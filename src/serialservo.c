// Servo axis control via TMC4671 position/velocity setpoint streaming
//
// Copyright (C) 2024  Nicola Zandegiacomo <nicola.zandegiacomo@flyingbasket.com>
// Copyright (C) 2026  Warmbird
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "spicmds.h" // spidev_transfer
#include "trsync.h" // trsync_add_signal

// The host streams "be at target_position with target_velocity at
// clock" segments.  Between segment endpoints velocity is interpolated
// linearly and position follows the resulting constant-acceleration
// parabola - exact for the trapezoidal profiles the host generates.
// A pacing timer wakes a task which evaluates the interpolant at the
// actual transfer time (plus a fixed SPI lead) and writes the TMC4671
// setpoint registers.  All SPI access happens in task context; the
// shutdown de-energize path is handled by config_spi_shutdown messages
// registered by the host, never by code here.

// TMC4671 registers accessed at runtime (chip setup is host-side)
#define TMC4671_PID_VELOCITY_OFFSET  0x67
#define TMC4671_PID_POSITION_TARGET  0x68
#define TMC4671_PID_VELOCITY_ACTUAL  0x6A
#define TMC4671_PID_POSITION_ACTUAL  0x6B

enum {
    SF_ACTIVE = 1<<0, SF_NEED_RESET = 1<<1, SF_STOP_PENDING = 1<<2,
    SF_HAVE_TIME = 1<<3,
};

struct serialservo_move {
    struct move_node node;
    int32_t target_position, target_velocity;
    uint32_t clock;
};

struct serialservo {
    struct timer time;
    struct spidev_s *spi;
    uint32_t interp_ticks, eval_lead;
    uint32_t vel_scale; // position lsb per clock tick per rpm (<<32)
    uint32_t ferror_window, ferror_ticks, ferror_last;
    // Active segment: interpolate (p0,v0)@t0 -> (p1,v1)@t1
    int32_t p0, p1;
    int32_t v0, v1;
    uint32_t t0, t1;
    int32_t last_written_position;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    uint8_t flags;
};

static struct task_wake serialservo_wake;


/****************************************************************
 * TMC4671 register access (40 bit datagrams, task context only)
 ****************************************************************/

static uint32_t
tmc_reg_read(struct spidev_s *spi, uint8_t addr)
{
    uint8_t msg[5] = { addr & 0x7f, 0, 0, 0, 0 };
    spidev_transfer(spi, 1, sizeof(msg), msg);
    return ((uint32_t)msg[1] << 24) | ((uint32_t)msg[2] << 16)
        | ((uint32_t)msg[3] << 8) | msg[4];
}

static void
tmc_reg_write(struct spidev_s *spi, uint8_t addr, uint32_t val)
{
    uint8_t msg[5] = { addr | 0x80, val >> 24, val >> 16, val >> 8, val };
    spidev_transfer(spi, 0, sizeof(msg), msg);
}


/****************************************************************
 * Segment interpolation
 ****************************************************************/

// Interpolated velocity (register units, electrical rpm) at t0 + dt
static int32_t
serialservo_calc_velocity(struct serialservo *s, uint32_t dt)
{
    uint32_t seg_ticks = s->t1 - s->t0;
    if (!seg_ticks || dt >= seg_ticks)
        return s->v1;
    return s->v0 + (int32_t)((int64_t)(s->v1 - s->v0) * dt / seg_ticks);
}

// Interpolated position (register units) at t0 + dt.  Velocity varies
// linearly across the segment, so displacement is the trapezoid
// integral: vel_scale * (v0*dt + (v1-v0)*dt^2/(2*T)).
static int32_t
serialservo_calc_position(struct serialservo *s, uint32_t dt)
{
    uint32_t seg_ticks = s->t1 - s->t0;
    if (!seg_ticks || dt >= seg_ticks)
        return s->p1;
    // Divide by seg_ticks before the second dt factor so the
    // intermediate cannot overflow int64 even for very long (anchor)
    // segments
    int64_t inner = (int64_t)s->v0 * dt;
    inner += (int64_t)(s->v1 - s->v0) * dt / seg_ticks * dt / 2;
    int64_t disp = (inner * s->vel_scale) >> 32;
    return s->p0 + (int32_t)disp;
}

// Load the next queued segment (caller must disable irqs)
static int
serialservo_load_next(struct serialservo *s)
{
    if (move_queue_empty(&s->mq))
        return -1;
    struct move_node *n = move_queue_pop(&s->mq);
    struct serialservo_move *m = container_of(
        n, struct serialservo_move, node);
    s->p0 = s->p1;
    s->v0 = s->v1;
    s->t0 = s->t1;
    s->p1 = m->target_position;
    s->v1 = m->target_velocity;
    s->t1 = m->clock;
    move_free(m);
    return 0;
}

// Pacing timer - wake the servo task for the next setpoint update
static uint_fast8_t
serialservo_event(struct timer *t)
{
    struct serialservo *s = container_of(t, struct serialservo, time);
    sched_wake_task(&serialservo_wake);
    s->time.waketime += s->interp_ticks;
    return SF_RESCHEDULE;
}

// Halt streaming on endstop trigger or shutdown (irq context)
static void
serialservo_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct serialservo *s = container_of(
        tss, struct serialservo, stop_signal);
    sched_del_timer(&s->time);
    s->flags = (s->flags & ~(SF_ACTIVE|SF_HAVE_TIME))
        | SF_NEED_RESET | SF_STOP_PENDING;
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct serialservo_move *m = container_of(
            mn, struct serialservo_move, node);
        move_free(m);
    }
    sched_wake_task(&serialservo_wake);
}


/****************************************************************
 * Servo update task
 ****************************************************************/

// Hold at the measured position and cancel feed-forward (Cat 2 stop -
// power-off paths are covered by config_spi_shutdown messages)
static void
serialservo_do_stop(struct serialservo *s)
{
    if (!s->spi)
        return;
    int32_t actual = tmc_reg_read(s->spi, TMC4671_PID_POSITION_ACTUAL);
    tmc_reg_write(s->spi, TMC4671_PID_VELOCITY_OFFSET, 0);
    tmc_reg_write(s->spi, TMC4671_PID_POSITION_TARGET, actual);
    s->last_written_position = actual;
}

static void
serialservo_update(struct serialservo *s)
{
    uint32_t now = timer_read_time();
    uint32_t eval_time = now + s->eval_lead;
    // Advance past completed segments
    for (;;) {
        irq_disable();
        if (!(s->flags & SF_ACTIVE)) {
            irq_enable();
            return;
        }
        if (timer_is_before(eval_time, s->t1)) {
            irq_enable();
            break;
        }
        int ret = serialservo_load_next(s);
        if (ret) {
            // Stream complete - write final target and go idle
            sched_del_timer(&s->time);
            s->flags &= ~SF_ACTIVE;
            irq_enable();
            tmc_reg_write(s->spi, TMC4671_PID_VELOCITY_OFFSET, s->v1);
            tmc_reg_write(s->spi, TMC4671_PID_POSITION_TARGET, s->p1);
            s->last_written_position = s->p1;
            return;
        }
        irq_enable();
    }
    uint32_t dt = eval_time - s->t0;
    if ((int32_t)dt < 0)
        dt = 0;
    int32_t pos = serialservo_calc_position(s, dt);
    int32_t vel = serialservo_calc_velocity(s, dt);
    tmc_reg_write(s->spi, TMC4671_PID_VELOCITY_OFFSET, vel);
    tmc_reg_write(s->spi, TMC4671_PID_POSITION_TARGET, pos);
    s->last_written_position = pos;
    // Periodic following error supervision
    if (s->ferror_window && !timer_is_before(now, s->ferror_last
                                             + s->ferror_ticks)) {
        s->ferror_last = now;
        int32_t actual = tmc_reg_read(s->spi, TMC4671_PID_POSITION_ACTUAL);
        int32_t ferror = pos - actual;
        if (ferror < 0)
            ferror = -ferror;
        if ((uint32_t)ferror > s->ferror_window)
            shutdown("serialservo following error exceeds window");
    }
}

void
command_config_serialservo(uint32_t *args);

void
serialservo_task(void)
{
    if (!sched_check_wake(&serialservo_wake))
        return;
    uint8_t oid;
    struct serialservo *s;
    foreach_oid(oid, s, command_config_serialservo) {
        irq_disable();
        uint8_t flags = s->flags;
        s->flags = flags & ~SF_STOP_PENDING;
        irq_enable();
        if (!s->spi)
            continue;
        if (flags & SF_STOP_PENDING) {
            serialservo_do_stop(s);
            continue;
        }
        if (flags & SF_ACTIVE)
            serialservo_update(s);
    }
}
DECL_TASK(serialservo_task);


/****************************************************************
 * Host commands
 ****************************************************************/

void
command_config_serialservo(uint32_t *args)
{
    struct serialservo *s = oid_alloc(
        args[0], command_config_serialservo, sizeof(*s));
    s->interp_ticks = args[1];
    s->vel_scale = args[2];
    s->eval_lead = args[3];
    s->ferror_window = args[4];
    s->ferror_ticks = args[5];
    if (!s->interp_ticks)
        shutdown("Invalid serialservo interp_ticks parameter");
    move_queue_setup(&s->mq, sizeof(struct serialservo_move));
    s->time.func = serialservo_event;
    s->flags = SF_NEED_RESET;
}
DECL_COMMAND(command_config_serialservo,
             "config_serialservo oid=%c interp_ticks=%u vel_scale=%u"
             " eval_lead=%u ferror_window=%u ferror_ticks=%u");

static struct serialservo *
serialservo_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_serialservo);
}

void
command_config_serialservo_spi(uint32_t *args)
{
    struct serialservo *s = serialservo_oid_lookup(args[0]);
    s->spi = spidev_oid_lookup(args[1]);
    if (!spidev_have_cs_pin(s->spi))
        shutdown("serialservo requires cs pin");
}
DECL_COMMAND(command_config_serialservo_spi,
             "config_serialservo_spi oid=%c spi_oid=%c");

void
command_serialservo_queue_step(uint32_t *args)
{
    struct serialservo *s = serialservo_oid_lookup(args[0]);
    if (!s->spi)
        shutdown("serialservo spi not configured");
    struct serialservo_move *m = move_alloc();
    m->target_position = args[1];
    m->target_velocity = args[2];
    m->clock = args[3];
    uint32_t now = timer_read_time();
    irq_disable();
    uint8_t flags = s->flags;
    if (flags & SF_NEED_RESET) {
        move_free(m);
    } else if (!timer_is_before(now, m->clock)) {
        irq_enable();
        shutdown("serialservo target clock in the past");
    } else if (flags & SF_ACTIVE) {
        move_queue_push(&m->node, &s->mq);
    } else {
        // Restart streaming from the current hold state.  A stale
        // time base (idle gap approaching the 32 bit clock wrap) is
        // re-anchored to now - the host's burst-start hold anchor
        // makes this a zero-motion segment either way.
        if (!(flags & SF_HAVE_TIME)
            || m->clock - s->t1 >= 0x40000000) {
            s->t1 = now;
            s->v1 = 0;
        }
        s->p0 = s->p1;
        s->v0 = s->v1;
        s->t0 = s->t1;
        s->p1 = m->target_position;
        s->v1 = m->target_velocity;
        s->t1 = m->clock;
        move_free(m);
        s->flags = (flags | SF_ACTIVE | SF_HAVE_TIME);
        s->ferror_last = now;
        s->time.waketime = now + s->interp_ticks;
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_serialservo_queue_step,
             "serialservo_queue_step oid=%c target_position=%i"
             " target_velocity=%i clock=%u");

void
command_serialservo_reset_step_clock(uint32_t *args)
{
    struct serialservo *s = serialservo_oid_lookup(args[0]);
    irq_disable();
    if (s->flags & SF_ACTIVE) {
        irq_enable();
        shutdown("Can't reset time when serialservo active");
    }
    s->flags &= ~(SF_NEED_RESET|SF_HAVE_TIME);
    irq_enable();
    // Re-anchor the interpolation state to the measured position
    if (s->spi) {
        int32_t actual = tmc_reg_read(s->spi, TMC4671_PID_POSITION_ACTUAL);
        s->p0 = s->p1 = actual;
        s->last_written_position = actual;
    }
    s->v0 = s->v1 = 0;
}
DECL_COMMAND(command_serialservo_reset_step_clock,
             "serialservo_reset_step_clock oid=%c clock=%u");

void
command_serialservo_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct serialservo *s = serialservo_oid_lookup(oid);
    int32_t pos = s->last_written_position;
    if (s->spi)
        pos = tmc_reg_read(s->spi, TMC4671_PID_POSITION_ACTUAL);
    sendf("serialservo_position oid=%c pos=%i", oid, pos);
}
DECL_COMMAND(command_serialservo_get_position,
             "serialservo_get_position oid=%c");

void
command_serialservo_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct serialservo *s = serialservo_oid_lookup(oid);
    uint32_t clock = timer_read_time();
    int32_t actual = 0, velocity = 0;
    if (s->spi) {
        actual = tmc_reg_read(s->spi, TMC4671_PID_POSITION_ACTUAL);
        velocity = tmc_reg_read(s->spi, TMC4671_PID_VELOCITY_ACTUAL);
    }
    sendf("serialservo_state oid=%c clock=%u target=%i actual=%i"
          " velocity=%i", oid, clock, s->last_written_position, actual
          , velocity);
}
DECL_COMMAND(command_serialservo_query_state,
             "serialservo_query_state oid=%c");

void
command_serialservo_stop_on_trigger(uint32_t *args)
{
    struct serialservo *s = serialservo_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, serialservo_stop);
}
DECL_COMMAND(command_serialservo_stop_on_trigger,
             "serialservo_stop_on_trigger oid=%c trsync_oid=%c");

void
serialservo_shutdown(void)
{
    uint8_t oid;
    struct serialservo *s;
    foreach_oid(oid, s, command_config_serialservo) {
        sched_del_timer(&s->time);
        s->flags = (s->flags & ~(SF_ACTIVE|SF_HAVE_TIME|SF_STOP_PENDING))
            | SF_NEED_RESET;
        move_queue_clear(&s->mq);
    }
}
DECL_SHUTDOWN(serialservo_shutdown);
