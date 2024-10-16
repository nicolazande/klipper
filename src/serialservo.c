/**
 * \file serialservo.c
 *
 * \brief Handling of serialservo.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "serialservo.h" // drive_event
#include "trsync.h" // trsync_add_signal
#include "spicmds.h" // spidev_transfer
//#include "tmc4671.h" // TMC4671 control functions


/****************************************************************
 * Registers
 ****************************************************************/
#define CHIPINFO_DATA 0x00
#define CHIPINFO_ADDR 0x01
#define MOTOR_TYPE_N_POLE_PAIRS 0x1B
#define PID_TORQUE_P_TORQUE_I 0x56
#define PID_VELOCITY_P_VELOCITY_I 0x58
#define PID_POSITION_P_POSITION_I 0x5A
#define MODE_RAMP_MODE_MOTION 0x63
#define PID_TORQUE_FLUX_TARGET 0x64
#define PID_VELOCITY_TARGET 0x66
#define PID_POSITION_TARGET 0x68
#define PID_VELOCITY_ACTUAL 0x6A
#define PID_POSITION_ACTUAL 0x6B

#define ABN_DECODER_COUNT 0x27
#define PHI_E_SELECTION 0x52
#define VELOCITY_SELECTION 0x50
#define MODE_RAMP_MODE_MOTION 0x63
#define STATUS_FLAGS 0x7C
#define OPENLOOP_VELOCITY_TARGET 0x21
#define TMC4671_INPUTS_RAW 0x76
#define TMC4671_OUTPUTS_RAW 0x77


/****************************************************************
 * Defines
 ****************************************************************/
/* flags */
enum
{ 
    DF_NEED_RESET = 1<<0 
};

/* drive move step */
struct serialservo_move
{
    struct move_node node;
    int32_t target_position;
    int32_t target_velocity;
    uint32_t time;
    uint8_t flags;
};

/* drive data */
struct serialservo
{
    struct timer time;
    struct spidev_s *spi;
    uint8_t chain_pos;
    uint8_t chain_len;

    int32_t start_position;
    int32_t start_velocity;
    uint32_t start_time;

    int32_t current_position;
    int32_t current_velocity;
    uint32_t current_time;

    int32_t target_position;
    int32_t target_velocity;
    uint32_t target_time;

    int32_t delta_position;
    int32_t delta_velocity;
    int32_t delta_time;
    uint32_t interpolation_steps;

    uint32_t count;
    uint32_t sampling_time;

    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    uint8_t flags : 8;
};


uint32_t reg_read(struct spidev_s *spi, uint8_t address)
{
	/* read data packet */
	uint8_t msg[5] = {address & 0x7F, 0, 0, 0, 0};

    spidev_transfer(spi, 1, sizeof(msg), msg);
	
    return (uint32_t)((msg[1] << 24) | (msg[2] << 16) | (msg[3] << 8) | msg[4]);
}

void reg_write(struct spidev_s *spi, uint8_t address, int32_t value)
{
	/* write data packet */
	uint8_t msg[5] = 
    {
        address | 0x80,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF
    };

	spidev_transfer(spi, 0, sizeof(msg), msg);
}


/****************************************************************
 * Private function prototypes
 ****************************************************************/
/** setup a serialservo for the next move in its queue */
static uint_fast8_t serialservo_load_next(struct serialservo *s, uint32_t event_time);

/* interpolation step (position and velocity) */
static uint_fast8_t serialservo_interpolation_step(struct serialservo *d, uint32_t event_time);

/** get serialservo for a given drive oid */
static struct serialservo *serialservo_oid_lookup(uint8_t oid);

/** current serialservo position (caller must disable irqs) */
static int32_t serialservo_get_position(struct serialservo *s);

/* stop all moves for a given serialservo (caller must disable IRQs) */
static void serialservo_stop(struct trsync_signal *tss, uint8_t reason);


/****************************************************************
 * Command function prototypes
 ****************************************************************/
/* command to configure the serialservo */
void command_config_serialservo(uint32_t *args);

/** command to associate a spi bus */
void command_config_serialservo_spi(uint32_t *args);

/* command to queue a set of moves with a given timing */
void command_queue_serialservo(uint32_t *args);

/* command to reset the serialservo clock */
void command_reset_serialservo_clock(uint32_t *args);

/* command to get the current position of the serialservo */
void command_serialservo_get_position(uint32_t *args);

/* set the serialservo to stop on a trigger event */
void command_serialservo_stop_on_trigger(uint32_t *args);

/* shutdown command */
void serialservo_shutdown(void);


/****************************************************************
 * Private functions
 ****************************************************************/
static struct serialservo *
serialservo_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_serialservo);
}

static uint_fast8_t
serialservo_load_next(struct serialservo *d, uint32_t event_time)
{
    if (move_queue_empty(&d->mq))
    {
        /* queue is empty (no steps) */
        d->interpolation_steps = d->count = 0;
        return SF_DONE;
    }

    /* load next move */
    struct move_node *n = move_queue_pop(&d->mq);
    struct serialservo_move *m = container_of(n, struct serialservo_move, node);

    /* set timer for rescheduling event */
    if (timer_is_before(d->time.waketime, event_time))
    {
        d->time.waketime = event_time + d->sampling_time;
    }

    /* update boundaries */
    d->start_position = d->current_position = d->target_position;
    d->start_velocity = d->current_velocity = d->target_velocity;
    d->target_position = m->target_position;
    d->target_velocity = m->target_velocity;
    d->start_time = d->current_time = d->target_time;
    d->target_time = m->time;

    /* compute delta */
    d->delta_time = d->target_time - d->start_time;
    d->delta_position = d->target_position - d->current_position;
    d->delta_velocity = d->target_velocity - d->current_velocity;

    /* calculate interpolation steps */
    d->interpolation_steps = d->delta_time / d->sampling_time;
    d->count--;

    // output("loaded move: p=%i, v=%i, t=%u, et=%u, dt=%i, s=%u", 
    //         m->target_position, m->target_velocity, m->time,
    //         event_time, d->delta_time, d->interpolation_steps);

    /* delete move */
    move_free(m);

    return SF_RESCHEDULE;
}

static uint_fast8_t
serialservo_interpolation_step(struct serialservo *d, uint32_t event_time)
{
    /* compute delta */
    int32_t delta_time = event_time - d->start_time;
    delta_time = (delta_time < d->delta_time) ? delta_time : d->delta_time;
    int32_t delta_position = (delta_time * d->delta_position) / d->delta_time;
    int32_t delta_velocity = (delta_time * d->delta_velocity) / d->delta_time;

    /* update */
    d->current_position = d->start_position + delta_position;
    d->current_velocity = d->start_velocity + delta_velocity;
    d->current_time = d->start_time + delta_time;

    d->interpolation_steps--;
    d->time.waketime += d->sampling_time;
    
    return 0;
}

/* return the current serialservo position */
static int32_t
serialservo_get_position(struct serialservo *d)
{
    return d->current_position;
}

static void serialservo_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct serialservo *d = container_of(tss, struct serialservo, stop_signal);
    sched_del_timer(&d->time);
    /** NOTE: important for offset calculation */
    //d->current_position = tmc4671_read_actual_position();
    d->target_time = d->start_time = d->time.waketime = 0;
    d->interpolation_steps = d->count = 0;
    d->flags |= DF_NEED_RESET; //stop accepting new moves

    /* stop the motor by setting velocity and torque to zero */
    // tmc4671_set_target_velocity(0);
    // tmc4671_set_target_torque(0);

    while (!move_queue_empty(&d->mq))
    {
        struct move_node *mn = move_queue_pop(&d->mq);
        struct serialservo_move *m = container_of(mn, struct serialservo_move, node);
        move_free(m);
    }
}


/****************************************************************
 * Public functions
 ****************************************************************/
uint_fast8_t serialservo_event(struct timer *t)
{
    /* get timer serialservo */
    struct serialservo *d = container_of(t, struct serialservo, time);

    /* TMC position and velocity setpoint */
    reg_write(d->spi, OPENLOOP_VELOCITY_TARGET, d->current_velocity);

    int32_t position_feedback = (int32_t)reg_read(d->spi, PID_POSITION_ACTUAL);
    uint32_t input_status = reg_read(d->spi, TMC4671_INPUTS_RAW);
    uint32_t output_status = reg_read(d->spi, TMC4671_OUTPUTS_RAW);

    output("==> position (target = %i, feedback = %i, input_status = %u, output_status = %u)",
    d->current_position, position_feedback, input_status, output_status);

    uint32_t event_time = timer_read_time();
    
    /* perform interpolation and send setpoint */
    if (timer_is_before(event_time, d->target_time))
    {
        serialservo_interpolation_step(d, event_time);
        return SF_RESCHEDULE;
    }

    return serialservo_load_next(d, event_time);
}


/****************************************************************
 * Klippy commands
 ****************************************************************/
void command_config_serialservo(uint32_t *args)
{
    /* allocate oid for serialservo */
    struct serialservo *d = oid_alloc(args[0], command_config_serialservo, sizeof(*d));
    d->sampling_time = args[1];
    d->current_position = 0;
    d->current_velocity = 0;
    d->current_time = 0;
    d->count = 0;
    move_queue_setup(&d->mq, sizeof(struct serialservo_move));
    d->time.func = serialservo_event;

    /* initialize serialservo in position control mode */
    // tmc4671_init(TMC4671_MODE_POSITION);
}
DECL_COMMAND(command_config_serialservo, "config_serialservo oid=%c sampling_time=%u");

void command_config_serialservo_spi(uint32_t *args)
{
    struct serialservo *d = serialservo_oid_lookup(args[0]);
    d->spi = spidev_oid_lookup(args[1]);
    d->chain_len = args[2];
    d->chain_pos = args[3];
    /* reset encoder count */
    //reg_write(d->spi, ABN_DECODER_COUNT, 0);
    /* witch to feedback control mode */
    //reg_write(d->spi, PHI_E_SELECTION, 0x3);
    //reg_write(d->spi, VELOCITY_SELECTION, 0x9);
    /* witch to position mode for closed-loop control */
    //reg_write(d->spi, MODE_RAMP_MODE_MOTION, 0x3);
}
DECL_COMMAND(command_config_serialservo_spi, "config_serialservo_spi oid=%c spi_oid=%c chain_len=%c chain_pos=%c");

void command_queue_serialservo(uint32_t *args)
{ 
    /* get serialservo and create move */
    struct serialservo *d = serialservo_oid_lookup(args[0]);

    uint32_t req_time = args[3];
    uint32_t event_time = timer_read_time();

    if (timer_is_before(req_time, event_time + d->sampling_time))
    {
        return;
    }

    /* create new move */
    struct serialservo_move *m = move_alloc();

    /* get data */
    m->target_position = args[1];
    m->target_velocity = args[2];
    m->time = args[3];
    m->flags = 0;

    irq_disable();

    if (d->interpolation_steps)
    {
        //output("move in queue");
        move_queue_push(&m->node, &d->mq);
    }
    else if (d->flags & DF_NEED_RESET)
    {
        //output("move free");
        move_free(m);
    }
    else 
    {
        //output("move restart");
        d->target_time = event_time;
        move_queue_push(&m->node, &d->mq);
        serialservo_load_next(d, event_time);
        sched_add_timer(&d->time);
    }

    /* increse move counter */
    d->count++;

    irq_enable();
}
DECL_COMMAND(command_queue_serialservo, "serialservo_queue_step oid=%c target_position=%i target_velocity=%i time=%u");

void command_reset_serialservo_clock(uint32_t *args)
{
    struct serialservo *d = serialservo_oid_lookup(args[0]);
    uint32_t waketime = args[1];

    irq_disable();
    if (d->interpolation_steps)
    {
        shutdown("Can't reset time when serialservo active");
    }
    d->time.waketime = d->target_time = d->start_time = waketime;
    d->flags &= ~DF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_serialservo_clock, "serialservo_reset_step_clock oid=%c clock=%u");

void command_serialservo_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct serialservo *d = serialservo_oid_lookup(oid);
    irq_disable();
    int32_t position = serialservo_get_position(d);
    irq_enable();
    sendf("serialservo_position oid=%c pos=%i", oid, position);
}
DECL_COMMAND(command_serialservo_get_position, "serialservo_get_position oid=%c");

void command_serialservo_stop_on_trigger(uint32_t *args)
{
    struct serialservo *d = serialservo_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &d->stop_signal, serialservo_stop);
}
DECL_COMMAND(command_serialservo_stop_on_trigger, "serialservo_stop_on_trigger oid=%c trsync_oid=%c");

void serialservo_shutdown(void)
{
    uint8_t i;
    struct serialservo *d;
    foreach_oid(i, d, command_config_serialservo) {
        move_queue_clear(&d->mq);
        serialservo_stop(&d->stop_signal, 0);
    }
}
DECL_SHUTDOWN(serialservo_shutdown);