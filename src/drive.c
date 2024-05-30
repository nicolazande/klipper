/**
 * \file drive.c
 *
 * \brief Handling of drive drivers.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "drive.h" // drive_event
#include "trsync.h" // trsync_add_signal


/****************************************************************
 * Defines
 ****************************************************************/
/* flags */
enum
{
    SF_LAST_DIR = 1<<0,
    SF_NEXT_DIR = 1<<1,
    SF_INVERT_STEP = 1<<2,
    SF_NEED_RESET = 1<<3,
    SF_SINGLE_SCHED = 1<<4,
    SF_HAVE_ADD = 1<<5
};


/****************************************************************
 * Custom data types
 ****************************************************************/
/* drive move step */
struct drive_move
{
    struct move_node node;
    int32_t position;
    int32_t velocity;
    uint32_t time;
    uint8_t flags;
};

/* drive data */
struct drive
{
    struct timer time;
    uint32_t next_step_time;
    uint32_t step_pulse_ticks;
    struct gpio_out step_pin;
    struct gpio_out dir_pin;
    int32_t position;
    int32_t velocity;
    uint32_t time;
    uint16_t count;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    uint8_t flags : 8;
};


/****************************************************************
 * Private function prototypes
 ****************************************************************/
/** setup a drive for the next move in its queue */
static uint_fast8_t drive_load_next(struct drive *s);

/** get drive for a given drive oid */
static struct drive *drive_oid_lookup(uint8_t oid);

/** current drive position (caller must disable irqs) */
static int32_t drive_get_position(struct drive *s);

/* stop all moves for a given drive (caller must disable IRQs) */
static void drive_stop(struct trsync_signal *tss, uint8_t reason);


/****************************************************************
 * Public function prototypes
 ****************************************************************/
/* drive event function */
uint_fast8_t drive_event(struct timer *t);

/** configure drive */
void command_config_drive(uint32_t *args);

/** schedule a set of steps with a given timing */
void drive_queue_step(uint32_t *args);

/** 
 * Set absolute time that the next step will be relative to.
 * NOTE: use in combination with drive control word.
 */
void drive_reset_step_clock(uint32_t *args);

/** report the current position of the drive */
void command_drive_get_position(uint32_t *args);

/** stop drive on trigger event (used in homing) */
void command_drive_stop_on_trigger(uint32_t *args);

/** drive shutdown function */
void drive_shutdown(void);


/****************************************************************
 * Private functions
 ****************************************************************/
static uint_fast8_t
drive_load_next(struct drive *s)
{
    if (move_queue_empty(&s->mq))
    {
        /* queue is empty */
        s->count = 0;
        return SF_DONE;
    }

    /* load next move */
    struct move_node *n = move_queue_pop(&s->mq);
    struct drive_move *m = container_of(n, struct drive_move, node);

    /* update drive data */
    s->position = m->position;
    s->velocity = m->velocity;
    s->next_step_time += m->time;
    s->time.waketime = s->next_step_time;
    s->count++;

    /* remove step */
    move_free(m);

    return SF_RESCHEDULE;
}

static struct drive *
drive_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_drive);
}

static int32_t
drive_get_position(struct drive *s)
{
    return s->position;
}

static void
drive_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct drive *s = container_of(tss, struct drive, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = drive_get_position(s);
    s->count = 0;
    s->flags = (s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED)) | SF_NEED_RESET;

    /* stop TMC */
    gpio_out_write(s->dir_pin, 0);

    while (!move_queue_empty(&s->mq))
    {
        /* remove all steps in queue */
        struct move_node *mn = move_queue_pop(&s->mq);
        struct drive_move *m = container_of(mn, struct drive_move, node);
        move_free(m);
    }
}


/****************************************************************
 * Public functions
 ****************************************************************/
uint_fast8_t
drive_event(struct timer *t)
{
    /* get drive associated with timer */
    struct drive *s = container_of(t, struct drive, time);

    /* command position setpoint */
    if (s->count)
    {
        gpio_out_toggle_noirq(s->step_pin);
        s->count--;
    }
    if (!s->count)
    {
        /* load next move */
        drive_load_next(s);
    }

    return SF_DONE;
}

void
command_config_drive(uint32_t *args)
{
    struct drive *s = oid_alloc(args[0], command_config_drive, sizeof(*s));
    // int_fast8_t invert_step = args[3];
    // s->flags = invert_step > 0 ? SF_INVERT_STEP : 0;
    // s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    // s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = 0;
    s->velocity = 0;
    s->time = 0;
    move_queue_setup(&s->mq, sizeof(struct drive_move));
    s->time.func = drive_event;
}
DECL_COMMAND(command_config_drive, "config_drive oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

void
drive_queue_step(uint32_t *args)
{
    /* get drive and create move */
    struct drive *s = drive_oid_lookup(args[0]);
    struct drive_move *m = move_alloc();

    /* read message data */
    m->position = args[1];
    m->velocity = args[2];
    m->time = args[3];
    m->flags = 0;

    /* disable interrupts */
    irq_disable();

    /* update flags */
    uint8_t flags = s->flags;

    if (s->count)
    {
        /* move queue nnt empty */
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    }
    else if (flags & SF_NEED_RESET)
    {
        /* move queue empty and reset */
        move_free(m);
    }
    else
    {
        /* move queue empty */
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        drive_load_next(s);
        sched_add_timer(&s->time);
    }

    /* enable interrupts */
    irq_enable();
}
DECL_COMMAND(drive_queue_step,
             "drive_queue_step oid=%c position=%i velocity=%i time=%u");

void
drive_reset_step_clock(uint32_t *args)
{
    /* get drive */
    struct drive *s = drive_oid_lookup(args[0]);

    /* absolute time (clock) */
    uint32_t waketime = args[1];

    /* disable interrupts */
    irq_disable();

    if (s->count)
    {
        shutdown("Can't reset time when drive active");
    }

    /* update drive wake time */
    s->next_step_time = s->time.waketime = waketime;

    /* clear reset flag */
    s->flags &= ~SF_NEED_RESET;

    /* enable interrupts */
    irq_enable();
}
DECL_COMMAND(drive_reset_step_clock, "drive_reset_step_clock oid=%c clock=%u");

void
command_drive_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct drive *s = drive_oid_lookup(oid);
    irq_disable();
    int32_t position = drive_get_position(s);
    irq_enable();
    sendf("drive_position oid=%c pos=%i", oid, position);
}
DECL_COMMAND(command_drive_get_position, "drive_get_position oid=%c");

void
command_drive_stop_on_trigger(uint32_t *args)
{
    struct drive *s = drive_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, drive_stop);
}
DECL_COMMAND(command_drive_stop_on_trigger,
             "drive_stop_on_trigger oid=%c trsync_oid=%c");

void
drive_shutdown(void)
{
    uint8_t i;
    struct drive *s;
    foreach_oid(i, s, command_config_drive)
    {
        move_queue_clear(&s->mq);
        drive_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(drive_shutdown);
