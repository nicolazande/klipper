/**
 * \file stepcompress.c
 *
 * \brief Step buffering and synchronization.
 * 
 * Take a set of steps, buffer and reorder (synchronize) them.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "compiler.h"
#include "pyhelper.h"
#include "serialqueue.h"
#include "serialservo_compress.h"


/****************************************************************
 * Defines
 ****************************************************************/
#define HISTORY_EXPIRE (30.0) //history time window in seconds
#define CLOCK_DIFF_MAX (3<<28) //maximium clock delta between messages in the queue


/****************************************************************
 * Custom data types
 ****************************************************************/
/*
 * The stepcompress object is used to represent each drive state,
 * to track timing and scheduled steps.
 */
struct stepcompress 
{
    // Buffer management
    uint32_t *queue, *queue_end, *queue_pos, *queue_next;
    // Internal tracking
    uint32_t max_error;
    double mcu_time_offset, mcu_freq, last_step_print_time;
    // Message generation
    uint64_t last_step_clock;
    struct list_head msg_queue;
    uint32_t oid;
    int32_t queue_step_msgtag, set_next_step_dir_msgtag;
    union
    {
        int sdir;
        int polePairs; //number of pole pairs on motor
    };
    union
    {
        int invert_sdir;
        int scaler; //integer scaler for position and velocity
    };
    // Step+dir+step filter
    uint64_t next_step_clock;
    int next_step_dir;
    // History tracking
    double last_position;
    struct list_head history_list;
};

struct history_steps 
{
    struct list_node node;
    uint64_t first_clock;
    uint64_t last_clock;
    double start_position;
    double velocity;
};

/****************************************************************
 * Private function prototypes
 ****************************************************************/
/** determine the print time of the last scheduled step */
static inline void calc_last_step_print_time(struct stepcompress *sc);

/** free items from the history list up to end_clock */
static inline void free_history(struct stepcompress *sc, uint64_t end_clock);


/****************************************************************
 * Private functions
 ****************************************************************/
static inline void
calc_last_step_print_time(struct stepcompress *sc)
{    
    /* last drive step clock */
    double lsc = sc->last_step_clock;

    /* convert it to host print time */
    sc->last_step_print_time = sc->mcu_time_offset + (lsc - .5) / sc->mcu_freq;
}

static inline void
free_history(struct stepcompress *sc, uint64_t end_clock)
{
    while (!list_empty(&sc->history_list))
    {
        struct history_steps *hs = list_last_entry(&sc->history_list, struct history_steps, node);
        if (hs->last_clock > end_clock)
        {
            break;
        }
        list_del(&hs->node);
        free(hs);
    }
}


/****************************************************************
 * Public functions
 ****************************************************************/
/** allocate a new stepcompress object */
struct stepcompress * __visible
serialservo_compress_alloc(uint32_t oid)
{
    struct stepcompress *sc = malloc(sizeof(*sc));
    memset(sc, 0, sizeof(*sc));
    list_init(&sc->msg_queue);
    list_init(&sc->history_list);
    sc->oid = oid;
    return sc;
}

/** fill message id information */
void __visible
serialservo_compress_fill(struct stepcompress *sc,
                          int32_t queue_step_msgtag,
                          int32_t polePairs,
                          int32_t scaler)
{
    sc->queue_step_msgtag = queue_step_msgtag;
    sc->polePairs = polePairs;
    sc->scaler = scaler;
}

/** free memory associated with a stepcompress object */
void __visible
serialservo_compress_free(struct stepcompress *sc)
{
    if (!sc)
    {
        return;
    }
    while (!list_empty(&sc->msg_queue))
    {
        struct queue_message *qm = list_first_entry(&sc->msg_queue, struct queue_message, node);
        list_del(&qm->node);
        free(qm);
    }
    free_history(sc, UINT64_MAX);
    free(sc);
}

/** get object id of a stepcompress object */
uint32_t
serialservo_compress_get_oid(struct stepcompress *sc)
{
    return sc->oid;
}

/** reset the internal state of the stepcompress object */
int __visible
serialservo_compress_reset(struct stepcompress *sc, uint64_t last_step_clock)
{
    sc->last_step_clock = last_step_clock;
    calc_last_step_print_time(sc);
    return 0;
}

/** set last_position in the stepcompress object */
double __visible
serialservo_compress_set_last_position(struct stepcompress *sc, uint64_t clock, int64_t last_position)
{
    /* update last position */
    sc->last_position  = ((double)last_position * sc->scaler) / (65536.0 * sc->polePairs);
    /* add a marker to the history list */
    struct history_steps *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->first_clock = hs->last_clock = clock;
    hs->start_position = sc->last_position;
    list_add_head(&hs->node, &sc->history_list);
    return sc->last_position;
}

/** search history of moves to find a past position at a given clock */
double __visible
serialservo_compress_find_past_position(struct stepcompress *sc, uint64_t clock)
{
    double last_position = sc->last_position;
    struct history_steps *hs;

    /* loop over history steps */
    list_for_each_entry(hs, &sc->history_list, node)
    {
        if (clock < hs->first_clock)
        {
            /* move back to previous step */
            last_position = hs->start_position;
            continue;
        }
        if (clock >= hs->last_clock)
        {
            /* 
             * Last step or high delay between steps. since thre is not the
             * next step, it is not possible to linearize between them, for
             * this reason the best option is to return the last known
             * move step start position.
             */
            return hs->start_position;
        }

        /* linearize in between first and last clock (constant velocity) */
        double interval = (clock - hs->first_clock)/sc->mcu_freq;
        return hs->start_position + interval*hs->velocity;
    }
    /* last position, clock is too recent */
    return last_position;
}

/** 
 * Return history of queue_step commands.
 */
int __visible
serialservo_compress_extract_old(struct stepcompress *sc,
                                 struct pull_history_serialservo_steps *p,
                                 int max,
                                 uint64_t start_clock,
                                 uint64_t end_clock)
{
    /* data */
    int res = 0;
    struct history_steps *hs;

    /* loop over step history queue */
    list_for_each_entry(hs, &sc->history_list, node)
    {
        if ((start_clock >= hs->last_clock) || (res >= max))
        {
            /* stop (no more steps in time window) */
            break;
        }
        if (end_clock <= hs->first_clock)
        {
            /* move backwords */
            continue;
        }

        /* populate new entry of step history pull array (moving backwards) */
        p->first_clock = hs->first_clock;
        p->last_clock = hs->last_clock;
        p->start_position = hs->start_position;

        /* update */
        p++; //move forward in step history pull array
        res++; //increase step counter
    }

    return res;
}

/** append step to compessor */
void
serialservo_compress_append(struct stepcompress *sc, struct pose *pose, double move_time)
{
    /* update next move clock (wrt main mcu clock) */
    double first_offset = pose->time - sc->last_step_print_time;
    double last_offset = first_offset + move_time;

    /* get time interval for the move */
    uint64_t first_clock = sc->last_step_clock;
    uint64_t last_clock = first_clock + (uint64_t)(last_offset * sc->mcu_freq);

    /**
     * Position conversion calculation:
     *   scaler    = [mm / mechanicalRotation]
     *   polePairs = [electricalRotation / mechanicalRotation]
     *   internal  = [1/65536 * electricalRotation]
     *   
     *   linear                              = [mm]
     *   linear / scaler                     = [mechanicalRotation]
     *   linear / scaler * polePairs         = [electricalRotation]
     *   linear / scaler * polePairs * 65536 = [1/65536 * electricalRotation]   
     */
    int32_t mcu_position = (int32_t)(pose->position * 65536 * sc->polePairs / sc->scaler);

    /**
     * Velocity onversion calculation:
	 *   scaler    = [mm / mechanicalRotation]
	 *   time:       [60s / minute]
     *   internal  = [mechanicalRotation / minute]
	 *
	 *   linear               = [mm / s]
	 *   linear / scaler      = [mechanicalRotation / s]
	 *   linear / scaler * 60 = [mechanicalRotation / minute]
     */
    int32_t mcu_velocity = (int32_t)(pose->velocity * 60. / sc->scaler);

    /* move time in mcu ticks */
    uint32_t mcu_time = (uint32_t)/*(move_time * sc->mcu_freq); */sc->last_step_clock;

    /* create and queue a queue step command */
    uint32_t msg[] = 
    {
        sc->queue_step_msgtag,
        sc->oid,
        mcu_position,
        mcu_velocity,
        mcu_time
    };

    /* allocate new queue mesage */
    struct queue_message *qm = message_alloc_and_encode(msg, ARRAY_SIZE(msg));

    /* assign min clock and required clock (same during initialization) */
    qm->min_clock = sc->last_step_clock;
    qm->req_clock = sc->last_step_clock;

    // errorf("-> serialservo [tag = %i, len = %u]: p = %i, v = %i, t = %u",
    // sc->queue_step_msgtag, (uint32_t)qm->len, (int32_t)qm->msg[2], (int32_t)qm->msg[3], qm->req_clock);

    /* take into account high delay between steps */
    uint64_t first_clock_offset = (uint64_t)((first_offset - .5) * sc->mcu_freq);
    if (first_clock_offset > CLOCK_DIFF_MAX)
    {
        /* 
         * The delta time between current and last step is too high to
         * be ignored, therefore move forward the required clock and
         * accept a delay between steps (this happens for both X and Y
         * axis --> no positioning error).
         */
        qm->req_clock = first_clock + first_clock_offset;
    }

    /* add message to queue */
    list_add_tail(&qm->node, &sc->msg_queue);

    /* update stepcompress time (start of next move) */
    sc->last_step_clock = last_clock;

    /* create and store move in history tracking */
    struct history_steps *hs = malloc(sizeof(*hs));
    hs->first_clock = first_clock;
    hs->last_clock = last_clock;
    hs->start_position = pose->position;
    hs->velocity = pose->velocity;
    list_add_head(&hs->node, &sc->history_list);

    /* open loop update of step print time */
    calc_last_step_print_time(sc);
}

int __visible
serialservo_compress_queue_msg(struct stepcompress *sc, uint32_t *data, int len)
{
    struct queue_message *qm = message_alloc_and_encode(data, len);
    qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}