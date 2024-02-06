/**
 * \file pvtcompress.c
 *
 * \brief Step buffering and synchronization.
 * 
 * Take a set of steps, buffer and reorder (synchronize) them.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdint.h> // uint32_t
#include <stdio.h> // fprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // DIV_ROUND_UP
#include "pyhelper.h" // errorf
#include "ethercatqueue.h" // struct pvtmsg
#include "pvtcompress.h" // pvtcompress_alloc


/****************************************************************
 * Defines
 ****************************************************************/
#define HISTORY_EXPIRE (30.0) //history time window in seconds
#define CLOCK_DIFF_MAX (3<<28) //maximium clock delta between messages in the queue
#define SEQ_NUM_MASK (0b00000111)


/****************************************************************
 * Custom data types
 ****************************************************************/
/*
 * The pvtcompress object is used to represent each drive state,
 * to track timing and scheduled steps.
 */
struct pvtcompress
{
    double mcu_time_offset; //offset for converting print time to mcu time
    double mcu_freq; //drive frequency
    double last_step_print_time; //print time of the last scheduled move
    uint64_t last_step_clock; //drive clock value of the last scheduled move
    struct list_head msg_queue; //linked list of commands (moves) to be executed on the drive
    uint32_t oid; //object id associated with the pvt step compressor (unique for each drive)
    uint32_t seq_num; //sequence number of last scheduled step
    double last_position; //last known position of the drive
    struct list_head history_list; //linked list of historical drive information
};

/* step history */
struct pvthistory
{
    struct list_node node;
    uint64_t first_clock;
    uint64_t last_clock;
    double start_position;
    double velocity;
};

/*
 * The drivesync object is used to syncronize the output of pvt step commands.
 * The drives can only queue a limited number of step commands, this code
 * tracks when items on the drive step queue become free so that new commands
 * can be transmitted. It also ensures the pvt step queue is ordered between
 * drives (no drive starves).
 */
struct drivesync
{
    struct ethercatqueue *sq; //ethercat queue
    struct command_queue *cq; //private command queue (isolate from unrelated commands)
    struct pvtcompress **sc_list; //storage for associated pvtcompress objects (one for each drive)
    int sc_num; //number of associated drives
    uint64_t *move_clocks; //list of pending move clocks
    int num_move_clocks; //size of move_clocks list
};


/****************************************************************
 * Private functions
 ****************************************************************/
/** free items from the history list up to end_clock */
static void
free_history(struct pvtcompress *sc, uint64_t end_clock)
{
    while (!list_empty(&sc->history_list))
    {
        struct pvthistory *hs = list_last_entry(&sc->history_list, struct pvthistory, node);
        if (hs->last_clock > end_clock)
        {
            break;
        }
        list_del(&hs->node);
        free(hs);
    }
}

/** determine the print time of the last scheduled step */
static void
calc_last_step_print_time(struct pvtcompress *sc)
{
    /* last drive step clock */
    double lsc = sc->last_step_clock;

    /* convert it to host print time */
    sc->last_step_print_time = sc->mcu_time_offset + (lsc - .5) / sc->mcu_freq;

    /* clear old part of history queue */
    if (unlikely(lsc > sc->mcu_freq * HISTORY_EXPIRE))
    {
        free_history(sc, lsc - sc->mcu_freq * HISTORY_EXPIRE);
    }
}

/** 
 * Set the conversion rate from host print time to mcu clock.
 * This function is indirectly called by the mcu module in
 * check_active() and adjust the time offset and frequency
 * with respect to the main mcu. Nothe that this time is not
 * criticat since is used only for ordering the steps.
 */
static void
pvtcompress_set_time(struct pvtcompress *sc, double time_offset, double mcu_freq)
{
    sc->mcu_time_offset = time_offset;
    sc->mcu_freq = mcu_freq;
    calc_last_step_print_time(sc);
}

/** append a move step command */
void
pvtcompress_append(struct pvtcompress *sc, struct pose *pose, double move_time)
{
    /* update next move clock (wrt main mcu clock) */
    double first_offset = pose->time - sc->last_step_print_time;
    double last_offset = first_offset + move_time;

    /* get time interval for the move */
    uint64_t first_clock = sc->last_step_clock;
    uint64_t last_clock = first_clock + (uint64_t)(last_offset * sc->mcu_freq);

    /* create a queue message */
    struct pvtmsg *qm = malloc(sizeof(*qm));
    memset(qm, 0, sizeof(*qm));

    /**
     * Cast the queue message buffer to a pvt move and fill it (avoid redundant copies).
     * This can be done only because the queue message msg field is guaranteed to be 8
     * bytes aligned.
     * TODO: check Copley manualfor proper position, velocity, time format, ...
     */
    struct pvtmove *move = (struct pvtmove *)qm->msg;
    move->header.type = 0; //0 = buffer mode, 1 = command mode
    move->header.seq_num = sc->seq_num & SEQ_NUM_MASK; //step sequence number
    move->header.format = 0; //0 = buffer mode, 1 = command mode
    move->position = pose->position; //move absolute start position [ticks].
    move->velocity = pose->velocity; //move constant velocity [ticks/s].
    move->time = (uint8_t)(move_time * 1000.); //move time duration [ms] (up to next pose)

    /*
     * Queue messages from different pvtcompress objects are merged in a single
     * command queue, but in the ethercat frame each drive pdo has a specific
     * position, therefore we need to store the source stepcompress object id
     * (drive) in each pvt queue mesage.
     */
    qm->oid = sc->oid;
    qm->len = sizeof(*move); //message data size

    /* assign min clock and required clock (same during initialization) */
    qm->min_clock = sc->last_step_clock;
    qm->req_clock = sc->last_step_clock;

    // /* take into account high delay between steps */
    // uint64_t first_clock_offset = (uint64_t)((first_offset - .5) * sc->mcu_freq);
    // if (first_clock_offset > CLOCK_DIFF_MAX)
    // {
    //     /* 
    //      * The delta time between current and last step is too high to
    //      * be ignored, therefore move forward the required clock and
    //      * accept a delay between steps (this happens for both X and Y
    //      * axis --> no positioning error).
    //      */
    //     qm->req_clock = first_clock + first_clock_offset;
    // }

    /* add message to queue */
    list_add_tail(&qm->node, &sc->msg_queue);

    /* update stepcompress time (start of next move) */
    sc->last_step_clock = last_clock;

    /* update stepcompress position (start of next move) */
    sc->last_position = pose->position + pose->velocity * move_time;

    /* update step sequence number (avoid overflow) */
    sc->seq_num++ % (SEQ_NUM_MASK + 1);

    /* create and store move in history tracking */
    struct pvthistory *hs = malloc(sizeof(*hs));
    hs->first_clock = first_clock;
    hs->last_clock = last_clock;
    hs->start_position = pose->position;
    hs->velocity = pose->velocity;
    list_add_head(&hs->node, &sc->history_list);

    /* 
     * Open loop update of step print time, the correction update based on
     * the mcu time offset is done periodically through the serial module.
     */
    calc_last_step_print_time(sc);
}

/** 
 * Implement a binary heap algorithm to track when the next available
 * pvt move the will be available.
 */
static void
heap_replace(struct drivesync *ss, uint64_t req_clock)
{
    uint64_t *mc = ss->move_clocks;
    int nmc = ss->num_move_clocks;
    int pos = 0;
    for (;;)
    {
        int child1_pos = 2*pos+1;
        int child2_pos = 2*pos+2;
        uint64_t child2_clock = child2_pos < nmc ? mc[child2_pos] : UINT64_MAX;
        uint64_t child1_clock = child1_pos < nmc ? mc[child1_pos] : UINT64_MAX;
        if (req_clock <= child1_clock && req_clock <= child2_clock)
        {
            mc[pos] = req_clock;
            break;
        }
        if (child1_clock < child2_clock)
        {
            mc[pos] = child1_clock;
            pos = child1_pos;
        }
        else
        {
            mc[pos] = child2_clock;
            pos = child2_pos;
        }
    }
}


/****************************************************************
 * Public functions
 ****************************************************************/
/** allocate a new pvtcompress object */
struct pvtcompress * __visible
pvtcompress_alloc(uint32_t oid)
{
    struct pvtcompress *sc = malloc(sizeof(*sc));
    memset(sc, 0, sizeof(*sc));
    list_init(&sc->msg_queue);
    list_init(&sc->history_list);
    sc->oid = oid;
    return sc;
}

/** free memory associated with a pvtcompress object */
void __visible
pvtcompress_free(struct pvtcompress *sc)
{
    if (!sc)
    {
        return;
    }
    while (!list_empty(&sc->msg_queue))
    {
        struct pvtmsg *qm = list_first_entry(&sc->msg_queue, struct pvtmsg, node);
        list_del(&qm->node);
        free(qm);
    }
    free_history(sc, UINT64_MAX);
    free(sc);
}

/** get object id of a pvtcompress object */
uint32_t
pvtcompress_get_oid(struct pvtcompress *sc)
{
    return sc->oid;
}

/** reset the internal state of the pvtcompress object */
int __visible
pvtcompress_reset(struct pvtcompress *sc, uint64_t last_step_clock)
{
    sc->last_step_clock = last_step_clock;
    calc_last_step_print_time(sc);
    return 0;
}

/** set last_position in the pvtcompress object */
int __visible
pvtcompress_set_last_position(struct pvtcompress *sc, uint64_t clock, double last_position)
{
    /* update last position */
    sc->last_position = last_position;

    /* add a marker to the history list */
    struct pvthistory *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->first_clock = hs->last_clock = clock;
    hs->start_position = last_position;
    list_add_head(&hs->node, &sc->history_list);
    return 0;
}

/** search history of moves to find a past position at a given clock */
double __visible
pvtcompress_find_past_position(struct pvtcompress *sc, uint64_t clock)
{
    double last_position = sc->last_position;
    struct pvthistory *hs;

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
 * TODO: check compatibility with pvthistory new format.
 */
int __visible
pvtcompress_extract_old(struct pvtcompress *sc,
                        struct pull_history_pvt_steps *p,
                        int max,
                        uint64_t start_clock,
                        uint64_t end_clock)
{
    /* data */
    int res = 0;
    struct pvthistory *hs;

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

/** allocate a new drivesync object */
struct drivesync * __visible
drivesync_alloc(struct ethercatqueue *sq, struct pvtcompress **sc_list, int sc_num, int move_num)
{
    /* create drive synchronization object */
    struct drivesync *ss = malloc(sizeof(*ss));
    memset(ss, 0, sizeof(*ss));
    ss->sq = sq; //assign ethercatqueue
    ss->cq = ethercatqueue_alloc_commandqueue(); //initialize command queue

    /* setup compressor list (one for each drive) */
    ss->sc_list = malloc(sizeof(*sc_list)*sc_num); 
    memcpy(ss->sc_list, sc_list, sizeof(*sc_list)*sc_num);
    ss->sc_num = sc_num;

    /* setup common move clock list (for scheduling and synchronization) */
    ss->move_clocks = malloc(sizeof(*ss->move_clocks)*move_num);
    memset(ss->move_clocks, 0, sizeof(*ss->move_clocks)*move_num);
    ss->num_move_clocks = move_num;

    return ss;
}

/** free memory associated with a drivesync object */
void __visible
drivesync_free(struct drivesync *ss)
{
    if (!ss)
    {
        return;
    }
    free(ss->sc_list);
    free(ss->move_clocks);
    ethercatqueue_free_commandqueue(ss->cq);
    free(ss);
}

/** 
 * Set the conversion rate of print time to mcu clock. This function is
 * indirectly alled by the mcu module in the check_active() function.
 */
void __visible
drivesync_set_time(struct drivesync *ss, double time_offset, double mcu_freq)
{
    /* loop over associated drives */
    for (int i = 0; i < ss->sc_num; i++)
    {
        struct pvtcompress *sc = ss->sc_list[i];
        pvtcompress_set_time(sc, time_offset, mcu_freq);
    }
}

/** 
 * Find and transmit any scheduled steps prior to the given move_clock for
 * the drivesync common object, i.e. reorder and merge into a single queue
 * the steps of all associated drives (in the command queue).
 */
int __visible
drivesync_flush(struct drivesync *ss, uint64_t move_clock)
{
    /* create common message list */
    struct list_head msgs;
    list_init(&msgs);

    errorf(".");
    errorf(" --> sent move_clock = %u", move_clock);
    errorf(".");

    /* order commands by requested clock for each drive */
    for (;;)
    {
        /* find message with lowest req_clock */
        uint64_t req_clock = MAX_CLOCK;
        struct pvtmsg *qm = NULL;

        /* 
         * Loop over drive compressor queues and get the min time among each drive compressor
         * queues (first element), this ensures all drives are fed with similar priority.
         */
        for (uint8_t i = 0; i < ss->sc_num; i++)
        {
            /* get compressor */
            struct pvtcompress *sc = ss->sc_list[i];
            if (!list_empty(&sc->msg_queue))
            {
                /* get requested clock of first step for the selected drive */
                struct pvtmsg *m = list_first_entry(&sc->msg_queue, struct pvtmsg, node);

                /* compare it with the current min (from previous drives) */
                if (m->req_clock < req_clock)
                {
                    qm = m;
                    req_clock = m->req_clock;
                }
            }
        }

        /* 
         * Check if the selected step is inside the flush time window.
         * Since the compressor queues are ordered (by earliest required time)
         * and min_clock <= req_clock, stop at the first step outside the
         * flush time window, it will be scheduled in the next cycle.
         */
        if ((!qm) || (qm->min_clock && req_clock > move_clock))
        {
            /* stop, all required steps selected */
            break;
        }

        /* 
         * Get next available move clock among all drives (computed in the previous cycle).
         * It represents the earliest possible scheduling time, i.e. the end time of the
         * last scheduled (added to msgs queue) move.
         */
        uint64_t next_avail = ss->move_clocks[0];

        if (qm->min_clock)
        {
            /* 
             * The qm->min_clock field is overloaded to indicate that the command uses the move
             * queue and to store the time that move queue item becomes available. Here min_clock
             * is used instead of req_clock specifically for command scheduling priority and it
             * can change at runtime, while req_time is the required time setpoint.
             * In this way the min_clock of the current step is taken into account and used to
             * update the heap (ordered queue) of min clocks for the next step, maintaining the
             * synchronization between all drives. The reason to maintain the heap is to quickly
             * insert a new min_clock and at the same time know the earliest next scheduling
             * clock, i.e. only the heap root is used in this logic.
             */
            heap_replace(ss, qm->min_clock);
        }
        /* 
         * Reset the min_clock to its normal meaning, the end time of the last
         * scheduled move step (from previous cycle).
         */
        qm->min_clock = next_avail;

        errorf("sent: min_clock = %u, req_clock = %u, oid = %u", qm->min_clock, qm->req_clock, qm->oid);

        /* add to message queue */
        list_del(&qm->node);
        list_add_tail(&qm->node, &msgs);
    }

    if (!list_empty(&msgs))
    {
        /* transmit atch of steps */
        ethercatqueue_send_batch(ss->sq, ss->cq, &msgs);
    }
    
    return 0;
}
