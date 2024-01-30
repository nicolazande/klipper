/**
 * \file ethercatqueue.c
 *
 * \brief CANopen over EtherCAT low level utilities.
 *
 * The goal of this code is to handle low-level CANopen over EtherCAT communication
 * directly with the drives. This code is written in C (instead of python) to reduce
 * communication latencies and to reduce scheduling jitter. The code queues messages
 * to be transmitted, schedules transmission of commands at specified clock times,
 * prioritizes commands. A background thread is launched to minimize latency.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <math.h> //fabs
#include <pthread.h> //pthread_mutex_lock
#include <stddef.h> //offsetof
#include <stdint.h> //uint64_t
#include <stdio.h> //snprintf
#include <stdlib.h> //malloc
#include <string.h> //memset
#include <termios.h> //tcflush
#include <unistd.h> //pipe
#include "compiler.h" //__visible
#include "list.h" //list_add_tail
#include "msgblock.h" // message_alloc
#include "pollreactor.h" //pollreactor_alloc
#include "pyhelper.h" //get_monotonic
#include "ethcatqueue.h" //struct pvtmsg


/****************************************************************
 * Defines
 ****************************************************************/
/* file descriptors */
#define EQPF_PIPE   0               //index of rx-cmd pipe (pipe where application stores cmd to be sent)
#define EQPF_NUM    1               //number of file descriptors used by a ethcatqueue
/* timers */
#define EQPT_INPUT      0           //position of input timer in ethcatqueue timer list
#define EQPT_COMMAND    1           //position of command timer in ethcatqueue timer list
#define EQPT_PROTOCOL   2           //position of command timer in ethcatqueue timer list
#define EQPT_NUM        3           //number of timer used by a ethcatqueue
/* transport layer */
#define EQT_ETHCAT      'e'         //id for ethernet transport layer (osi 2)
#define EQT_DEBUGFILE   'f'         //id for output to debug file (no commnication)
/* time and memory limits */
#define WR_TIME_OFFSET        0     //offset between write and next read operation (without frame_time)
#define MIN_REQTIME_DELTA 0.250     //min delta time (in advance) to send a command
/* helpers */
#define TIMES2NS(time) ((uint64_t)(time * 1e9)) //convert internal time to nanaoseconds
#define HANDLE_ERROR(condition, exit) if(condition) {goto exit;} //error handling
/* switches */
#define MESSAGE_CHECK_FORMAT (0U)


/****************************************************************
 * Data
 ****************************************************************/
/* external command parser table */
extern struct command_parser *command_parser_table[ETH_MAX_CP];


/****************************************************************
 * Private function prototypes
 ****************************************************************/
/**
 * Wake up the ethercat high level thread if it is waiting. It needs
 * to further process the messages in the receive queue.
 */
static void
check_wake_receive(struct ethcatqueue *sq);

/**
 * Write to the ethcatqueue internal pipe to force the execution 
 * of the background low level ethercat immediately (don't wait
 * for associated timer). This function is called asynchronously
 * by the high level thread since it cannot update directly the
 * command event timer.
 */
static void kick_bg_thread(struct ethcatqueue *sq, uint8_t cmd);

/** 
 * Process a single request from the high level thread, execute the associated
 * callback and append the result to the response queue so that the high level
 * thread can read and further process it. The process is always synchronous
 * and only one command at a time can be processed since there is a single
 * instance of sharedmonitor in the ethcatqueue.
 */
static double process_request(struct ethcatqueue *sq, double eventtime);

/**
 * Read EtherCAT frame coming back from the drives and update internal structures.
 */
static double
input_event(struct ethcatqueue *sq, double eventtime);

/**
 * Precess a request coming from the EtherCAT high level thread.
 */
static double
protocol_event(struct ethcatqueue *sq, double eventtime);

/**
 * Callback for input activity on the pipe. This function is continuosly
 * run by the poll reactor and, as soon as data (dummy bytes) is found
 * on the rx side of the internal pipe, it updates the command timer (to
 * zero), indirectly forcing an execution of command_event(). This is
 * the low level thread counterpart of kick_bg_thread.
 */
static void
kick_event(struct ethcatqueue *sq, double eventtime);

/** 
 * Populate ethercat domain for the current cycle, all mapped objects
 * are transmitted in the current cycle frame.
 */
static int
build_and_send_command(struct ethcatqueue *sq);

/**
 * Determine schedule time of next command event and move messages
 * from pending queue to ready queue for each drive.
 */
static double
check_send_command(struct ethcatqueue *sq, int pending, double eventtime);

/* callback timer to send data to the ethercat port */
static double
command_event(struct ethcatqueue *sq, double eventtime);

/** main background thread for reading/writing to ethcat port */
static void *
background_thread(void *data);


/****************************************************************
 * Private functions
 ****************************************************************/
static void
check_wake_receive(struct ethcatqueue *sq)
{
    if (sq->receive_waiting)
    {
        /* signal condition to ethcatqueue_pull */
        sq->receive_waiting = 0;
        pthread_cond_signal(&sq->cond);
    }
}

static void
kick_bg_thread(struct ethcatqueue *sq, uint8_t cmd)
{
    /* write a dummy value just to wake up the poll reactor */
    int ret = write(sq->pipe_sched[1], &cmd, 1);
    if (ret < 0)
    {
        report_errno("pipe write", ret);
    }
}

static double process_request(struct ethcatqueue *sq, double eventtime)
{
    double next_event = PR_NEVER;

    /* check request from high level thread */
    if (!list_empty(&sq->request_queue))
    {
        /* data */
        struct sharedmonitor *ifc = &sq->klippyifc; //shared interface
        struct queue_message *request = list_first_entry(&sq->request_queue, struct queue_message, node); //request messgae
        struct queue_message *response = message_alloc(); //response message (TODO: populate sent and receive time)
        uint8_t msglen; //message length in byte
        const struct command_parser *cp; //command parser

#if MESSAGE_CHECK_FORMAT
        /* check message format (sync, crc, ...) */
        if (check_command(request->msg, MESSAGE_MAX, &msglen) <= 0)
        {
            return;
        }
#else
        /* get request length in bytes (with header and trailer) */
        msglen = request->msg[MESSAGE_POS_LEN];
#endif

        /* get boundaries */
        uint8_t *in = &request->msg[MESSAGE_HEADER_SIZE];
        uint8_t *inend = &request->msg[msglen-MESSAGE_TRAILER_SIZE];
        uint8_t *out = response->msg;
        uint8_t *outend = &response->msg[MESSAGE_MAX-MESSAGE_TRAILER_SIZE];

        /* loop over request bytes (allow multiple commands per message) */
        while (in < inend)
        {
            /* get command id and move forward to arguments */
            uint8_t cmdid = *in++;

            /* check command */
            if (cmdid < ETH_MAX_CP)
            {
                /* get command parser */
                cp = ifc->cp_table[cmdid];
            }
            else
            {
                /* unknown command */
                break;
            }
            
            /* variable length array for arguments */
            uint32_t args[cp->num_args];

            /* parse an incoming command into args */
            in = command_parsef(in, inend, cp, args);
            
            /* get request handler */
            int (*func)(struct ethcatqueue *, void *, uint32_t *) = cp->func;

            /* check callback */
            if (func)
            {
                /* check if command response fits msg buffer */
                if (out >= outend)
                {
                    /* save old message reference */
                    struct queue_message *old_response = response;

                    /* append previous response to response queue */
                    list_add_tail(response, &sq->response_queue);
                    
                    /* create response message and update indexes */
                    response = malloc(sizeof(*response));
                    response->notify_id = old_response->notify_id;
                    response->sent_time = old_response->sent_time;
                    response->receive_time = old_response->receive_time;
                    out = response->msg;
                    outend = &response->msg[MESSAGE_MAX-MESSAGE_TRAILER_SIZE];
                }

                /**
                 * Execute command specific handler. NOTE: out is updated by func
                 * according to response size, also in case of out of bounds so
                 * that a new response message can be created automatically.
                 */
                int ret = func(sq, out, args);
                if (ret)
                {
                    message_free(response);
                    return PR_NOW;
                }
            }
        }

        /* remove message from request queue */
        list_del(&request->node);

        /* update response message */
        response->len = response->msg[MESSAGE_POS_LEN];
        response->receive_time = eventtime;

        if (request->notify_id)
        {
            /* message requires notification (add to notify list) */
            request->req_clock = sq->send_seq;
            list_add_tail(&request->node, &sq->notify_queue);
        }
        else
        {
            /* delete request message */
            message_free(request);
        }

        /* 
         * Add response to response queue, in case of error it will
         * notify the high level thread which will take action.
         */
        if (response->len > 0)
        {
            list_add_tail(&response->node, &sq->response_queue);
        }
        else
        {
            message_free(response);
        }
    }

    //uint32_t rseq_delta = ((response->msg[MESSAGE_POS_SEQ] - sq->receive_seq) & MESSAGE_SEQ_MASK);
    //uint64_t rseq = sq->receive_seq + rseq_delta;
    while (!list_empty(&sq->notify_queue))
    {
        struct queue_message *qm = list_first_entry(&sq->notify_queue, struct queue_message, node);
        // uint64_t wake_seq = rseq - 1 - (len > MESSAGE_MIN ? 1 : 0);
        // uint64_t notify_msg_sent_seq = qm->req_clock;
        // if (notify_msg_sent_seq > wake_seq)
        // {
        //     break;
        // }
        list_del(&qm->node);
        qm->len = 0;
        qm->sent_time = 0; //sq->last_receive_sent_time;
        qm->receive_time = eventtime; //eventtime;
        list_add_tail(&qm->node, &sq->response_queue);
    }

    /* wake up high level thread */
    check_wake_receive(sq);

    return next_event;
};

static double
input_event(struct ethcatqueue *sq, double eventtime)
{
    /* get master interface */
    struct mastermonitor *ifc = &sq->masterifc;
    
    /* receive process data */
    ecrt_master_receive(ifc->master);

    /* process received common data (from datagram to domain) */
    ecrt_domain_process(ifc->domain);

    /** update domain state (TODO: add error handling) */
    ecrt_domain_state(ifc->domain, &ifc->domain_state);

    /* read associated slaves window status (as soon as possible) */
    for (uint8_t i = 0; i < ETHCAT_DRIVES; i++)
    {
        ifc->monitor[i].slave_window = EC_READ_U32(ifc->domain_pd + ifc->monitor[i].off_slave_window);
    }

    /* process a high level thread request */
    //process_request(sq);

    /*wake up high level thread (will process response) */
    check_wake_receive(sq);

    /* reset input event timer (command_event will reschedule it when needed)  */
    return PR_NEVER;
}

static double
protocol_event(struct ethcatqueue *sq, double eventtime)
{
    double next_event = PR_NEVER;

    pthread_mutex_lock(&sq->lock);

    /* process a high level thread request */
    next_event = process_request(sq, eventtime);

    pthread_mutex_unlock(&sq->lock);

    return next_event;
};

static void
kick_event(struct ethcatqueue *sq, double eventtime)
{
    /* protocol command */
    uint8_t cmd;
    /* read rx-command pipe */
    int ret = read(sq->pipe_sched[0], &cmd, sizeof(cmd));
    /* check data */
    if (ret < 0)
    {
        report_errno("pipe read", ret);
    }
    /* update command timer (force ethercat command transmission) */
    pollreactor_update_timer(sq->pr, cmd, PR_NOW);
}

static int
build_and_send_command(struct ethcatqueue *sq)
{
    /* data */
    int len = 0; //number of bytes added in to the current frame
    struct mastermonitor *master = &sq->masterifc; //ethercat master interface
    struct slavemonitor *slave; //ethercat slave interface
    
    /* 
     * Loop over bytes ready to be transmitted. This is like going through
     * the ready queue of all command queues (normally just one).
     * Send only if there is free space in the drive pvt buffers.
     */
    while (sq->ready_bytes > 0)
    {
        /* data */
        uint64_t min_clock = MAX_CLOCK; //min clock
        struct command_queue *q; //iterator for ethercatqueue associated command queues
        struct command_queue *cq = NULL; //command queue
        struct pvtmsg *qm = NULL; //step queue message

        /* 
         * Find highest priority message, the one with the lowest req_clock among all
         * command queues. The single ready queues are already ordered therefore it
         * is sufficient to check the first message of each drive queue.
         */
        list_for_each_entry(q, &sq->pending_queues, node)
        {
            /* check list */
            if (!list_empty(&q->ready_queue))
            {
                /* get first message in ordered ready queue */
                struct pvtmsg *m = list_first_entry(&q->ready_queue, struct pvtmsg, node);
                /* check for priority */
                if (m->req_clock < min_clock)
                {
                    min_clock = m->req_clock;
                    cq = q;
                    qm = m;
                }
            }
        }

        /* get target slave */
        if (qm->oid < ETHCAT_DRIVES)
        {
            slave = &master->monitor[qm->oid];
        }
        else
        {
            /* oid out of bounds */
            report_errno("ethercat oid out of bounds", 0);
            return 0;
        }

        /* check for available space */
        if ((slave->master_window < slave->tx_size) && (slave->slave_window < slave->rx_size))    
        {
            /* populate domain data (directly mapped to kernel) */
            memcpy(slave->pvtdata[slave->master_window], qm->msg, qm->len);

            /* increase slave counter in domain */
            master->pvtdomain[slave->master_window].mask |= (1 << qm->oid);
            
            /* increase master tx index */
            slave->master_window++;

            /* increase slave rx index */
            slave->slave_window++;
        }
        else
        {
            /* no space (tx or rx) */
            master->full_counter = 1;

            /* stop as soon as one of the drive buffers (tx or rx) is full */
            break;
        }

        /* remove message from ready queue */
        list_del(&qm->node);

        /* delete ready queue and upcoming queue if both empty */
        if (list_empty(&cq->ready_queue) && list_empty(&cq->upcoming_queue))
        {
            /* remove message from command_queue */
            list_del(&cq->node);
        }

        /* update stats (messages with wrong old are discarded) */
        len += qm->len;
        sq->ready_bytes -= qm->len;

        /* delete message (message content copied) */
        free(qm);
    }

    /* total bytes sent */
    return len;
}

static double
check_send_command(struct ethcatqueue *sq, int pending, double eventtime)
{
    /* data */
    struct mastermonitor *master = &sq->masterifc; //ethercat master interface
    struct slavemonitor *slave; //ethercat slave interface

    /* 
     * Check for free buffer slots on slave side only. There is no need to check tx side
     * since when this function is called it is guaranteed to have all pdo slots free.
     */
    for (uint8_t i = 0; i < ETHCAT_DRIVES; i++)
    {
        /* get slave */
        slave = &master->monitor[i];

        if ((slave->slave_window >= slave->rx_size) /* || (sq->masterifc.full_counter) */)
        {
            /* stop (drive buffer is full) */
            return PR_NEVER;
        }
    }

    /* time parameters */
    double idletime = eventtime > sq->idle_time ? eventtime : sq->idle_time; //current host time (limited by idle time)
    idletime += master->frame_time; //current transmission estimated drive reception time (frame is back to master).
    uint64_t ack_clock = clock_from_time(&sq->ce, idletime); //current transmission estimated drive reception clock
    uint64_t min_stalled_clock = MAX_CLOCK; //min clock among stalled messages (for next cycle)
    uint64_t min_ready_clock = MAX_CLOCK; //min required clock among pending messages

    /* check for upcoming messages now ready (loop over pending queues) */
    struct command_queue *cq; //single command queue (normally just one)
    list_for_each_entry(cq, &sq->pending_queues, node)
    {
        /* move messages from the upcoming queue to the ready queue */
        while (!list_empty(&cq->upcoming_queue))
        {
            /* get first message in queue */
            struct pvtmsg *qm = list_first_entry(&cq->upcoming_queue, struct pvtmsg, node);
            /* 
             * Select only the pending messages that can be sent before the current estimated drive
             * reception clock. In this case qm->min_clock represents the earliest time when the
             * message can be sent, if it is greater than the estimated frame reception time
             * (ack_clock) stop immediately.
             */
            if (ack_clock < qm->min_clock)
            {
                if (qm->min_clock < min_stalled_clock)
                {
                    /* 
                     * The current message is not added to the ready queue, however the
                     * min_stalled_clock is updated for the next function call (start
                     * with the min possible value of min_stalled_clock).
                     */
                    min_stalled_clock = qm->min_clock;
                }
                /*
                 * The ack clock (estimated drive reception clock) is greater than the pending queue
                 * min clock, i.e. the first message in the ordered upcoming queue is still too far
                 * in the future, skip the entire queue and move to next command queue.
                 */
                break;
            }
            /* remove message from upcoming queue */
            list_del(&qm->node);
            /* add message from ready queue */
            list_add_tail(&qm->node, &cq->ready_queue);
            /* update stats */
            sq->upcoming_bytes -= qm->len;
            sq->ready_bytes += qm->len;
        }

        /* check updated ready queue */
        if (!list_empty(&cq->ready_queue))
        {
            /* get first message in ordered ready queue */
            struct pvtmsg *qm = list_first_entry(&cq->ready_queue, struct pvtmsg, node);
            /* clock at which time the first queue message has to be executed on drive side */
            uint64_t req_clock = qm->req_clock;
            /* get min ready clock among command queues */
            if (req_clock < min_ready_clock)
            {
                min_ready_clock = req_clock;
            }
        }
    }

    /* 
     * Check if the current cycle pvt domains data is ready, a drive 
     * specific check is performed in build_and_send_command(), but
     * this avoids to have an undefinitely growing ready queue.
     */
    if (sq->ready_bytes >= master->frame_size)
    {
        /* 
         * Cannot add more bytes, data has to be transmitted in the current cycle.
         * The remaining messages will be transmitted in the next cycle.
         */
        return PR_NOW;
    }
    /* check clock estimate */
    if (!sq->ce.est_freq)
    {
        /* no clock estimate specified (initialization, reset, error) */
        if (sq->ready_bytes)
        {
            /* 
             * Something to send, add to transmission buffer. This may
             * happen in configuration phase where drive parameters
             * can be tuned also asynchronously but shouldn't happen.
             */
            return PR_NOW;
        }
        /* disable timer and stop (error or stopped) */
        sq->need_kick_clock = MAX_CLOCK;
        return PR_NEVER;
    }

    /* 
     * Calculate the amount of time in advance a command has to be sent to the
     * drive before being executed: higher it is, safer the operation is,
     * but fuller the drive buffer will be (potentially completely full).
     * By taking into account also the synch cycle time we ensure that the
     * next command_event() is at least MIN_REQTIME_DELTA in advance with
     * respect to the earliest schedulable step.
     */
    uint64_t reqclock_delta = (master->sync0_ct + MIN_REQTIME_DELTA) * sq->ce.est_freq;

    /* check ready queue timing requirements */
    if (min_ready_clock <= ack_clock + reqclock_delta)
    {
        /* 
         * Next command event time is too late to send ready queue 
         * messages, add them to the transmission buffer and send 
         * them in the current command event.
         */
        return PR_NOW;
    }

    /*
     * Min requested clock for scheduling next command event without
     * taking into account the synch cycle time, i.e. in normal
     * operation timer scheduling happens always earlier than the
     * poll one, but if it fails we have a second way to send the
     * commands in time (sq->need_kick_clock).
     */
    uint64_t wantclock = min_ready_clock - (MIN_REQTIME_DELTA * sq->ce.est_freq);

    /* take into account min clock of stalled messages */
    if (min_stalled_clock < wantclock)
    {
        /* 
         * This condition should not happen (upcoming_queue should be
         * ordered), however if it is not the case, update wantclock
         * so that it takes into account the first non scheduled
         * message in the pending queues.
         */
        wantclock = min_stalled_clock;
    }

    /* 
     * Update ethercatqueue poll option, wantclock should always be
     * greater than the next timer command_event() execution time.
     */
    sq->need_kick_clock = wantclock;

    /* 
     * Ideal walue of next command_event(), unused since the ethercat
     * synch cycle time has to be constant.
     */
    return idletime + (wantclock - ack_clock) / sq->ce.est_freq;
}

static double
command_event(struct ethcatqueue *sq, double eventtime)
{
    /* acquire mutex */
    pthread_mutex_lock(&sq->lock);

    /* data */
    struct mastermonitor *master = &sq->masterifc; //ethercat master interface
    int buflen = 0; //length (in bytes) of data to be transmitted
    double waketime; //wake time of next command event

    /** set master application time (TODO: check delayed read operation) */
    ecrt_master_application_time(master->master, TIMES2NS(eventtime));

    /**
     * Synchronize the reference clock (first DC capable slave) with the
     * master internal time. TODO: this operation doesn't need to be
     * performed every cycle, therefore add a counter enable logic.
     */
    ecrt_master_sync_reference_clock_to(master->master, TIMES2NS(eventtime));

    /** 
     * Queue the DC clock drift compensation datagram, all slaves are
     * synchronized with the reference clock (first DC capable slave).
     */
    ecrt_master_sync_slave_clocks(master->master);

    /* prepare data for frame transmission */
    for (;;)
    {
        /**
         * Check step timing constraints, move messages from upcoming to ready
         * queue so that they can be transmitted in the current event.
         * The waketime variable is not used as reference time for the next
         * command event, since the delta time between two consecutive events
         * is consatant and equal to the sync cycle time.
         */
        waketime = check_send_command(sq, buflen, eventtime);

        /* 
         * Check if there is enought time and space to buffer upcoming commands:
         *  - if waketime > 0: ready queue commands can be added to the ethercat frame
         *                     and transmitted in the next cycle, send the frame now.
         *  - if waketime == 0: ready queue commands have to be added to the ethercat
         *                      frame and transmitted in the current cycle, pending
         *                      queue commands can still be added to ready queue.
         *  - if there are too many messages in the transmission buffer (pdo instances)
         *    of the domain and ethercat frame, a transmission needs to be performed.
         *    This is an extreme case where more than one frame per synch cycle needs
         *    to be sent --> find a proper way to handle this (i.e. synch datagram
         *    doesn't need to be added to the frame).
         */
        if ((waketime != PR_NOW) || (master->full_counter))
        {
            /* upate common domain datagram (always) */
            ecrt_domain_queue(master->domain);

            /* check if there is something to send */
            if (buflen)
            {
                /* loop over domains */
                for (uint8_t i = 0; i < ETHCAT_PVT_DOMAINS; i++)
                {
                    /* get domain */
                    struct pvtdomain *domain = master->pvtdomain[i].domain;

                    /* data consistency check (step for each axis) */
                    if (domain->mask == ETHCAT_DRIVE_MASK)
                    {
                        /* update domain */
                        ecrt_domain_queue(master->pvtdomain[i].domain);

                        /* reset domain drive mask */
                        domain->mask = 0;
                    }
                    else
                    {
                        /* stop (domains are ordered) */
                        break;
                    }
                }

                /* update idle time (take into account current transmission) */
                double idletime = (eventtime > sq->idle_time ? eventtime : sq->idle_time);
                sq->idle_time = idletime + master->frame_time;

                /* reset transmission parameters */
                buflen = 0; //buffer free
                master->full_counter = 0; //pdo slots free

                /* reset slaves tx counter */
                for (uint8_t i = 0; i < ETHCAT_DRIVES; i++)
                {
                    master->monitor[i].master_window = 0;
                }
            }

            /* write ethercat frame (always) */
            ecrt_master_send(master->master);

            if (waketime != PR_NOW)
            {
                /* stop (frame transmitted and nothing else to send) */
                break;
            }
        }

        /* build or update data to be sent */
        buflen += build_and_send_command(sq);
    }

    /* set input event timer (when the frame is expected to be received back) */
    pollreactor_update_timer(sq->pr, EQPT_INPUT, sq->idle_time + WR_TIME_OFFSET);

    /* releas mutex */
    pthread_mutex_unlock(&sq->lock);

    /* update next timer event (in pollreactor_check_timers) */
    return eventtime + master->sync0_ct;
}

static void *
background_thread(void *data)
{
    /* pointer to shared ethercatqueue */
    struct ethcatqueue *sq = data;

    /* 
     * Cyclic function checking for timers and fd events and invoking
     * their callbacks. This function runs until the thread exit.
     */
    pollreactor_run(sq->pr);

    /* acquire mutex */
    pthread_mutex_lock(&sq->lock);

    /* 
     * Wake up receiver thread (ethercat high level) for
     * last message parsing using klipper main reactor.
     */
    check_wake_receive(sq);

    /* release mutex */
    pthread_mutex_unlock(&sq->lock);
    return NULL;
}


/****************************************************************
 * Public functions
 ****************************************************************/
/** initialize ethercat slave */
void __visible
ethcatqueue_slave_config(struct ethcatqueue *sq,
                         uint8_t index,
                         uint16_t alias,
                         uint16_t position,
                         uint32_t vendor_id,
                         uint32_t product_code)
{
    struct mastermonitor *master = &sq->masterifc;
    struct slavemonitor *slave = &master->monitor[index];
    slave->alias = alias;
    slave->position = position;
    slave->vendor_id = vendor_id;
    slave->product_code = product_code;
}

/** configure a list of pdos for a sync manager of an ethercat slave */
void __visible
ethcatqueue_slave_config_pdos(struct ethcatqueue *sq,
                              uint8_t slave_index,
                              uint8_t sync_index,
                              uint8_t direction,
                              uint8_t n_pdo_entries,
                              ec_pdo_entry_info_t *pdo_entries,
                              uint8_t n_pdos,
                              ec_pdo_info_t *pdos)
{
    struct mastermonitor *master = &sq->masterifc;
    struct slavemonitor *slave = &master->monitor[slave_index];

    slave->n_pdo_entries = n_pdo_entries;
    for (uint8_t i = 0; i < n_pdo_entries; i++)
    {
        slave->pdo_entries[i] = pdo_entries[i];
    }

    slave->n_pdos = n_pdos;
    for (uint8_t i = 0; i < n_pdos; i++)
    {
        slave->pdos[i].index = pdos[i].index;
        slave->pdos[i].n_entries = pdos[i].n_entries;
        uint8_t position = (uint8_t)pdos[i].entries;
        slave->pdos[i].entries = &slave->pdo_entries[position];
    }

    uint8_t local_index = direction - 1;
    if (local_index < EC_DIR_OUTPUT)
    {
        slave->syncs[local_index].index = sync_index;
        slave->syncs[local_index].dir = direction;
        slave->syncs[local_index].n_pdos = n_pdos;
        slave->syncs[local_index].pdos = &slave->pdos[0];
    }
}

/** configure ethercat slave private registers */
void __visible
ethcatqueue_slave_config_registers(struct ethcatqueue *sq,
                                   uint8_t index,
                                   uint8_t n_registers,
                                   ec_pdo_entry_reg_t *registers)
{
    struct mastermonitor *master = &sq->masterifc;
    struct slavemonitor *slave = &master->monitor[index];
    
    slave->n_registers = n_registers;
    for (uint8_t i = 0; i < n_registers; i++)
    {
        slave->registers[i] = registers[i];
    }
}

/** configure ethercat master common registers */
void __visible 
ethcatqueue_master_config_registers(struct ethcatqueue *sq,
                                    uint8_t n_registers,
                                    ec_pdo_entry_reg_t *registers)
{
    struct mastermonitor *master = &sq->masterifc;

    for (uint8_t i = 0; i < n_registers; i++)
    {
        master->registers[i] = registers[i];
    }
}

/** create an empty ethcatqueue object */
struct ethcatqueue * __visible
ethcatqueue_alloc(void)
{
    /* allocate serialqueue */
    struct ethcatqueue *sq = malloc(sizeof(*sq));
    memset(sq, 0, sizeof(*sq));

    return sq;
}

/** initialize ethcatqueue */
int __visible
ethcatqueue_init(struct ethcatqueue *sq)
{
    /* shared error code */
    int ret;

    /* get ethercat master interface */
    struct mastermonitor *master = &sq->masterifc;

    /* get ethercat master */
    master->master = ecrt_request_master(0);
    HANDLE_ERROR(!master->master, klipper)

    /* create common data domain */
    master->domain = ecrt_master_create_domain(master->master);
    HANDLE_ERROR(!master->domain, klipper)

    /* create pvt specific domains */
    for (uint8_t i = 0; i < ETHCAT_PVT_DOMAINS; i++)
    {
        master->pvtdomain[i].domain = ecrt_master_create_domain(master->master);
        HANDLE_ERROR(!master->pvtdomain[i].domain, klipper)
    }

    /* initialize ethercat slaves */
    for (uint8_t i = 0; i < ETHCAT_DRIVES; i++)
    {
        /* get slave monitor */
        struct slavemonitor *slave = &master->monitor[i];

        /* create slave configuration */
        ec_slave_config_t *sc = ecrt_master_slave_config(master->master,
                                                         slave->alias,
                                                         slave->position,
                                                         slave->vendor_id,
                                                         slave->product_code);
        HANDLE_ERROR(!sc, klipper)

        /* configure slave pdos */
        ret = ecrt_slave_config_pdos(sc, EC_END, slave->syncs);
        report_errno("ecrt_slave_config_pdos", ret);
        HANDLE_ERROR(ret, klipper)

        /* configure slave domain specific registers */
        for (uint8_t j = 0; j < ETHCAT_PVT_DOMAINS; j++)
        {
            ret = ecrt_domain_reg_pdo_entry_list(master->pvtdomain[j].domain, slave->registers);
            report_errno("ecrt_domain_reg_pdo_entry_list", ret);
            HANDLE_ERROR(ret, klipper)
        }

        /* configure slave dc clock */
        ecrt_slave_config_dc(sc,
                             master->monitor[i].assign_activate, //dc channel used
                             TIMES2NS(master->sync0_ct), //sync0 cycle time
                             TIMES2NS(slave->sync0_st),  //sync0 shift time
                             TIMES2NS(master->sync1_ct), //sync0 cycle time
                             TIMES2NS(slave->sync1_st)); //sync0 shift time
    }

    /* activate master */
    ret = ecrt_master_activate(master->master);
    HANDLE_ERROR(ret, fail)

    /* get common data domain starting address */
    master->domain_pd = ecrt_domain_data(master->domain);
    HANDLE_ERROR(!master->domain_pd, fail)

    /* get pdo data domain starting address */
    for (uint8_t i = 0; i < ETHCAT_PVT_DOMAINS; i++)
    {
        master->pvtdomain[i].domain_pd = ecrt_domain_data(master->pvtdomain[i].domain);
        HANDLE_ERROR(!master->pvtdomain[i].domain_pd, fail)
    }

klipper:

    /* crete pipe for internal event scheduling */
    ret = pipe(sq->pipe_sched);
    HANDLE_ERROR(ret, fail)

    /* 
     * Ethercat low level thread reactor setup. It handles low level
     * ethercet communication where:
     *  - pipe_sched[0]: RX side of scheduling pipe used for triggering the low level
     *                   reactor. Once input activity on the pipe is detected, the
     *                   associated "kick_event" callback is run and it forces a new
     *                   execution of command_event().
     *  - pipe_sched[1]: TX side of the scheduling pipe, used by the low level ethercat
     *                   thread to schedule unexpected command events (command events 
     *                   that send additional frames in the same sync cycle).
     *  - timers: there is only one associated timer:
     *            1) EQPT_COMMAND: used to trigger the command_event callback,
     *                             forcing the transmission of data to the drives.
     *                             It is fundamental for the synchronization between
     *                             host and drives, i.e. the command event is run at
     *                             a precise time so that the drives receive data
     *                             in real time exactly when they need it. It needs
     *                             to match the drives sync cycle time (or a multiple
     *                             if the drive support different sync and frame
     *                             cycle time).
     *            2) EQPT_INPUT: used to trigger input_event, its value is defined
     *                           directly by command_event after a fixed amount of
     *                           time when the frame is guaranteed to be received
     *                           back by the master.
     *            3) EQPT_PROTOCOL: used to trigger protocol_event, its value is defined
     *                              can be defined by the input_event (in the low level
     *                              EtherCAT thread) or scheduled immediately by the
     *                              high level thread through the pipe.
     */
    sq->pr = pollreactor_alloc(EQPF_NUM, EQPT_NUM, sq);
    pollreactor_add_fd(sq->pr, EQPF_PIPE, sq->pipe_sched[0], kick_event, 0); //build and send frame
    pollreactor_add_timer(sq->pr, EQPT_INPUT, input_event); //input timer (rx operation)
    pollreactor_add_timer(sq->pr, EQPT_COMMAND, command_event); //command timer (tx operation)
    pollreactor_add_timer(sq->pr, EQPT_PROTOCOL, protocol_event); //protocol timer

    /* 
     * Set pipe for low level thread scheduling in non blocking mode.
     * This is fundamental in order to be reactive to events, if there
     * is no activity on the pipe, return immediately and don't wait
     * in blocking mode. 
     */
    ret = fd_set_non_blocking(sq->pipe_sched[0]);
    ret = fd_set_non_blocking(sq->pipe_sched[1]);

    /* queues */
    sq->need_kick_clock = MAX_CLOCK; //background thread wuke up time
    list_init(&sq->pending_queues);  //messages waiting to be sent
    list_init(&sq->request_queue);   //request messages form high to low level thread
    list_init(&sq->response_queue);  //response messages form low to high level thread
    list_init(&sq->notify_queue);    //notify queue for protocol messages

    /* associate external ethercat callback table */
    sq->klippyifc.cp_table = command_parser_table;

    /* initialize ethercatqueue mutex used to protect its data fields */
    ret = pthread_mutex_init(&sq->lock, NULL);
    HANDLE_ERROR(ret, fail)

    /* 
     * Initialize ethercatqueue condition variable used for synchronization with
     * the ethercat high level thread that post processes incoming messages
     * requiring notification of special handling (not standard).
     */
    ret = pthread_cond_init(&sq->cond, NULL);
    HANDLE_ERROR(ret, fail)

    /* create and run the background ethercat low level thread */
    ret = pthread_create(&sq->tid, NULL, background_thread, sq);
    HANDLE_ERROR(ret, fail)

fail:
    /* error handling */
    report_errno("Ethercat queue allocation", ret);
    
    return ret;
}

/** 
 * Request background thread exit, from this time on the
 * communication with the drives is interrupted.
 */
void __visible
ethcatqueue_exit(struct ethcatqueue *sq)
{
    /* signal must exit */
    pollreactor_do_exit(sq->pr);
    /* wake ethercat high level thread (last time) */
    //kick_bg_thread(sq, EQPT_PROTOCOL);
    /* join and stop current thread */
    int ret = pthread_join(sq->tid, NULL);
    if (ret)
    {
        report_errno("pthread_join", ret);
    }
}

/** free all resources associated with a ethcatqueue */
void __visible
ethcatqueue_free(struct ethcatqueue *sq)
{
    if (!sq)
    {
        return;
    }
    if (!pollreactor_is_exit(sq->pr))
    {
        ethcatqueue_exit(sq);
    }
    /* acquire mutex */
    pthread_mutex_lock(&sq->lock);
    message_queue_free(&sq->request_queue);
    message_queue_free(&sq->response_queue);
    message_queue_free(&sq->notify_queue);
    /* free all pending queues */
    while (!list_empty(&sq->pending_queues))
    {
        struct command_queue *cq = list_first_entry(&sq->pending_queues, struct command_queue, node);
        list_del(&cq->node);
        while (!list_empty(&cq->ready_queue))
        {
            struct pvtmsg *qm = list_first_entry(&cq->ready_queue, struct pvtmsg, node);
            list_del(&qm->node);
            free(qm);
        }
        while (!list_empty(&cq->upcoming_queue))
        {
            struct pvtmsg *qm = list_first_entry(&cq->upcoming_queue, struct pvtmsg, node);
            list_del(&qm->node);
            free(qm);
        }
    }
    /* release mutex */
    pthread_mutex_unlock(&sq->lock);
    /* free associated poll reactor */
    pollreactor_free(sq->pr);
    /* delete ethercatqueue */
    free(sq);
}

/** allocate a command_queue */
struct command_queue * __visible
ethcatqueue_alloc_commandqueue(void)
{
    struct command_queue *cq = malloc(sizeof(*cq));
    memset(cq, 0, sizeof(*cq));
    list_init(&cq->ready_queue);
    list_init(&cq->upcoming_queue);
    return cq;
}

/** free a command_queue */
void __visible
ethcatqueue_free_commandqueue(struct command_queue *cq)
{
    /* check command queue */
    if (!cq)
    {
        return;
    }
    /* ready_queue and upcoming_queue need to be already deallocated */
    if (!list_empty(&cq->ready_queue) || !list_empty(&cq->upcoming_queue))
    {
        errorf("Memory leak! Can't free non-empty commandqueue");
        return;
    }
    free(cq);
}

/** send a single synchronous command from high to low level thread */
void __visible
ethcatqueue_send_command(struct ethcatqueue *sq,
                         uint8_t *msg,
                         int len,
                         uint64_t min_clock,
                         uint64_t req_clock,
                         uint64_t notify_id)
{
    /* crete new queue message */
    struct queue_message *qm = message_alloc();
    uint8_t *buf = &qm->msg[0];
    memcpy(&buf[MESSAGE_HEADER_SIZE], msg, len);
    qm->len = len;
    qm->min_clock = min_clock;
    qm->req_clock = req_clock;
    qm->notify_id = notify_id;

    /* fill header and trailer */
    len += MESSAGE_HEADER_SIZE + MESSAGE_TRAILER_SIZE;
    buf[MESSAGE_POS_LEN] = len;
    buf[MESSAGE_POS_SEQ] = MESSAGE_DEST | (1 & MESSAGE_SEQ_MASK);
    uint16_t crc = msgblock_crc16_ccitt(buf, len - MESSAGE_TRAILER_SIZE);
    buf[len-MESSAGE_TRAILER_CRC] = crc >> 8;
    buf[len-MESSAGE_TRAILER_CRC+1] = crc & 0xff;
    buf[len-MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;
 
    /* acquire mutex (this operation has to be as fast as possible) */
    pthread_mutex_lock(&sq->lock);

    /* merge message queue with upcoming queue */
    list_add_tail(&qm->node, &sq->request_queue);

    /* release mutex */
    pthread_mutex_unlock(&sq->lock);

    kick_bg_thread(sq, EQPT_PROTOCOL);    
}

/** add a batch of messages (pvt only) to the upcoming command queue */
void
ethcatqueue_send_batch(struct ethcatqueue *sq, struct command_queue *cq, struct list_head *msgs)
{
    /* data */
    int len = 0;
    struct pvtmsg *qm;

    /* pre-process messages */
    list_for_each_entry(qm, msgs, node)
    {
        /* check message timing */
        if (qm->min_clock + (1LL<<31) < qm->req_clock)
        {
            /* 
             * Limit time difference between min_clock and req_clock,
             * this helps in the scheduling procedure.
             */
            qm->min_clock = qm->req_clock - (1LL<<31);
        }
        /* get total amount of bytes in batch */
        len += qm->len;
    }
    if (!len)
    {
        /* nothing to transmit */
        return;
    }

    /* get first message */
    qm = list_first_entry(msgs, struct pvtmsg, node);

    /* 
     * Acquire mutex, this operation has to be as fast as possible
     * since it can block the ethercat real time thread.
     */
    pthread_mutex_lock(&sq->lock);

    /* check ready queue and upcoming queue */
    if (list_empty(&cq->ready_queue) && list_empty(&cq->upcoming_queue))
    {
        /* new command queue, add it to pending queues */
        list_add_tail(&cq->node, &sq->pending_queues);
    }

    /* merge message queue with upcoming queue */
    list_join_tail(msgs, &cq->upcoming_queue);
    sq->upcoming_bytes += len;

    /* 
     * Schedule new transmission if the current message min clock
     * is lower than the current need_kick_clock, otherwise the
     * message will not be sent in time but should not happen in
     * in normal cases since it means that either the queues
     * are not ordered or the command_event has been delayed for
     * a very long time. The effect of an early call is to send
     * an additional ethercat frame in the same sync cycle that
     * will not contain the DC clock reference.
     */
    int mustwake = 0;
    if (qm->min_clock < sq->need_kick_clock)
    {
        /* min possible kick clock */
        sq->need_kick_clock = 0;
        /* force background thread execution */
        mustwake = 1;
    }

    /* release mutex */
    pthread_mutex_unlock(&sq->lock);

    /* wake the background thread if necessary (limit case) */
    if (mustwake)
    {
        //kick_bg_thread(sq, EQPT_COMMAND);
    }
}

/**
 * Return a response message from the ethercat low level thread (or wait
 * for one if none available). It is called directly from the ethercat
 * high level thread. This function takes a pre-processed message from
 * the low levelthread and handles it properly using the main reactor.
 */
void __visible
ethcatqueue_pull(struct ethcatqueue *sq, struct pull_queue_message *pqm)
{
    /* lock mutex */
    pthread_mutex_lock(&sq->lock);

    /* wait for message to be available */
    while (list_empty(&sq->response_queue))
    {
         /* stop if reactor not running */
        if (pollreactor_is_exit(sq->pr))
        {
            pqm->len = -1;
            pthread_mutex_unlock(&sq->lock);
            return;
        }

        /* signal waiting to low level thread */
        sq->receive_waiting = 1;

        /* wait in non busy mode (mutex is released) */
        int ret = pthread_cond_wait(&sq->cond, &sq->lock);
        if (ret)
        {
            report_errno("pthread_cond_wait", ret);
        }
    }

    /* get first message */
    struct queue_message *response = list_first_entry(&sq->response_queue, struct queue_message, node);

    /* remove message from response queue */
    list_del(&response->node);

    /* update message contents */
    memcpy(pqm->msg, response->msg, response->len);
    pqm->len = response->len;
    pqm->sent_time = response->sent_time;
    pqm->receive_time = response->receive_time;
    pqm->notify_id = response->notify_id;

    /* delete request message */
    message_free(response);

    /* unlock mutex */
    pthread_mutex_unlock(&sq->lock);
}

/** 
 * Set ethercat frequency, it defines the time a frame needs to
 * be received back by the ethercat master.
 */
void __visible
ethcatqueue_set_wire_frequency(struct ethcatqueue *sq, double frequency)
{
    /* acquire mutex */
    pthread_mutex_lock(&sq->lock);

    /* time to transmit one bit (baud) */
    sq->masterifc.frame_time = 1. / frequency;

    /* unlock mutex */
    pthread_mutex_unlock(&sq->lock);
}

/**
 * Set the estimated clock rate of the main mcu. This function is
 * used to maintain synchronization between main mcu, host and
 * drives. The main mcu is taken as reference clock.
 */
void __visible
ethcatqueue_set_clock_est(struct ethcatqueue *sq,
                          double est_freq,
                          double conv_time,
                          uint64_t conv_clock,
                          uint64_t last_clock)
{
    pthread_mutex_lock(&sq->lock);
    sq->ce.est_freq = est_freq;
    sq->ce.conv_time = conv_time;
    sq->ce.conv_clock = conv_clock;
    sq->ce.last_clock = last_clock;
    pthread_mutex_unlock(&sq->lock);
}

/** return the latest clock estimate */
void
ethcatqueue_get_clock_est(struct ethcatqueue *sq, struct clock_estimate *ce)
{
    pthread_mutex_lock(&sq->lock);
    memcpy(ce, &sq->ce, sizeof(sq->ce));
    pthread_mutex_unlock(&sq->lock);
}

/* return a string buffer containing statistics for the ethcat port */
void __visible
ethcatqueue_get_stats(struct ethcatqueue *sq, char *buf, int len)
{
    struct ethcatqueue stats;
    pthread_mutex_lock(&sq->lock);
    memcpy(&stats, sq, sizeof(stats));
    pthread_mutex_unlock(&sq->lock);
    /* print to bufffer */
    snprintf(buf, len,
            " ready_bytes=%u upcoming_bytes=%u",
            stats.ready_bytes, stats.upcoming_bytes);
}
