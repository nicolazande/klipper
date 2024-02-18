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
#include <stddef.h> //offsetof
#include <stdio.h> //snprintf
#include <stdlib.h> //malloc
#include <string.h> //memset
#include <termios.h> //tcflush
#include <unistd.h> //pipe
#include "compiler.h" //__visible
#include "pollreactor.h" //pollreactor_alloc
#include "pyhelper.h" //get_monotonic
#include "ethercatqueue.h" //struct pvtmsg


/****************************************************************
 * Defines
 ****************************************************************/
/* file descriptors */
#define EQPF_PIPE   0               //index of rx-cmd pipe (pipe where application stores cmd to be sent)
#define EQPF_NUM    1               //number of file descriptors used by a ethercatqueue
/* timers */
#define EQPT_INPUT      0           //position of input timer in ethercatqueue timer list
#define EQPT_COMMAND    1           //position of command timer in ethercatqueue timer list
#define EQPT_NUM        2           //number of timer used by a ethercatqueue
/* transport layer */
#define EQT_ETHERCAT    'e'         //id for ethernet transport layer (osi 2)
#define EQT_DEBUGFILE   'f'         //id for output to debug file (no commnication)
/* time and memory limits */
#define MIN_REQTIME_DELTA 0.250     //min delta time (in advance) to send a command
#define PR_OFFSET (INT32_MAX)       //poll reactor time offset (disable poll)
/* helpers */
#define TIMES2NS(time) ((uint64_t)(time * 1e9)) //convert internal time to nanaoseconds
#define HANDLE_ERROR(condition, exit) if(condition) {goto exit;} //error handling
/* switches */
#define MESSAGE_CHECK_FORMAT (0U)


/****************************************************************
 * Data
 ****************************************************************/
/** external command parser table */
extern struct command_parser *command_parser_table[ETH_MAX_CP];


/****************************************************************
 * Private function prototypes
 ****************************************************************/
/** callback timer to send data to the ethercat port */
static double command_event(struct ethercatqueue *sq, double eventtime);

/**
 * Read EtherCAT frame coming back from the drives and update internal structures.
 */
static double input_event(struct ethercatqueue *sq, double eventtime);

/**
 * Callback for input activity on the pipe. This function is continuosly
 * run by the poll reactor and, as soon as data (dummy bytes) is found
 * on the rx side of the internal pipe, it updates the command timer (to
 * zero), indirectly forcing an execution of command_event(). This is
 * the low level thread counterpart of kick_bg_thread.
 */
static void kick_event(struct ethercatqueue *sq, double eventtime);

/**
 * Write to the ethercatqueue internal pipe to force the execution 
 * of the background low level ethercat immediately (don't wait
 * for associated timer). This function is called asynchronously
 * by the high level thread since it cannot update directly the
 * command event timer.
 */
static void kick_bg_thread(struct ethercatqueue *sq, uint8_t cmd);

/**
 * Wake up the ethercat high level thread if it is waiting. It needs
 * to further process the messages in the receive queue.
 */
static void check_wake_receive(struct ethercatqueue *sq);

/** 
 * Process a single request from the high level thread, execute the associated
 * callback and append the result to the response queue so that the high level
 * thread can read and further process it. Thois function is is executed also
 * in real time context, therefore find a proper time positioning, i.e. after
 * an input event and before the next command event.
 */
static void process_request(struct ethercatqueue *sq, double eventtime);

/** process ethercat frame (reset and configuration check) */
static void process_frame(struct ethercatqueue *sq);

/** 
 * Populate ethercat domain for the current cycle, all mapped objects
 * are transmitted in the current cycle frame.
 */
static int build_and_send_command(struct ethercatqueue *sq);

/**
 * Determine schedule time of next command event and move messages
 * from pending queue to ready queue for each drive.
 */
static double check_send_command(struct ethercatqueue *sq, int pending, double eventtime);

/** main background thread for reading/writing to ethercat port */
static void *background_thread(void *data);


/****************************************************************
 * Private functions
 ****************************************************************/
static void
check_wake_receive(struct ethercatqueue *sq)
{
    if (sq->receive_waiting)
    {
        /* signal condition to ethercatqueue_pull */
        sq->receive_waiting = 0;
        pthread_cond_signal(&sq->cond);
    }
}

static void
kick_bg_thread(struct ethercatqueue *sq, uint8_t cmd)
{
    /* write a dummy value just to wake up the poll reactor */
    int ret = write(sq->pipe_sched[1], &cmd, 1);
    if (ret < 0)
    {
        report_errno("pipe write", ret);
    }
}

static void 
process_request(struct ethercatqueue *sq, double eventtime)
{
    /* check request from high level thread */
    if (!list_empty(&sq->request_queue))
    {
        /* data */
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

        /* get message boundaries */
        uint8_t *in = &request->msg[MESSAGE_HEADER_SIZE]; //request data start address
        uint8_t *inend = &request->msg[msglen-MESSAGE_TRAILER_SIZE]; //request end address
        uint8_t *out = response->msg; //response start address
        uint8_t *outend = &response->msg[MESSAGE_MAX-MESSAGE_TRAILER_SIZE]; //response end address

        /**
         * Loop over request bytes, isolate and execute single commands.
         * NOTE: for the moment each request message buffer contains a
         *       single command for simplicity but these logic allows
         *       to handle multiple commands per request.
         */
        while (in < inend)
        {
            /* get command id and move forward to arguments */
            uint8_t cmdid = *in++;

            /* check command */
            if (cmdid < ETH_MAX_CP)
            {
                /* get command parser */
                cp = sq->cp_table[cmdid];
            }
            else
            {
                /* unknown command */
                errorf("Unknown protocol command: cmdid = %u", cmdid);
                break;
            }
            
            /* variable length array for arguments */
            uint32_t args[cp->num_args];

            /* parse an incoming command into args */
            in = command_parsef(in, inend, cp, args);
            
            /* get request handler */
            int (*func)(struct ethercatqueue *, void *, uint32_t *) = cp->func;

            /* check callback */
            if (func)
            {
                /* check if response data fits msg buffer */
                if (out >= outend)
                {
                    /* save old message reference */
                    struct queue_message *old_response = response;

                    /* append previous response to response queue */
                    list_add_tail(&response->node, &sq->response_queue);
                    
                    /* create additional response message and update indexes */
                    response = malloc(sizeof(*response));
                    response->notify_id = old_response->notify_id;
                    response->sent_time = old_response->sent_time;
                    response->receive_time = old_response->receive_time;
                    out = response->msg;
                    outend = &response->msg[MESSAGE_MAX-MESSAGE_TRAILER_SIZE];
                }

                /**
                 * Execute command specific handler.
                 * NOTE: out is updated directly in the callback according to the
                 *       response size. In case of out of bounds a new response
                 *       message is created automatically.
                 */
                int ret = func(sq, out, args);
                if (ret)
                {
                    /* error (release response) */
                    message_free(response);
                    return;
                }
            }
        }

        /* remove message from request queue */
        list_del(&request->node);

        /* update response message */
        response->len = response->msg[MESSAGE_POS_LEN];
        response->receive_time = eventtime;

        /* check for message notification request */
        if (request->notify_id)
        {
            /* message requires notification (add to notify list) */
            list_add_tail(&request->node, &sq->notify_queue);
        }
        else
        {
            /* delete request message */
            message_free(request);
        }

        /* check response data */
        if (response->len > 0)
        {
            /* add response to response queue */
            list_add_tail(&response->node, &sq->response_queue);
        }
        else
        {
            /* nothing to be sent (release respons) */
            message_free(response);
        }
    }

    /* process notification queue */
    while (!list_empty(&sq->notify_queue))
    {
        /* get message */
        struct queue_message *qm = list_first_entry(&sq->notify_queue, struct queue_message, node);
        list_del(&qm->node);
        qm->len = 0; //signal high level thread of request end
        qm->sent_time = 0; //unused (keep zero)
        qm->receive_time = eventtime; //current event time
        /* add to response queue */
        list_add_tail(&qm->node, &sq->response_queue);
    }

    /* wake up high level thread */
    check_wake_receive(sq);
};

static double
input_event(struct ethercatqueue *sq, double eventtime)
{
    /* acquire mutex */
    pthread_mutex_lock(&sq->lock);

    /* get master interface */
    struct mastermonitor *master = &sq->masterifc;
    
    /* receive process data */
    ecrt_master_receive(master->master);

    /* loop over domains */
    for (uint8_t i = 0; i < ETHERCAT_DOMAINS; i++)
    {
        /* get domain */
        struct domainmonitor *dm = &master->domains[i];

        /* process received data (from datagram to domain) */
        ecrt_domain_process(dm->domain);

        /** update domain state (TODO: add error handling) */
        ecrt_domain_state(dm->domain, &dm->domain_state);
    }

    /**
     * Read associated slaves window status (as soon as possible).
     */
    for (uint8_t i = 0; i < ETHERCAT_DRIVES; i++)
    {
        /* get slave */
        struct slavemonitor *slave = &master->monitor[i];

        /* update slave window */
        if (slave->off_slave_window)
        {
            slave->slave_window = EC_READ_U16(slave->off_slave_window);
        }
        /* update actual position */
        if (slave->off_position_actual)
        {
            slave->position_actual = EC_READ_S32(slave->off_position_actual);
        }
        /* update actual velocity */
        if (slave->off_velocity_actual)
        {
            slave->velocity_actual = EC_READ_S32(slave->off_velocity_actual);
        }
        
        /** NOTE: following lines only for test purpose, remove it!!!! */
        {
            struct coe_control_word *cw = (struct coe_control_word *)slave->off_control_word;
            struct coe_status_word *sw = (struct coe_status_word *)slave->off_status_word;   
            sw->homing_attained = 1;
        }
    }

    /* update last clock for protocol */
    sq->last_clock = clock_from_time(&sq->ce, eventtime);

    /* prepare frame for next cycle */
    process_frame(sq);

    /* process a high level thread request */
    process_request(sq, eventtime);

    /* release mutex */
    pthread_mutex_unlock(&sq->lock);

    /* reset input event timer (command_event will reschedule it when needed)  */
    return PR_NEVER;
}

static void
kick_event(struct ethercatqueue *sq, double eventtime)
{
    /* protocol command */
    uint8_t timer;

    /* read rx-command pipe */
    int ret = read(sq->pipe_sched[0], &timer, sizeof(timer));

    /* check data */
    if (ret < 0)
    {
        report_errno("pipe read error", ret);
    }
    
    /* check timer index */
    if (timer < EQPT_NUM)
    {
        /* update command timer */
        pollreactor_update_timer(sq->pr, timer, PR_NOW);
    }
}

static int
build_and_send_command(struct ethercatqueue *sq)
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
        if (qm->oid < ETHERCAT_DRIVES)
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
            memcpy(slave->movedata[slave->master_window], qm->msg, qm->len);
            
            /* increase master tx index */
            slave->master_window++;

            /* increase slave rx index in advance */
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
check_send_command(struct ethercatqueue *sq, int pending, double eventtime)
{
    /* data */
    struct mastermonitor *master = &sq->masterifc; //ethercat master interface
    struct slavemonitor *slave; //ethercat slave interface

    /* 
     * Check for free buffer slots on slave side only. There is no need to check tx side
     * since when this function is called it is guaranteed to have all pdo slots free.
     * Stop as soon as one drive receive buffer is full, even if the others are not, this
     * allows to keep better drive synchronization (common steps in the same frame).
     */
    for (uint8_t i = 0; i < ETHERCAT_DRIVES; i++)
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
    double idletime = eventtime + master->sync0_st; //current transmission estimated drive reception time (frame is back to master).
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

            /**
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
     * Early stop check (step space in domain is full). A drive specific
     * check is performed in build_and_send_command(), but this avoids
     * having an undefinitely growing ready queue.
     */
    if (sq->ready_bytes >= master->frame_pvt_size)
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
        return PR_NEVER;
    }

    /**
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
     * taking into account the synch cycle time.
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
     * Ideal walue of next command_event(), unused since the ethercat
     * synch cycle time has to be constant.
     */
    return idletime + (wantclock - ack_clock) / sq->ce.est_freq;
}

/** process ethercat frame (reset and configuration check) */
static void
process_frame(struct ethercatqueue *sq)
{
    /* get master */
    struct mastermonitor *master = &sq->masterifc;

    /* loop over domains */
    for (uint8_t i = 0; i < ETHERCAT_DOMAINS; i++)
    {
        /* loop over domain associated drives */
        for (uint8_t j = 0; j < ETHERCAT_DRIVES; j++)
        {
            /* get slave */
            struct slavemonitor *slave = &master->monitor[j];

            /* get control word */
            struct coe_control_word *cw = (struct coe_control_word *)slave->off_control_word;

            /* get slave segment buffer data */
            struct coe_ip_move *move = (struct coe_ip_move *)slave->movedata[i];

            /*
             * Reset segment slot in doamin allowing to schedule the command
             * events at regular intervals, even without active segments.
             */
            if (move)
            {
                move->command.type = COPLEY_MODE_CMD;
                move->command.code = COPLEY_CMD_NO_OPERATION;
                move->time = 0;
                move->position = 0;
                move->velocity = 0;
            }

            /** 
             * Configure slave specific control word.
             * NOTE: if using multiple domains register the control word object only
             *       once so that for the other ones off_control_word is NULL.
             */
            if ((slave->operation_mode == COLPEY_OPERATION_MODE_INTERPOLATION) && cw)
            {
                /**
                 * Check receive buffer status and perform automatic transition
                 * of enable operation command, i.e. automatic start when there
                 * are enough samples in the buffer and automatic stop when the
                 * low limit of segments in the drive budder is reached.
                 */
                if (slave->slave_window < slave->interpolation_window)
                {
                    /** NOTE: this causes hard stop (remove if unwanted) */
                    cw->enable_operation = 0;
                }
                else
                {
                    /* enable operation (interpolation can be performed) */
                    cw->enable_operation = 1;
                }
            }
        }
    }
}

static double
command_event(struct ethercatqueue *sq, double eventtime)
{
    /* acquire mutex */
    pthread_mutex_lock(&sq->lock);

    /* data */
    struct mastermonitor *master = &sq->masterifc; //ethercat master interface
    int buflen = 0; //length (in bytes) of data to be transmitted
    double waketime; //wake time of next command event
    uint64_t sync_clock = TIMES2NS(eventtime); //distributed clock value

    /** set master application time */
    ecrt_master_application_time(master->master, sync_clock);

    /**
     * Synchronize the reference clock (first DC capable slave) with the
     * master internal time (use eventtime for stability).
     * TODO: this operation doesn't need to be performed every cycle,
     *       therefore add a counter enable logic.
     */
    ecrt_master_sync_reference_clock_to(master->master, sync_clock);

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
            /* loop over domains */
            for (uint8_t i = 0; i < ETHERCAT_DOMAINS; i++)
            {
                /* get domain */
                struct domainmonitor *dm = &master->domains[i];

                /* update domain */
                ecrt_domain_queue(dm->domain);
            }

            /* write ethercat frame (always) */
            ecrt_master_send(master->master);

            /* reset transmission parameters */
            buflen = 0; //buffer free
            master->full_counter = 0; //pdo slots free

            /* reset slaves tx counter */
            for (uint8_t i = 0; i < ETHERCAT_DRIVES; i++)
            {
                /* get slave */
                struct slavemonitor *slave = &master->monitor[i];

                /* reset window */
                slave->master_window = 0;
            }

            /* stop (frame transmitted) */
            break;
        }

        /* build or update data to be sent */
        buflen += build_and_send_command(sq);
    }

    /* compute time of next input (read) event */
    double inputtime = eventtime + master->sync0_st + master->frame_time;

    /* set input event timer (when the frame is expected to be received back) */
    pollreactor_update_timer(sq->pr, EQPT_INPUT, inputtime);

    /* releas mutex */
    pthread_mutex_unlock(&sq->lock);

    /* update next timer event (in pollreactor_check_timers) */
    return eventtime + master->sync0_ct;
}

static void *
background_thread(void *data)
{
    /* pointer to shared ethercatqueue */
    struct ethercatqueue *sq = data;

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
/** configure ethercat low level thread dedicated cpu */
void __visible
ethercatqueue_config_cpu(struct ethercatqueue *sq, int cpu)
{
    sq->cpu = cpu;
}

/** initialize ethercat slave */
void __visible
ethercatqueue_slave_config(struct ethercatqueue *sq,
                           uint8_t index,
                           uint16_t alias,
                           uint16_t position,
                           uint32_t vendor_id,
                           uint32_t product_code,
                           uint16_t assign_activate,
                           uint8_t rx_size,
                           uint8_t interpolation_window)
{
    /* get master and slave monitor */
    struct mastermonitor *master = &sq->masterifc;
    struct slavemonitor *slave = &master->monitor[index];

    /* populate slave */
    slave->alias = alias;
    slave->position = position;
    slave->vendor_id = vendor_id;
    slave->product_code = product_code;
    slave->assign_activate = assign_activate;
    slave->rx_size = rx_size;
    slave->tx_size = ETHERCAT_DOMAINS;
    slave->oid = index;
    slave->n_pdo_entries = 0;
    slave->n_pdos = 0;
    slave->interpolation_window = interpolation_window;
}

/** configure an ethercat sync manager */
void __visible
ethercatqueue_slave_config_sync(struct ethercatqueue *sq,
                                uint8_t slave_index,
                                uint8_t sync_index,
                                uint8_t direction,
                                uint8_t n_pdo_entries,
                                ec_pdo_entry_info_t *pdo_entries,
                                uint8_t n_pdos,
                                ec_pdo_info_t *pdos)
{
    /* get master and slave monitor */
    struct mastermonitor *master = &sq->masterifc;
    struct slavemonitor *slave = &master->monitor[slave_index];

    /* store pdo entries */
    uint8_t old_n_pdo_entries = slave->n_pdo_entries;
    slave->n_pdo_entries += n_pdo_entries;
    for (uint8_t i = 0; i < n_pdo_entries; i++)
    {
        uint8_t idx = i + old_n_pdo_entries;
        slave->pdo_entries[idx] = pdo_entries[i];
    }

    /* store pdos */
    uint8_t old_n_pdos = slave->n_pdos;
    slave->n_pdos += n_pdos;
    for (uint8_t i = 0; i < n_pdos; i++)
    {
        uint8_t idx = i + old_n_pdos;
        slave->pdos[idx].index = pdos[i].index;
        slave->pdos[idx].n_entries = pdos[i].n_entries;
        slave->pdos[idx].entries = &slave->pdo_entries[old_n_pdo_entries];
        old_n_pdo_entries += pdos[i].n_entries;
    }

    /* get number of available syncs */
    int8_t sync_size = sizeof(slave->syncs)/sizeof(slave->syncs[0]) - 1;
    if ((sync_size > 0) && (sync_index < sync_size))
    {
        slave->syncs[sync_index].index = sync_index;
        slave->syncs[sync_index].dir = direction;
        slave->syncs[sync_index].n_pdos = n_pdos;
        slave->syncs[sync_index].pdos = &slave->pdos[old_n_pdos];
    }

    /* reset stop flag (last sync) */
    slave->syncs[sync_size] = (ec_sync_info_t){EC_END, 0, 0, 0, 0};
}

/** configure ethercat master */
void __visible 
ethercatqueue_master_config(struct ethercatqueue *sq,
                            double sync0_ct,
                            double sync0_st,
                            double sync1_ct,
                            double sync1_st,
                            double frame_time)
{
    /* get master domain monitor */
    struct mastermonitor *master = &sq->masterifc;

    /* reset and initialize master */
    master->sync0_ct = sync0_ct;
    master->sync0_st = sync0_st;
    master->sync1_ct = sync1_ct;
    master->sync1_st = sync1_st;
    master->frame_size = 0;
    master->frame_pvt_size = 0;
    master->frame_time = frame_time;
    master->full_counter = 0;

    /* reset domain register counter (incremental) */
    for (uint8_t i = 0; i < ETHERCAT_DOMAINS; i++)
    {
        struct domainmonitor *dm = &master->domains[i];
        dm->n_registers = 0;
    }
}

/** configure ethercat master domain registers */
void __visible 
ethercatqueue_master_config_registers(struct ethercatqueue *sq,
                                      uint8_t index,
                                      uint8_t n_registers,
                                      ec_pdo_entry_reg_t *registers)
{
    /* get master domain monitor */
    struct mastermonitor *master = &sq->masterifc;
    struct domainmonitor *dm = &master->domains[index];

    /* store registers */
    uint8_t old_n_registers = dm->n_registers;
    dm->n_registers += n_registers;
    for (uint8_t i = 0; i < n_registers; i++)
    {
        /* cumulative index */
        uint8_t idx = i + old_n_registers;
        /* assign register */
        dm->registers[idx] = registers[i];
        /* assign register offset (overwrite) */
        dm->registers[idx].offset = &dm->offsets[idx];
    }

    /* reset stop flag (last empty register) */
    dm->registers[dm->n_registers] = (ec_pdo_entry_reg_t){};
}

/** create an empty ethercatqueue object */
struct ethercatqueue * __visible
ethercatqueue_alloc(void)
{
    /* allocate serialqueue */
    struct ethercatqueue *sq = malloc(sizeof(*sq));
    memset(sq, 0, sizeof(*sq));
    return sq;
}

/** initialize ethercatqueue */
int __visible
ethercatqueue_init(struct ethercatqueue *sq)
{
    /* shared error code */
    int ret;

    /* get ethercat master interface */
    struct mastermonitor *master = &sq->masterifc;

    /* create ethercat master */
    master->master = ecrt_request_master(0);

    /* initialize ethercat slaves */
    for (uint8_t i = 0; i < ETHERCAT_DRIVES; i++)
    {
        /* get slave monitor */
        struct slavemonitor *slave = &master->monitor[i];

        /* create slave configuration */
        ec_slave_config_t *sc = ecrt_master_slave_config(master->master,
                                                         slave->alias,
                                                         slave->position,
                                                         slave->vendor_id,
                                                         slave->product_code);

        /* register slave */
        slave->slave = sc;

        /* configure slave pdos */        
        ret = ecrt_slave_config_pdos(sc, EC_END, slave->syncs);

        /* configure slave dc clock */
        ecrt_slave_config_dc(sc,
                             master->monitor[i].assign_activate, //dc channel used
                             TIMES2NS(master->sync0_ct),  //sync0 cycle time
                             TIMES2NS(master->sync0_st),  //sync0 shift time
                             TIMES2NS(master->sync1_ct),  //sync1 cycle time
                             TIMES2NS(master->sync1_st)); //sync1 shift time
    }

    /* configure master domains */
    for (uint8_t i = 0; i < ETHERCAT_DOMAINS; i++)
    {
        /* get domain monitor */
        struct domainmonitor *dm = &master->domains[i];

        /* create domain */
        dm->domain = ecrt_master_create_domain(master->master);

        /* register mapped pdo entries to domain */
        ret = ecrt_domain_reg_pdo_entry_list(dm->domain, dm->registers);
    }

    /* activate master */
    ret = ecrt_master_activate(master->master);

    /**
     * Get domain data addresses.
     * NOTE: perform this operation only after master activation since the process
     *       data image map from kernel to userspace has to be already valid.
     */
    for (uint8_t i = 0; i < ETHERCAT_DOMAINS; i++)
    {
        /* get domain monitor */
        struct domainmonitor *dm = &master->domains[i];

        /* initialize domain data */
        dm->domain_pd = ecrt_domain_data(dm->domain);

        /* get domian data size */
        dm->domain_size = ecrt_domain_size(dm->domain);

        /* udate expected frame total size */
        master->frame_size += dm->domain_size;

        /* setup domain drive specific objects */
        for (uint8_t j = 0; j < ETHERCAT_DRIVES; j++)
        {
            /* offset data */
            uint16_t offset_idx;
            uint32_t offset;

            /* update expected pvt frame size */
            master->frame_pvt_size += ETHERCAT_PVT_SIZE;

            /* get slave monitor */
            struct slavemonitor *slave = &master->monitor[j];

            /* setup interpolation move segment */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_MOVE_SEGMENT;
            offset = dm->offsets[offset_idx];
            slave->movedata[i] = (uint8_t *)(dm->domain_pd + offset);

            /* setup buffer free slot count */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_BUFFER_FREE_COUNT;
            offset = dm->offsets[offset_idx];
            slave->off_slave_window = (uint8_t *)(dm->domain_pd + offset);

            /* setup slave control word */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_CONTROL_WORD;
            offset = dm->offsets[offset_idx];
            slave->off_control_word = (uint8_t *)(dm->domain_pd + offset);
            struct coe_control_word *cw = slave->off_control_word;
            if (cw)
            {
                /* ensure that enable operation is disabled at startup */
                cw->enable_operation = 0;
            }

            /* setup slave status word */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_STATUS_WORD;
            offset = dm->offsets[offset_idx];
            slave->off_status_word = (uint8_t *)(dm->domain_pd + offset);

            /* setup slave operation mode */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_MODE_OF_OPERATION;
            offset = dm->offsets[offset_idx];
            slave->off_operation_mode = (uint8_t *)(dm->domain_pd + offset);

            /* setup slave position actual offset */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_POSITION_ACTUAL;
            offset = dm->offsets[offset_idx];
            slave->off_position_actual = (uint8_t *)(dm->domain_pd + offset);

            /* setup slave velocity actual offset */
            offset_idx = j * ETHERCAT_OFFSET_MAX + ETHERCAT_OFFSET_VELOCITY_ACTUAL;
            offset = dm->offsets[offset_idx];
            slave->off_velocity_actual = (uint8_t *)(dm->domain_pd + offset);
        }
    }

    /** 
     * Crete pipe for internal event scheduling. NOTE: this can be
     * removed in the future since command and input event run at
     * with a constant interval (ethercat sync cycle time).
     */
    ret = pipe(sq->pipe_sched);
    HANDLE_ERROR(ret, fail)

    /**
     * Ethercat low level thread reactor setup. It handles low level
     * ethercet communication where:
     *  - pipe_sched[0]: RX side of scheduling pipe used for triggering the low level
     *                   reactor. Once input activity on the pipe is detected, the
     *                   associated "kick_event" callback is run and it forces a new
     *                   execution of command_event().
     *  - pipe_sched[1]: TX side of the scheduling pipe, used by the low level ethercat
     *                   thread to schedule unexpected command events (command events 
     *                   that send additional frames in the same sync cycle).
     *                   NOTE: pipe_sched will be removed in the future since the ethercat
     *                         communication model avoids pipes for real time requirements.
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
     */
    sq->pr = pollreactor_alloc(EQPF_NUM, EQPT_NUM, sq, 0., 10., PR_OFFSET);
    pollreactor_add_fd(sq->pr, EQPF_PIPE, sq->pipe_sched[0], kick_event, 0); //build and send frame
    pollreactor_add_timer(sq->pr, EQPT_INPUT, input_event); //input timer (rx operation)
    pollreactor_add_timer(sq->pr, EQPT_COMMAND, command_event); //command timer (tx operation)

    /* 
     * Set pipe for low level thread scheduling in non blocking mode.
     * This is fundamental in order to be reactive to events, if there
     * is no activity on the pipe, return immediately and don't wait
     * in blocking mode. 
     */
    fd_set_non_blocking(sq->pipe_sched[0]);
    fd_set_non_blocking(sq->pipe_sched[1]);

    /* queues */
    list_init(&sq->pending_queues);  //messages waiting to be sent
    list_init(&sq->request_queue);   //request messages form high to low level thread
    list_init(&sq->response_queue);  //response messages form low to high level thread
    list_init(&sq->notify_queue);    //notify queue for protocol messages

    /* associate external ethercat callback table */
    sq->cp_table = command_parser_table;

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

    /**
     * Set scheduling fifo policy so that backgrond_thread never releases the
     * cpu and no time consuming context switching is performed (only specific
     * system interrupts can preempt it but they don't require a complete
     * context switching).
     */
    pthread_attr_init(&sq->sched_policy);
    pthread_attr_setschedpolicy(&sq->sched_policy, SCHED_FIFO);

    /* set max possible background_thread scheduling priority */
    sq->sched_param.sched_priority = 1;
    pthread_attr_setschedparam(&sq->sched_policy, &sq->sched_param);    

    /* create and run the background ethercat low-level thread */
    ret = pthread_create(&sq->tid, &sq->sched_policy, background_thread, sq);
    HANDLE_ERROR(ret, fail)

    /** 
     * Set cpu affinity for background_thread. The selected cpu has been
     * isolated, therefor background_thread is the only process running
     * there, meaning only system interrupts can preempt it.
     */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(sq->cpu, &cpuset);
    ret = pthread_setaffinity_np(sq->tid, sizeof(cpu_set_t), &cpuset);
    HANDLE_ERROR(ret, fail)

    /* start cyclic ethercat frame transmission */
    pollreactor_update_timer(sq->pr, EQPT_COMMAND, PR_NOW);

fail:
    /* error handling */
    if (ret)
    {
        report_errno("Ethercat queue allocation error", ret);
    }
    
    return ret;
}

/** 
 * Request background thread exit, from this time on the
 * communication with the drives is interrupted.
 */
void __visible
ethercatqueue_exit(struct ethercatqueue *sq)
{
    /* signal must exit */
    pollreactor_do_exit(sq->pr);

    /* process last eventual request */
    process_request(sq, PR_NOW);

    /* join and stop current thread */
    int ret = pthread_join(sq->tid, NULL);

    if (ret)
    {
        report_errno("pthread_join", ret);
    }
}

/** free all resources associated with a ethercatqueue */
void __visible
ethercatqueue_free(struct ethercatqueue *sq)
{
    if (!sq)
    {
        return;
    }
    if (!pollreactor_is_exit(sq->pr))
    {
        ethercatqueue_exit(sq);
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
ethercatqueue_alloc_commandqueue(void)
{
    struct command_queue *cq = malloc(sizeof(*cq));
    memset(cq, 0, sizeof(*cq));
    list_init(&cq->ready_queue);
    list_init(&cq->upcoming_queue);
    return cq;
}

/** free a command_queue */
void __visible
ethercatqueue_free_commandqueue(struct command_queue *cq)
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

/** send a single command from high to low level thread */
void __visible
ethercatqueue_send_command(struct ethercatqueue *sq,
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

    /* add message to request queue */
    list_add_tail(&qm->node, &sq->request_queue);

    /* release mutex */
    pthread_mutex_unlock(&sq->lock);
}

/** add a batch of messages (pvt only) to the upcoming command queue */
void
ethercatqueue_send_batch(struct ethercatqueue *sq, struct command_queue *cq, struct list_head *msgs)
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

    /* release mutex */
    pthread_mutex_unlock(&sq->lock);
}

/**
 * Return a response message from the ethercat low level thread (or wait
 * for one if none available). It is called directly from the ethercat
 * high level thread. This function takes a pre-processed message from
 * the low levelthread and handles it properly using the main reactor.
 */
void __visible
ethercatqueue_pull(struct ethercatqueue *sq, struct pull_queue_message *pqm)
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
 * Set the estimated clock rate of the main mcu. This function is
 * used to maintain synchronization between main mcu, host and
 * drives. The main mcu is taken as reference clock.
 */
void __visible
ethercatqueue_set_clock_est(struct ethercatqueue *sq,
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

/** 
 * Return a string buffer containing statistics for the ethercat queue.
 * NOTE: this function does not lock the ethercat queue while copying
 *       data, therefore it can happen that stats is corrupted.
 *       It is not worth to add potential delay in the background
 *       thread for these statistics.
 */
void __visible
ethercatqueue_get_stats(struct ethercatqueue *sq, char *buf, int len)
{
    struct ethercatqueue stats;
    /* get data */
    memcpy(&stats, sq, sizeof(stats));
    /* print to bufffer */
    snprintf(buf, len, " ready_bytes=%u upcoming_bytes=%u",
             stats.ready_bytes, stats.upcoming_bytes);
}