#ifndef ETHERCATQUEUE_H
#define ETHERCATQUEUE_H

/****************************************************************
 * Configuration
 ****************************************************************/
#define __USE_GNU 1


/****************************************************************
 * Includes
 ****************************************************************/
#include <stdint.h>
#include "list.h" 
#include <pthread.h>
#include "msgblock.h"
#include "ethercatmsg.h"
#include "command.h"
#include "ecrt.h"


/****************************************************************
 * Defines
 ****************************************************************/
#ifndef MAX_CLOCK //max clock
#define MAX_CLOCK 0x7fffffffffffffffLL
#endif
#define ETHERCAT_DRIVES 2  //number of ethercat drives
#define ETHERCAT_DOMAINS 1 //total number of domains
#define ETHERCAT_MAX_SYNCS 3 //max number of syncs per slave
#define ETHERCAT_MAX_REGISTERS 10 //max number of master registers
#define ETHERCAT_MAX_PDOS 10 //max number of slave pdos (per slave)
#define ETHERCAT_MAX_PDO_ENTRIES 10 //max number of pdo enries per slave


/****************************************************************
 * Custom data types
 ****************************************************************/
/* ethercat slave offsets */
enum
{
    ETHERCAT_OFFSET_MOVE_SEGMENT,
    ETHERCAT_OFFSET_BUFFER_FREE_COUNT,
    ETHERCAT_OFFSET_BUFFER_STATUS,
    ETHERCAT_OFFSET_MAX
};

/* pull queue message complete forward declaration */
struct pull_queue_message
{
    uint8_t msg[MESSAGE_MAX];
    int len;
    double sent_time;
    double receive_time;
    uint64_t notify_id;
};

/* command queue */
struct command_queue
{
    struct list_head upcoming_queue; //messages that need to be processed before send
    struct list_head ready_queue;    //messages ready to be sent
    struct list_node node;
};

/* 
 * Domain dedicated to a single pvt instance. A pvt instance is composed by
 * a pdo for each of the associated drives (X and Y coordinates have to be
 * transmitted together).
 */
struct domainmonitor
{
    ec_domain_t *domain;                //domain used for drive pdos
    ec_domain_state_t domain_state;     //domain state
    uint8_t *domain_pd;                 //domain process data pointer
    uint16_t domain_size;               //size in bytes of the domain
    uint8_t n_registers;                //number of registers in the domain
    uint32_t offsets[ETHERCAT_MAX_REGISTERS]; //domain register offsets
    ec_pdo_entry_reg_t registers[ETHERCAT_MAX_REGISTERS]; //domain registers
};

/* EtherCAT slave status monitor */
struct slavemonitor
{
    uint16_t master_window;                 //number of commands currently in frame buffer (1 or more pdo instances).
    uint8_t *off_slave_window;              //offset for slave window in the domain.
    uint16_t slave_window;                  //number of commands currently in drive buffer.
    uint16_t tx_size;                       //number tx pdo instances in the frame.
    uint16_t rx_size;                       //size of pvt buffer on slave side.
    uint16_t oid;                           //pvtcompress object id (position in mastermonitor->monitor)
    uint16_t alias;                         //slave alias
    uint16_t position;                      //slave position
    uint32_t vendor_id;                     //slave vendor_id
    uint32_t product_code;                  //slave product_code
    uint16_t assign_activate;               //bitmask for dc clock channel used for synchronization
    double sync0_st;                        //ethercat sync0 shify time (in seconds)
    double sync1_st;                        //ethercat sync1 shify time (in seconds)
    uint8_t n_pdo_entries;                  //number of slave pdo entries
    ec_pdo_entry_info_t pdo_entries[ETHERCAT_MAX_PDO_ENTRIES]; //slave pdo entries
    uint8_t n_pdos;                           //number of slave pdos
    ec_pdo_info_t pdos[ETHERCAT_MAX_PDOS];    //slave pdos
    ec_sync_info_t syncs[ETHERCAT_MAX_SYNCS]; //pdo sync manager configuration
    uint8_t *pvtdata[ETHERCAT_DOMAINS];   //pvt domain addresses
};

/* EtherCAT master status monitor */
struct mastermonitor
{
    ec_master_t *master;                          //ethercat master
    uint16_t frame_size;                          //total size in bytes of data (all domains)
    uint16_t frame_pvt_size;                      //total size in bytes of data (pvt domains only)
    uint8_t full_counter;                         //counter for signaling number of slaves for which pdo slots are full
    double sync0_ct;                              //ethercat sync0 cycle time (in seconds)
    double sync1_ct;                              //ethercat sync1 cycle time (in seconds)
    double wr_offset;                             //time offset between next write and read operation
    double frame_time;                            //time needed for a frame to be received back by the master (frame_time << min(sync0_ct, sync1_ct))
    uint8_t n_domains;                            //number of ethercat domains in use
    struct domainmonitor domains[ETHERCAT_DOMAINS]; //pvt private domains
    struct slavemonitor monitor[ETHERCAT_DRIVES];   //storage for associated slave monitors
};

/* Shared interface (between high and low level ethercat threads) */
struct sharedmonitor
{
    /* object dictionary */
    uint16_t obj_index;
    uint8_t obj_subindex;
    uint8_t obj_len;
    /* raw data */
    void *data_in;
    void *data_out;
    uint16_t data_len;
    /* callback table */
    struct command_parser **cp_table;
};

/* EtherCAT message queue */
struct ethercatqueue
{
    /* reactor and scheduling */
    struct pollreactor *pr; //ethercat low level reactor
    int pipe_sched[2]; //pipe_sched[0] = rx-command pipe, pipe_sched[1] = tx-command-pipe
    /* threading */
    int cpu; //ethercat low level thread dedicated cpu
    pthread_t tid; //ethercat low level thread id
    pthread_attr_t sched_policy; //thread scheduling policy
    struct sched_param sched_param; //thread schedulting parameters (priority)
    pthread_mutex_t lock; //protects variables below
    pthread_cond_t cond; //condition variable used for thread synchronization
    int receive_waiting; //flag indicating whether the ethercatqueue is waiting to receive data
    /* baud and clock tracking */
    double idle_time; //next time when the ethernet port is idle (ready for next operation)
    struct clock_estimate ce; //mcu clock estimate (same as serialqueue)
    /* message queues */
    struct list_head pending_queues; //drive queues of pending messages waiting to be sent
    int ready_bytes; //number of bytes ready to be sent (depends on drive pvt buffer size)
    int upcoming_bytes; //number of bytes in upcoming messages to be sent (depends on drive pvt buffer size)
    uint64_t need_kick_clock; //clock value at which the background thread needs to be woken up
    /* ethercat interface data */
    struct mastermonitor masterifc; //EtherCAT master interface
    struct sharedmonitor klippyifc; //klippy interface
    /* internal protocol data */
    uint64_t send_seq;
    uint64_t receive_seq;
    struct list_head notify_queue;
    struct list_head request_queue; //list of high level thread requests
    struct list_head response_queue; //list of low level thread responses
};


/****************************************************************
 * Public functions
 ****************************************************************/
/** configure ethercat low level thread dedicated cpu */
void ethercatqueue_config_cpu(struct ethercatqueue *sq, int cpu);

/** initialize ethercat slave */
void ethercatqueue_slave_config(struct ethercatqueue *sq,
                                uint8_t index,
                                uint16_t alias,
                                uint16_t position,
                                uint32_t vendor_id,
                                uint32_t product_code,
                                uint16_t assign_activate,
                                double sync0_st,
                                double sync1_st,
                                uint16_t rx_size);

/** configure an ethercat sync manager */
void ethercatqueue_slave_config_sync(struct ethercatqueue *sq,
                                     uint8_t slave_index,
                                     uint8_t sync_index,
                                     uint8_t direction,
                                     uint8_t n_pdo_entries,
                                     ec_pdo_entry_info_t *pdo_entries,
                                     uint8_t n_pdos,
                                     ec_pdo_info_t *pdos);

/** configure ethercat master */
void ethercatqueue_master_config(struct ethercatqueue *sq,
                                 double sync0_ct,
                                 double sync1_ct);

/** configure ethercat master domain registers */
void ethercatqueue_master_config_registers(struct ethercatqueue *sq,
                                           uint8_t index,
                                           uint8_t n_registers,
                                           ec_pdo_entry_reg_t *registers);

/** create an empty ethercatqueue object */
struct ethercatqueue *ethercatqueue_alloc(void);

/** initialize ethercatqueue */
int ethercatqueue_init(struct ethercatqueue *sq);

/** request that the background thread exit */
void ethercatqueue_exit(struct ethercatqueue *sq);

/** free all resources associated with a ethercatqueue */
void ethercatqueue_free(struct ethercatqueue *sq);

/** allocate a command_queue */
struct command_queue *ethercatqueue_alloc_commandqueue(void);

/** free a command_queue */
void ethercatqueue_free_commandqueue(struct command_queue *cq);

/** send a single synchronous command from high to low level thread (blocking) */
void
ethercatqueue_send_command(struct ethercatqueue *sq,
                           uint8_t *msg,
                           int len,
                           uint64_t min_clock,
                           uint64_t req_clock,
                           uint64_t notify_id);

/** 
 * Add a batch of messages to the given command_queue. This function is called
 * from the flush_moves in the mcu module.
 */
void ethercatqueue_send_batch(struct ethercatqueue *sq,
                              struct command_queue *cq,
                              struct list_head *msgs);

/**
 * Return a message read from the ethercat port (or wait for one if none
 * available). It is called directly from the ethercat high level thread.
 * It is a ffi c-helper function that takes a pre-processed message from
 * the low level thread and handles it properly using the main reactor.
 */
void ethercatqueue_pull(struct ethercatqueue *sq, struct pull_queue_message *pqm);

/**
 * Set the estimated clock rate of the mcu on the other end of the
 * ethercat port.
 */
void ethercatqueue_set_clock_est(struct ethercatqueue *sq,
                                 double est_freq,
                                 double conv_time,
                                 uint64_t conv_clock,
                                 uint64_t last_clock);

/* return a string buffer containing statistics for the ethercat port */
void ethercatqueue_get_stats(struct ethercatqueue *sq, char *buf, int len);

#endif // ethercatqueue.h
