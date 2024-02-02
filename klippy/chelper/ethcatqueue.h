#ifndef ETHCATQUEUE_H
#define ETHCATQUEUE_H

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
#define ETHCAT_DRIVES 2  //number of ethercat drives
#define ETHCAT_PVT_DOMAINS 2 //number of pvt domains (one for each pdo instance)
#define ETHCAT_STANDARD_DOMAINS 1  //number of pvt domains (one for each pdo instance)
#define ETHCAT_DOMAINS (ETHCAT_PVT_DOMAINS + ETHCAT_STANDARD_DOMAINS) //total number of domains
#define ETHCAT_MAX_SYNCS 3 //max number of syncs per slave
#define ETHCAT_MAX_REGISTERS 3 //max number of master registers
#define ETHCAT_MAX_PDOS 3 //max number of slave pdos (per slave)
#define ETHCAT_MAX_PDO_ENTRIES 3 //max number of pdo enries per slave
#define ETHCAT_DRIVE_MASK ((1 << (ETHCAT_DRIVES)) - 1) //operation complete mask


/****************************************************************
 * Custom data types
 ****************************************************************/
/* pull queue message complete forward declaration */
struct pull_queue_message
{
    uint8_t msg[MESSAGE_MAX];
    int len;
    double sent_time, receive_time;
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
    uint8_t mask;                       //number of active associated drives (in current cycle)
    uint8_t n_registers;                //number of registers in the domain
    uint32_t offsets[ETHCAT_MAX_REGISTERS]; //domain register offsets
    ec_pdo_entry_reg_t registers[ETHCAT_MAX_REGISTERS]; //domain registers
};

/* EtherCAT slave status monitor */
struct slavemonitor
{
    uint16_t master_window;                 //number of commands currently in frame buffer (1 or more pdo instances).
    uint32_t off_slave_window;              //offset for slave window in the domain.
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
    ec_pdo_entry_info_t pdo_entries[ETHCAT_MAX_PDO_ENTRIES]; //slave pdo entries
    uint8_t n_pdos;                         //number of slave pdos
    ec_pdo_info_t pdos[ETHCAT_MAX_PDOS];    //slave pdos
    ec_sync_info_t syncs[ETHCAT_MAX_SYNCS]; //pdo sync manager configuration
    uint8_t *pvtdata[ETHCAT_PVT_DOMAINS];   //pvt domain addresses
};

/* EtherCAT master status monitor */
struct mastermonitor
{
    ec_master_t *master;                          //ethercat master
    uint16_t frame_size;                          //total size in bytes of pvt data (sum of domains.domain_size)
    uint8_t full_counter;                         //counter for signaling number of slaves for which pdo slots are full
    double sync0_ct;                              //ethercat sync0 cycle time (in seconds)
    double sync1_ct;                              //ethercat sync1 cycle time (in seconds)
    double frame_time;                            //time needed for a frame to be received back by the master (frame_time << min(sync0_ct, sync1_ct))
    uint8_t n_domains;                            //number of ethercat domains in use
    struct domainmonitor domains[ETHCAT_DOMAINS]; //pvt private domains
    struct slavemonitor monitor[ETHCAT_DRIVES];   //storage for associated slave monitors
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
struct ethcatqueue
{
    /* reactor and scheduling */
    struct pollreactor *pr; //ethercat low level reactor
    int pipe_sched[2]; //pipe_sched[0] = rx-command pipe, pipe_sched[1] = tx-command-pipe
    /* threading */
    pthread_t tid; //ethcat low level thread id
    pthread_attr_t sched_policy; //thread scheduling policy
    struct sched_param sched_param; //thread schedulting parameters (priority)
    pthread_mutex_t lock; //protects variables below
    pthread_cond_t cond; //condition variable used for thread synchronization
    int receive_waiting; //flag indicating whether the ethcatqueue is waiting to receive data
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
/** initialize ethercat slave */
void ethcatqueue_slave_config(struct ethcatqueue *sq,
                              uint8_t index,
                              uint16_t alias,
                              uint16_t position,
                              uint32_t vendor_id,
                              uint32_t product_code,
                              uint16_t assign_activate,
                              double sync0_st,
                              double sync1_st);

/** configure ethercat slava pdos for a sync manager */
void ethcatqueue_slave_config_pdos(struct ethcatqueue *sq,
                                   uint8_t slave_index,
                                   uint8_t sync_index,
                                   uint8_t direction,
                                   uint8_t n_pdo_entries,
                                   ec_pdo_entry_info_t *pdo_entries,
                                   uint8_t n_pdos,
                                   ec_pdo_info_t *pdos);

/** configure ethercat master */
void ethcatqueue_master_config(struct ethcatqueue *sq,
                               double sync0_ct,
                               double sync1_ct);

/** configure ethercat master domain registers */
void ethcatqueue_master_config_registers(struct ethcatqueue *sq,
                                         uint8_t index,
                                         uint8_t n_registers,
                                         ec_pdo_entry_reg_t *registers);

/** create an empty ethcatqueue object */
struct ethcatqueue *ethcatqueue_alloc(void);

/** initialize ethcatqueue */
int ethcatqueue_init(struct ethcatqueue *sq);

/** request that the background thread exit */
void ethcatqueue_exit(struct ethcatqueue *sq);

/** free all resources associated with a ethcatqueue */
void ethcatqueue_free(struct ethcatqueue *sq);

/** allocate a command_queue */
struct command_queue *ethcatqueue_alloc_commandqueue(void);

/** free a command_queue */
void ethcatqueue_free_commandqueue(struct command_queue *cq);

/** send a single synchronous command from high to low level thread (blocking) */
void
ethcatqueue_send_command(struct ethcatqueue *sq,
                         uint8_t *msg,
                         int len,
                         uint64_t min_clock,
                         uint64_t req_clock,
                         uint64_t notify_id);

/** 
 * Add a batch of messages to the given command_queue. This function is called
 * from the flush_moves in the mcu module.
 */
void ethcatqueue_send_batch(struct ethcatqueue *sq,
                            struct command_queue *cq,
                            struct list_head *msgs);

/**
 * Return a message read from the ethcat port (or wait for one if none
 * available). It is called directly from the ethercat high level thread.
 * It is a ffi c-helper function that takes a pre-processed message from
 * the low level thread and handles it properly using the main reactor.
 */
void ethcatqueue_pull(struct ethcatqueue *sq, struct pull_queue_message *pqm);

/** ffi c-helper function used to set ethercat frequency */
void ethcatqueue_set_wire_frequency(struct ethcatqueue *sq, double frequency);

/**
 * Set the estimated clock rate of the mcu on the other end of the
 * ethcat port.
 */
void ethcatqueue_set_clock_est(struct ethcatqueue *sq,
                               double est_freq,
                               double conv_time,
                               uint64_t conv_clock,
                               uint64_t last_clock);

/** return the latest clock estimate */
void ethcatqueue_get_clock_est(struct ethcatqueue *sq, struct clock_estimate *ce);

/* return a string buffer containing statistics for the ethcat port */
void ethcatqueue_get_stats(struct ethcatqueue *sq, char *buf, int len);

#endif // ethcatqueue.h
