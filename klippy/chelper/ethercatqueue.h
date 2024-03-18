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
#include "command.h"
#include "ecrt.h"
#include "pvtsolve.h"
#include "ethercatmsg.h"


/****************************************************************
 * Defines
 ****************************************************************/
#ifndef MAX_CLOCK //max clock
#define MAX_CLOCK 0x7fffffffffffffffLL
#endif
#define ETHERCAT_DRIVES 2  //number of ethercat drives
#define ETHERCAT_DOMAINS 1 //total number of domains
#define ETHERCAT_MAX_SYNCS 5 //max number of syncs per slave
#define ETHERCAT_MAX_REGISTERS 20 //max number of master registers
#define ETHERCAT_MAX_PDOS 10 //max number of slave pdos (per slave)
#define ETHERCAT_MAX_PDO_ENTRIES 20 //max number of pdo enries per slave
#define ETHERCAT_PVT_SIZE 8U //ethercat message size in bytes


/****************************************************************
 * Custom data types
 ****************************************************************/
/* pull queue message */
struct pull_queue_message
{
    uint8_t msg[MESSAGE_MAX];
    int len;
    double sent_time;
    double receive_time;
    uint64_t notify_id;
};

/* ethercat domain wrapper */
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

/* ethercat slave wrapper */
struct slavemonitor
{
    /* etherlab master */
    ec_slave_config_t *slave;      //ethercat slave
    /* objects */
    uint8_t *off_slave_window;     //offset for slave window in the domain.
    int16_t slave_window;         //number of commands currently in drive buffer (local copy).
    uint8_t *off_control_word;     //control word image offset
    uint16_t control_word;         //local copy of control word
    uint8_t *off_status_word;      //status word image offset
    uint16_t status_word;          //local copy of status word
    uint8_t *off_operation_mode;   //operation mode offset
    int8_t operation_mode;         //local copy of operation mode
    uint8_t *off_position_actual;  //actual position offset
    int32_t position_actual;       //local copy of actual position
    uint8_t *off_velocity_actual;  //actual velocity offset
    int32_t velocity_actual;       //local copy of actual velocity
    uint8_t *off_buffer_status;    //segment buffer status
    uint32_t buffer_status;        //local copy of segment buffer status
    /* sdo */
    ec_sdo_request_t *interpolation_mode_sdo; //sdo for setting interpolation mode
    ec_sdo_request_t *operation_mode_sdo; //sdo for setting operation mode
    ec_sdo_request_t *homing_method_sdo; //sdo for setting operation mode
    /* monitoring */
    uint16_t seq_num;
    uint16_t tx_size;              //number tx pdo instances in the frame
    uint16_t rx_size;              //size of pvt buffer on slave side
    uint8_t interpolation_window;  //slave windom minimum active size for interpolation
    uint16_t master_window;        //number of commands currently in frame buffer (1 or more pdo instances)
    /* configuration */
    uint16_t alias;                //slave alias
    uint16_t position;             //slave position
    uint32_t vendor_id;            //slave vendor_id
    uint32_t product_code;         //slave product_code
    uint16_t assign_activate;      //bitmask for dc clock channel used for synchronization
    uint16_t oid;                  //compressor object id (position in mastermonitor->monitor)
    /* pdo data */
    uint8_t n_pdo_entries;         //number of slave pdo entries
    ec_pdo_entry_info_t pdo_entries[ETHERCAT_MAX_PDO_ENTRIES]; //slave pdo entries
    uint8_t n_pdos;                //number of slave pdos
    ec_pdo_info_t pdos[ETHERCAT_MAX_PDOS]; //slave pdos
    ec_sync_info_t syncs[ETHERCAT_MAX_SYNCS]; //pdo sync manager configuration
    uint8_t *movedata[ETHERCAT_DOMAINS]; //ip segment move offset (support for multiple domains)
};

/* ethecat master wrapper */
struct mastermonitor
{
    /* etherlab master */
    ec_master_t *master;                            //ethercat master
    ec_master_state_t state;                        //master state
    /* monitoring */
    uint16_t frame_size;                            //total size in bytes of data (all domains)
    uint16_t frame_segment_size;                    //total size in bytes of data (pvt domains only)
    uint8_t full_counter;                           //counter for signaling number of slaves for which pdo slots are full
    /* configuration */
    double sync0_ct;                                //ethercat sync0 cycle time (in seconds)
    double sync0_st;                                //ethercat sync0 shify time (in seconds)
    double sync1_ct;                                //ethercat sync1 cycle time (in seconds)
    double sync1_st;                                //ethercat sync1 shify time (in seconds)
    double frame_time;                              //time needed for a frame to be received back by the master (frame_time << min(sync0_ct, sync1_ct))
    /* data buffers */
    uint8_t n_domains;                              //number of ethercat domains in use
    struct domainmonitor domains[ETHERCAT_DOMAINS]; //pvt private domains
    struct slavemonitor monitor[ETHERCAT_DRIVES];   //storage for associated slave monitors
};

/* ethercat message queue */
struct ethercatqueue
{
    /* reactor and scheduling */
    struct pollreactor *pr; //ethercat low level reactor
    /* threading */
    int cpu; //ethercat low level thread dedicated cpu
    pthread_t tid; //ethercat low level thread id
    pthread_attr_t sched_policy; //thread scheduling policy
    struct sched_param sched_param; //thread schedulting parameters (priority)
    pthread_mutex_t lock; //protects variables below
    pthread_cond_t cond; //condition variable used for thread synchronization
    int receive_waiting; //flag indicating whether the ethercatqueue is waiting to receive data
    /* baud and clock tracking */
    struct clock_estimate ce; //mcu clock estimate (same as serialqueue)
    uint32_t last_clock; //last input event (read operation) clock time
    /* message queues */
    struct list_head ready_queue; //list of messages ready to be sent
    struct list_head upcoming_queue; //list of upcoming messages
    int ready_bytes; //number of bytes ready to be sent (depends on drive pvt buffer size)
    int upcoming_bytes; //number of bytes in upcoming messages to be sent (depends on drive pvt buffer size)
    /* ethercat interface data */
    struct mastermonitor masterifc; //EtherCAT master interface
    /* internal protocol data */
    struct list_head notify_queue;  //list of message completitions
    struct list_head request_queue; //list of high level thread requests
    struct list_head response_queue; //list of low level thread responses
    struct command_parser **cp_table; //external list of protocol commands
    /* allocation */
    struct move_msgpool msgpool;
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
                                uint8_t rx_size,
                                uint8_t interpolation_window);

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
                                 double sync0_st,
                                 double sync1_ct,
                                 double sync1_st,
                                 double frame_time);

/** configure ethercat master domain registers */
void ethercatqueue_master_config_registers(struct ethercatqueue *sq,
                                           uint8_t index,
                                           uint8_t n_registers,
                                           ec_pdo_entry_reg_t *registers);

/** get ethercatqueue data */
struct ethercatqueue *ethercatqueue_get(void);

/** initialize ethercatqueue */
int ethercatqueue_init(struct ethercatqueue *sq);

/** request that the background thread exit */
void ethercatqueue_exit(struct ethercatqueue *sq);

/** free all resources associated with a ethercatqueue */
void ethercatqueue_free(struct ethercatqueue *sq);

/** send a single synchronous command from high to low level thread (blocking) */
void
ethercatqueue_send_command(struct ethercatqueue *sq,
                           uint8_t *msg,
                           int len,
                           uint64_t min_clock,
                           uint64_t req_clock,
                           uint64_t notify_id);

/** 
 * Add a batch of messages to the upcoming queue. This function is called
 * from the flush_moves in the mcu module.
 */
void ethercatqueue_send_batch(struct ethercatqueue *sq, struct list_head *msgs);

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
