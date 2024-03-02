/**
 * \file ethercatmsg.h
 *
 * \brief EtherCAT message specific implementation.
 */

#ifndef ETHERCATMSG_H
#define ETHERCATMSG_H

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdint.h> // uint8_t
#include "list.h" // struct list_node


/****************************************************************
 * Defines
 ****************************************************************/
#define ETHERCAT_PVT_SIZE 8U //ethercat message size in bytes
#define MAX_MOVE_SEGMENTS 1024U //drive move segment buffer size
#define MAX_MSGPOOL_SLOTS 2U //number of messagepool consumers
#define ETHERCAT_CACHE_ALIGNMENT 64U //data alignment to limit cach miss


/****************************************************************
 * Custom data types
 ****************************************************************/
/* drive specific queue message */
struct move_segment_msg
{
    int len; //message size in bytes
    uint32_t oid; //associated compressor id
    __attribute__((aligned(8))) uint8_t msg[ETHERCAT_PVT_SIZE]; //message data (keep alignment)
    union
    {
        /* filled when on a command queue */
        struct
        {
            uint64_t min_clock;
            uint64_t req_clock;
        };
        /* filled when in sent/receive  */
        struct
        {
            double sent_time;
            double receive_time;
        };
    };
    uint64_t notify_id; //notify id for high level thread
    struct list_node node;
} __attribute__((aligned(ETHERCAT_CACHE_ALIGNMENT)));

/* move message pool */
struct move_msgpool
{
    struct move_segment_msg messages[MAX_MOVE_SEGMENTS]; //message buffer
    pthread_mutex_t lock; //mutex
    uint8_t n_slots; //number of messagepool consumers
    int alloc_idx[MAX_MSGPOOL_SLOTS]; //allocation index
    int free_idx[MAX_MSGPOOL_SLOTS]; //deallocation index
} __attribute__((aligned(ETHERCAT_CACHE_ALIGNMENT)));


/****************************************************************
 * Public functions
 ****************************************************************/
/* initialize message pool */
void init_msg_pool(struct move_msgpool* pool, uint8_t n_slots);

/* allocate a message from the pool */
struct move_segment_msg *emsg_alloc(struct move_msgpool* pool, uint8_t slot);

/* deallocate a message and make it available for reuse */
void emsg_free(struct move_msgpool* pool, struct move_segment_msg* msg, uint8_t slot);

#endif // ethercatmsg.h
