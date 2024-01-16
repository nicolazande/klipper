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
#define ETHERCATPVT_SIZE 8U //ethercat message size in bytes


/****************************************************************
 * Custom data types
 ****************************************************************/
/* drive specific queue message */
struct pvtmsg
{
    int len; //message size in bytes
    uint32_t oid; //associated compressor id
    __attribute__((aligned(8))) uint8_t msg[ETHERCATPVT_SIZE]; //message data (keep alignment)
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
};

#endif // ethercatmsg.h
