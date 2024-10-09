/**
 * \file ethercatmsg.c
 *
 * \brief EtherCAT message specific implementation.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <pthread.h>
#include "ethercatmsg.h" // message_alloc
#include "pyhelper.h" // errorf
#include "compiler.h"


/****************************************************************
 * Public functions
 ****************************************************************/
/* initialize message pool */
void init_msg_pool(struct move_msgpool* pool, uint8_t n_slots)
{
    /* initialize data */
    pool->n_slots = n_slots;
    for (uint8_t i = 0; i < n_slots; i++)
    {
        /* shift consumer by one slot */
        pool->alloc_idx[i] = pool->free_idx[i] = i;
    }

    /* clear message buffer */
    memset(pool->messages, 0, sizeof(pool->messages));

    /**
     * Initialize message pool mutex.
     * NOTE: with the current implementation there is no need to block
     *       since one thread only moves allocation index and the other
     *       moves the free index.
     */
    pthread_mutex_init(&pool->lock, NULL);
}

/* allocate a message from the pool */
struct move_segment_msg *emsg_alloc(struct move_msgpool* pool, uint8_t slot)
{
    /* lock mutex */
    //pthread_mutex_lock(&pool->lock);

    /* data */
    struct move_segment_msg *move;
    int alloc_idx = pool->alloc_idx[slot];
    int free_idx = pool->free_idx[slot];

    /* check if the pool is full */
    if ((alloc_idx + pool->n_slots) % MAX_MOVE_SEGMENTS == free_idx)
    {
        /**
         * Direct memory allocation, in case the allocated buffer is
         * to small allocate dynamic memory in the heap.
         * NOTE: this is a backup strategy and should be avoided as
         *       much as possible since allocation and deallocation
         *       times are not deterministic.
         */
        move = malloc(sizeof(*move));
        memset(move, 0, sizeof(*move));
        //errorf("error: dynamic alloc = %d", pool->alloc_idx[slot]);
    }
    else
    {
        /* get move from message pool */
        move = &pool->messages[alloc_idx];
        memset(move, 0, sizeof(*move));

        /* update message pool allocation index */
        pool->alloc_idx[slot] = (alloc_idx + pool->n_slots) % MAX_MOVE_SEGMENTS;
    }

    /* release mutex */
    //pthread_mutex_unlock(&pool->lock);

    return move;
}

/* deallocate a message and make it available for reuse */
void emsg_free(struct move_msgpool* pool, struct move_segment_msg* msg, uint8_t slot)
{
    /* lock mutex */
    //pthread_mutex_lock(&pool->lock);

    /* data */
    //int alloc_idx = pool->alloc_idx[slot];
    int free_idx = pool->free_idx[slot];
    int index = msg - pool->messages;

    /* ceck if the message is within the valid range */
    if ((msg < pool->messages) || (msg >= pool->messages + MAX_MOVE_SEGMENTS))
    {
        /* check message pointer */
        if (msg)
        {
            /**
             * The move segment message was allocated with the backup
             * strategy and is in the heap, therfore free its memory.
             * NOTE: this is a backup strategy and should be avoided as
             *       much as possible since allocation and deallocation
             *       times are not deterministic.
             */
            free(msg);
            //errorf("error: dynamic free = %d", pool->free_idx[slot]);
        }
    }
    else
    {
        /* update message pool deallocation index */
        if (free_idx == index)
        {
            pool->free_idx[slot] = (free_idx + pool->n_slots) % MAX_MOVE_SEGMENTS;
        }
        else
        {
            errorf("error: out of order (free_idx = %d, index = %d)", pool->free_idx[slot], index);
        }
    }

    /* release mutex */
    //pthread_mutex_unlock(&pool->lock);
}