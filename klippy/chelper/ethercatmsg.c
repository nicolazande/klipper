/**
 * \file ethercatmsg.c
 *
 * \brief EtherCAT message specific implementation.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <pthread.h>
#include "pyhelper.h" // errorf
#include "compiler.h"
#include "ethercatmsg.h" // message_alloc


/****************************************************************
 * Public functions
 ****************************************************************/
/* initialize message pool */
void init_msg_pool(struct move_msgpool* pool)
{
    /* initialize indexes */
    pool->alloc_idx = 0;
    pool->free_idx = 0;
    /**
     * Initialize message pool mutex.
     * NOTE: with the current implementation there is no need to block
     *       since one thread only moves allocation index and the other
     *       moves the free index.
     */
    pthread_mutex_init(&pool->lock, NULL);
}

/* allocate a message from the pool */
struct move_segment_msg *emsg_alloc(struct move_msgpool* pool)
{
    /* lock mutex */
    //pthread_mutex_lock(&pool->lock);

    /* segment move */
    struct move_segment_msg *move;

    /* check if the pool is full */
    if ((pool->alloc_idx + 1) % MAX_MOVE_SEGMENTS == pool->free_idx)
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
        errorf("error: allocate message memory");
    }
    else
    {
        /* get move from message pool */
        move = &(pool->messages[pool->alloc_idx]);
        memset(move, 0, sizeof(*move));

        /* update message pool allocation index */
        pool->alloc_idx = (pool->alloc_idx + 1) % MAX_MOVE_SEGMENTS;
    }

    /* release mutex */
    //pthread_mutex_unlock(&pool->lock);

    return move;
}

/* deallocate a message and make it available for reuse */
void emsg_free(struct move_msgpool* pool, struct move_segment_msg* msg)
{
    /* lock mutex */
    //pthread_mutex_lock(&pool->lock);

    /* calculate the index of the message */
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
            errorf("error: free message memory");
        }
    }
    else
    {
        /* update message pool deallocation index */
        if (pool->free_idx == index)
        {
            pool->free_idx = (pool->free_idx + 1) % MAX_MOVE_SEGMENTS;
        }
    }

    /* release mutex */
    //pthread_mutex_unlock(&pool->lock);
}