#include "serialservo_solve.h"


/**
 * \file serialservo_compress.h
 *
* \brief Step buffering and synchronization.
 * 
 * Take a set of steps, buffer and reorder (synchronize) them.
 */

#ifndef SERIALSERVO_COMPRESS_H
#define SERIALSERVO_COMPRESS_H

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdint.h> // uint32_t
#include "serialservo_solve.h"


/****************************************************************
 * Defines
 ****************************************************************/


/****************************************************************
 * Custom data types
 ****************************************************************/
/* pull history step used for data dump only */
struct pull_history_serialservo_steps
{
    uint64_t first_clock;
    uint64_t last_clock;
    int64_t start_position;
    int64_t velocity;
};


/****************************************************************
 * Public functions
 ****************************************************************/
/** allocate a new compressor */
struct stepcompress *serialservo_compress_alloc(uint32_t oid);

/** fill message id information */
void serialservo_compress_fill(struct stepcompress *sc,
                               int32_t queue_step_msgtag,
                               int32_t polePairs,
                               int32_t scaler);

/** free memory associated with a compressor */
void serialservo_compress_free(struct stepcompress *sc);

/** get object id of a compressor */
uint32_t serialservo_compress_get_oid(struct stepcompress *sc);

/** append step to compessor */
void serialservo_compress_append(struct stepcompress *sc, struct pose *pose, double move_time);

/** reset the internal state of the compressor */
int serialservo_compress_reset(struct stepcompress *sc, uint64_t last_step_clock);

/** set last position in the compressor */
double serialservo_compress_set_last_position(struct stepcompress *sc, uint64_t clock, int64_t last_position);

/** search history of moves to find a past position at a given clock */
double serialservo_compress_find_past_position(struct stepcompress *sc, uint64_t clock);

/** return history of queue_step commands */
int serialservo_compress_extract_old(struct stepcompress *sc,
                                     struct pull_history_serialservo_steps *p,
                                     int max,
                                     uint64_t start_clock,
                                     uint64_t end_clock);

/* queue an mcu command to go out in order with stepper commands */
int serialservo_compress_queue_msg(struct stepcompress *sc, uint32_t *data, int len);

#endif // serialservo_compress.h
