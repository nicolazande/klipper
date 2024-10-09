/**
 * \file ethercatservo_compress.h
 *
* \brief Step buffering and synchronization.
 * 
 * Take a set of steps, buffer and reorder (synchronize) them.
 */

#ifndef ETHERCATSERVO_COMPRESS_H
#define ETHERCATSERVO_COMPRESS_H

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdint.h> // uint32_t
#include "ethercatservo_solve.h"


/****************************************************************
 * Defines
 ****************************************************************/


/****************************************************************
 * Custom data types
 ****************************************************************/
/* pull history step used for data dump only */
struct pull_history_pvt_steps
{
    uint64_t first_clock;
    uint64_t last_clock;
    double start_position;
    double velocity;
};

/* ethercat queue forward declaration */
struct ethercatqueue;


/****************************************************************
 * Public functions
 ****************************************************************/
/** allocate a new compressor */
struct ethercatservo_compress *ethercatservo_compress_alloc(uint32_t oid, double position_scaling, double velocity_scaling);

/** free memory associated with a compressor */
void ethercatservo_compress_free(struct ethercatservo_compress *sc);

/** get object id of a compressor */
uint32_t ethercatservo_compress_get_oid(struct ethercatservo_compress *sc);

/** append step to compessor */
void ethercatservo_compress_append(struct ethercatservo_compress *sc, struct pose *pose, double move_time);

/** reset the internal state of the compressor */
int ethercatservo_compress_reset(struct ethercatservo_compress *sc, uint64_t last_step_clock);

/** set last position in the compressor */
double ethercatservo_compress_set_last_position(struct ethercatservo_compress *sc, uint64_t clock, int32_t last_position);

/** search history of moves to find a past position at a given clock */
double ethercatservo_compress_find_past_position(struct ethercatservo_compress *sc, uint64_t clock);

/** return history of queue_step commands */
int ethercatservo_compress_extract_old(struct ethercatservo_compress *sc,
                            struct pull_history_pvt_steps *p,
                            int max,
                            uint64_t start_clock,
                            uint64_t end_clock);

/** allocate a new drivesync object */
struct drivesync *drivesync_alloc(struct ethercatqueue *sq,
                                    struct ethercatservo_compress **sc_list,
                                    int sc_num,
                                    int move_num);

/** free memory associated with a drivesync object */
void drivesync_free(struct drivesync *ss);

/** set the conversion rate of print_time to mcu clock */
void drivesync_set_time(struct drivesync *ss, double time_offset, double mcu_freq);

/** 
 * Find and transmit any scheduled steps prior to the given move_clock for
 * the drivesync common object, i.e. reorder and merge into a single queue
 * the steps of all associated drives (in the command queue).
 */
int drivesync_flush(struct drivesync *ss, uint64_t move_clock, uint64_t clear_history_clock);

#endif // ethercatservo_compress.h
