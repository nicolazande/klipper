#ifndef SERIALSERVO_COMPRESS_H
#define SERIALSERVO_COMPRESS_H

#include <stdint.h> // uint32_t
#include "serialservo_solve.h" // struct pose

// Setpoint history entry for motion_report dumps (wire units)
struct pull_history_serialservo_steps {
    uint64_t first_clock;
    uint64_t last_clock;
    int64_t start_position;
    int64_t velocity;
};

struct stepcompress *serialservo_compress_alloc(uint32_t oid);
void serialservo_compress_fill(struct stepcompress *sc
                               , int32_t queue_step_msgtag
                               , int32_t pole_pairs
                               , double rotation_distance);
void serialservo_compress_set_position_offset(struct stepcompress *sc
                                              , double offset);
void serialservo_compress_free(struct stepcompress *sc);
uint32_t serialservo_compress_get_oid(struct stepcompress *sc);
int serialservo_compress_append(struct stepcompress *sc, struct pose *pose
                                , double move_time);
int serialservo_compress_queue_msg(struct stepcompress *sc, uint32_t *data
                                   , int len);
int serialservo_compress_reset(struct stepcompress *sc
                               , uint64_t last_step_clock);
double serialservo_compress_set_last_position(struct stepcompress *sc
                                              , uint64_t clock
                                              , int64_t last_position);
double serialservo_compress_find_past_position(struct stepcompress *sc
                                               , uint64_t clock);
int serialservo_compress_extract_old(struct stepcompress *sc
                                     , struct pull_history_serialservo_steps *p
                                     , int max, uint64_t start_clock
                                     , uint64_t end_clock);

#endif // serialservo_compress.h
