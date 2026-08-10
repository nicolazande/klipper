// Serialservo setpoint buffering and unit/clock conversion
//
// Copyright (C) 2024  Nicola Zandegiacomo <nicola.zandegiacomo@flyingbasket.com>
// Copyright (C) 2026  Warmbird
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // llrint
#include <stddef.h> // offsetof
#include <stdint.h> // uint32_t
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "pyhelper.h" // errorf
#include "serialqueue.h" // struct queue_message
#include "serialservo_compress.h" // serialservo_compress_append
#include "stepcompress.h" // ERROR_RET

// Each queued message tells the MCU "be at target_position with
// target_velocity at clock" - the end state of one constant
// acceleration segment sampled from the trapq.  Wire units are the
// TMC4671 register units established by the chip configuration:
// position in 1/65536 of an electrical revolution (16.16 multi-turn),
// velocity in electrical rpm.  Wire positions are in the MCU frame:
// the toolhead frame offset is applied here so kinematic re-bases
// (homing) never produce a physical jump.

#define HISTORY_EXPIRE (30.0) // history time window in seconds

// This struct must stay layout-compatible with struct stepcompress in
// stepcompress.c up to and including history_list: these objects are
// registered with the stock steppersync, whose set_time/flush code
// accesses mcu_time_offset, mcu_freq, msg_queue and the (empty) step
// queue pointers through that layout.  Serialservo-specific fields
// extend the struct after the shared prefix.
struct stepcompress {
    // Stock stepcompress layout (do not reorder)
    uint32_t *queue, *queue_end, *queue_pos, *queue_next;
    uint32_t max_error;
    double mcu_time_offset, mcu_freq, last_step_print_time;
    uint64_t last_step_clock;
    struct list_head msg_queue;
    uint32_t oid;
    int32_t queue_step_msgtag, set_next_step_dir_msgtag;
    int sdir, invert_sdir;
    uint64_t next_step_clock;
    int next_step_dir;
    int64_t stock_last_position;
    struct list_head history_list;
    // Serialservo extension
    int pole_pairs;
    double rotation_distance; // mm of travel per mechanical motor rev
    double units_per_mm; // wire position lsb per mm
    double vel_units_per_mms; // wire velocity lsb per mm/s
    double position_offset; // mcu frame minus toolhead frame (mm)
    double last_position; // last streamed mcu-frame position (mm)
};

struct history_steps {
    struct list_node node;
    uint64_t first_clock, last_clock;
    double start_position; // mcu-frame mm at first_clock
    double velocity; // mean mm/s across the segment
};


/****************************************************************
 * Internal helpers
 ****************************************************************/

// Determine the print time of the last scheduled setpoint
static void
calc_last_step_print_time(struct stepcompress *sc)
{
    double lsc = sc->last_step_clock;
    sc->last_step_print_time = sc->mcu_time_offset + (lsc - .5) / sc->mcu_freq;
}

// Free items from the history list up to end_clock
static void
free_history(struct stepcompress *sc, uint64_t end_clock)
{
    while (!list_empty(&sc->history_list)) {
        struct history_steps *hs = list_last_entry(
            &sc->history_list, struct history_steps, node);
        if (hs->last_clock > end_clock)
            break;
        list_del(&hs->node);
        free(hs);
    }
}

// Expire old history entries
static void
clean_history(struct stepcompress *sc)
{
    uint64_t hist_ticks = HISTORY_EXPIRE * sc->mcu_freq;
    if (sc->last_step_clock > hist_ticks)
        free_history(sc, sc->last_step_clock - hist_ticks);
}


/****************************************************************
 * Allocation and configuration
 ****************************************************************/

// Allocate a new serialservo compressor
struct stepcompress * __visible
serialservo_compress_alloc(uint32_t oid)
{
    struct stepcompress *sc = malloc(sizeof(*sc));
    memset(sc, 0, sizeof(*sc));
    list_init(&sc->msg_queue);
    list_init(&sc->history_list);
    sc->oid = oid;
    return sc;
}

// Fill message id and unit conversion information
void __visible
serialservo_compress_fill(struct stepcompress *sc, int32_t queue_step_msgtag
                          , int32_t pole_pairs, double rotation_distance)
{
    sc->queue_step_msgtag = queue_step_msgtag;
    sc->pole_pairs = pole_pairs;
    sc->rotation_distance = rotation_distance;
    sc->units_per_mm = pole_pairs * 65536. / rotation_distance;
    sc->vel_units_per_mms = pole_pairs * 60. / rotation_distance;
}

// Set the mcu frame offset applied to outgoing positions
void __visible
serialservo_compress_set_position_offset(struct stepcompress *sc
                                         , double offset)
{
    sc->position_offset = offset;
}

// Free memory associated with a compressor
void __visible
serialservo_compress_free(struct stepcompress *sc)
{
    if (!sc)
        return;
    while (!list_empty(&sc->msg_queue)) {
        struct queue_message *qm = list_first_entry(
            &sc->msg_queue, struct queue_message, node);
        list_del(&qm->node);
        free(qm);
    }
    free_history(sc, UINT64_MAX);
    free(sc);
}

uint32_t
serialservo_compress_get_oid(struct stepcompress *sc)
{
    return sc->oid;
}


/****************************************************************
 * Setpoint generation
 ****************************************************************/

// Queue one segment-end setpoint.  pose describes the state at the
// END of the segment (pose->time absolute print time), move_time is
// the segment duration.
int
serialservo_compress_append(struct stepcompress *sc, struct pose *pose
                            , double move_time)
{
    double mcu_pos_mm = pose->position + sc->position_offset;
    double end_time = pose->time - sc->mcu_time_offset;
    double clock_end_d = end_time * sc->mcu_freq;
    double clock_start_d = (end_time - move_time) * sc->mcu_freq;
    if (!(clock_start_d >= 0. && clock_end_d < 9e18)) {
        errorf("serialservo clock conversion out of range oid=%d"
               " time=%.3f", sc->oid, pose->time);
        return ERROR_RET;
    }
    double pos_units = mcu_pos_mm * sc->units_per_mm;
    double vel_units = pose->velocity * sc->vel_units_per_mms;
    if (!(pos_units > -2147483647. && pos_units < 2147483647.)
        || !(vel_units > -2147483647. && vel_units < 2147483647.)) {
        errorf("serialservo setpoint out of range oid=%d pos=%.3f"
               " vel=%.3f", sc->oid, mcu_pos_mm, pose->velocity);
        return ERROR_RET;
    }
    uint64_t clock_start = (uint64_t)clock_start_d;
    uint64_t clock_end = (uint64_t)clock_end_d;
    uint32_t msg[5] = {
        (uint32_t)sc->queue_step_msgtag, sc->oid,
        (uint32_t)(int32_t)llrint(pos_units),
        (uint32_t)(int32_t)llrint(vel_units),
        (uint32_t)clock_end,
    };
    struct queue_message *qm = message_alloc_and_encode(msg, ARRAY_SIZE(msg));
    qm->min_clock = sc->last_step_clock;
    qm->req_clock = clock_start;
    list_add_tail(&qm->node, &sc->msg_queue);
    // History entry (mcu frame, mean velocity is exact for linear-V)
    struct history_steps *hs = malloc(sizeof(*hs));
    hs->first_clock = clock_start;
    hs->last_clock = clock_end;
    hs->start_position = sc->last_position;
    hs->velocity = move_time > 0.
        ? (mcu_pos_mm - sc->last_position) / move_time : 0.;
    list_add_head(&hs->node, &sc->history_list);
    sc->last_position = mcu_pos_mm;
    sc->last_step_clock = clock_end;
    calc_last_step_print_time(sc);
    clean_history(sc);
    return 0;
}

// Queue an mcu command to go out in order with setpoint commands
int __visible
serialservo_compress_queue_msg(struct stepcompress *sc, uint32_t *data
                               , int len)
{
    struct queue_message *qm = message_alloc_and_encode(data, len);
    qm->min_clock = qm->req_clock = sc->last_step_clock;
    list_add_tail(&qm->node, &sc->msg_queue);
    return 0;
}

// Reset the internal state of the compressor
int __visible
serialservo_compress_reset(struct stepcompress *sc, uint64_t last_step_clock)
{
    sc->last_step_clock = last_step_clock;
    calc_last_step_print_time(sc);
    return 0;
}


/****************************************************************
 * Position tracking
 ****************************************************************/

// Note the actual mcu position (wire units) measured at clock
double __visible
serialservo_compress_set_last_position(struct stepcompress *sc
                                       , uint64_t clock
                                       , int64_t last_position)
{
    sc->last_position = (double)last_position / sc->units_per_mm;
    struct history_steps *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->first_clock = hs->last_clock = clock;
    hs->start_position = sc->last_position;
    list_add_head(&hs->node, &sc->history_list);
    return sc->last_position;
}

// Search history of moves to find the mcu position at a given clock
double __visible
serialservo_compress_find_past_position(struct stepcompress *sc
                                        , uint64_t clock)
{
    double last_position = sc->last_position;
    struct history_steps *hs;
    list_for_each_entry(hs, &sc->history_list, node) {
        if (clock < hs->first_clock) {
            last_position = hs->start_position;
            continue;
        }
        if (clock >= hs->last_clock)
            return hs->start_position + hs->velocity
                * (double)(hs->last_clock - hs->first_clock) / sc->mcu_freq;
        double interval = (double)(clock - hs->first_clock) / sc->mcu_freq;
        return hs->start_position + interval * hs->velocity;
    }
    return last_position;
}

// Return history of queued setpoints (positions in wire units)
int __visible
serialservo_compress_extract_old(struct stepcompress *sc
                                 , struct pull_history_serialservo_steps *p
                                 , int max, uint64_t start_clock
                                 , uint64_t end_clock)
{
    int res = 0;
    struct history_steps *hs;
    list_for_each_entry(hs, &sc->history_list, node) {
        if (start_clock >= hs->last_clock || res >= max)
            break;
        if (end_clock <= hs->first_clock)
            continue;
        p->first_clock = hs->first_clock;
        p->last_clock = hs->last_clock;
        p->start_position = (int64_t)llrint(
            hs->start_position * sc->units_per_mm);
        p->velocity = (int64_t)llrint(
            hs->velocity * sc->vel_units_per_mms);
        p++;
        res++;
    }
    return res;
}
