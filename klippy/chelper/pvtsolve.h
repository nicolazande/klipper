/**
 * \file pvtsolve.h
 *
 * \brief PVT specific iterative solver for kinematic moves.
 *        TODO: it depends on the specific drive selected (Copley),
 *              maybe it is metter to move this definition in an
 *              external hardware dependent only file and keep here
 *              only a forward declaration.
 */

#ifndef PVTSOLVE_H
#define PVTSOLVE_H

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdint.h>
#include "copley.h"


/****************************************************************
 * Custom data types
 ****************************************************************/
/* drive axes enum */
enum
{
    AF_X = 1 << 0, AF_Y = 1 << 1, AF_Z = 1 << 2,
};

/* drive pose */
struct pose
{
    double position; //start position
    double velocity; //current speed
    double time; //absolute start time
};

/* drive_kinematics forward declaration */
struct drive_kinematics;

/* trapezoidal queue move forward declaration */
struct move;

/* drive_kinematics calc position callback */
typedef struct pose (*sk_fwk_callback)(struct drive_kinematics *sk, struct move *m, double move_time);

/* drive_kinematics post calc position callback (polar kinematics only) */
typedef void (*sk_post_callback)(struct drive_kinematics *sk);

/* 
 * Drive kinematics struct, we can keep the same name and include this
 * header for PVT drives and the standard pvtsolve.h for steppers.
 * The struct size will be the same and the order of the common fields
 * as well. In this way we achieve a sort of runtime polimorphism.
 */
struct drive_kinematics
{
    union 
    {
        double step_dist; //step distance (compatibility with stepper)
        double simtime; //simulation time (pvt specific)
    };
    double commanded_pos; //commanded position (updated at the end of each move)
    struct pvtcompress *sc; //pvt compress
    double last_flush_time; //last time move queue flushed
    double last_move_time; //last move time
    struct trapq *tq; //trapezoidal queue to sample
    int active_flags; //active flags utils
    double gen_steps_pre_active; //pre-move max time window
    double gen_steps_post_active; //post-move max time window
    sk_fwk_callback kinematics_cb; //forward kinematics callback
    sk_post_callback post_cb; //compatibility with polar kinematics
};


/****************************************************************
 * Public Functions
 ****************************************************************/
/** generate step times for a range of moves on the trapq */
int32_t pvtsolve_generate_steps(struct drive_kinematics *sk, double flush_time);

/** check if the given drive is likely to be active in the given time range */
double pvtsolve_check_active(struct drive_kinematics *sk, double flush_time);

/** report if the given stepper is registered for the given axis */
int32_t pvtsolve_is_active_axis(struct drive_kinematics *sk, char axis);

/** set drive_kinematics trapezoidal queue */
void pvtsolve_set_trapq(struct drive_kinematics *sk, struct trapq *tq);

/** set drive_kinematics pvtcompress */
void pvtsolve_set_pvtcompress(struct drive_kinematics *sk, struct pvtcompress *sc, double simtime);

/** calculate toolhead position from coordinates */
double pvtsolve_calc_position_from_coord(struct drive_kinematics *sk, double x, double y, double z);

/** set toolhead position */
void pvtsolve_set_position(struct drive_kinematics *sk, double x, double y, double z);

/** get toolhead position */
double pvtsolve_get_commanded_pos(struct drive_kinematics *sk);

#endif // pvtsolve.h
