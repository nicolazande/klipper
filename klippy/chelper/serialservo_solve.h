/**
 * \file serialservo_solve.h
 *
 * \brief PVT specific iterative solver for kinematic moves.
 *        TODO: it depends on the specific drive selected (Copley),
 *              maybe it is metter to move this definition in an
 *              external hardware dependent only file and keep here
 *              only a forward declaration.
 */

#ifndef SERIALSERVO_SOLVE_H
#define SERIALSERVO_SOLVE_H

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdint.h>
#include "servo_solve.h"


/****************************************************************
 * Custom data types
 ****************************************************************/
/* serialservo_kinematics forward declaration */
struct serialservo_kinematics;

/* trapezoidal queue move forward declaration */
struct move;

/* 
 * Drive kinematics struct, we can keep the same name and include this
 * header for PVT drives and the standard serialservo_solve.h for steppers.
 * The struct size will be the same and the order of the common fields
 * as well. In this way we achieve a sort of runtime polimorphism.
 */
struct serialservo_kinematics
{
    union 
    {
        double step_dist; //step distance (compatibility with stepper)
        double simtime; //simulation time (pvt specific)
    };
    double commanded_pos; //commanded position (updated at the end of each move)
    struct stepcompress *sc; //step compressor
    double last_flush_time; //last time move queue flushed
    double last_move_time; //last move time
    struct trapq *tq; //trapezoidal queue to sample
    int active_flags; //active flags utils
    double gen_steps_pre_active; //pre-move max time window
    double gen_steps_post_active; //post-move max time window
    struct pose (*kinematics_cb)(struct move *m, double move_time); //forward kinematics callback
};


/****************************************************************
 * Public Functions
 ****************************************************************/
/** generate step times for a range of moves on the trapq */
int32_t serialservo_solve_generate_steps(struct serialservo_kinematics *sk, double flush_time);

/** check if the given drive is likely to be active in the given time range */
double serialservo_solve_check_active(struct serialservo_kinematics *sk, double flush_time);

/** report if the given stepper is registered for the given axis */
int32_t serialservo_solve_is_active_axis(struct serialservo_kinematics *sk, char axis);

/** set serialservo_kinematics trapezoidal queue */
void serialservo_solve_set_trapq(struct serialservo_kinematics *sk, struct trapq *tq);

/** set serialservo_kinematics stepcompress */
void serialservo_solve_set_stepcompress(struct serialservo_kinematics *sk, struct stepcompress *sc, double simtime);

/** calculate toolhead position from coordinates */
double serialservo_solve_calc_position_from_coord(struct serialservo_kinematics *sk, double x, double y, double z);

/** set toolhead position */
void serialservo_solve_set_position(struct serialservo_kinematics *sk, double x, double y, double z);

/** get toolhead position */
double serialservo_solve_get_commanded_pos(struct serialservo_kinematics *sk);

#endif // serialservo_solve.h
