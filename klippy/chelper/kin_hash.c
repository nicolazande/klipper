/**
 * \file kin_hash.c
 *
 * \brief Hash kinematics drive pose generation.
 * 
 */


/****************************************************************
 * Includes
 ****************************************************************/
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "pvtsolve.h" // struct drive_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord


/****************************************************************
 * Private functions
 ****************************************************************/
/** velocity step update helper */
static inline double
move_get_velocity(struct move *m, double move_time)
{
    return (m->start_v + 2 * m->half_accel * move_time);
}

/** forward kinematics for x axis */
static struct pose
x_axis_forward_kinematics(struct drive_kinematics *sk, struct move *m, double move_time)
{
    struct pose pose;
    pose.position = move_get_coord(m, move_time).x;
    pose.velocity = move_get_velocity(m, move_time) * m->axes_r.x;
    pose.time = m->print_time + move_time;
    return pose;
}

/** forward kinematics for y axis */
static struct pose
y_axis_forward_kinematics(struct drive_kinematics *sk, struct move *m, double move_time)
{
    struct pose pose;
    pose.position = move_get_coord(m, move_time).y;
    pose.velocity = move_get_velocity(m, move_time) * m->axes_r.y;
    pose.time = m->print_time + move_time;
    return pose;
}

/** forward kinematics for z axis */
static struct pose
z_axis_forward_kinematics(struct drive_kinematics *sk, struct move *m, double move_time)
{
    struct pose pose;
    pose.position = move_get_coord(m, move_time).z;
    pose.velocity = move_get_velocity(m, move_time) * m->axes_r.z;
    pose.time = m->print_time + move_time;
    return pose;
}


/****************************************************************
 * Public functions
 ****************************************************************/
/** allocate drive kinematics object and associate specific callback */
struct drive_kinematics * __visible
hash_drive_alloc(char axis)
{
    /* create kinematic structure */
    struct drive_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));

    /* associate axis callbacks at runtime */
    if (axis == 'x')
    {
        sk->kinematics_cb = x_axis_forward_kinematics;
        sk->active_flags = AF_X;
    }
    else if (axis == 'y')
    {
        sk->kinematics_cb = y_axis_forward_kinematics;
        sk->active_flags = AF_Y;
    }
    else if (axis == 'z')
    {
        sk->kinematics_cb = z_axis_forward_kinematics;
        sk->active_flags = AF_Z;
    }
    
    return sk;
}


