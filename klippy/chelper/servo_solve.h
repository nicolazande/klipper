/**
 * \file servo_solve.h
 *
 * \brief This file contains the common definitions that are used
 *        by all drives (serialservo, ethercatdrive, ...).
 */

#ifndef SERVO_SOLVE_H
#define SERVO_SOLVE_H


/****************************************************************
 * Includes
 ****************************************************************/


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

#endif // servo_solve.h