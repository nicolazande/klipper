#ifndef TRAPQ_H
#define TRAPQ_H

/****************************************************************
 * Includes
 ****************************************************************/
#include "list.h" // list_node


/****************************************************************
 * Custom datatypes
 ****************************************************************/
/* coordinate */
struct coord
{
    union
    {
        struct
        {
            double x, y, z;
        };
        double axis[3];
    };
};

/* move */
struct move
{
    double print_time, move_t;
    double start_v, half_accel;
    struct coord start_pos, axes_r;
    struct list_node node;
};

/* trapezoidal queue */
struct trapq
{
    struct list_head moves, history;
};

/* move pull */
struct pull_move
{
    double print_time, move_t;
    double start_v, accel;
    double start_x, start_y, start_z;
    double x_r, y_r, z_r;
};


/****************************************************************
 * Public functions
 ****************************************************************/
/** allocate a move object */
struct move *move_alloc(void);

/** return the distance moved given a time in a move */
double move_get_distance(struct move *m, double move_time);

/** return the coordinates given a time in a move */
struct coord move_get_coord(struct move *m, double move_time);

/** allocate a trapq object */
struct trapq *trapq_alloc(void);

/** free memory associated with a trapq object */
void trapq_free(struct trapq *tq);

/** update the list sentinels */
void trapq_check_sentinels(struct trapq *tq);

/** add a move to the trapezoid velocity queue */
void trapq_add_move(struct trapq *tq, struct move *m);

/** fill and add a move to the trapezoid velocity queue */
void trapq_append(struct trapq *tq, double print_time,
                  double accel_t, double cruise_t, double decel_t,
                  double start_pos_x, double start_pos_y, double start_pos_z,
                  double axes_r_x, double axes_r_y, double axes_r_z,
                  double start_v, double cruise_v, double accel);

/** expire any moves older than `print_time from the trapezoid velocity queue */
void trapq_finalize_moves(struct trapq *tq, double print_time, double clear_history_time);

/** note a position change in the trapq history */
void trapq_set_position(struct trapq *tq, double print_time, double pos_x, double pos_y, double pos_z);

/** return history of movement queue */
int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max, double start_time, double end_time);

#endif // trapq.h
