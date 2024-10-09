/**
 * \file ethercatservo_solve.c
 *
 * \brief Solver and sampler for kinematic moves in position, velocity time format.
 * 
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <math.h> // fabs
#include <stddef.h> // offsetof
#include <string.h> // memset
#include "compiler.h" // __visible
#include "ethercatservo_solve.h" // ethercatservo_solve_generate_steps for pvt
#include "pyhelper.h" // errorf
#include "ethercatservo_compress.h" // queue_append_start
#include "trapq.h" // struct move


/****************************************************************
 * Defines
 ****************************************************************/
#define MIN_MOVE_TIME (0.001)


/****************************************************************
 * Private functions
 ****************************************************************/
/** generate steps for a portion of a move */
static int32_t
gen_steps_range(struct drive_kinematics *sk, struct move *m, double abs_start, double abs_end)
{
    /* simulation data */
    double start = abs_start - m->print_time; //move relative start time
    double end = abs_end - m->print_time; //move relative end time
    double dt = sk->simtime; //sampling time

    /* clamp time range */
    if (start < 0.)
    {
        /*
         * When abs_start = sk->last_flush_time is greater than m->print_time, it means
         * the last flush operation terminated in between the last move. The first move
         * of the current flush operation is the same of the last one. In this case we
         * need to take into account the time simulated in the previous one (maybe some
         * samples were generated as well in this time range) and restart the current
         * simulation from there (avoid duplicated samples). Should also never be
         * negative, therefore clamp it to zero.
         */
        start = 0.;
    }
    if (end > m->move_t)
    {
        /* one move at a time */
        end = m->move_t;
    }

    /* simulation pose (initilaize with previous step values) */
    struct pose pose =
    {
        .position = sk->commanded_pos,
        .velocity = 0.,
        .time = sk->last_move_time
    };

    /* run simulation (avoid zero moves) */
    while (start + MIN_MOVE_TIME <= end)
    {
        /**
         * Smooth stop check. NOTE: the delta time needs to be a positive
         * integer multiple of 1ms in order for the drive to perform a
         * proper interpolation, therfore try to always round up time.
         */
        if (end - start < 2. * dt)
        {
            /* adapt last step time duration */
            dt = end - start;
        }

        /* 
         * Calculate current pose using incremental simulation time in order
         * to avoid cumulative error due to approximation.
         */
        pose = sk->kinematics_cb(m, start);

        /* append move to message queue */
        ethercatservo_compress_append(sk->sc, &pose, dt);

        /* update start time and move on */
        start += dt;
    }

    /* 
     * Update final position, take into account also the delta
     * time of the last step since it produces a movement too.
     */
    sk->commanded_pos = sk->kinematics_cb(m, start).position;

    /* execute optinal post callback */
    if (sk->post_cb)
    {
        sk->post_cb(sk);
    }

    return 0;
}

/**
 * Check if a move is likely to cause movement on the drive.
 */
static int
check_active(struct drive_kinematics *sk, struct move *m)
{
    int af = sk->active_flags;
    return ((af & AF_X && m->axes_r.x != 0.) ||
            (af & AF_Y && m->axes_r.y != 0.) ||
            (af & AF_Z && m->axes_r.z != 0.));
}


/****************************************************************
 * Public functions
 ****************************************************************/
/** 
 * Generate steps for a range of moves on the trapezoidal queue in
 * the time interval [last_flush_time, flush_time].
 */
int32_t __visible
ethercatservo_solve_generate_steps(struct drive_kinematics *sk, double flush_time)
{
    /* save last flush */
    double last_flush_time = sk->last_flush_time;

    /* update flush time horizon */
    sk->last_flush_time = flush_time;
    if (!sk->tq)
    {
        return 0;
    }

    /* check trapezoidal queue sentinels */
    trapq_check_sentinels(sk->tq);

    /* get first move in queue */
    struct move *m = list_first_entry(&sk->tq->moves, struct move, node);

    /* 
     * Skip already flushed moves. If the last flush time was in between a
     * move (not completed), this step generation restarts from the same
     * move, taking into account the already generated steps.
     */
    while (last_flush_time >= m->print_time + m->move_t)
    {
        m = list_next_entry(m, node);
    }

    /* 
     * Get force steps time for post active move (an active move cannot be
     * interrupted immediately due to extruder and kinematic shaper).
     */
    double force_steps_time = sk->last_move_time + sk->gen_steps_post_active;
    
    /* counter of moves not procucing real movement on the drive */
    int skip_count = 0;

    /* process all moves in the trapezoidal queue */
    for (;;)
    {
        /* get current move time boundaries */
        double move_start = m->print_time;
        double move_end = move_start + m->move_t;

        /* check if move causes a move on drive */
        if (check_active(sk, m)) 
        {
            /* consider also previously skipped (inactive) moves */
            if (skip_count && sk->gen_steps_pre_active) 
            {
                /* 
                 * Must generate steps leading up to motor activity for the
                 * extruder and the kinematic shaper.
                 */
                double abs_start = move_start - sk->gen_steps_pre_active;
                if (abs_start < last_flush_time)
                {
                    abs_start = last_flush_time;
                }
                if (abs_start < force_steps_time)
                {
                    abs_start = force_steps_time;
                }

                /*
                 * Recover all inactive moves not executed. The sum of multiple inactive
                 * moves may be not negligible therefore move back to first not executed.
                 */
                struct move *pm = list_prev_entry(m, node);
                while ((--skip_count > 0) && (pm->print_time > abs_start))
                {
                    /* find first inactive move */
                    pm = list_prev_entry(pm, node);
                }

                /* 
                 * Generate steps for all previous inactive moves. They may be all
                 * describing the same position if no movement is needed, but this
                 * helps the drive to maintain the proper position.
                 */
                do 
                {
                    /* one move at a time (even if flush_time far in future) */
                    int32_t ret = gen_steps_range(sk, pm, abs_start, flush_time);
                    if (ret)
                    {
                        return ret;
                    }
                    pm = list_next_entry(pm, node);
                } while (pm != m);
            }

            /* 
             * Generate steps for current move. Each move is first divided in three parts:
             * [acceleration, constant speed, deceleration] in the trapezoidal queue and
             * in this second step each part is sampled with a constant time interval.
             * This approach allows to have a higher control on the executed trajectory
             * and guarantees more independence from the drive itself. In the limit case,
             * when using a high time interval between steps, the trapezoidal queue and the
             * sampled queue correspond, i.e. there are no intermediate samples.
             */
            int32_t ret = gen_steps_range(sk, m, last_flush_time, flush_time);
            if (ret)
            {
                return ret;
            }

            /* time window expired (correct last move time and stop) */
            if (move_end >= flush_time)
            {
                /* update end time of last move step time */
                sk->last_move_time = flush_time;
                return 0;
            }
            /* motion executed (no more inactive moves) */
            skip_count = 0;
            /* update end time of last move step time */
            sk->last_move_time = move_end;
            /* update force step time */
            force_steps_time = sk->last_move_time + sk->gen_steps_post_active;
        }
        else
        {
            if (move_start < force_steps_time)
            {
                /* 
                 * Must generates steps just past drive activity. Steps in the time
                 * range between move_start and force_steps_time are generated here.
                 * This ensure proper positioning of the drive after an active move.
                 */
                double abs_end = force_steps_time;
                if (abs_end > flush_time)
                {
                    /* move doesn't need to be flushed completely */
                    abs_end = flush_time;
                }
                int32_t ret = gen_steps_range(sk, m, last_flush_time, abs_end);
                if (ret)
                {
                    return ret;
                }
                /* 
                 * This condition happens only once since in the next iteration it is ensured
                 * that move_start >= force_steps_time, therfore skip_count can be incremented.
                 */
                skip_count = 1;
            }
            else
            {
                /* this move doesn't impact this drive (postpone it) */
                skip_count++;
            }
            /* time elapsed */
            if (flush_time + sk->gen_steps_pre_active <= move_end)
            {
                return 0;
            }
        }
        /* next move */
        m = list_next_entry(m, node);
    }
}

/** check if the given drive is likely to be active in the given time range */
double __visible
ethercatservo_solve_check_active(struct drive_kinematics *sk, double flush_time)
{
    /* check trapezoidal queue */
    if (!sk->tq)
    {
        return 0.;
    }

    /* check trapezoidal queue sentinels */
    trapq_check_sentinels(sk->tq);

    /* get first move in queue */
    struct move *m = list_first_entry(&sk->tq->moves, struct move, node);

    /* skip already flushed moves */
    while (sk->last_flush_time >= m->print_time + m->move_t)
    {
        m = list_next_entry(m, node);
    }

    /* loop over move queue and check for an active move */
    for (;;) 
    {
        if (check_active(sk, m))
        {
            /* stop (active move found) */
            return m->print_time;
        }
        if (flush_time <= m->print_time + m->move_t)
        {
            /* stop )no active actions in time horizon) */
            return 0.;
        }
        /* check next move */
        m = list_next_entry(m, node);
    }
}

/** report if the given stepper is registered for the given axis */
int32_t __visible
ethercatservo_solve_is_active_axis(struct drive_kinematics *sk, char axis)
{
    /* check axis name */
    if (axis < 'x' || axis > 'z')
    {
        return 0;
    }
    /* return updated flag */
    return (sk->active_flags & (AF_X << (axis - 'x'))) != 0;
}

/** set drive_kinematics trapezoidal queue */
void __visible
ethercatservo_solve_set_trapq(struct drive_kinematics *sk, struct trapq *tq)
{
    sk->tq = tq;
}

/** set drive kinematics compressor */
void __visible
ethercatservo_solve_set_ethercatservo_compress(struct drive_kinematics *sk, struct ethercatservo_compress *sc, double simtime)
{
    sk->sc = sc;
    sk->simtime = simtime;
}

/** calculate toolhead position from coordinates */
double __visible
ethercatservo_solve_calc_position_from_coord(struct drive_kinematics *sk,
                                  double x,
                                  double y,
                                  double z)
{
    struct move m;
    memset(&m, 0, sizeof(m));
    m.start_pos.x = x;
    m.start_pos.y = y;
    m.start_pos.z = z;
    m.move_t = 0.;
    return sk->kinematics_cb(&m, 0.).position;
}

/** set toolhead position */
void __visible
ethercatservo_solve_set_position(struct drive_kinematics *sk,
                      double x,
                      double y,
                      double z)
{
    sk->commanded_pos = ethercatservo_solve_calc_position_from_coord(sk, x, y, z);
}

/** get toolhead position (last move sampled) */
double __visible
ethercatservo_solve_get_commanded_pos(struct drive_kinematics *sk)
{
    return sk->commanded_pos;
}
