/**
 * \file pollreactor.h
 *
 * \brief Code for dispatching timer and file descriptor events.
 * 
 */

#ifndef POLLREACTOR_H
#define POLLREACTOR_H

/****************************************************************
 * Includes
 ****************************************************************/


/****************************************************************
 * Defines
 ****************************************************************/
#define PR_NOW   0.
#define PR_NEVER 9999999999999999.


/****************************************************************
 * Public Functions
 ****************************************************************/
/** allocate a pollreactor object */
struct pollreactor *pollreactor_alloc(int num_fds, int num_timers, void *callback_data,
                                      double tmin, double tmax, double offset);

/** free resources associated with a pollreactor object */
void pollreactor_free(struct pollreactor *pr);

/** add a callback for when a file descriptor (fd) becomes readable */
void pollreactor_add_fd(struct pollreactor *pr, int pos, int fd, void *callback, int write_only);

/** add a timer callback */
void pollreactor_add_timer(struct pollreactor *pr, int pos, void *callback);

/** return the last schedule wake-up time for a timer */
double pollreactor_get_timer(struct pollreactor *pr, int pos);

/** set the wake-up time for a given timer */
void pollreactor_update_timer(struct pollreactor *pr, int pos, double waketime);

/** repeatedly check for timer and fd events and invoke their callbacks */
void pollreactor_run(struct pollreactor *pr);

/** request that a currently running pollreactor loop exit */
void pollreactor_do_exit(struct pollreactor *pr);

/** check if a pollreactor loop has been requested to exit */
int pollreactor_is_exit(struct pollreactor *pr);

/** set file descriptor in non blocking mode */
int fd_set_non_blocking(int fd);

#endif // pollreactor.h
