# Wrapper around C helper code
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging
import cffi


######################################################################
# c_helper.so compiling
######################################################################
# additional libraries
ADDITIONAL_LIBS = "-lethercat" #space + -l{lib_name} in the same string to add a new lib
ADDITIONAL_LIB_PATHS = "-L/usr/local/lib" #space + -L{lib_path} in the same string to add a new lib path
ADDITIONAL_INCLUDES = ['/usr/local/include/ecrt.h', '/usr/local/include/ectty.h'] #add needed library headers in list

GCC_CMD = "gcc"
COMPILE_ARGS = ("-Wall -g -O2 -shared -fPIC"
                " -flto -fwhole-program -fno-use-linker-plugin"
                f" -o %s %s {ADDITIONAL_LIB_PATHS} {ADDITIONAL_LIBS}")
SSE_FLAGS = "-mfpmath=sse -msse2"
SOURCE_FILES = [
    'command.c', 'ethcatqueue.c', 'kin_hash.c', 'pvtcompress.c', 'pvtsolve.c',
    'pyhelper.c', 'serialqueue.c', 'stepcompress.c', 'itersolve.c', 'trapq.c',
    'pollreactor.c', 'msgblock.c', 'trdispatch.c',
    'kin_cartesian.c', 'kin_corexy.c', 'kin_corexz.c', 'kin_delta.c',
    'kin_deltesian.c', 'kin_polar.c', 'kin_rotary_delta.c', 'kin_winch.c',
    'kin_extruder.c', 'kin_shaper.c', 'kin_idex.c',
]
DEST_LIB = "c_helper.so"
OTHER_FILES = [
    'command.h', 'ethcatqueue.h', 'ethercatmsg.h', 'pvtcompress.h',
    'list.h', 'serialqueue.h', 'stepcompress.h', 'itersolve.h', 'pyhelper.h',
    'trapq.h', 'pollreactor.h', 'msgblock.h'
]

defs_pvtcompress = """
    struct pull_history_pvt_steps
    {
        uint64_t first_clock;
        uint64_t last_clock;
        double start_position;
        double velocity;
    };
    struct pvtcompress *pvtcompress_alloc(uint32_t oid);
    void pvtcompress_free(struct pvtcompress *sc);
    uint32_t pvtcompress_get_oid(struct pvtcompress *sc);
    void pvtcompress_append(struct pvtcompress *sc, struct pose *pose, double move_time);
    int pvtcompress_reset(struct pvtcompress *sc, uint64_t last_step_clock);
    int pvtcompress_set_last_position(struct pvtcompress *sc, uint64_t clock, double last_position);
    double pvtcompress_find_past_position(struct pvtcompress *sc, uint64_t clock);
    int pvtcompress_queue_msg(struct pvtcompress *sc, uint32_t *data, int len);
    int pvtcompress_extract_old(struct pvtcompress *sc,
                                struct pull_history_pvt_steps *p,
                                int max,
                                uint64_t start_clock,
                                uint64_t end_clock);
    struct drivesync *drivesync_alloc(struct ethcatqueue *sq,
                                      struct pvtcompress **sc_list,
                                      int sc_num,
                                      int move_num);
    void drivesync_free(struct drivesync *ss);
    void drivesync_set_time(struct drivesync *ss, double time_offset, double mcu_freq);
    int drivesync_flush(struct drivesync *ss, uint64_t move_clock);
"""

defs_pvtsolve = """
    int32_t pvtsolve_generate_steps(struct drive_kinematics *sk, double flush_time);
    double pvtsolve_check_active(struct drive_kinematics *sk, double flush_time);
    int32_t pvtsolve_is_active_axis(struct drive_kinematics *sk, char axis);
    void pvtsolve_set_trapq(struct drive_kinematics *sk, struct trapq *tq);
    void pvtsolve_set_pvtcompress(struct drive_kinematics *sk, struct pvtcompress *sc, double simtime);
    double pvtsolve_calc_position_from_coord(struct drive_kinematics *sk, double x, double y, double z);
    void pvtsolve_set_position(struct drive_kinematics *sk, double x, double y, double z);
    double pvtsolve_get_commanded_pos(struct drive_kinematics *sk);
"""

defs_kin_hash = """
    struct drive_kinematics *hash_drive_alloc(char axis);
"""

defs_ethcatqueue = """
    typedef struct
    {
        uint16_t alias;
        uint16_t position;
        uint32_t vendor_id;
        uint32_t product_code;
        uint16_t index;
        uint8_t subindex;
        unsigned int *offset;
        unsigned int *bit_position;
    } ec_pdo_entry_reg_t;
    typedef struct
    {
        uint16_t index;
        uint8_t subindex;
        uint8_t bit_length;
    } ec_pdo_entry_info_t;
    typedef struct
    {
        uint16_t index;
        unsigned int n_entries;
        ec_pdo_entry_info_t *entries;
    } ec_pdo_info_t;
    void ethcatqueue_slave_config(struct ethcatqueue *sq,
                              uint8_t index,
                              uint16_t alias,
                              uint16_t position,
                              uint32_t vendor_id,
                              uint32_t product_code,
                              uint16_t assign_activate,
                              double sync0_st,
                              double sync1_st);
    void ethcatqueue_slave_config_pdos(struct ethcatqueue *sq,
                                    uint8_t slave_index,
                                    uint8_t sync_index,
                                    uint8_t direction,
                                    uint8_t n_pdo_entries,
                                    ec_pdo_entry_info_t *pdo_entries,
                                    uint8_t n_pdos,
                                    ec_pdo_info_t *pdos);
    void ethcatqueue_master_config_registers(struct ethcatqueue *sq,
                                            uint8_t index,
                                            uint8_t n_registers,
                                            ec_pdo_entry_reg_t *registers);
    struct ethcatqueue *ethcatqueue_alloc(void);
    int ethcatqueue_init(struct ethcatqueue *sq);
    void ethcatqueue_exit(struct ethcatqueue *sq);
    void ethcatqueue_free(struct ethcatqueue *sq);
    struct command_queue *ethcatqueue_alloc_commandqueue(void);
    void ethcatqueue_free_commandqueue(struct command_queue *cq);
    void ethcatqueue_send_command(struct ethcatqueue *sq,
                                  uint8_t *msg,
                                  int len,
                                  uint64_t min_clock,
                                  uint64_t req_clock,
                                  uint64_t notify_id);
    void ethcatqueue_send_batch(struct ethcatqueue *sq,
                                struct command_queue *cq,
                                struct list_head *msgs);
    void ethcatqueue_pull(struct ethcatqueue *sq, struct pull_queue_message *pqm);
    void ethcatqueue_set_wire_frequency(struct ethcatqueue *sq, double frequency);
    void ethcatqueue_set_clock_est(struct ethcatqueue *sq,
                                   double est_freq,
                                   double conv_time,
                                   uint64_t conv_clock,
                                   uint64_t last_clock);
    void ethcatqueue_get_clock_est(struct ethcatqueue *sq, struct clock_estimate *ce);
    void ethcatqueue_get_stats(struct ethcatqueue *sq, char *buf, int len);
"""

defs_stepcompress = """
    struct pull_history_steps {
        uint64_t first_clock, last_clock;
        int64_t start_position;
        int step_count, interval, add;
    };

    struct stepcompress *stepcompress_alloc(uint32_t oid);
    void stepcompress_fill(struct stepcompress *sc, uint32_t max_error
        , int32_t queue_step_msgtag, int32_t set_next_step_dir_msgtag);
    void stepcompress_set_invert_sdir(struct stepcompress *sc
        , uint32_t invert_sdir);
    void stepcompress_free(struct stepcompress *sc);
    int stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock);
    int stepcompress_set_last_position(struct stepcompress *sc
        , uint64_t clock, int64_t last_position);
    int64_t stepcompress_find_past_position(struct stepcompress *sc
        , uint64_t clock);
    int stepcompress_queue_msg(struct stepcompress *sc
        , uint32_t *data, int len);
    int stepcompress_queue_mq_msg(struct stepcompress *sc, uint64_t req_clock
        , uint32_t *data, int len);
    int stepcompress_extract_old(struct stepcompress *sc
        , struct pull_history_steps *p, int max
        , uint64_t start_clock, uint64_t end_clock);

    struct steppersync *steppersync_alloc(struct serialqueue *sq
        , struct stepcompress **sc_list, int sc_num, int move_num);
    void steppersync_free(struct steppersync *ss);
    void steppersync_set_time(struct steppersync *ss
        , double time_offset, double mcu_freq);
    int steppersync_flush(struct steppersync *ss, uint64_t move_clock
        , uint64_t clear_history_clock);
"""

defs_itersolve = """
    int32_t itersolve_generate_steps(struct stepper_kinematics *sk
        , double flush_time);
    double itersolve_check_active(struct stepper_kinematics *sk
        , double flush_time);
    int32_t itersolve_is_active_axis(struct stepper_kinematics *sk, char axis);
    void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);
    void itersolve_set_stepcompress(struct stepper_kinematics *sk
        , struct stepcompress *sc, double step_dist);
    double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
        , double x, double y, double z);
    void itersolve_set_position(struct stepper_kinematics *sk
        , double x, double y, double z);
    double itersolve_get_commanded_pos(struct stepper_kinematics *sk);
"""

defs_trapq = """
    struct pull_move {
        double print_time, move_t;
        double start_v, accel;
        double start_x, start_y, start_z;
        double x_r, y_r, z_r;
    };

    struct trapq *trapq_alloc(void);
    void trapq_free(struct trapq *tq);
    void trapq_append(struct trapq *tq, double print_time
        , double accel_t, double cruise_t, double decel_t
        , double start_pos_x, double start_pos_y, double start_pos_z
        , double axes_r_x, double axes_r_y, double axes_r_z
        , double start_v, double cruise_v, double accel);
    void trapq_finalize_moves(struct trapq *tq, double print_time
        , double clear_history_time);
    void trapq_set_position(struct trapq *tq, double print_time
        , double pos_x, double pos_y, double pos_z);
    int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
        , double start_time, double end_time);
"""

defs_kin_cartesian = """
    struct stepper_kinematics *cartesian_stepper_alloc(char axis);
"""

defs_kin_corexy = """
    struct stepper_kinematics *corexy_stepper_alloc(char type);
"""

defs_kin_corexz = """
    struct stepper_kinematics *corexz_stepper_alloc(char type);
"""

defs_kin_delta = """
    struct stepper_kinematics *delta_stepper_alloc(double arm2
        , double tower_x, double tower_y);
"""

defs_kin_deltesian = """
    struct stepper_kinematics *deltesian_stepper_alloc(double arm2
        , double arm_x);
"""

defs_kin_polar = """
    struct stepper_kinematics *polar_stepper_alloc(char type);
"""

defs_kin_rotary_delta = """
    struct stepper_kinematics *rotary_delta_stepper_alloc(
        double shoulder_radius, double shoulder_height
        , double angle, double upper_arm, double lower_arm);
"""

defs_kin_winch = """
    struct stepper_kinematics *winch_stepper_alloc(double anchor_x
        , double anchor_y, double anchor_z);
"""

defs_kin_extruder = """
    struct stepper_kinematics *extruder_stepper_alloc(void);
    void extruder_set_pressure_advance(struct stepper_kinematics *sk
        , double pressure_advance, double smooth_time);
"""

defs_kin_shaper = """
    double input_shaper_get_step_generation_window(
        struct stepper_kinematics *sk);
    int input_shaper_set_shaper_params(struct stepper_kinematics *sk, char axis
        , int n, double a[], double t[]);
    int input_shaper_set_sk(struct stepper_kinematics *sk
        , struct stepper_kinematics *orig_sk);
    struct stepper_kinematics * input_shaper_alloc(void);
"""

defs_kin_idex = """
    void dual_carriage_set_sk(struct stepper_kinematics *sk
        , struct stepper_kinematics *orig_sk);
    int dual_carriage_set_transform(struct stepper_kinematics *sk
        , char axis, double scale, double offs);
    struct stepper_kinematics * dual_carriage_alloc(void);
"""

defs_serialqueue = """
    #define MESSAGE_MAX 64
    struct pull_queue_message {
        uint8_t msg[MESSAGE_MAX];
        int len;
        double sent_time, receive_time;
        uint64_t notify_id;
    };

    struct serialqueue *serialqueue_alloc(int serial_fd, char serial_fd_type
        , int client_id);
    void serialqueue_exit(struct serialqueue *sq);
    void serialqueue_free(struct serialqueue *sq);
    struct command_queue *serialqueue_alloc_commandqueue(void);
    void serialqueue_free_commandqueue(struct command_queue *cq);
    void serialqueue_send(struct serialqueue *sq, struct command_queue *cq
        , uint8_t *msg, int len, uint64_t min_clock, uint64_t req_clock
        , uint64_t notify_id);
    void serialqueue_pull(struct serialqueue *sq
        , struct pull_queue_message *pqm);
    void serialqueue_set_wire_frequency(struct serialqueue *sq
        , double frequency);
    void serialqueue_set_receive_window(struct serialqueue *sq
        , int receive_window);
    void serialqueue_set_clock_est(struct serialqueue *sq, double est_freq
        , double conv_time, uint64_t conv_clock, uint64_t last_clock);
    void serialqueue_get_stats(struct serialqueue *sq, char *buf, int len);
    int serialqueue_extract_old(struct serialqueue *sq, int sentq
        , struct pull_queue_message *q, int max);
"""

defs_trdispatch = """
    void trdispatch_start(struct trdispatch *td, uint32_t dispatch_reason);
    void trdispatch_stop(struct trdispatch *td);
    struct trdispatch *trdispatch_alloc(void);
    struct trdispatch_mcu *trdispatch_mcu_alloc(struct trdispatch *td
        , struct serialqueue *sq, struct command_queue *cq, uint32_t trsync_oid
        , uint32_t set_timeout_msgtag, uint32_t trigger_msgtag
        , uint32_t state_msgtag);
    void trdispatch_mcu_setup(struct trdispatch_mcu *tdm
        , uint64_t last_status_clock, uint64_t expire_clock
        , uint64_t expire_ticks, uint64_t min_extend_ticks);
"""

defs_pyhelper = """
    void set_python_logging_callback(void (*func)(const char *));
    double get_monotonic(void);
"""

defs_std = """
    void free(void*);
"""

defs_all = [
    defs_pvtcompress, defs_pvtsolve, defs_kin_hash, defs_ethcatqueue,
    defs_pyhelper, defs_serialqueue, defs_std, defs_stepcompress,
    defs_itersolve, defs_trapq, defs_trdispatch,
    defs_kin_cartesian, defs_kin_corexy, defs_kin_corexz, defs_kin_delta,
    defs_kin_deltesian, defs_kin_polar, defs_kin_rotary_delta, defs_kin_winch,
    defs_kin_extruder, defs_kin_shaper, defs_kin_idex,
]

# Update filenames to an absolute path
def get_abs_files(srcdir, filelist):
    return [os.path.join(srcdir, fname) for fname in filelist]

# Return the list of file modification times
def get_mtimes(filelist):
    out = []
    for filename in filelist:
        try:
            t = os.path.getmtime(filename)
        except os.error:
            continue
        out.append(t)
    return out

# Check if the code needs to be compiled
def check_build_code(sources, target):
    src_times = get_mtimes(sources)
    obj_times = get_mtimes([target])
    return not obj_times or max(src_times) > min(obj_times)

# Check if the current gcc version supports a particular command-line option
def check_gcc_option(option):
    cmd = "%s %s -S -o /dev/null -xc /dev/null > /dev/null 2>&1" % (
        GCC_CMD, option)
    res = os.system(cmd)
    return res == 0

# Check if the current gcc version supports a particular command-line option
def do_build_code(cmd):
    res = os.system(cmd)
    if res:
        msg = "Unable to build C code module (error=%s)" % (res,)
        logging.error(msg)
        raise Exception(msg)

FFI_main = None
FFI_lib = None
pyhelper_logging_callback = None

# Hepler invoked from C errorf() code to log errors
def logging_callback(msg):
    logging.error(FFI_main.string(msg))

# Return the Foreign Function Interface api to the caller
def get_ffi():
    global FFI_main, FFI_lib, pyhelper_logging_callback
    if FFI_lib is None:
        srcdir = os.path.dirname(os.path.realpath(__file__))
        srcfiles = get_abs_files(srcdir, SOURCE_FILES)
        ofiles = get_abs_files(srcdir, OTHER_FILES)
        ofiles += ADDITIONAL_INCLUDES
        destlib = get_abs_files(srcdir, [DEST_LIB])[0]

        if check_build_code(srcfiles+ofiles+[__file__], destlib):
            if check_gcc_option(SSE_FLAGS):
                cmd = "%s %s %s" % (GCC_CMD, SSE_FLAGS, COMPILE_ARGS)
            else:
                cmd = "%s %s" % (GCC_CMD, COMPILE_ARGS)
            logging.info("Building C code module %s", DEST_LIB)
            do_build_code(cmd % (destlib, ' '.join(srcfiles)))
        FFI_main = cffi.FFI()

        for d in defs_all:
            '''
            Add all definitions that can be used directly from python.
            '''
            try:
                FFI_main.cdef(d)
            except Exception as e:
                raise

        try:
            FFI_lib = FFI_main.dlopen(destlib)
        except Exception as e:
            raise
            
        # Setup error logging
        pyhelper_logging_callback = FFI_main.callback("void func(const char *)", logging_callback)
        FFI_lib.set_python_logging_callback(pyhelper_logging_callback)
    return FFI_main, FFI_lib


######################################################################
# hub-ctrl hub power controller
######################################################################

HC_COMPILE_CMD = "gcc -Wall -g -O2 -o %s %s -lusb"
HC_SOURCE_FILES = ['hub-ctrl.c']
HC_SOURCE_DIR = '../../lib/hub-ctrl'
HC_TARGET = "hub-ctrl"
HC_CMD = "sudo %s/hub-ctrl -h 0 -P 2 -p %d"

def run_hub_ctrl(enable_power):
    srcdir = os.path.dirname(os.path.realpath(__file__))
    hubdir = os.path.join(srcdir, HC_SOURCE_DIR)
    srcfiles = get_abs_files(hubdir, HC_SOURCE_FILES)
    destlib = get_abs_files(hubdir, [HC_TARGET])[0]
    if check_build_code(srcfiles, destlib):
        logging.info("Building C code module %s", HC_TARGET)
        do_build_code(HC_COMPILE_CMD % (destlib, ' '.join(srcfiles)))
    os.system(HC_CMD % (hubdir, enable_power))


if __name__ == '__main__':
    get_ffi()
