/**
 * \file copley.h
 *
 * \brief Copley drives specific structures and definitions.
 */

#ifndef COPLEY_H
#define COPLEY_H

/****************************************************************
 * Defines
 ****************************************************************/
/* ethercat slave offsets register position */
enum
{
    ETHERCAT_OFFSET_MOVE_SEGMENT,
    ETHERCAT_OFFSET_BUFFER_FREE_COUNT,
    ETHERCAT_OFFSET_BUFFER_STATUS,
    ETHERCAT_OFFSET_CONTROL_WORD,
    ETHERCAT_OFFSET_STATUS_WORD,
    ETHERCAT_OFFSET_MODE_OF_OPERATION,
    ETHERCAT_OFFSET_POSITION_ACTUAL,
    ETHERCAT_OFFSET_VELOCITY_ACTUAL,
    ETHERCAT_OFFSET_MAX
};

/* copley drives operation mode */
enum
{
    COPLEY_OPERATION_MODE_POSITION = 1,
    COLPEY_OPERATION_MODE_VELOCITY = 3,
    COLPEY_OPERATION_MODE_TORQUE = 4,
    COLPEY_OPERATION_MODE_HOMING = 6,
    COLPEY_OPERATION_MODE_INTERPOLATION = 7,
    COLPEY_OPERATION_MODE_SYNC_POSITION = 8,
    COLPEY_OPERATION_MODE_SYNC_VELOCITY = 9,
    COLPEY_OPERATION_MODE_SYNC_TORQUE = 10,
    COLPEY_OPERATION_MODE_SYNC_ANGLE = 11,
};

/* copley drives ip segment mode */
enum
{
    COPLEY_MODE_BUFFER, //add new sample to buffer
    COPLEY_MODE_CMD     //buffer command
};

/* copley drives buffer commands */
enum
{
    COPLEY_CMD_CLEAR_BUFFER,
    COPLEY_CMD_POP_SEGMENTS,
    COPLEY_CMD_CLEAR_ERRORS,
    COPLEY_CMD_RESET_SEGMENT_ID,
    COPLEY_CMD_NO_OPERATION
};


/****************************************************************
 * CANopen data types
 ****************************************************************/
/* control word */
struct coe_control_word
{
    uint8_t power_switch:1;
    uint8_t voltage_switch:1;
    uint8_t quick_stop:1;
    uint8_t enable_operation:1;
    uint8_t operation_mode:3; //operation mode specific
    uint8_t reset_fault:1;
    uint8_t halt:1;
    uint8_t padding:7;
} __attribute((aligned(1), packed));

/* status word */
struct coe_status_word
{
    uint8_t switch_ready:1;
    uint8_t switch_on:1;
    uint8_t operation_enabled:1;
    uint8_t fault:1;
    uint8_t voltage_enabled:1;
    uint8_t quick_stop:1;
    uint8_t switch_diabled:1;
    uint8_t warning:1;
    uint8_t aborted:1;
    uint8_t remote:1;
    uint8_t target_reached:1;
    uint8_t limit_active:1;
    uint8_t homing_attained:1;
    uint8_t generic_error:1;
    uint8_t moving:1;
    uint8_t homed:1;
} __attribute((aligned(1), packed));

/* interpolation move commandon*/
struct coe_ip_move
{
    /* common header (keep duplicate header) */
    union
    {
        /* used in buffer mode */
        struct
        {
            uint8_t seq_num:3; //segment sequence number (increases always)
            uint8_t format:4; //buffer format code (0 or 1)
            uint8_t type:1; //always zero, identifies buffer segment
        } header;
        /* used in command mode */
        struct
        {
            uint8_t code:7; //command code
            uint8_t type:1; //always one, identifies drive command
        } command;
    };
    uint8_t time; //time in ms
    int32_t position:24; //current position
    int32_t velocity:24; //current speed
} __attribute((aligned(1), packed));

#endif // copley.h
