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
    COE_OFFSET_MOVE_SEGMENT,
    COE_OFFSET_BUFFER_FREE_COUNT,
    COE_OFFSET_BUFFER_STATUS,
    COE_OFFSET_CONTROL_WORD,
    COE_OFFSET_STATUS_WORD,
    COE_OFFSET_MODE_OF_OPERATION,
    COE_OFFSET_POSITION_ACTUAL,
    COE_OFFSET_VELOCITY_ACTUAL,
    COE_OFFSET_MAX
};

/* copley drives segment interpolation mode */
enum
{
    COE_SEGMENT_LINEAR_INTERPOLATION = 0,
    COE_SEGMENT_LINEAR_INTERPOLATION_CT = -1,
    COE_SEGMENT_LINEAR_INTERPOLATION_VT = -2,
    COE_SEGMENT_CUBIC_INTERPOLATION = -3
};

/* copley drives operation mode */
enum
{
    COE_OPERATION_MODE_POSITION = 1,
    COE_OPERATION_MODE_VELOCITY = 3,
    COE_OPERATION_MODE_TORQUE = 4,
    COE_OPERATION_MODE_HOMING = 6,
    COE_OPERATION_MODE_INTERPOLATION = 7,
    COE_OPERATION_MODE_SYNC_POSITION = 8,
    COE_OPERATION_MODE_SYNC_VELOCITY = 9,
    COE_OPERATION_MODE_SYNC_TORQUE = 10,
    COE_OPERATION_MODE_SYNC_ANGLE = 11,
};

/* copley drives ip segment mode */
enum
{
    COE_SEGMENT_MODE_BUFFER, //add new sample to buffer
    COE_SEGMENT_MODE_CMD     //buffer command
};

/* copley drives buffer commands */
enum
{
    COE_CMD_CLEAR_BUFFER,
    COE_CMD_POP_SEGMENTS,
    COE_CMD_CLEAR_ERRORS,
    COE_CMD_RESET_SEGMENT_ID,
    COE_CMD_NO_OPERATION
};

/* copley drives sdo */
#define COE_SDO_INTERPOLATION_MODE(axis) 0x60C0 + axis*0x800, 0, 2


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
} __attribute((packed));

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
} __attribute((packed));

/* interpolation move command */
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
} __attribute((packed));

#endif // copley.h
