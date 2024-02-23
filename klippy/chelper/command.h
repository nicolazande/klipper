#ifndef __COMMAND_H
#define __COMMAND_H

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdarg.h> // va_list
#include <stddef.h>
#include <stdint.h> // uint8_t
#include "ethercatqueue.h" //struct drivemsg


/****************************************************************
 * Defines
 ****************************************************************/
/* message format */
#define MESSAGE_MIN 5
#define MESSAGE_MAX 64
#define MESSAGE_HEADER_SIZE  2
#define MESSAGE_TRAILER_SIZE 3
#define MESSAGE_POS_LEN 0
#define MESSAGE_POS_SEQ 1
#define MESSAGE_TRAILER_CRC  3
#define MESSAGE_TRAILER_SYNC 1
#define MESSAGE_PAYLOAD_MAX (MESSAGE_MAX - MESSAGE_MIN)
#define MESSAGE_SEQ_MASK 0x0f
#define MESSAGE_DEST 0x10
#define MESSAGE_SYNC 0x7E
#define HF_IN_SHUTDOWN   0x01   //handler can run even when in emergency stop


/****************************************************************
 * Custom data types
 ****************************************************************/
/* command parser ids */
enum 
{
    ETH_DEFAULT_CP,
    ETH_RESET_STEP_CLOCK_CP,
    ETH_STEPPER_GET_POSITION_CP,
    ETH_ENDSTOP_HOME_CP,
    ETH_ENDSTOP_QUERY_STATE_CP,
    ETH_STEPPER_STOP_ON_TRIGGER_CP,
    ETH_MAX_CP
};

/* command encoder ids */
enum 
{
    ETH_DEFAULT_CE,
    ETH_STEPPER_POSITION_CE,
    ETH_ENDSTOP_STATE_CE,
    ETH_MAX_CE
};

/* command supported data types */
enum 
{
    PT_uint32,
    PT_int32, 
    PT_uint16, 
    PT_int16, 
    PT_byte,
    PT_string, 
    PT_progmem_buffer, 
    PT_buffer,
};

/* response command encoder */
struct command_encoder
{
    uint8_t msg_id; //response id
    uint8_t max_size; //max command or response data size in bytes
    uint8_t num_params; //list of parameter types (external)
    const uint8_t *param_types;
};

/* command parser */
struct ethercatqueue;
struct command_parser
{
    uint8_t msg_id; //command id
    uint8_t num_args; //number of arguments (function parametes + buffers)
    uint8_t flags; //command flags
    uint8_t num_params; //number of function parameters
    const uint8_t *param_types; //list of parameter types (external)
    int (*func)(struct ethercatqueue *, void *, uint32_t *); //command callback
};


/****************************************************************
 * Public functions
 ****************************************************************/
/* parse an incoming command into args */
uint8_t *command_parsef(uint8_t *p, uint8_t *maxend, const struct command_parser *cp, uint32_t *args);

/* check message */
int8_t check_command(uint8_t *buf, uint8_t buf_len, uint8_t *pop_count);

/* encode a response message buffer (used directly by callbacks) */
uint8_t command_encode_and_frame(uint8_t *buf, const struct command_encoder *ce, ...);

#endif // command.h
