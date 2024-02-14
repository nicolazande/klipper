/**
 * \file command.c
 *
 * \brief Internal protocol for communication between high and low
 *        level ethercat threads.
 */

/****************************************************************
 * Includes
 ****************************************************************/
#include <stdarg.h> // va_start
#include <string.h> // memcpy
#include "command.h" // output_P


/****************************************************************
 * Defines
 ****************************************************************/
/* message operation */
enum
{ 
    CF_NEED_SYNC  = 1<<0, 
    CF_NEED_VALID = 1<<1 
};


/****************************************************************
 * Data
 ****************************************************************/
/* message sequence number */
static uint8_t next_sequence = MESSAGE_DEST;
extern struct command_parser *command_parser_table[ETH_MAX_CP];
extern struct command_encoder *command_encoder_table[ETH_MAX_CE]; 


/****************************************************************
 * Private function prototypes
 ****************************************************************/
/* encode an integer as a variable length quantity (vlq) */
static uint8_t *encode_int(uint8_t *p, uint32_t v);

/* parse a vlq integer */
static uint32_t parse_int(uint8_t **pp);

/* encode a response message */
static uint8_t command_encodef(uint8_t *buf, const struct command_encoder *ce, va_list args);

/* add header and trailer bytes to a response message */
static void command_add_frame(uint8_t *buf, uint8_t msglen);


/****************************************************************
 * Private functions
 ****************************************************************/
/* encode an integer as a variable length quantity (vlq) */
static uint8_t *
encode_int(uint8_t *p, uint32_t v)
{
    /* unsigned to signed integer for range checking */
    int32_t sv = v;

    if (sv < (3L<<5)  && sv >= -(1L<<5))
    {
        /* single byte encoding */
        goto f4;
    }
    if (sv < (3L<<12) && sv >= -(1L<<12))
    {
        /* two bytes encoding */
        goto f3;
    }
    if (sv < (3L<<19) && sv >= -(1L<<19))
    {
        /* three bytes encoding */
        goto f2;
    }
    if (sv < (3L<<26) && sv >= -(1L<<26))
    {
        /* four bytes encoding */
        goto f1;
    }

    *p++ = (v>>28) | 0x80;          //more than four bytes
f1: *p++ = ((v>>21) & 0x7f) | 0x80; //four bytes
f2: *p++ = ((v>>14) & 0x7f) | 0x80; //three bytes
f3: *p++ = ((v>>7) & 0x7f) | 0x80;  //two bytes
f4: *p++ = v & 0x7f;                //one byte
    return p;
}

/* parse a vlq integer */
static uint32_t
parse_int(uint8_t **pp)
{
    uint8_t *p = *pp;
    uint8_t c = *p++;

    /* extract data */
    uint32_t v = c & 0x7f;

    /* check if more bytes are needed */
    if ((c & 0x60) == 0x60)
    {
        /* sign extension */
        v |= -0x20;
    }
    /* loop for additional bytes */
    while (c & 0x80)
    {
        /* get next byte */
        c = *p++;
        /* get byte value and update total value */
        v = (v<<7) | (c & 0x7f);
    }

    /* update data pointer */
    *pp = p;

    return v;
}

/* encode a response message */
static uint8_t
command_encodef(uint8_t *buf, const struct command_encoder *ce, va_list args)
{
    /* get message max size in bytes */
    uint8_t max_size = ce->max_size;
    if (max_size <= MESSAGE_MIN)
    {
        return max_size;
    }

    /* get first byte in command */
    uint8_t *p = &buf[MESSAGE_HEADER_SIZE];
    uint8_t *maxend = &p[max_size - MESSAGE_MIN];
    
    /* get number and type of parameters */
    uint8_t num_params = ce->num_params;
    const uint8_t *param_types = ce->param_types;
    
    /* store message id */
    *p++ = ce->msg_id;

    /* loop through parameters and encode it */
    while (num_params--)
    {
        if (p > maxend)
        {
            /* beyond maximum allowed position */
            return 0;
        }

        /* get parameter type */
        uint8_t t = *param_types;
        param_types++;
        uint32_t v;

        /* handle different parameter types */
        switch (t)
        {
            /* integer types */
            case PT_uint32:
            case PT_int32:
            case PT_uint16:
            case PT_int16:
            case PT_byte:
            {
                /* check data tybe base on cpu architecture */
                if ((sizeof(v) > sizeof(int)) && (t >= PT_uint16))
                {
                    /* never happens on 64 bit processor */
                    if (t == PT_int16)
                    {
                        v = (int32_t)va_arg(args, int);
                    }
                    else
                    {
                        v = va_arg(args, unsigned int);
                    }
                }
                else
                {
                    v = va_arg(args, uint32_t);
                }
                /* encode integer into message buffer */
                p = encode_int(p, v);
                break;
            }
            /* string */
            case PT_string:
            {
                uint8_t *s = va_arg(args, uint8_t*);
                uint8_t *lenp = p++;
                while (*s && p < maxend)
                {
                    /* copy string bytes */
                    *p++ = *s++;
                }
                /* move forward */
                *lenp = p-lenp-1;
                break;
            }
            /* buffer */
            case PT_progmem_buffer:
            case PT_buffer:
            {
                /* get buffer size */
                v = va_arg(args, int);
                if (v > maxend - p)
                {
                    v = maxend - p;
                }
                /* store buffer size */
                *p++ = v;
                /* get buffer pointer */
                uint8_t *s = va_arg(args, uint8_t*);
                /* copy data */
                memcpy(p, s, v);
                /* move forward */
                p += v;
                break;
            }
            default:
            {
                /* erorr */
                goto error;
            }
        }
    }
    /* move forward */
    return p - buf + MESSAGE_TRAILER_SIZE;

error:
    errorf("Message encode error");
    return 0;
}

/* add header and trailer bytes to a response message block */
static void
command_add_frame(uint8_t *buf, uint8_t msglen)
{
    buf[MESSAGE_POS_LEN] = msglen;
    buf[MESSAGE_POS_SEQ] = next_sequence;
    //uint16_t crc = crc16_ccitt(buf, msglen - MESSAGE_TRAILER_SIZE);
    uint16_t crc = 0;
    buf[msglen - MESSAGE_TRAILER_CRC + 0] = crc >> 8;
    buf[msglen - MESSAGE_TRAILER_CRC + 1] = crc;
    buf[msglen - MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;
}


/****************************************************************
 * Public functions
 ****************************************************************/
/* parse an incoming command into args */
uint8_t *
command_parsef(uint8_t *p, uint8_t *maxend, const struct command_parser *cp, uint32_t *args)
{
    /* get number of parameters and parameter types */
    uint8_t num_params = cp->num_params;
    const uint8_t *param_types = cp->param_types;

    /* loop over parameter */
    while (num_params--)
    {
        if (p > maxend)
        {
            /* beyond maximum allowed position */
            return NULL;
        }

        /* parameter type */
        uint8_t t = *param_types;
        param_types++;

        /* get data type */
        switch (t)
        {
            /* integer data types (all stored in four bytes) */
            case PT_uint32:
            case PT_int32:
            case PT_uint16:
            case PT_int16:
            case PT_byte:
            {
                /* store argument */
                *args++ = parse_int(&p);
                break;
            }
            /* buffer data type */
            case PT_buffer:
            {
                /* read buffer size */
                uint8_t len = *p++;
                if (p + len > maxend)
                {
                    /* beyond maximum allowed position */
                    return NULL;
                }
                *args++ = len; //store buffer size (duplicated but needed for buffer memcpy)
                *args++ = (size_t)p; //store buffer initial address
                p += len; //move forward
                break;
            }
            default:
            {
                /* unknown command */
                return NULL;
            }
        }
    }
    /* updated buffer position */
    return p;
}

/* check command or response format */
int8_t
check_command(uint8_t *buf, uint8_t buf_len, uint8_t *pop_count)
{
    /* data */
    static uint8_t sync_state;

    /* check if synchronization is needed and there is data in the buffer */
    if ((buf_len && sync_state) & CF_NEED_SYNC)
    {
        goto need_sync;
    }

    /* check if enough data for a complete message */
    if (buf_len < MESSAGE_MIN)
    {
        goto need_more_data;
    }

    /* extract message length from the buffer */
    uint8_t msglen = buf[MESSAGE_POS_LEN];
    if ((msglen < MESSAGE_MIN) || (msglen > MESSAGE_MAX))
    {
        goto error;
    }

    /* extract message sequence number from the buffer */
    uint8_t msgseq = buf[MESSAGE_POS_SEQ];
    if ((msgseq & ~MESSAGE_SEQ_MASK) != MESSAGE_DEST)
    {
        goto error;
    }

    /* check if enough data for the complete message */
    if (buf_len < msglen)
    {
        goto need_more_data;
    }

    /* check sync byte at the end of message */
    if (buf[msglen-MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC)
    {
        goto error;
    }

    /* extract and calculate crc for the message */
    uint16_t msgcrc = ((buf[msglen-MESSAGE_TRAILER_CRC] << 8) | buf[msglen-MESSAGE_TRAILER_CRC+1]);
    //uint16_t crc = crc16_ccitt(buf, msglen-MESSAGE_TRAILER_SIZE);
    uint16_t crc = 0;
    if (crc != msgcrc)
    {
        goto error;
    }

    /* update synchronization state */
    sync_state &= ~CF_NEED_VALID;

    /* return real (sent or received) message length */
    *pop_count = msglen;

    /* check sequence number */
    if (msgseq != next_sequence)
    {
        /* lost message (discard messages until it is retransmitted) */
        goto nak;
    }

    /* update sequence number */
    next_sequence = ((msgseq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
    return 1;

need_more_data:
    /* discard all data */
    *pop_count = 0;
    return 0;

error:
    if (buf[0] == MESSAGE_SYNC) 
    {
        /* ignore (do not nak) leading SYNC bytes */
        *pop_count = 1;
        return -1;
    }
    sync_state |= CF_NEED_SYNC;

need_sync: ;
    /* discard bytes until next sync found */
    uint8_t *next_sync = memchr(buf, MESSAGE_SYNC, buf_len);
    if (next_sync)
    {
        /* update synchronization state */
        sync_state &= ~CF_NEED_SYNC;
        /* return the count of bytes to discard */
        *pop_count = next_sync - buf + 1;
    } 
    else
    {
        /* no sync found (discard the entire buffer) */
        *pop_count = buf_len;
    }
    /* check if synchronization is needed */
    if (sync_state & CF_NEED_VALID)
    {
        return -1;
    }
    /* set need for validation */
    sync_state |= CF_NEED_VALID;

nak:
    return -1;
}

/* encode a response message buffer */
uint8_t
command_encode_and_frame(uint8_t *buf, const struct command_encoder *ce, ...)
{
    /* response parameters */
    va_list args;
    va_start(args, ce);
    /* encode message (based on arguments and command parser) */
    uint8_t msglen = command_encodef(buf, ce, args);
    va_end(args);
    /* add header and trailer to message */
    command_add_frame(buf, msglen);
    return msglen;
}


/****************************************************************
 * Command parser list
 ****************************************************************/
/* default command parser */
static int cp_f_default(struct ethercatqueue *sq, void *out, uint32_t *args)
{
    return 0;
}
static int cp_p_default[0];
static struct command_parser cp_default = 
{
    .msg_id = ETH_DEFAULT_CP,
    .num_args = 0,
    .flags = 0,
    .num_params = 0,
    .param_types = (const uint8_t *)&cp_p_default,
    .func = &cp_f_default,
};

/* reset step clock command parser */
static int cp_f_command_reset_step_clock(struct ethercatqueue *sq, void *out, uint32_t *args)
{
    struct stepper *s = NULL;
    /* get drive oid and target buffer */
    uint8_t oid = args[0];
    uint32_t waketime = args[1];
    /** TODO: add logic for next step time, it should use adapted
     *        copley firmware and send a segment representing the
     *        absolute time of the following step.
     */
    return 0;
}
static int cp_p_reset_step_clock[2] = {PT_byte, PT_uint32}; //oid, clock
static struct command_parser cp_reset_step_clock = 
{
    .msg_id = ETH_RESET_STEP_CLOCK_CP,
    .num_args = 2,
    .flags = 0,
    .num_params = 2,
    .param_types = (const uint8_t *)&cp_p_reset_step_clock,
    .func = &cp_f_command_reset_step_clock,
};

/** stepper get position command parser */
static int cp_f_stepper_get_position(struct ethercatqueue *sq, void *out, uint32_t *args)
{
    /* get drive oid and target buffer */
    uint8_t oid = args[0];
    uint8_t *buf = (uint8_t *)out;

    /* check oid */
    if (oid < ETHERCAT_DRIVES)
    {
        /* get slave */
        struct slavemonitor *slave = &sq->masterifc.monitor[oid];
        /* get actual position */
        int32_t position = slave->position_actual;
        /* get response command parser */
        struct command_encoder *ce = command_encoder_table[ETH_STEPPER_POSITION_CE];
        /* create response  */
        uint8_t msglen = command_encode_and_frame(buf, ce, oid, position);
    }    
    return 0;
}
static int cp_p_stepper_get_position[1] = {PT_byte}; //oid
static struct command_parser cp_stepper_get_position = 
{
    .msg_id = ETH_STEPPER_GET_POSITION_CP,
    .num_args = 1,
    .flags = 0,
    .num_params = 1,
    .param_types = (const uint8_t *)&cp_p_stepper_get_position,
    .func = &cp_f_stepper_get_position,
};

/** endstop home command parser */
static int cp_f_endstop_home(struct ethercatqueue *sq, void *out, uint32_t *args)
{
    /* get endstop oid (corresponds to drive oid) */
    uint8_t oid = args[0];
    uint8_t *buf = (uint8_t *)out;
    
    /* check oid */
    if (oid < ETHERCAT_DRIVES)
    {
        /* get slave */
        struct slavemonitor *slave = &sq->masterifc.monitor[oid];
        /* control word */
        struct coe_control_word *cw = (struct coe_control_word *)slave->off_control_word;

        /* set homing mode */
        if (slave->off_operation_mode)
        {
            /* operation mode in frame */
            *slave->off_operation_mode = COLPEY_OPERATION_MODE_HOMING;
            /* local copy of operation mode */
            slave->operation_mode = COLPEY_OPERATION_MODE_HOMING;
            /* start homing */
            cw->enable_operation = 1;
        }
    }
    return 0;
}
static int cp_p_endstop_home[1] = {PT_byte}; //oid
static struct command_parser cp_endstop_home = 
{
    .msg_id = ETH_ENDSTOP_HOME_CP,
    .num_args = 1,
    .flags = 0,
    .num_params = 1,
    .param_types = (const uint8_t *)&cp_p_endstop_home,
    .func = &cp_f_endstop_home,
};

/** endstop query state command parser */
static int cp_f_endstop_query_state(struct ethercatqueue *sq, void *out, uint32_t *args)
{
    /* get endstop oid (corresponds to drive oid) */
    uint8_t oid = args[0];
    uint8_t *buf = (uint8_t *)out;
    
    /* check oid */
    if (oid < ETHERCAT_DRIVES)
    {
        /* get slave */
        struct slavemonitor *slave = &sq->masterifc.monitor[oid];
        /* control word */
        struct coe_control_word *cw = (struct coe_control_word *)slave->off_control_word;
        /* status word */
        struct coe_status_word *sw = (struct coe_status_word *)slave->off_status_word;
        /* check data */
        if (cw && sw)
        {
            /* get data */
            int8_t homing = (cw->operation_mode == COLPEY_OPERATION_MODE_HOMING);
            int8_t finished = sw->homing_attained; //homed
            uint32_t next_clock = sq->last_clock; //current input event clock
            /* get command encoder */
            struct command_encoder *ce = command_encoder_table[ETH_ENDSTOP_STATE_CE];
            /* create response  */
            uint8_t msglen = command_encode_and_frame(buf, ce, oid, homing, finished, next_clock);
            /* check if homing finished */
            if (finished)
            {
                /* reset standard interpolation mode */
                cw->operation_mode = COLPEY_OPERATION_MODE_INTERPOLATION;
            }
        }
    }
    return 0;
}
static int cp_p_endstop_query_state[1] = {PT_byte}; //oid
static struct command_parser cp_endstop_query_state = 
{
    .msg_id = ETH_ENDSTOP_QUERY_STATE_CP,
    .num_args = 1,
    .flags = 0,
    .num_params = 1,
    .param_types = (const uint8_t *)&cp_p_endstop_query_state,
    .func = &cp_f_endstop_query_state,
};

/** stepper stop on trigger command parser */
static int cp_f_stepper_stop_on_trigger(struct ethercatqueue *sq, void *out, uint32_t *args)
{
    /* get endstop oid (corresponds to drive oid) */
    uint8_t oid = args[0];
    uint8_t *buf = (uint8_t *)out;
    
    /* check oid */
    if (oid < ETHERCAT_DRIVES)
    {
        /* get slave */
        struct slavemonitor *slave = &sq->masterifc.monitor[oid];
        /* control word */
        struct coe_control_word *cw = (struct coe_control_word *)slave->off_control_word;
        /** hard stop (NOTE: maybe sustitute with soft one) */
        if (cw)
        {
            cw->enable_operation = 0;
        }
    }

    return 0;
}
static int cp_p_stepper_stop_on_trigger[1] = {PT_byte}; //oid
static struct command_parser cp_stepper_stop_on_trigger = 
{
    .msg_id = ETH_STEPPER_STOP_ON_TRIGGER_CP,
    .num_args = 1,
    .flags = 0,
    .num_params = 1,
    .param_types = (const uint8_t *)&cp_p_stepper_stop_on_trigger,
    .func = &cp_f_stepper_stop_on_trigger,
};


/****************************************************************
 * Command encoder list
 ****************************************************************/
/* default command encoder */
static int ce_p_default[0];
static struct command_encoder ce_default = 
{
    .msg_id = ETH_DEFAULT_CE + ETH_MAX_CP,
    .max_size = 0,
    .num_params = 0,
    .param_types = (const uint8_t *)&ce_p_default,
};

/* stepper position command encoder */
static int ce_p_stepper_position[2] = {PT_byte, PT_int32}; //oid, position
static struct command_encoder ce_stepper_position = 
{
    .msg_id = ETH_STEPPER_POSITION_CE + ETH_MAX_CP,
    .max_size = MESSAGE_MAX-MESSAGE_TRAILER_SIZE,
    .num_params = 2,
    .param_types = (const uint8_t *)&ce_p_stepper_position,
};

/* endstop state command encoder */
static int ce_p_endstop_state[4] = {PT_byte, PT_byte, PT_byte, PT_uint32}; //oid, homing, finished, next_clock
static struct command_encoder ce_endstop_state = 
{
    .msg_id = ETH_ENDSTOP_STATE_CE + ETH_MAX_CP,
    .max_size = MESSAGE_MAX-MESSAGE_TRAILER_SIZE,
    .num_params = 4,
    .param_types = (const uint8_t *)&ce_p_endstop_state,
};


/****************************************************************
 * Command command parser public table
 ****************************************************************/
struct command_parser *command_parser_table[ETH_MAX_CP] =
{
    [ETH_DEFAULT_CP] = &cp_default, //default
    [ETH_RESET_STEP_CLOCK_CP] = &cp_reset_step_clock, //reset step clock
    [ETH_STEPPER_GET_POSITION_CP] = &cp_stepper_get_position, //stepper get position
    [ETH_ENDSTOP_HOME_CP] = &cp_endstop_home, //endstop home
    [ETH_ENDSTOP_QUERY_STATE_CP] = &cp_endstop_query_state, //query endstop state
    [ETH_STEPPER_STOP_ON_TRIGGER_CP] = &cp_stepper_stop_on_trigger, //stepper stop on trigger
};


/****************************************************************
 * Response command parser public table
 ****************************************************************/
struct command_encoder *command_encoder_table[ETH_MAX_CE] =
{
    [ETH_DEFAULT_CE] = &ce_default, //default
    [ETH_STEPPER_POSITION_CE] = &ce_stepper_position, //stepper position
    [ETH_ENDSTOP_STATE_CE] = &ce_endstop_state, //endstop state
};