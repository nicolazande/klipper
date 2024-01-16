/**
 * \file command.c
 *
 * \brief .
 *
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
extern struct command_parser *command_parser_table[ETHCAT_MAX_CMD];
extern struct command_parser *response_parser_table[ETHCAT_MAX_RES]; 


/****************************************************************
 * Private function prototypes
 ****************************************************************/
/* encode an integer as a variable length quantity (vlq) */
static uint8_t *encode_int(uint8_t *p, uint32_t v);

/* parse a vlq integer */
static uint32_t parse_int(uint8_t **pp);

/* encode a response message */
static uint8_t command_encodef(uint8_t *buf, const struct command_parser *cp, va_list args);

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
command_encodef(uint8_t *buf, const struct command_parser *cp, va_list args)
{
    /* get message max size in bytes */
    uint8_t max_size = cp->max_size;
    if (max_size <= MESSAGE_MIN)
    {
        return max_size;
    }

    /* get first byte in command */
    uint8_t *p = &buf[MESSAGE_HEADER_SIZE];
    uint8_t *maxend = &p[max_size - MESSAGE_MIN];
    
    /* get number and type of parameters */
    uint8_t num_params = cp->num_params;
    const uint8_t *param_types = cp->param_types;
    
    /* store message id */
    *p++ = cp->msg_id;

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
                /* do nothing */
                return 0;
            }
        }
    }
    /* move forward */
    return p - buf + MESSAGE_TRAILER_SIZE;
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
command_parsef(uint8_t *p, uint8_t *maxend, struct command_parser *cp, uint32_t *args)
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
command_encode_and_frame(uint8_t *buf, struct command_parser *ce, ...)
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
 * Command command parsers
 ****************************************************************/
/** default dummy function */
static int command_general_dummy(struct ethcatqueue *sq, void *out, uint32_t *args)
{
    return 0;
}
static int command_dummy_param_list[0];
static struct command_parser generic_dummy_parser = 
{
    .msg_id = ETHCAT_NO_CMD,
    .num_args = 0,
    .flags = 0,
    .num_params = 0,
    .max_size = 0,
    .param_types = &command_dummy_param_list,
    .func = &command_general_dummy,
};

/** get position command */
static int command_stepper_get_position(struct ethcatqueue *sq, void *out, uint32_t *args)
{
    /* get drive oid and target buffer */
    uint8_t oid = args[0];
    uint8_t *buf = (uint8_t *)out;
    /* get position from slave moinitor */
    int32_t position = sq->masterifc.monitor[oid].position; /** TODO: add int32_t specific field */
    /* get response command parser */
    struct command_parser *ce = response_parser_table[ETHCAT_GET_POSITION_RES];
    /* create response  */
    uint8_t msglen = command_encode_and_frame(buf, ce, oid, position);
}
static int command_stepper_get_position_param_list[1] = {PT_byte}; //oid
static struct command_parser command_stepper_get_position_parser = 
{
    .msg_id = ETHCAT_GET_POSITION_CMD,
    .num_args = 1,
    .flags = 0,
    .num_params = 1,
    .param_types = &command_stepper_get_position_param_list,
    .func = &command_stepper_get_position,
};

/* set drive absolute clock (next step) */
static int command_reset_step_clock(struct ethcatqueue *sq, void *out, uint32_t *args)
{
    struct stepper *s = NULL;
    /* get drive oid and target buffer */
    uint8_t oid = args[0];
    uint32_t waketime = args[1];
    /** TODO: add logic for next step time */
    return 0;
}
static int command_reset_step_clock_param_list[2] = {PT_byte, PT_uint32}; //oid, clock
static struct command_parser command_reset_step_clock_parser = 
{
    .msg_id = ETHCAT_SET_CLOCK_CMD,
    .num_args = 2,
    .flags = 0,
    .num_params = 2,
    .param_types = &command_reset_step_clock_param_list,
    .func = &command_reset_step_clock,
};


/****************************************************************
 * Response command parsers
 ****************************************************************/
/** get position response */
static int response_stepper_get_position_param_list[2] = {PT_byte, PT_int32}; //oid, position
static struct command_parser response_stepper_get_position_parser = 
{
    .msg_id = ETHCAT_GET_POSITION_RES,
    .flags = 0,
    .num_params = 2,
    .max_size = MESSAGE_MAX-MESSAGE_TRAILER_SIZE,
    .param_types = &response_stepper_get_position_param_list,
    .func = NULL,
};


/****************************************************************
 * Command command parser public table
 ****************************************************************/
struct command_parser *command_parser_table[ETHCAT_MAX_CMD] =
{
    [ETHCAT_NO_CMD] = &generic_dummy_parser, //no command
    [ETHCAT_GET_POSITION_CMD] = &command_stepper_get_position_parser, //query drive position
    [ETHCAT_SET_CLOCK_CMD] = &command_reset_step_clock_parser, //reset drive clock
};


/****************************************************************
 * Response command parser public table
 ****************************************************************/
struct command_parser *response_parser_table[ETHCAT_MAX_RES] =
{
    [ETHCAT_NO_RES] = &generic_dummy_parser, //no response
    [ETHCAT_GET_POSITION_RES] = &response_stepper_get_position_parser, //query drive position
};
