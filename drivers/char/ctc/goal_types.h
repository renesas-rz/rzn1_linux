/** @file
 *
 * @brief Generic Types and Defines for GOAL
 *
 * This module defines data types and macros used by all components of GOAL
 * (Generic Open Abstraction Layer).
 *
 * @copyright
 * Copyright 2010-2017.
 * This software is protected Intellectual Property and may only be used
 * according to the license agreement.
 */

#ifndef GOAL_TYPES_H
#define GOAL_TYPES_H


/****************************************************************************/
/* Public constant definitions */
/****************************************************************************/

/* define compile-time assert */
#define GOAL_CASSERT_merge(a, b)    a##b
#define GOAL_CASSERT_cond(p, l)     typedef char GOAL_CASSERT_merge(assertion_failed_, l)[2 * !!(p) - 1]
#define GOAL_CASSERT(p)             GOAL_CASSERT_cond(p, __LINE__)

/* comparison */
#define GOAL_CMP_EQUAL              0

/* timer specific defines */
#define GOAL_TIMER_INVALID      NULL
#define GOAL_TIMER_FUNC(f, p)   void f(void *p)

/* generic lock handle defines */
#define GOAL_LOCK_INFINITE                 0

/**< string terminator length */
#define GOAL_STR_TERM_LEN 1


/* endianness convert defines
 *
 * GOAL_htobe16 - host to big endian, 16 bit
 * GOAL_htole16 - host to little endian, 16 bit
 * GOAL_be16toh - big endian to host, 16 bit
 * GOAL_le16toh - little endian to host, 16 bit
 *
 * GOAL_htobe32 - host to big endian, 32 bit
 * GOAL_htole32 - host to little endian, 32 bit
 * GOAL_be32toh - big endian to host, 32 bit
 * GOAL_le32toh - little endian to host, 32 bit
 */

/* Swap a 16 bit value */
#define GOAL_bswap16(x)   ((uint16_t)(                                   \
                        (((uint16_t)(x) & (uint16_t) 0x00ffU) << 8) | \
                        (((uint16_t)(x) & (uint16_t) 0xff00U) >> 8)))
/* Swap a 32 bit value */
#define GOAL_bswap32(x)   ((uint32_t)(                                          \
                        (((uint32_t)(x) & (uint32_t) 0x000000ffUL) << 24) |  \
                        (((uint32_t)(x) & (uint32_t) 0x0000ff00UL) <<  8) |  \
                        (((uint32_t)(x) & (uint32_t) 0x00ff0000UL) >>  8) |  \
                        (((uint32_t)(x) & (uint32_t) 0xff000000UL) >> 24)))
/* Swap a 64 bit value */
#define GOAL_bswap64(x)   ((uint64_t)(                                          \
                        (((uint64_t)(x) & (uint64_t) 0x00000000000000ffUL) << 56) |  \
                        (((uint64_t)(x) & (uint64_t) 0x000000000000ff00UL) << 40) |  \
                        (((uint64_t)(x) & (uint64_t) 0x0000000000ff0000UL) << 24) |  \
                        (((uint64_t)(x) & (uint64_t) 0x00000000ff000000UL) <<  8) |  \
                        (((uint64_t)(x) & (uint64_t) 0x000000ff00000000UL) >>  8) |  \
                        (((uint64_t)(x) & (uint64_t) 0x0000ff0000000000UL) >> 24) |  \
                        (((uint64_t)(x) & (uint64_t) 0x00ff000000000000UL) >> 40) |  \
                        (((uint64_t)(x) & (uint64_t) 0xff00000000000000UL) >> 56)))

#if (GOAL_CONFIG_TARGET_LITTLE_ENDIAN == 1) && (GOAL_CONFIG_TARGET_BIG_ENDIAN == 1)
#  error "Target endianness cannot be both: GOAL_CONFIG_TARGET_LITTLE_ENDIAN and GOAL_CONFIG_TARGET_BIG_ENDIAN"
#endif

#if GOAL_CONFIG_TARGET_LITTLE_ENDIAN == 1

#  define GOAL_htole16(x)             (x)
#  define GOAL_htole16_p(p, x)        GOAL_set_le16_ua((uint8_t *) (p), x)
#  define GOAL_htole32(x)             (x)
#  define GOAL_htole32_p(p, x)        GOAL_set_le32_ua((uint8_t *) (p), x)
#  define GOAL_htole64(x)             (x)
#  define GOAL_htole64_p(p, x)        GOAL_set_le64_ua((uint8_t *) (p), x)

#  define GOAL_htobe16(x)             GOAL_bswap16(x)
#  define GOAL_htobe16_p(p, x)        GOAL_set_be16_ua((uint8_t *) (p), x)
#  define GOAL_htobe32(x)             GOAL_bswap32(x)
#  define GOAL_htobe24_p(p, x)        GOAL_set_be24_ua((uint8_t *) (p), x)
#  define GOAL_htobe32_p(p, x)        GOAL_set_be32_ua((uint8_t *) (p), x)
#  define GOAL_htobe48_p(p, x)        GOAL_set_be48_ua((uint8_t *) (p), x)
#  define GOAL_htobe64(x)             GOAL_bswap64(x)
#  define GOAL_htobe64_p(p, x)        GOAL_set_be64_ua((uint8_t *) (p), x)

#  define GOAL_le16toh(x)             (x)
#  define GOAL_le16toh_p(p)           GOAL_get_le16_ua((const uint8_t *) (p))
#  define GOAL_le32toh(x)             (x)
#  define GOAL_le32toh_p(p)           GOAL_get_le32_ua((const uint8_t *) (p))
#  define GOAL_le64toh(x)             (x)
#  define GOAL_le64toh_p(p)           GOAL_get_le64_ua((const uint8_t *) (p))

#  define GOAL_be16toh(x)             GOAL_bswap16(x)
#  define GOAL_be16toh_p(p)           GOAL_get_be16_ua((const uint8_t *) (p))
#  define GOAL_be24toh_p(p)           GOAL_get_be24_ua((const uint8_t *) (p))
#  define GOAL_be32toh(x)             GOAL_bswap32(x)
#  define GOAL_be32toh_p(p)           GOAL_get_be32_ua((const uint8_t *) (p))
#  define GOAL_be48toh_p(p)           GOAL_get_be48_ua((const uint8_t *) (p))
#  define GOAL_be64toh(x)             GOAL_bswap64(x)
#  define GOAL_be64toh_p(p)           GOAL_get_be64_ua((const uint8_t *) (p))

#elif GOAL_CONFIG_TARGET_BIG_ENDIAN == 1

#  define GOAL_htole16(x)             GOAL_bswap16(x)
#  define GOAL_htole16_p(p, x)        GOAL_set_le16_ua((uint8_t *) (p), x)
#  define GOAL_htole32(x)             GOAL_bswap32(x)
#  define GOAL_htole32_p(p, x)        GOAL_set_le32_ua((uint8_t *) (p), x)
#  define GOAL_htole64(x)             GOAL_bswap64(x)
#  define GOAL_htole64_p(p, x)        GOAL_set_le64_ua((uint8_t *) (p), x)

#  define GOAL_htobe16(x)             (x)
#  define GOAL_htobe16_p(p, x)        GOAL_set_be16_ua((uint8_t *) (p), x)
#  define GOAL_htobe32(x)             (x)
#  define GOAL_htobe24_p(p, x)        GOAL_set_be24_ua((uint8_t *) (p), x)
#  define GOAL_htobe32_p(p, x)        GOAL_set_be32_ua((uint8_t *) (p), x)
#  define GOAL_htobe48_p(p, x)        GOAL_set_be48_ua((uint8_t *) (p), x)
#  define GOAL_htobe64(x)             (x)
#  define GOAL_htobe64_p(p, x)        GOAL_set_be64_ua((uint8_t *) (p), x)

#  define GOAL_le16toh(x)             GOAL_bswap16(x)
#  define GOAL_le16toh_p(p)           GOAL_get_le16_ua((const uint8_t *) (p))
#  define GOAL_le32toh(x)             GOAL_bswap32(x)
#  define GOAL_le32toh_p(p)           GOAL_get_le32_ua((const uint8_t *) (p))
#  define GOAL_le64toh(x)             GOAL_bswap64(x)
#  define GOAL_le64toh_p(p)           GOAL_get_le64_ua((const uint8_t *) (p))

#  define GOAL_be16toh(x)             (x)
#  define GOAL_be16toh_p(p)           GOAL_get_be16_ua((const uint8_t *) (p))
#  define GOAL_be24toh_p(p)           GOAL_get_be24_ua((const uint8_t *) (p))
#  define GOAL_be32toh(x)             (x)
#  define GOAL_be32toh_p(p)           GOAL_get_be32_ua((const uint8_t *) (p))
#  define GOAL_be48toh_p(p)           GOAL_get_be48_ua((const uint8_t *) (p))
#  define GOAL_be64toh(x)             (x)
#  define GOAL_be64toh_p(p)           GOAL_get_be64_ua((const uint8_t *) (p))

#else
#  error "Target endianness not set: GOAL_CONFIG_TARGET_LITTLE_ENDIAN or GOAL_CONFIG_TARGET_BIG_ENDIAN"
#endif

/* alignment */

/* align the position at 4 bytes and fill the gap with zeros */
#define GOAL_align32(ptr, pos)        do {                                             \
                                        uint8_t _cnt = (4 - (pos & 0x03)) & 0x03;   \
                                        for (; _cnt; _cnt--, ptr[pos++] = 0);          \
                                    } while (0)

/* logic to align memory pointer at GOAL_MEM_ALIGN defines */
#if (GOAL_TARGET_MEM_ALIGN_CPU > GOAL_TARGET_MEM_ALIGN_NET)
#  define GOAL_ALIGN_PAD GOAL_TARGET_MEM_ALIGN_CPU
#else
#  define GOAL_ALIGN_PAD GOAL_TARGET_MEM_ALIGN_NET
#endif

#if (GOAL_ALIGN_PAD < GOAL_TARGET_MEM_ALIGN_CPU) || (GOAL_ALIGN_PAD < GOAL_TARGET_MEM_ALIGN_NET)
#  error "GOAL_ALIGN_PAD < GOAL_MEM_ALIGN_*"
#endif


#if ((2 != GOAL_TARGET_MEM_ALIGN_CPU) && (4 != GOAL_TARGET_MEM_ALIGN_CPU) && (8 != GOAL_TARGET_MEM_ALIGN_CPU)) || \
    ((2 != GOAL_TARGET_MEM_ALIGN_NET) && (4 != GOAL_TARGET_MEM_ALIGN_NET) && (8 != GOAL_TARGET_MEM_ALIGN_NET))
#  error "GOAL_TARGET_MEM_ALIGN_CPU or GOAL_TARGET_MEM_ALIGN_NET: only 2, 4 and 8 alignments supported"
#endif


#define GOAL_align_2(ptr, ptrcast)                                                        \
    ((3 == (ptrcast ptr & 3)) ? 3 : (2 - (ptrcast ptr & 3)))

#define GOAL_align_4(ptr, ptrcast)                                                        \
    ((ptrcast ptr & 3) ? (4 - (ptrcast ptr & 3)) : 0)

#define GOAL_align_8(ptr, ptrcast)                                                        \
    ((ptrcast ptr & 7) ? (8 - (ptrcast ptr & 7)) : 0)

#define GOAL_alignPtr(align, ptr)                                                         \
    (void *) ((PtrCast) ptr + (                                                         \
        (2 == align) ? GOAL_align_2(ptr, (PtrCast)) : (                                   \
        (4 == align) ? GOAL_align_4(ptr, (PtrCast)) :                                     \
                       GOAL_align_8(ptr, (PtrCast)))))

#define GOAL_alignInt(align, val)                                                         \
    (val + ((2 == align) ? GOAL_align_2(val, ) : (                                        \
            (4 == align) ? GOAL_align_4(val, ) :                                          \
                           GOAL_align_8(val, ))))
/* data type casts */

/* define maximum values */
#define GOAL_MAX_I16 0x7fff
#define GOAL_MAX_U16 0xffff
#define GOAL_MAX_U32 0xffffffff

/* sane convert functions for larger to smaller widths */
#define GOAL_INT2U16(x)   (((0 < (x)) && (GOAL_MAX_I16 >= (x))) ? (uint16_t) (x) : 0)
#define GOAL_UINT2I16(x)  (((0 < (x)) && (GOAL_MAX_I16 >= (x))) ? (int16_t)  (x) : 0)
#define GOAL_UINT2U16(x)  (((0 < (x)) && (GOAL_MAX_U16 >= (x))) ? (uint16_t) (x) : 0)
#define GOAL_UINT2U32(x)  (((0 < (x)) && (GOAL_MAX_U32 >= (x))) ? (uint32_t) (x) : 0)

/* set and clear bit macros
 * explanation: the C standard converts unsigned char operands to integers.
 * Therefore the casting is necessary - otherwise for example IAR will give you
 * the warning:
 *     Remark[Pa091]: operator operates on value promoted to int (with possibly
 *     unexpected result)
 */
#define GOAL_bitSet(x, y)     (x | (1 << y))
#define GOAL_bitClear(x, y)   (x & ~(1 << y))
#define GOAL_maskSet(x, y)    (x | y)
#define GOAL_maskClear(x, y)  (x & ~(unsigned int) y)

/* Halt the device if a pointer is zero */
#define GOAL_HALT_IF_NULL(x) if (!x) { goal_targetHalt(); }

/* Halt the device if a structure isn't active */
#define GOAL_HALT_IF_INACTIVE(x) if (GOAL_TRUE != x->active) { goal_targetHalt(); }

/* Mark an element as unused to avoid compiler warnings */
#ifndef UNUSEDARG
#  define UNUSEDARG(x) ((void)(x))
#endif

/* get number of elements of an array */
#define ARRAY_ELEMENTS(x) (sizeof(x) / sizeof(x[0]))

/* offsetof implementation */
#ifndef offsetof
#  define offsetof(_struct, _elem) ((size_t) &(((_struct *) 0)->_elem))
#endif


/****************************************************************************/
/* Public data types */
/****************************************************************************/

/** GOAL_BUFFER_T prototype */
struct GOAL_BUFFER_T;

/** boolean data type */
typedef enum {
    GOAL_FALSE = 0,                             /**< false */
    GOAL_TRUE = 1                               /**< true */
} GOAL_BOOL_T;

/** Error status */
typedef uint32_t GOAL_STATUS_T;

#define GOAL_OK                         0       /**< success */
#define GOAL_OK_TIMER_RUN               1       /**< timer currently running */
#define GOAL_OK_SHUTDOWN                2       /**< issue normal shutdown */
#define GOAL_OK_SUPPORTED               3       /**< supported */
#define GOAL_OK_DELAYED                 4       /**< delayed */
#define GOAL_OK_ALREADY_INITIALIZED     5       /**< already initialized */
#define GOAL_OK_EXISTS                  6       /**< exists already */
#define GOAL_OK_PARTIAL                 7       /**< partial transfer */

#define GOAL_ERROR                      (1u << 31) /**< general error */
#define GOAL_ERR_NULL_POINTER           (GOAL_ERROR | 103) /**< null pointer error */
#define GOAL_ERR_UNSUPPORTED            (GOAL_ERROR | 104) /**< functionality unsupported */
#define GOAL_ERR_WRONG_STATE            (GOAL_ERROR | 105) /**< wrong state */
#define GOAL_ERR_ACCESS                 (GOAL_ERROR | 106) /**< access right violated */
#define GOAL_ERR_ALLOC                  (GOAL_ERROR | 107) /**< failed to allocate memory */
#define GOAL_ERR_WOULDBLOCK             (GOAL_ERROR | 108) /**< resource not available */
#define GOAL_ERR_NODATA                 (GOAL_ERROR | 109) /**< no data to process */
#define GOAL_ERR_FRAME_SEND_FAILED      (GOAL_ERROR | 110) /**< failed to send out an Ethernet frame */
#define GOAL_ERR_LOCK_GET_TIMEOUT       (GOAL_ERROR | 111) /**< timeout while acquiring lock */
#define GOAL_ERR_TARGET_INIT            (GOAL_ERROR | 112) /**< failed to initialize Low Level module */
#define GOAL_ERR_ALLOC_INIT             (GOAL_ERROR | 113) /**< failed to initialize Allocation module */
#define GOAL_ERR_TIMER_INIT             (GOAL_ERROR | 114) /**< failed to initialize Timer module */
#define GOAL_ERR_ETH_INIT               (GOAL_ERROR | 115) /**< failed to initialize Ethernet module */
#define GOAL_ERR_ETH_DUPLEX_GET         (GOAL_ERROR | 116) /**< eth: error reading duplex settings */
#define GOAL_ERR_ETH_SPEED_GET          (GOAL_ERROR | 117) /**< eth: error reading speed settings */
#define GOAL_ERR_ETH_LINK_GET           (GOAL_ERROR | 118) /**< eth: error reading link state */
#define GOAL_ERR_BUFF_GET               (GOAL_ERROR | 119) /**< failed to get a Buffer from a Queue */
#define GOAL_ERR_PARAM                  (GOAL_ERROR | 120) /**< invalid parameter */
#define GOAL_ERR_QUEUE_EMPTY            (GOAL_ERROR | 121) /**< Queue is empty */
#define GOAL_ERR_QUEUE_FULL             (GOAL_ERROR | 122) /**< Queue is full */
#define GOAL_ERR_NVS_READ               (GOAL_ERROR | 123) /**< failed to read from non-volatile memory */
#define GOAL_ERR_NVS_WRITE              (GOAL_ERROR | 124) /**< failed to write to non-volatile memory */
#define GOAL_ERR_IP_SET                 (GOAL_ERROR | 125) /**< failed to set IP address configuration */
#define GOAL_ERR_IP_GET                 (GOAL_ERROR | 126) /**< failed to get IP address configuration */
#define GOAL_ERR_OUT_OF_TIMERS          (GOAL_ERROR | 127) /**< timer module is out of resources */
#define GOAL_ERR_TIMER_NOT_USED         (GOAL_ERROR | 128) /**< timer was not initialized */
#define GOAL_ERR_TIMER_STARTED          (GOAL_ERROR | 129) /**< timer to start is already running */
#define GOAL_ERR_INVALID_FRAME          (GOAL_ERROR | 130) /**< received frame with invalid parameters */
#define GOAL_ERR_OUT_OF_RANGE           (GOAL_ERROR | 131) /**< index/value is out of range */
#define GOAL_ERR_NET_INIT               (GOAL_ERROR | 132) /**< failed to initialize Net module */
#define GOAL_ERR_NET_OPEN               (GOAL_ERROR | 133) /**< failed to open socket */
#define GOAL_ERR_NET_CLOSE              (GOAL_ERROR | 134) /**< failed to close socket */
#define GOAL_ERR_NET_REOPEN             (GOAL_ERROR | 135) /**< failed to reopen socket */
#define GOAL_ERR_NET_SEND               (GOAL_ERROR | 136) /**< failed to send on socket */
#define GOAL_ERR_NET_SEND_CHAN_DISABLED (GOAL_ERROR | 137) /**< failed to send on socket because channel is disabled */
#define GOAL_ERR_NET_OPTION             (GOAL_ERROR | 138) /**< failed to change option of socket */
#define GOAL_ERR_NET_CALLBACK           (GOAL_ERROR | 139) /**< failed to set callback for socket */
#define GOAL_ERR_NET_ACTIVATE           (GOAL_ERROR | 140) /**< failed to activate socket */
#define GOAL_ERR_NET_ADDRESS            (GOAL_ERROR | 141) /**< failed to get remote address */
#define GOAL_ERR_NET_MCAST_ADD          (GOAL_ERROR | 142) /**< failed to add multicast address */
#define GOAL_ERR_NET_MCAST_DEL          (GOAL_ERROR | 143) /**< failed to delete multicast address */
#define GOAL_ERR_OVERFLOW               (GOAL_ERROR | 144) /**< overflow error */
#define GOAL_ERR_TIMER_CREATE           (GOAL_ERROR | 145) /**< failed to create timer */
#define GOAL_ERR_TIMER_START            (GOAL_ERROR | 146) /**< failed to start timer */
#define GOAL_ERR_TIMER_STOP             (GOAL_ERROR | 147) /**< failed to stop timer */
#define GOAL_ERR_TIMER_DELETE           (GOAL_ERROR | 148) /**< failed to delete timer */
#define GOAL_ERR_THREAD_CREATE          (GOAL_ERROR | 149) /**< failed to create thread */
#define GOAL_ERR_THREAD_CLOSE           (GOAL_ERROR | 150) /**< failed to close thread */
#define GOAL_ERR_MAC_GET                (GOAL_ERROR | 151) /**< failed to get MAC address */
#define GOAL_ERR_MAC_SET                (GOAL_ERROR | 152) /**< failed to set MAC address */
#define GOAL_ERR_MAC_DEL                (GOAL_ERROR | 153) /**< failed to delete MAC address */
#define GOAL_ERR_NET_IP_SET             (GOAL_ERROR | 154) /**< failed to set IP configuration */
#define GOAL_ERR_NET_IP_GET             (GOAL_ERROR | 155) /**< failed to get IP configuration */
#define GOAL_ERR_LOCK_WRONG_STATE       (GOAL_ERROR | 156) /**< lock has wrong state */
#define GOAL_ERR_INIT                   (GOAL_ERROR | 157) /**< initialization failed */
#define GOAL_ERR_ALREADY_USED           (GOAL_ERROR | 158) /**< already in use */
#define GOAL_ERR_NOT_INITIALIZED        (GOAL_ERROR | 159) /**< not initialized */
#define GOAL_ERR_TASK_CREATE            (GOAL_ERROR | 160) /**< task: create failed */
#define GOAL_ERR_TASK_EXIT              (GOAL_ERROR | 161) /**< task: exit failed */
#define GOAL_ERR_TASK_JOIN              (GOAL_ERROR | 162) /**< task: join failed */
#define GOAL_ERR_TASK_SLEEP             (GOAL_ERROR | 163) /**< task: sleep failed */
#define GOAL_ERR_TASK_SLEEP_INT         (GOAL_ERROR | 164) /**< task: sleep interrupted  can be resumed */
#define GOAL_ERR_TASK_START             (GOAL_ERROR | 165) /**< task: start failed */
#define GOAL_ERR_TASK_SELF              (GOAL_ERROR | 166) /**< task: get self-id failed */
#define GOAL_ERR_TASK_SUSPEND           (GOAL_ERROR | 167) /**< task: suspend failed */
#define GOAL_ERR_TASK_RESUME            (GOAL_ERROR | 168) /**< task: resume failed */
#define GOAL_ERR_TASK_PRIORITY_SET      (GOAL_ERROR | 169) /**< task: priorty set failed */
#define GOAL_ERR_BUSY                   (GOAL_ERROR | 170) /**< busy */
#define GOAL_ERR_TIMEOUT                (GOAL_ERROR | 171) /**< timeout occured */
#define GOAL_ERR_NOT_FOUND              (GOAL_ERROR | 172) /**< not found */
#define GOAL_ERR_NET_DHCP_START         (GOAL_ERROR | 173) /**< DHCP: start failed */
#define GOAL_ERR_NET_DHCP_STOP          (GOAL_ERROR | 174) /**< DHCP: stop failed */
#define GOAL_ERR_NET_DHCP_RENEW         (GOAL_ERROR | 175) /**< DHCP: renew failed */
#define GOAL_ERR_NET_DHCP_RELEASE       (GOAL_ERROR | 176) /**< DHCP: release failed */
#define GOAL_ERR_NET_DHCP_INFORM        (GOAL_ERROR | 177) /**< DHCP: inform failed */
#define GOAL_ERR_NET_DHCP_ADDR          (GOAL_ERROR | 178) /**< DHCP: address retrieval failed */
#define GOAL_ERR_SPECIFIC               (GOAL_ERROR | 179) /**< specific error set in (meta-) data */
#define GOAL_ERR_EMPTY                  (GOAL_ERROR | 180) /**< selected part is empty */
#define GOAL_ERR_FULL                   (GOAL_ERROR | 181) /**< selected part is full */
#define GOAL_ERR_DELAYED                (GOAL_ERROR | 182) /**< delay error */
#define GOAL_ERR_UNDERFLOW              (GOAL_ERROR | 183) /**< underflow error */
#define GOAL_ERR_NET_DEACTIVATE         (GOAL_ERROR | 184) /**< failed to deactivate socket */
#define GOAL_ERR_NET_SET_SEND           (GOAL_ERROR | 185) /**< failed to set send function for channel */
#define GOAL_ERR_EXISTS                 (GOAL_ERROR | 186) /**< ressource already exists */
#define GOAL_ERR_NOT_EMPTY              (GOAL_ERROR | 187) /**< ressource not empty */
#define GOAL_ERR_TASK_PRIORITY_GET      (GOAL_ERROR | 188) /**< task: priorty get failed */

#define GOAL_RES_OK(x) (!(x & GOAL_ERROR))      /**< positive result verification */
#define GOAL_RES_ERR(x) (x & GOAL_ERROR)        /**< negative result verification */

/**< GOAL result with integrated id */
#define GOAL_RES_ID(_id, _status) ((_id << 15) | _status)

/**< GOAL status areas */
#define GOAL_RES_MASK                   0x0000ffff /**< GOAL result mask */
#define GOAL_RES_AREA                   0xefff0000 /**< GOAL result area mask */

#define GOAL_RES_GOAL                   0x00000000 /**< GOAL */
#define GOAL_RES_APPL                   0x00200000 /**< Application */
#define GOAL_RES_PNIO                   0x00300000 /**< PROFINET */
#define GOAL_RES_EIP                    0x00400000 /**< EtherNet/IP */
#define GOAL_RES_EPL                    0x00500000 /**< POWERLINK */
#define GOAL_RES_CO                     0x00600000 /**< CANopen */
#define GOAL_RES_RPC                    0x00700000 /**< Remote Procedure Call */

/* Number Base Definitions */
#define GOAL_NUM_BASE_DEC               10      /**< base decimal */
#define GOAL_NUM_BASE_HEX               16      /**< base hexadecimal */
#define GOAL_NUM_PERCENT_100            100     /**< 100 percent */

/** generic GOAL_STATUS_T void ptr-argument function */
typedef GOAL_STATUS_T (* GOAL_FUNC_RET_T)(void *pArg);

/** generic GOAL_STATUS_T no-argument function */
typedef GOAL_STATUS_T (* GOAL_FUNC_RET_NOARG_T)(void);

/** generic no-return void ptr-argument function */
typedef void (* GOAL_FUNC_NORET_T)(void *pArg);

/** generic no-return and no-argument function */
typedef void (* GOAL_FUNC_NORET_NOARG_T)(void);

/** timer callback */
typedef void (*GOAL_TIMER_CB_T)(void *);

/** timer type */
typedef enum {
    GOAL_TIMER_SINGLE = 0,                      /**< single shot timer */
    GOAL_TIMER_PERIODIC = 1                     /**< periodic timer */
} GOAL_TIMER_TYPE_T;

/** timer priority */
typedef enum {                                  /**< timer priorities */
    GOAL_TIMER_LOW = 0,
    GOAL_TIMER_HIGH = 1
} GOAL_TIMER_PRIO_T;

/** 64 bit timestamp */
typedef uint64_t GOAL_TIMESTAMP_T;              /**< timestamp */

/** Ethernet Tx Type */
typedef enum {
    GOAL_NET_TX_LOW = 0,                        /**< low priority */
    GOAL_NET_TX_LLDP,                           /**< LLDP */
    GOAL_NET_TX_RT,                             /**< real-time data */
} GOAL_NET_TX_TYPE_T;

/** endian type */
typedef enum {
    GOAL_ENDIAN_LITTLE = 0,
    GOAL_ENDIAN_BIG
} GOAL_ENDIAN_T;

/* Locking data types */
/**< lock type */
typedef enum {
    GOAL_LOCK_BINARY,
    GOAL_LOCK_COUNT
} GOAL_LOCK_TYPE_T;


/**< usage type */
typedef uint32_t GOAL_USAGE_T;


/**< id type */
typedef uint16_t GOAL_ID_T;


/**< usage types (see goal/goal_id.h) */
#define GOAL_USAGE_UNKNOWN      GOAL_ID_ZERO
#define GOAL_USAGE_MEM_POOL     GOAL_ID_MEM
#define GOAL_USAGE_TIMER        GOAL_ID_TMR
#define GOAL_USAGE_ETH          GOAL_ID_ETH
#define GOAL_USAGE_ETH_RECV     GOAL_ID_ETH_RECV
#define GOAL_USAGE_ETH_SEND     GOAL_ID_ETH_SEND
#define GOAL_USAGE_LOG          GOAL_ID_LOG
#define GOAL_USAGE_LOG_SYSLOG   GOAL_ID_LOG_SYSLOG
#define GOAL_USAGE_NET          GOAL_ID_NET
#define GOAL_USAGE_LIST         GOAL_ID_LIST
#define GOAL_USAGE_APPL         GOAL_ID_APPL
#define GOAL_USAGE_CTC          GOAL_ID_CTC
#define GOAL_USAGE_RPC          GOAL_ID_RPC
#define GOAL_USAGE_DLR          GOAL_ID_DLR
#define GOAL_USAGE_CAN_SEND     GOAL_ID_CAN_SEND
#define GOAL_USAGE_CAN_TXBUF    GOAL_ID_CAN_TXBUF
#define GOAL_USAGE_CANOPEN      GOAL_ID_CANOPEN
#define GOAL_USAGE_PTP          GOAL_ID_PTP
#define GOAL_USAGE_MI_CTC_SPI   GOAL_ID_MI_CTC_SPI


/** lock data */
typedef GOAL_TARGET_PACKED_PRE struct {
    GOAL_BOOL_T active;                         /**< lock active flag */
    GOAL_LOCK_TYPE_T type;                      /**< lock type */
    GOAL_USAGE_T usage;                         /**< usage indicator */

    void *pTgt;                                 /**< target data */
} GOAL_TARGET_PACKED GOAL_LOCK_T;

/** network connection type */
typedef enum {
    GOAL_NET_UDP_SERVER,                        /**< UDP server connection */
    GOAL_NET_UDP_CLIENT,                        /**< UDP client connection */
    GOAL_NET_TCP_LISTENER,                      /**< TCP server connection (waiting state) */
    GOAL_NET_TCP,                               /**< active TCP client or server connection */
    GOAL_NET_TCP_CLIENT,                        /**< unconnected TCP client connection */
    GOAL_NET_INTERNAL,                          /**< internal connection between GOAL channels */
} GOAL_NET_TYPE_T;

/** network address */
typedef struct {
    uint32_t      localIp;                      /**< local IP address */
    uint16_t      localPort;                    /**< local port number */
    uint32_t      remoteIp;                     /**< remote IP address */
    uint16_t      remotePort;                   /**< remote port number */
} GOAL_NET_ADDR_T;

/** callback types */
typedef enum {
    GOAL_NET_CB_NEW_DATA,                       /**< callback: data available */
    GOAL_NET_CB_NEW_SOCKET,                     /**< callback: new socket opened */
    GOAL_NET_CB_CLOSING,                        /**< callback: socket closing */
    GOAL_NET_CB_CONNECTED,                      /**< callback: connected to server */
} GOAL_NET_CB_TYPE_T;

/* GOAL net buffer forward declaration */
struct GOAL_NET_CHAN_T;

/** network receive callback */
typedef void (*GOAL_NET_CB_T)(
    GOAL_NET_CB_TYPE_T cbType,                  /**< callback type */
    struct GOAL_NET_CHAN_T *pChan,              /**< channel descriptor */
    struct GOAL_BUFFER_T *pBuf                  /**< GOAL buffer */
);

/** network channel send function */
typedef GOAL_STATUS_T (*GOAL_NET_SEND_FUNC_T)(
    struct GOAL_NET_CHAN_T *pChan,              /**< channel descriptor */
    struct GOAL_BUFFER_T *pBuf                  /**< GOAL buffer */
);

/** network channel close function */
typedef GOAL_STATUS_T (*GOAL_NET_CLOSE_FUNC_T)(
    struct GOAL_NET_CHAN_T *pChan               /**< channel descriptor */
);


typedef struct GOAL_NET_CHAN_T {
    struct GOAL_NET_CHAN_T *pChanSendDir;       /**< next channel in sending direction */
    struct GOAL_NET_CHAN_T *pChanApplDir;       /**< next channel in application direction */
    GOAL_NET_SEND_FUNC_T send;                  /**< channel sending function */
    GOAL_NET_CLOSE_FUNC_T close;                /**< channel closing function */
    GOAL_NET_CB_T callback;                     /**< callback function */
    GOAL_NET_TYPE_T type;                       /**< channel type */
    void *pData;                                /**< generic data pointer */
    GOAL_BOOL_T used;                           /**< channel is in use flag */
    GOAL_BOOL_T active;                         /**< channel is in use flag */
    GOAL_NET_ADDR_T addr;                       /**< net address */
} GOAL_NET_CHAN_T;

/** network option */
typedef enum {
    GOAL_NET_OPTION_NONBLOCK,                   /**< set socket to non-blocking */
    GOAL_NET_OPTION_BROADCAST,                  /**< broadcast reception */
    GOAL_NET_OPTION_TTL,                        /**< set TTL value in IP header */
    GOAL_NET_OPTION_TOS,                        /**< set TOS value in IP header */
    GOAL_NET_OPTION_MCAST_IF,                   /**< choose multicast interface */
    GOAL_NET_OPTION_MCAST_ADD,                  /**< join multicast group */
    GOAL_NET_OPTION_MCAST_DROP,                 /**< leave multicast group  */
    GOAL_NET_OPTION_REUSEADDR,                  /**< make address reusable */
} GOAL_NET_OPTION_T;

/** IP Multicast group type */
typedef struct {
    uint32_t mcastIp;                           /**< multicast address */
    uint32_t localIp;                           /**< local IP address */
} GOAL_NET_MCAST_GROUP_T;


/** Generic GOAL Callback */
typedef struct GOAL_CB_T {
    struct GOAL_CB_T *pNext;                    /**< next element */
    uint32_t id;                                /**< callback type */
    void *pFunc;                                /**< generic function ptr */
} GOAL_CB_T;


/****************************************************************************/
/* List of public functions */
/****************************************************************************/


/****************************************************************************/
/** Read a 16 bit value as little endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint16_t GOAL_get_le16_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint16_t)(
            ((uint16_t)(p8[0])) |
            ((uint16_t)(p8[1] << 8))));
}


/****************************************************************************/
/** Read a 16 bit value as big endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint16_t GOAL_get_be16_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint16_t)(
            ((uint16_t)(p8[0]) << 8) |
            ((uint16_t)(p8[1]))));
}


/****************************************************************************/
/** Write a 16 bit value as little endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_le16_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint16_t x                                  /**< uint16_t value */
)
{
    p8[0] = (uint8_t) (((uint16_t)(x) & (uint16_t) 0x00ffU));
    p8[1] = (uint8_t) (((uint16_t)(x) & (uint16_t) 0xff00U) >> 8);
}


/****************************************************************************/
/** Write a 16 bit value as big endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_be16_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint16_t x                                  /**< uint16_t value */
)
{
    p8[0] = (uint8_t) (((uint16_t)(x) & (uint16_t) 0xff00U) >> 8);
    p8[1] = (uint8_t) (((uint16_t)(x) & (uint16_t) 0x00ffU));
}


/****************************************************************************/
/** Read a 24 bit value as big endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint32_t GOAL_get_be24_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint32_t)(
            ((uint32_t)(p8[0]) << 16) |
            ((uint32_t)(p8[1]) <<  8) |
            ((uint32_t)(p8[2]))));
}


/****************************************************************************/
/** Write a 24 bit value as big endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_be24_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint32_t x                                  /**< uint32_t value */
)
{
    p8[0] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x00ff0000U) >> 16);
    p8[1] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x0000ff00U) >>  8);
    p8[2] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x000000ffU));
}


/****************************************************************************/
/** Read a 32 bit value as little endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint32_t GOAL_get_le32_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint32_t)(
            ((uint32_t)(p8[0]))       |
            ((uint32_t)(p8[1]) <<  8) |
            ((uint32_t)(p8[2]) << 16) |
            ((uint32_t)(p8[3]) << 24)));
}


/****************************************************************************/
/** Read a 32 bit value as big endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint32_t GOAL_get_be32_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint32_t)(
            ((uint32_t)(p8[0]) << 24) |
            ((uint32_t)(p8[1]) << 16) |
            ((uint32_t)(p8[2]) <<  8) |
            ((uint32_t)(p8[3]))));
}


/****************************************************************************/
/** Write a 32 bit value as little endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_le32_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint32_t x                                  /**< uint32_t value */
)
{
    p8[0] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x000000ffU));
    p8[1] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x0000ff00U) >>  8);
    p8[2] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x00ff0000U) >> 16);
    p8[3] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0xff000000U) >> 24);
}


/****************************************************************************/
/** Write a 32 bit value as big endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_be32_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint32_t x                                  /**< uint32_t value */
)
{
    p8[0] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0xff000000U) >> 24);
    p8[1] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x00ff0000U) >> 16);
    p8[2] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x0000ff00U) >>  8);
    p8[3] = (uint8_t) (((uint32_t)(x) & (uint32_t) 0x000000ffU));
}


/****************************************************************************/
/** Read a 64 bit value as little endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint64_t GOAL_get_le64_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint64_t)(
            ((uint64_t)(p8[0]))       |
            ((uint64_t)(p8[1]) <<  8) |
            ((uint64_t)(p8[2]) << 16) |
            ((uint64_t)(p8[3]) << 24) |
            ((uint64_t)(p8[4]) << 32) |
            ((uint64_t)(p8[5]) << 40) |
            ((uint64_t)(p8[6]) << 48) |
            ((uint64_t)(p8[7]) << 56)));
}


/****************************************************************************/
/** Read a 64 bit value as big endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint64_t GOAL_get_be64_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint64_t)(
            ((uint64_t)(p8[0]) << 56) |
            ((uint64_t)(p8[1]) << 48) |
            ((uint64_t)(p8[2]) << 40) |
            ((uint64_t)(p8[3]) << 32) |
            ((uint64_t)(p8[4]) << 24) |
            ((uint64_t)(p8[5]) << 16) |
            ((uint64_t)(p8[6]) <<  8) |
            ((uint64_t)(p8[7]))));
}


/****************************************************************************/
/** Read a 48 bit value as big endian byte-wise from a pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE uint64_t GOAL_get_be48_ua(
    const uint8_t *p8                           /**< uint8_t pointer */
)
{
    return ((uint64_t)(
            ((uint64_t)(p8[0]) << 40) |
            ((uint64_t)(p8[1]) << 32) |
            ((uint64_t)(p8[2]) << 24) |
            ((uint64_t)(p8[3]) << 16) |
            ((uint64_t)(p8[4]) <<  8) |
            ((uint64_t)(p8[5]))));
}


/****************************************************************************/
/** Write a 64 bit value as little endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_le64_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint64_t x                                  /**< uint64_t value */
)
{
    p8[0] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00000000000000ffU));
    p8[1] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x000000000000ff00U) >>  8);
    p8[2] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x0000000000ff0000U) >> 16);
    p8[3] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00000000ff000000U) >> 24);
    p8[4] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x000000ff00000000U) >> 32);
    p8[5] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x0000ff0000000000U) >> 40);
    p8[6] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00ff000000000000U) >> 48);
    p8[7] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0xff00000000000000U) >> 56);
}


/****************************************************************************/
/** Write a 64 bit value as big endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_be64_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint64_t x                                  /**< uint64_t value */
)
{
    p8[0] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0xff00000000000000U) >> 56);
    p8[1] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00ff000000000000U) >> 48);
    p8[2] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x0000ff0000000000U) >> 40);
    p8[3] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x000000ff00000000U) >> 32);
    p8[4] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00000000ff000000U) >> 24);
    p8[5] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x0000000000ff0000U) >> 16);
    p8[6] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x000000000000ff00U) >>  8);
    p8[7] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00000000000000ffU));
}


/****************************************************************************/
/** Write a 48 bit value as big endian byte-wise to pointer destination
 *
 * ua - (assume) unaligned
 */
static GOAL_TARGET_INLINE void GOAL_set_be48_ua(
    uint8_t *p8,                                /**< uint8_t pointer */
    uint64_t x                                  /**< uint64_t value */
)
{
    p8[0] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x0000ff0000000000U) >> 40);
    p8[1] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x000000ff00000000U) >> 32);
    p8[2] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00000000ff000000U) >> 24);
    p8[3] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x0000000000ff0000U) >> 16);
    p8[4] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x000000000000ff00U) >>  8);
    p8[5] = (uint8_t) (((uint64_t)(x) & (uint64_t) 0x00000000000000ffU));
}


/****************************************************************************/
/** Convert a 16-bit value with a given endianness to host-endianness
 *
 * @param [in] pVal value pointer
 * @param [in] endian endianness
 *
 * @returns 16-bit value in host-format
 */

#define GOAL_en16toh_p(pVal, endian)  ((GOAL_ENDIAN_BIG == endian) ?  \
                                      GOAL_get_be16_ua((const uint8_t *) (pVal)) : \
                                      GOAL_get_le16_ua((const uint8_t *) (pVal)))


/****************************************************************************/
/** Convert a 32-bit value with a given endianness to host-endianness
 *
 * @param [in] pVal value pointer
 * @param [in] endian endianness
 *
 * @returns 32-bit value in host-format
 */
#define GOAL_en32toh_p(pVal, endian)  ((GOAL_ENDIAN_BIG == endian) ?  \
                                      GOAL_get_be32_ua((const uint8_t *) (pVal)) : \
                                      GOAL_get_le32_ua((const uint8_t *) (pVal)))


/****************************************************************************/
/** Convert a 64-bit value with a given endianness to host-endianness
 *
 * @param [in] pVal value pointer
 * @param [in] endian endianness
 *
 * @returns 64-bit value in host-format
 */
#define GOAL_en64toh_p(pVal, endian)  ((GOAL_ENDIAN_BIG == endian) ?  \
                                      GOAL_get_be64_ua((const uint8_t *) (pVal)) : \
                                      GOAL_get_le64_ua((const uint8_t *) (pVal)))


/****************************************************************************/
/** Convert a 16-bit value from host-endianness to a given endianness
 *
 * @param [out] pVal value pointer
 * @param [in] valHost host-value
 * @param [in] endian endianness
 */
#define GOAL_htoen16_p(pVal, valHost, endian)                         \
                            do {                                      \
                                if (GOAL_ENDIAN_BIG == endian)        \
                                    GOAL_set_be16_ua((uint8_t *) (pVal), valHost); \
                                else                                  \
                                    GOAL_set_le16_ua((uint8_t *) (pVal), valHost); \
                            } while (0)


/****************************************************************************/
/** Convert a 32-bit value from host-endianness to a given endianness
 *
 * @param [out] pVal value pointer
 * @param [in] valHost host-value
 * @param [in] endian endianness
 */
#define GOAL_htoen32_p(pVal, valHost, endian)                         \
                            do {                                      \
                                if (GOAL_ENDIAN_BIG == endian)        \
                                    GOAL_set_be32_ua((uint8_t *) (pVal), valHost); \
                                else                                  \
                                    GOAL_set_le32_ua((uint8_t *) (pVal), valHost); \
                            } while (0)


/****************************************************************************/
/** Convert a 64-bit value from host-endianness to a given endianness
 *
 * @param [out] pVal value pointer
 * @param [in] valHost host-value
 * @param [in] endian endianness
 */
#define GOAL_htoen64_p(pVal, valHost, endian)                         \
                            do {                                      \
                                if (GOAL_ENDIAN_BIG == endian)        \
                                    GOAL_set_be64_ua((uint8_t *) (pVal), valHost); \
                                else                                  \
                                    GOAL_set_le64_ua((uint8_t *) (pVal), valHost); \
                            } while (0)


#endif /* GOAL_TYPES_H */
