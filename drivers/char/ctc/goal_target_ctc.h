/** @file
 *
 * @brief Core to core module
 *
 * This module implements the core to core functionality between the M3 and A7 core
 * of the RZ-N.
 *
 * @copyright
 * (C) Copyright 2017 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef GOAL_TARGET_CTC_H
#define GOAL_TARGET_CTC_H

#include <linux/module.h>
#include <asm/byteorder.h>


/****************************************************************************/
/* editable defines */
/****************************************************************************/
/**< device defines */
#define CTC_DEVICE_NAME "ctc"                   /**< Name of sysfs device */
#define CTC_DEVICE_FILE "c2c_chan%u"            /**< Device file name in /dev */
#define CTC_DEVICE_FILE_SIZE 19                 /**< size of device file name in /dev */

/**< shared memory configuration */
#define GOAL_TGT_CTC_CHN_MAX 5                  /**< Number of channels */

/**< 16 pages of 64 bytes = 1024 Bytes write buffer per channel */
#define GOAL_TGT_CTC_PAGE_COUNT_EXPONENT 4      /**< exponent of the number of pages */
#define GOAL_TGT_CTC_PAGE_SIZE_EXPONENT 6       /**< exponent of the size of a page */

/**< configuration of the mailbox handling */
#ifndef GOAL_TGT_CTC_MBOX_TIME_MS
#  define GOAL_TGT_CTC_MBOX_TIME_MS 500         /**< time till mailbox message shall be send in ms */
#endif


/****************************************************************************/
/* not editable defines */
/****************************************************************************/
/**< CTC specific defines */
#define GOAL_CONFIG_CTC_FIFO_CNT_EXPONENT 4     /**< Number of elements in fifo. kfifo requires a power of 2 */

#define GOAL_CTC_AC 1                           /**< kernel module is an application core */
#define GOAL_CTC_CC 0                           /**< kernel module is an application core */

/**< ctc target spezific defiens */
#define GOAL_TGT_CTC_PAGE_COUNT (1 << GOAL_TGT_CTC_PAGE_COUNT_EXPONENT) /**< number of pages */
#define GOAL_TGT_CTC_PAGE_SIZE (1 << GOAL_TGT_CTC_PAGE_SIZE_EXPONENT) /**< size of a page */


/****************************************************************************/
/* Endianness */
/****************************************************************************/
#ifndef GOAL_CONFIG_TARGET_LITTLE_ENDIAN
#  define GOAL_CONFIG_TARGET_LITTLE_ENDIAN 0
#endif

#ifndef GOAL_CONFIG_TARGET_BIG_ENDIAN
#  define GOAL_CONFIG_TARGET_BIG_ENDIAN 0
#endif

#ifdef __LITTLE_ENDIAN
#    undef GOAL_CONFIG_TARGET_LITTLE_ENDIAN
#    define GOAL_CONFIG_TARGET_LITTLE_ENDIAN 1
#elif __BIG_ENDIAN
#    undef GOAL_CONFIG_TARGET_BIG_ENDIAN
#    define GOAL_CONFIG_TARGET_BIG_ENDIAN 1
#endif

#if (GOAL_CONFIG_TARGET_LITTLE_ENDIAN == 0) && (GOAL_CONFIG_TARGET_BIG_ENDIAN == 0)
#  error "Byte order not detected and not set."
#endif

#define GOAL_TARGET_MEM_ALIGN_CPU 4
#define GOAL_TARGET_MEM_ALIGN_NET 4

#define GOAL_TARGET_PACKED_PRE
#define GOAL_TARGET_PACKED __attribute__ ((packed))

#define GOAL_TARGET_INLINE __inline__

#ifndef GOAL_MEMCPY
#  define GOAL_MEMCPY memcpy
#endif

#ifndef GOAL_MEMCMP
#  define GOAL_MEMCMP memcmp
#endif

#  define goal_logDbg(fmt, args...) printk(KERN_DEBUG "[AC_D|%s:%d] " fmt, __func__, __LINE__, ## args)
#  define goal_logInfo(fmt, args...) printk(KERN_INFO "[AC_I|%s:%d] " fmt, __func__, __LINE__, ## args)
#  define goal_logWarn(fmt, args...) printk(KERN_WARNING "[AC_W|%s:%d] " fmt, __func__, __LINE__, ## args)
#  define goal_logErr(fmt, args...)  printk(KERN_NOTICE "[AC_E|%s:%d] " fmt, __func__, __LINE__, ## args)


/****************************************************************************/
/* Typedefs */
/****************************************************************************/
typedef __u32 uint32_t ;
typedef __u16 uint16_t;
typedef __u8 uint8_t;

typedef uint32_t GOAL_STATUS_T;


/****************************************************************************/
/* Makros */
/****************************************************************************/
#define GOAL_MEMSET memset                      /**< set memory */

#endif /* #define GOAL_TARGET_CTC_H */
