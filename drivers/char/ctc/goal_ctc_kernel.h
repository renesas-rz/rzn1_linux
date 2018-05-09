/** @file
 *
 * @brief Core to core
 *
 * This header contains
 *
 * @copyright
 * (C) Copyright 2017 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef GOAL_CTC_KERNEL_H
#define GOAL_CTC_KERNEL_H

#include "goal_ctc_util.h"


/****************************************************************************/
/* Defines */
/****************************************************************************/
#define CTC_IOCTL_NUM 0x0C2C                    /**< magic number */


/****************************************************************************/
/* Typedefs */
/****************************************************************************/
typedef struct {
    uint32_t offset;
    struct GOAL_CTC_CONFIG_T *pConfig;
} GOAL_CTC_IOCTL_DEF_CONF_T;


/****************************************************************************/
/* io ctl defines */
/****************************************************************************/
#define CTC_IOCTL_DEFAULT_CONF _IOWR(CTC_IOCTL_NUM, 0, GOAL_CTC_IOCTL_DEF_CONF_T *) /**< get the default configuration */
#define CTC_IOCTL_SET_MBOX_MS _IOW(CTC_IOCTL_NUM, 1, int) /**< set the maximal mailbox waiting time */
#define CTC_IOCTL_MBOX_SEND _IOW(CTC_IOCTL_NUM, 2, GOAL_CTC_PRECIS_MSG_T *) /**< io control cmd for setup a cyclic channel */
#define CTC_IOCTL_USED_BY_USER _IOW(CTC_IOCTL_NUM, 3, GOAL_BOOL_T) /**< io control cmd configures a channel to evaluate the precis at user space */

#endif /* define GOAL_CTC_KERNEL_H */
