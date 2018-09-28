/** @file
 *
 * @brief mailbox module
 *
 * This module implements a mailbox device for communication between the M3 and A7 core
 * of the RZ-N. Its based on the PL320 driver.
 *
 * A master device /dev/mbox is created during boot-up. Reading or writing to this device
 * is not permitted. It is used to create sub-devices by writing an specific event ID via
 * IOCTL. The event ID is a unsigned 32 bit value, therefore 0 - 0xFFFFFFFF is possible.
 * The sub devices, e.g. /dev/mbox_123, are readable and writable.
 * Writing transmits the message to the foreign core. The user has to set an event ID for
 * the destination core on the beginning of his data.
 * Reading blocks the call until a message with the specific event ID arrives. The user
 * application gets the complete mailbox contend.
 * Sub-devices cannot be removed.
 *
 * @copyright
 * (C) Copyright 2018 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef MAILBOX_H
#define MAILBOX_H

/* editable defines */
#define MBOX_DEV_NAME "mbox"                    /* Name of sysfs device */
#define MBOX_DEV_FILE "mbox_%u"                 /* Device file name in /dev */
#define MBOX_DEV_FILE_SIZE (5 + 10 + 1)         /* "mbox_" + max length of event ID + null-terminator */
#define MBOX_DEVS_MAX 5                         /* umber of char devices */

#define MBOX_FIFO_MAX (1 << 3)                  /* number of fifo elements per device for receiving messages */

#define MBOX_TIMEOUT_MS 500                     /* default timeout for mailbox write */

/* not editable defines */
#define MBOX_IOCTL_NUM 0x1337                   /* magic number */
#define MBOX_DATA_CNT (7)                       /* number mailbox data entries on PL320 driver */
#define MBOX_DATA_SIZE (sizeof(uint32_t))       /* size of a single mailbox data entry on the PL320 driver */

/* io ctl defines */
/* create a new mailbox sub-device */
#define MBOX_IOCTL_CREATE _IOW(MBOX_IOCTL_NUM, 0, unsigned int)

/* timeout of the mailbox sub-device in ms */
#define MBOX_IOCTL_TIMEOUT_SET _IOW(MBOX_IOCTL_NUM, 1, unsigned int)

/* get the version of the mailbox driver as 32 bit value (higher 16 bit: major, lower 16 bit: minor) */
#define MBOX_IOCTL_VER_GET _IOR(MBOX_IOCTL_NUM, 2, unsigned int *)

#endif /* #define MAILBOX_H */
