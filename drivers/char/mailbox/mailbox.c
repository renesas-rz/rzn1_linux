/** @file
 *
 * @brief mailbox module
 *
 * This module implements a mailbox device for communication between the M3 and A7 core
 * of the RZ/N1. It's based on the PL320 driver.
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

#include <asm/byteorder.h>
#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pl320-ipc.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mailbox.h"

/* not editable defines */
#define MBOX_VER_MAJOR 0                        /* major version of mailbox module */
#define MBOX_VER_MINOR 3                        /* mainor version of mailbox module */
#define MBOX_MSG_SIZE (MBOX_DATA_CNT * MBOX_DATA_SIZE) /* complete size of a single mailbox message */

/* Local structs/enums */
enum MBOX_DEV_GET_T {
	MBOX_DEV_GET_BY_EVENT_ID = 0,               /* look for device with given event ID */
	MBOX_DEV_GET_BY_MINOR_ID,                   /* look for device with given minor ID */
};

struct MBOX_DEV_SUB_INFO_T {
	uint32_t eventId;                           /* event ID */

	/* writing section */
	uint32_t *pDataWr;                          /* data buffer for writing */
	struct mutex mtxWrite;                      /* mutex for writing */

	/* reading section */
	struct circ_buf dataRd;                     /* circular buffer of received messages */
	wait_queue_head_t queueRxData;              /* wait queue for received messages */
	unsigned int timeoutMs;                     /* mailbox waiting time */
};

struct MBOX_DEV_T {
	struct MBOX_DEV_T *pNext;                   /* next element */
	int minor;                                  /* minor number */
	struct MBOX_DEV_SUB_INFO_T *pInfo;          /* private data of sub-device */
	struct cdev *pCharDev;                      /* character device */
	struct device *pSysDev;                     /* sysfs device reference */
	pid_t pid;                                  /* PID of the user space process */
};

struct MBOX_T {
	dev_t dev;                                  /* mailbox device */
	int major;                                  /* major number */
	struct class *pCharClass;                   /* mailbox class */
	struct MBOX_DEV_T *pListMboxDev;            /* mailbox devices */
	int listEntries;                            /* number of elements in the list */
};

/* get the time from a timer handle in ms */
#define TIME_MS_GET(hdlTime) \
	((uint64_t) ((hdlTime.tv_sec * 1000) + (hdlTime.tv_nsec / 1000000)))

/* get the data index on the circular buffer - consider wrap around */
#define MBOX_CIRC_DATA_IDX_GET(x) ((x % MBOX_FIFO_MAX) * MBOX_MSG_SIZE)

static struct MBOX_T *pMbox;
static const struct file_operations mbox_fops;  /* forward declaration */

/**
 * This function iterates the list of existing mailbox devices, looking up for
 * for the given ID. The ID is either an event ID or a minor ID.
 * The match is returned if a reference is set.
 *
 * Return 0 if successful, otherwise an error code.
 *
 * @ppMboxDev: mailbox device reference
 *
 * @type: Kind of type the id belongs to. This is
 *            the minor ID (MBOX_DEV_GET_BY_MINOR_ID) or
 *            the event ID (MBOX_DEV_GET_BY_EVENT_ID)
 *
 * @id: ID the list of mailbox devices is looked for.
 */
static int mbox_dev_get(
	struct MBOX_DEV_T **ppMboxDev,
	enum MBOX_DEV_GET_T type,
	int id
)
{
	struct MBOX_DEV_T *pIdxDev;                 /* device index */

	switch (type) {
	/* get device based on the event ID */
	case MBOX_DEV_GET_BY_EVENT_ID:
		for (pIdxDev = pMbox->pListMboxDev; pIdxDev != NULL; pIdxDev = pIdxDev->pNext) {
			if (pIdxDev->pInfo) {
				if ((uint32_t) id == pIdxDev->pInfo->eventId) {
					if (ppMboxDev != NULL)
						*ppMboxDev = pIdxDev;
					return 0;
				}
			}
		}
		break;

	/* get device based on the minor ID */
	case MBOX_DEV_GET_BY_MINOR_ID:
		for (pIdxDev = pMbox->pListMboxDev; pIdxDev != NULL; pIdxDev = pIdxDev->pNext) {
			if (id == pIdxDev->minor) {
				if (ppMboxDev != NULL)
					*ppMboxDev = pIdxDev;
				return 0;
			}
		}
		break;

	default:
		pr_notice("mbox: get type %u is invalid.\n", type);
		return -EINVAL;
	}

	return -ENODEV;
}

/**
 * Opens a mailbox device. Reading, writing and IOCTL is only possible on
 * opened devices.
 *
 * Return 0 if successful, otherwise an error code.
 *
 * @pInode: pointer to inode
 *
 * @pFile: pointer to file
 */
static int mbox_open(
	struct inode *pInode,
	struct file *pFile
)
{
	struct MBOX_DEV_T *pMboxDev = NULL;         /* mailbox device */
	int id;                                     /* minor id */
	int res;                                    /* result */

	id = MINOR(pFile->f_inode->i_rdev);

	res = mbox_dev_get(&pMboxDev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_notice("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}

	/* check if the file is already open */
	if (pMboxDev->pid)
		return -EACCES;

	/* save the pid */
	pMboxDev->pid = current->pid;

	return 0;
}

/**
 * This function releases the mailbox char device.
 *
 * Return 0 if successful, otherwise an error code.
 *
 * @pInode: pointer to inode
 *
 * @pFile: pointer to file
 */
static int mbox_release(
	struct inode *pInode,
	struct file *pFile
)
{
	struct MBOX_DEV_T *pMboxDev = NULL;         /* mailbox device */
	int id;                                     /* minor id */
	int res;                                    /* result */

	id = MINOR(pFile->f_inode->i_rdev);

	res = mbox_dev_get(&pMboxDev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_notice("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}

	pMboxDev->pid = 0;

	return 0;
}

/**
 * This function provides a blocking reading of new messages on the mailbox
 * FIFO.
 *
 * Returns Number of received data if successful, otherwise error code.
 *
 * @pFile: file handler
 *
 * @pBuf: message buffer in user space. Used for returning the mailbox message.
 *
 * @len: length of the message buffer
 *
 * @pOffset: buffer offset
 */
static ssize_t mbox_read(
	struct file *pFile,
	char *pBuf,
	size_t len,
	loff_t *pOffset
)
{
	int id;                                     /* id */
	int res;                                    /* result */
	struct circ_buf *pCirc;                     /* reference to structure of circular buffers */
	struct MBOX_DEV_T *pMboxDev = NULL;         /* mailbox device */

	id = MINOR(pFile->f_inode->i_rdev);
	if (id == 0)
		return -EACCES;

	res = mbox_dev_get(&pMboxDev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_notice("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}
	if (pMboxDev->pInfo == NULL) {
		pr_notice("mbox: Invalid mailbox device %d selected.\n", id);
		return -EACCES;
	}

	/* check if the device is open */
	if (!pMboxDev->pid) {
		pr_notice("mbox: device is not open.\n");
		return -EACCES;
	}

	/* check the buffer reference and the buffer length */
	if (len < MBOX_MSG_SIZE || (pBuf == NULL)) {
		pr_notice("mbox: buffer reference for reading is invalid.\n");
		return -EINVAL;
	}

	/* set the circular buffer reference to reduce the dereference operations */
	pCirc = &(pMboxDev->pInfo->dataRd);

	/* set module to sleep until there is an entry on the mailbox fifo */
	res = wait_event_interruptible(pMboxDev->pInfo->queueRxData, (
			CIRC_CNT(pCirc->head, pCirc->tail, MBOX_FIFO_MAX) > 0));
	if (res)
		return (ssize_t) res;

	/* return the data from the circular buffer */
	res = copy_to_user(pBuf, &pCirc->buf[MBOX_CIRC_DATA_IDX_GET(pCirc->tail)], MBOX_MSG_SIZE);

	/* confirm the data by increasing the tail */
	pCirc->tail++;

	return (!res) ? (MBOX_MSG_SIZE) : (-EMSGSIZE);
}

/**
 * This function writes a mailbox message to the foreign core. If the mailbox
 * is busy, this function will block up to 500ms for repeating. It is possible
 * to configure this waiting time by ioctl.
 *
 * Returns number of written bytes if successful, otherwise an error code.
 *
 * @pFile: file handler
 *
 * @pBuf: message buffer in user space containing the writable data
 *
 * @len: length of the message buffer
 *
 * @pOffset: buffer offset
 */
static ssize_t mbox_write(
	struct file *pFile,
	const char *pBuf,
	size_t len,
	loff_t *pOffset
)
{
	int id;                                     /* id */
	int res;                                    /* result */
	struct MBOX_DEV_T *pMboxDev = NULL;         /* mailbox device */
	uint32_t *pData = NULL;                     /* mailbox data */
	struct timespec mboxTimeout;                /* mailbox timeout */
	struct timespec mboxTime;                   /* elapsed mailbox time */
	int resMbox;                                /* mailbox result */

	id = MINOR(pFile->f_inode->i_rdev);
	if (id == 0)
		return -EACCES;

	/* validate the write arguments */
	if ((pBuf == NULL) || (len > MBOX_MSG_SIZE))
		return -EINVAL;

	res = mbox_dev_get(&pMboxDev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_notice("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}
	if (pMboxDev->pInfo == NULL) {
		pr_notice("mbox: Invalid mailbox device %d selected.\n", id);
		return -EACCES;
	}

	/* check if the device is open */
	if (!pMboxDev->pid) {
		pr_notice("mbox: device is not open.\n");
		return -EACCES;
	}

	/* lock the write mutex */
	mutex_lock(&pMboxDev->pInfo->mtxWrite);

	/* reset the data buffer */
	pData = pMboxDev->pInfo->pDataWr;
	memset(pData, 0, MBOX_MSG_SIZE);

	/* Copy the data from user space. */
	if (copy_from_user(pData, pBuf, len)) {
		pr_notice("mbox: Unable to write user data.\n");
		mutex_unlock(&pMboxDev->pInfo->mtxWrite);
		return -EACCES;
	}

	/* send the mailbox message */
	/* get the current time stamp */
	getnstimeofday(&mboxTimeout);
	do {
		/* send the mailbox message */
		resMbox = pl320_ipc_transmit(pData);
		if (resMbox == 0) {
			/* sending succeed */
			mutex_unlock(&pMboxDev->pInfo->mtxWrite);
			return (ssize_t) len;
		} else if (-EBUSY != resMbox) {
			/* quit immediately in case of non busy error */
			mutex_unlock(&pMboxDev->pInfo->mtxWrite);
			return (ssize_t) resMbox;
		}

		/* busy error - retry until timeout */
		getnstimeofday(&mboxTime);
	} while ((TIME_MS_GET(mboxTimeout) + pMboxDev->pInfo->timeoutMs) > TIME_MS_GET(mboxTime));

	/* sending mailbox messages causes an error */
	mutex_unlock(&pMboxDev->pInfo->mtxWrite);

	return -ETIMEDOUT;
}

/**
 * This function destroys a mailbox sub-device and frees allocated
 * memory.
 *
 * @pInfo: private data of sub-device
 */
static void mbox_destroy(
	struct MBOX_DEV_SUB_INFO_T *pInfo
)
{
	if (pInfo == NULL)
		return;

	/* destroy the mutex */
	mutex_destroy(&pInfo->mtxWrite);

	/* free the circular buffer for receiving messages */
	kfree(pInfo->dataRd.buf);
	pInfo->dataRd.head = 0;
	pInfo->dataRd.tail = 0;
	pInfo->dataRd.buf = NULL;

	/* free the data buffer for writing */
	kfree(pInfo->pDataWr);

	/* free the device data */
	kfree(pInfo);
}

/**
 * This function removes and frees all parts of the referenced mailbox
 * device.
 *
 * @pInst: mailbox device instance
 */
static void mbox_dev_destroy(
	struct MBOX_DEV_T *pInst
)
{
	if (pInst == NULL)
		return;

	mbox_destroy(pInst->pInfo);

	if (pInst->pCharDev)
		cdev_del(pInst->pCharDev);

	if (pInst->pSysDev)
		device_destroy(pMbox->pCharClass, MKDEV(pMbox->major, pInst->minor));

	kfree(pInst);
}

/**
 * This notifier is called, after a new mailbox message has been received.
 * If the event ID related mailbox sub-device has been created by ioctl, the
 * message is passed to a circle buffer and provided to the read function of
 * the related sub-device.
 *
 * Returns NOTIFY_STOP if the notifier was for the device,
 * otherwise NOTIFY_DONE
 *
 * @pNb: notifier block
 *
 * @idEvent: mailbox event ID on the message
 *
 * @pData: mailbox data
 */
static int mbox_notifier(
	struct notifier_block *pNb,
	unsigned long idEvent,
	void *pData
)
{
	struct MBOX_DEV_T *pMboxDev = NULL;         /* mailbox device */
	struct circ_buf *pCirc;                     /* reference to structure of circular buffers */
	uint8_t *pCircData;                         /* circular buffer */
	int res;                                    /* result */

	/* get event ID corresponding mailbox sub device */
	res = mbox_dev_get(&pMboxDev, MBOX_DEV_GET_BY_EVENT_ID, (int) idEvent);
	if (res)
		return NOTIFY_DONE;

	/* verify the device data */
	if (pMboxDev->pInfo == NULL)
		return NOTIFY_STOP;

	/* set the circular buffer reference to reduce the dereference operations */
	pCirc = &(pMboxDev->pInfo->dataRd);

	/* check the free space on the circular receive buffer */
	if (CIRC_SPACE(pCirc->head, pCirc->tail, MBOX_FIFO_MAX) == 0)
		return NOTIFY_STOP;

	/* reference to the first byte on the circular buffer for adding a new message */
	pCircData = (uint8_t *) (&pCirc->buf[MBOX_CIRC_DATA_IDX_GET(pCirc->head)]);

	/* idEvent is the first element on mailbox data */
	memcpy(pCircData, &idEvent, MBOX_DATA_SIZE);

	/* increase buffer reference to second element */
	pCircData += MBOX_DATA_SIZE;

	/* copy the remaining part of the mailbox data */
	memcpy(pCircData, pData, MBOX_MSG_SIZE - MBOX_DATA_SIZE);

	/* announce the new data on the circular buffer by increasing the head */
	pCirc->head++;

	/* wake up the read process */
	wake_up_interruptible(&pMboxDev->pInfo->queueRxData);

	return NOTIFY_STOP;
}

/* mailbox notifier */
static struct notifier_block mbox_nb = {
	.notifier_call = mbox_notifier,
};

/**
 * This function allocates, creates and initializes a mailbox device.
 *
 * Returns 0 if successful, otherwise an error code.
 *
 * @pName: device name
 *
 * @pInfo: Private data of the sub-device. This argument is NULL for the main device
 * (/dev/mbox).
 */
static int mbox_dev_create(
	char *pName,
	struct MBOX_DEV_SUB_INFO_T *pInfo
)
{
	struct MBOX_DEV_T *pInstNew;                /* new instance */
	struct MBOX_DEV_T **pInstRef;               /* instance reference */
	int res = 0;                                /* result */

	if (pMbox == NULL) {
		pr_notice("mbox: Basic mailbox module doesn't exists.\n");
		return -EFAULT;
	}

	/* allocate the device structure */
	pInstNew = kzalloc(sizeof(*pInstNew), GFP_ATOMIC);
	if (!pInstNew)
		return -ENOMEM;

	/* initialize the content of the structure */
	pInstNew->minor = pMbox->listEntries;
	pInstNew->pInfo = pInfo;

	/* allocate the char device of the base class */
	pInstNew->pCharDev = cdev_alloc();
	if (!pInstNew->pCharDev) {
		pr_notice("mbox: Unable to allocate basic char device.\n");
		mbox_dev_destroy(pInstNew);
		return -EFAULT;
	}

	/* initialize and add the base class */
	cdev_init(pInstNew->pCharDev, &mbox_fops);
	pInstNew->pCharDev->owner = THIS_MODULE;
	res = cdev_add(pInstNew->pCharDev, MKDEV(pMbox->major, pInstNew->minor), 1);
	if (res) {
		pr_notice("mbox: Failed to add base mailbox char device.\n");
		mbox_dev_destroy(pInstNew);
		return res;
	}

	/* create the sys device */
	pInstNew->pSysDev = device_create(pMbox->pCharClass, NULL, MKDEV(pMbox->major, pInstNew->minor), NULL, pName);
	if (!pInstNew->pSysDev) {
		pr_notice("mbox: Device creation %s failed.\n", pName);
		mbox_dev_destroy(pInstNew);
		return -EFAULT;
	}

	/* add new instance */
	for (pInstRef = &pMbox->pListMboxDev; *pInstRef; pInstRef = &(*pInstRef)->pNext)
		;
	*pInstRef = pInstNew;

	/* count the elements on the list */
	pMbox->listEntries++;

	pr_info("mbox: Created device %s.\n", pName);

	return res;
}

/**
 * This function creates and initializes a new mailbox sub-device.
 *
 * Returns 0 if successful,
 *         1 if the device already exists
 *         otherwise error code.
 *
 * @eventId: event ID of the new sub-device
 */
static int mbox_create(
	uint32_t eventId
)
{
	char buffer[MBOX_DEV_FILE_SIZE];            /* buffer for device name */
	struct MBOX_DEV_SUB_INFO_T *pInfo;          /* private data of sub-device */
	int res;                                    /* result */

	/* check if the event ID is already used */
	res = mbox_dev_get(NULL, MBOX_DEV_GET_BY_EVENT_ID, (int) eventId);
	if (!res) {
		pr_notice("mbox: The mailbox event ID %u is already in use.\n", eventId);
		return 1;
	}

	/* allocate the device structure */
	pInfo = kzalloc(sizeof(*pInfo), GFP_ATOMIC);
	if (!pInfo)
		return -ENOMEM;

	/* initialize the device structure */
	pInfo->pDataWr = kzalloc(MBOX_MSG_SIZE, GFP_ATOMIC);
	if (!pInfo->pDataWr) {
		mbox_destroy(pInfo);
		return -ENOMEM;
	}
	pInfo->timeoutMs = MBOX_TIMEOUT_MS;
	pInfo->eventId = eventId;

	/* initialize the circular buffer for receiving messages */
	pInfo->dataRd.head = 0;
	pInfo->dataRd.tail = 0;
	pInfo->dataRd.buf = kzalloc(MBOX_MSG_SIZE * MBOX_FIFO_MAX, GFP_ATOMIC);
	if (!pInfo->dataRd.buf) {
		mbox_destroy(pInfo);
		return -ENOMEM;
	}

	/* initialize the mutex */
	mutex_init(&pInfo->mtxWrite);

	 /* initialize the wait queue for the channel */
	init_waitqueue_head(&(pInfo->queueRxData));

	/* register the new sub-device on the list of devices */
	memset(buffer, 0, MBOX_DEV_FILE_SIZE);
	sprintf(buffer, MBOX_DEV_FILE, eventId);
	res = mbox_dev_create(buffer, pInfo);
	if (res) {
		pr_notice("mbox: Creation of ident structure failed.\n");
		mbox_destroy(pInfo);
		return res;
	}

	return res;
}

/**
 * This function handles the io control commands of the basic mailbox device.
 *
 * Returns 0 if successful, otherwise an error code.
 *
 * @pFile: file handler
 *
 * @cmd: io control command
 *
 * @arg: user data argument
 */
static long mbox_ioctl(
	struct file *pFile,
	unsigned int cmd,
	unsigned long arg
)
{
	int res = 0;                                /* result */
	int id;                                     /* id */
	struct MBOX_DEV_T *pMboxDev = NULL;         /* mailbox device */

	id = MINOR(pFile->f_inode->i_rdev);

	res = mbox_dev_get(&pMboxDev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_notice("mbox: Minor device %d is unknown.\n", id);
		return res;
	}

	/* check the PID of the write process */
	if (!pMboxDev->pid) {
		pr_notice("mbox: device is not open.\n");
		return -EACCES;
	}

	switch (cmd) {
	case MBOX_IOCTL_CREATE:
		/* Creating of new mailbox sub-device is only allowed by the master device. Its minor ID is 0. */
		if (id != 0)
			return -EACCES;

		/* Creating of new mailbox sub-device is only allowed by the process who opens the master device. */
		if (current->pid != pMboxDev->pid) {
			pr_notice("mbox: ioctl access by invalid process.\n");
			return -EACCES;
		}

		res = mbox_create((uint32_t __user) arg);
		break;

	case MBOX_IOCTL_TIMEOUT_SET:
		/* Setting the timeout is only allowed on mailbox sub-devices. */
		if ((id == 0) || (pMboxDev->pInfo == NULL))
			return -EACCES;

		/* set the mailbox send timeout */
		pMboxDev->pInfo->timeoutMs =  *(unsigned int __user *) arg;
		break;

	case MBOX_IOCTL_VER_GET:
		res = put_user(((MBOX_VER_MAJOR << 16) | MBOX_VER_MINOR), (unsigned int __user *) arg);
		break;

	default:
		res = -EPERM;
	}

	return res;
}

/* file operations for this device */
static const struct file_operations mbox_fops = {
	.open = mbox_open,
	.release = mbox_release,
	.read = mbox_read,
	.write = mbox_write,
	.unlocked_ioctl = mbox_ioctl,
};

/**
 * This function cleans up the complete mailbox module.
 */
static void mbox_cleanup(
	void
)
{
	struct MBOX_DEV_T *pIdx;                    /* mailbox device index */

	/* unregister the notifier */
	pl320_ipc_unregister_notifier(&mbox_nb);

	if (pMbox) {
		/* destroy all sub-devices */
		for (pIdx = pMbox->pListMboxDev; pIdx; pIdx = pIdx->pNext)
			mbox_dev_destroy(pIdx);

		/*  destroy the class */
		if (pMbox->pCharClass)
			class_destroy(pMbox->pCharClass);

		/* Unregister region */
		if (pMbox->dev)
			unregister_chrdev_region(pMbox->dev, MBOX_DEVS_MAX + 1);

		kfree(pMbox);
	}

	pr_info("mbox: Mailbox module cleaned up.\n");
}

/**
 * This function initialize the mailbox module on Linux by creating the master
 * device and setup the PL320 driver.
 *
 * Returns 0 if successful, otherwise error an code.
 */
static int __init mbox_init(
	void
)
{
	int res;                                    /* result */

	/* allocate the device structure */
	pMbox = kzalloc(sizeof(*pMbox), GFP_ATOMIC);
	if (!pMbox)
		return -ENOMEM;

	/* Alloc char dev regions. MBOX_DEVS_MAX doesn't include the base class */
	res = alloc_chrdev_region(&pMbox->dev, 0, MBOX_DEVS_MAX + 1, MBOX_DEV_NAME);
	if (res) {
		pr_notice("mbox: Failed to allocate char devices.\n");
		mbox_cleanup();
		return res;
	}

	pMbox->major = MAJOR(pMbox->dev);

	/* Create class in sysfs */
	pMbox->pCharClass = class_create(THIS_MODULE, MBOX_DEV_NAME);
	if (!pMbox->pCharClass) {
		pr_notice("mbox: Failed to create mailbox class.\n");
		mbox_cleanup();
		return -EFAULT;
	}

	/* create the device */
	res = mbox_dev_create(MBOX_DEV_NAME, (struct MBOX_DEV_SUB_INFO_T *) NULL);
	if (res) {
		pr_notice("mbox: failed create device %s\n", MBOX_DEV_NAME);
		mbox_cleanup();
		return res;
	}

	/* initialize the pl320 driver */
	res = pl320_ipc_register_notifier(&mbox_nb);
	if (res) {
		pr_notice("mbox: failed initialize pl320 driver\n");
		mbox_cleanup();
		return res;
	}

	pr_info("mbox: module version %u.%u initialized\n", MBOX_VER_MAJOR, MBOX_VER_MINOR);

	return 0;
}


module_init(mbox_init);
module_exit(mbox_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GOAL");
MODULE_DESCRIPTION("Mailbox driver");
