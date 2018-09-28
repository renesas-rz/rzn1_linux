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
 * (C) Copyright 2018-22 Renesas Electronics Corporation
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
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pl320-ipc.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mailbox.h"

/* not editable defines */
#define MBOX_VER_MAJOR	0	/* major version of mailbox module */
#define MBOX_VER_MINOR	3	/* minor version of mailbox module */
/* complete size of a single mailbox message */
#define MBOX_MSG_SIZE	(MBOX_DATA_CNT * MBOX_DATA_SIZE)

/* Local structs/enums */
enum MBOX_DEV_GET_T {
	MBOX_DEV_GET_BY_EVENT_ID = 0,	/* look for device with given event ID */
	MBOX_DEV_GET_BY_MINOR_ID,	/* look for device with given minor ID */
};

struct mbox_dev_sub_info_t {
	u32 event_id;	/* event ID */

	/* writing section */
	u32 *data_wr;		/* data buffer for writing */
	struct mutex mtx_wr;	/* mutex for writing */

	/* reading section */
	struct circ_buf data_rd;		/* circular buffer of received messages */
	wait_queue_head_t queue_rx_data;	/* wait queue for received messages */
	unsigned int timeout_ms;		/* mailbox waiting time */
};

struct mbox_dev_t {
	struct mbox_dev_t *next;		/* next element */
	int minor;				/* minor number */
	struct mbox_dev_sub_info_t *info;	/* private data of sub-device */
	struct cdev *char_dev;			/* character device */
	struct device *sys_dev;			/* sysfs device reference */
	pid_t pid;				/* PID of the user space process */
};

struct mbox_t {
	dev_t dev;                              /* mailbox device */
	int major;                              /* major number */
	struct class *char_class;               /* mailbox class */
	struct mbox_dev_t *list_mbox_dev;       /* mailbox devices */
	int list_entries;                       /* number of elements in the list */
};

/* get the time from a timer handle in ms */
#define TIME_MS_GET(hdl_time) \
	((uint64_t)((hdl_time.tv_sec * 1000) + (hdl_time.tv_nsec / 1000000)))

/* get the data index on the circular buffer - consider wrap around */
#define MBOX_CIRC_DATA_IDX_GET(x) (((x) % MBOX_FIFO_MAX) * MBOX_MSG_SIZE)

static struct mbox_t *mbox;
static const struct file_operations mbox_fops;  /* forward declaration */

/**
 * This function iterates the list of existing mailbox devices, looking up for
 * the given ID. The ID is either an event ID or a minor ID.
 * The match is returned if a reference is set.
 *
 * Return 0 if successful, otherwise an error code.
 *
 * @mbox_dev: mailbox device reference
 *
 * @type: Kind of type the id belongs to. This is
 *            the minor ID (MBOX_DEV_GET_BY_MINOR_ID) or
 *            the event ID (MBOX_DEV_GET_BY_EVENT_ID)
 *
 * @id: ID the list of mailbox devices is looked for.
 */
static int mbox_dev_get(struct mbox_dev_t **mbox_dev,
			enum MBOX_DEV_GET_T type,
			int id)
{
	struct mbox_dev_t *idx_dev;	/* device index */

	switch (type) {
	/* get device based on the event ID */
	case MBOX_DEV_GET_BY_EVENT_ID:
		for (idx_dev = mbox->list_mbox_dev; idx_dev != NULL; idx_dev = idx_dev->next) {
			if (idx_dev->info) {
				if ((uint32_t)id == idx_dev->info->event_id) {
					if (mbox_dev)
						*mbox_dev = idx_dev;
					return 0;
				}
			}
		}
		break;

	/* get device based on the minor ID */
	case MBOX_DEV_GET_BY_MINOR_ID:
		for (idx_dev = mbox->list_mbox_dev; idx_dev != NULL; idx_dev = idx_dev->next) {
			if (id == idx_dev->minor) {
				if (mbox_dev)
					*mbox_dev = idx_dev;
				return 0;
			}
		}
		break;

	default:
		pr_err("mbox: get type %u is invalid.\n", type);
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
 * @inode: pointer to inode
 *
 * @file: pointer to file
 */
static int mbox_open(struct inode *inode, struct file *file)
{
	struct mbox_dev_t *mbox_dev = NULL;         /* mailbox device */
	int res;                                    /* result */
	int id;                                     /* minor id */

	id = MINOR(file->f_inode->i_rdev);
	res = mbox_dev_get(&mbox_dev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_err("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}

	/* check if the file is already open */
	if (mbox_dev->pid)
		return -EACCES;

	/* save the pid */
	mbox_dev->pid = current->pid;

	return 0;
}

/**
 * This function releases the mailbox char device.
 *
 * Return 0 if successful, otherwise an error code.
 *
 * @inode: pointer to inode
 *
 * @file: pointer to file
 */
static int mbox_release(struct inode *inode, struct file *file)
{
	struct mbox_dev_t *mbox_dev = NULL;         /* mailbox device */
	int res;                                    /* result */
	int id;                                     /* minor id */

	id = MINOR(file->f_inode->i_rdev);
	res = mbox_dev_get(&mbox_dev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_err("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}

	mbox_dev->pid = 0;

	return 0;
}

/**
 * This function provides a blocking reading of new messages on the mailbox
 * FIFO.
 *
 * Returns Number of received data if successful, otherwise error code.
 *
 * @file: file handler
 *
 * @buf: message buffer in user space. Used for returning the mailbox message.
 *
 * @len: length of the message buffer
 *
 * @offset: buffer offset
 */
static ssize_t mbox_read(struct file *file, char *buf, size_t len, loff_t *offset)
{
	struct mbox_dev_t *mbox_dev = NULL;	/* mailbox device */
	struct circ_buf *circ;			/* reference to structure of circular buffers */
	int res;				/* result */
	int id;					/* id */

	id = MINOR(file->f_inode->i_rdev);
	if (id == 0)
		return -EACCES;

	res = mbox_dev_get(&mbox_dev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_err("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}
	if (!mbox_dev->info) {
		pr_err("mbox: Invalid mailbox device %d selected.\n", id);
		return -EACCES;
	}

	/* check if the device is open */
	if (!mbox_dev->pid) {
		pr_err("mbox: device is not open.\n");
		return -EACCES;
	}

	/* check the buffer reference and the buffer length */
	if (len < MBOX_MSG_SIZE || !buf) {
		pr_err("mbox: buffer reference for reading is invalid.\n");
		return -EINVAL;
	}

	/* set the circular buffer reference to reduce the dereference operations */
	circ = &mbox_dev->info->data_rd;

	/* set module to sleep until there is an entry on the mailbox fifo */
	res = wait_event_interruptible(mbox_dev->info->queue_rx_data,
				       (CIRC_CNT(circ->head, circ->tail, MBOX_FIFO_MAX) > 0));
	if (res)
		return (ssize_t)res;

	/* return the data from the circular buffer */
	res = copy_to_user(buf, &circ->buf[MBOX_CIRC_DATA_IDX_GET(circ->tail)], MBOX_MSG_SIZE);

	/* confirm the data by increasing the tail */
	circ->tail++;

	return (!res) ? (MBOX_MSG_SIZE) : (-EMSGSIZE);
}

/**
 * This function writes a mailbox message to the foreign core. If the mailbox
 * is busy, this function will block up to 500ms for repeating. It is possible
 * to configure this waiting time by ioctl.
 *
 * Returns number of written bytes if successful, otherwise an error code.
 *
 * @file: file handler
 *
 * @buf: message buffer in user space containing the writable data
 *
 * @len: length of the message buffer
 *
 * @offset: buffer offset
 */
static ssize_t mbox_write(struct file *file, const char *buf, size_t len, loff_t *offset)
{
	struct mbox_dev_t *mbox_dev = NULL;	/* mailbox device */
	struct timespec64 mboxTimeout;		/* mailbox timeout */
	struct timespec64 mboxTime;		/* elapsed mailbox time */
	u32 *data = NULL;			/* mailbox data */
	int resMbox;				/* mailbox result */
	int res;				/* result */
	int id;					/* id */

	id = MINOR(file->f_inode->i_rdev);
	if (id == 0)
		return -EACCES;

	/* validate the write arguments */
	if (!buf || len > MBOX_MSG_SIZE)
		return -EINVAL;

	res = mbox_dev_get(&mbox_dev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (res) {
		pr_err("mbox: Minor device %d is unknown.\n", id);
		return -ENODEV;
	}
	if (!mbox_dev->info) {
		pr_err("mbox: Invalid mailbox device %d selected.\n", id);
		return -EACCES;
	}

	/* check if the device is open */
	if (!mbox_dev->pid) {
		pr_err("mbox: device is not open.\n");
		return -EACCES;
	}

	/* lock the write mutex */
	mutex_lock(&mbox_dev->info->mtx_wr);

	/* reset the data buffer */
	data = mbox_dev->info->data_wr;
	memset(data, 0, MBOX_MSG_SIZE);

	/* Copy the data from user space. */
	if (copy_from_user(data, buf, len)) {
		pr_err("mbox: Unable to write user data.\n");
		mutex_unlock(&mbox_dev->info->mtx_wr);
		return -EACCES;
	}

	/* send the mailbox message */
	/* get the current time stamp */
	ktime_get_ts64(&mboxTimeout);
	do {
		/* send the mailbox message */
		resMbox = pl320_ipc_transmit(data);
		if (resMbox == 0) {
			/* sending succeed */
			mutex_unlock(&mbox_dev->info->mtx_wr);
			return (ssize_t)len;
		} else if (-EBUSY != resMbox) {
			/* quit immediately in case of non busy error */
			mutex_unlock(&mbox_dev->info->mtx_wr);
			return (ssize_t)resMbox;
		}

		/* busy error - retry until timeout */
		ktime_get_ts64(&mboxTime);
	} while ((TIME_MS_GET(mboxTimeout) + mbox_dev->info->timeout_ms) > TIME_MS_GET(mboxTime));

	/* sending mailbox messages causes an error */
	mutex_unlock(&mbox_dev->info->mtx_wr);

	return -ETIMEDOUT;
}

/**
 * This function destroys a mailbox sub-device and frees allocated
 * memory.
 *
 * @info: private data of sub-device
 */
static void mbox_destroy(struct mbox_dev_sub_info_t *info)
{
	if (!info)
		return;

	/* destroy the mutex */
	mutex_destroy(&info->mtx_wr);

	/* free the circular buffer for receiving messages */
	kfree(info->data_rd.buf);
	info->data_rd.head = 0;
	info->data_rd.tail = 0;
	info->data_rd.buf = NULL;

	/* free the data buffer for writing */
	kfree(info->data_wr);

	/* free the device data */
	kfree(info);
}

/**
 * This function removes and frees all parts of the referenced mailbox
 * device.
 *
 * @inst: mailbox device instance
 */
static void mbox_dev_destroy(struct mbox_dev_t *inst)
{
	if (!inst)
		return;

	mbox_destroy(inst->info);

	if (inst->char_dev)
		cdev_del(inst->char_dev);

	if (inst->sys_dev)
		device_destroy(mbox->char_class, MKDEV(mbox->major, inst->minor));

	kfree(inst);
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
 * @nb: notifier block
 *
 * @id_event: mailbox event ID on the message
 *
 * @data: mailbox data
 */
static int mbox_notifier(struct notifier_block *nb, unsigned long id_event, void *data)
{
	struct mbox_dev_t *mbox_dev = NULL;	/* mailbox device */
	struct circ_buf *circ;			/* reference to structure of circular buffers */
	u8 *circ_data;				/* circular buffer */
	int res;				/* result */

	/* get event ID corresponding mailbox sub device */
	res = mbox_dev_get(&mbox_dev, MBOX_DEV_GET_BY_EVENT_ID, (int)id_event);
	if (res)
		return NOTIFY_DONE;

	/* verify the device data */
	if (!mbox_dev->info)
		return NOTIFY_STOP;

	/* set the circular buffer reference to reduce the dereference operations */
	circ = &mbox_dev->info->data_rd;

	/* check the free space on the circular receive buffer */
	if (CIRC_SPACE(circ->head, circ->tail, MBOX_FIFO_MAX) == 0)
		return NOTIFY_STOP;

	/* reference to the first byte on the circular buffer for adding a new message */
	circ_data = (uint8_t *)(&circ->buf[MBOX_CIRC_DATA_IDX_GET(circ->head)]);

	/* id_event is the first element on mailbox data */
	memcpy(circ_data, &id_event, MBOX_DATA_SIZE);

	/* increase buffer reference to second element */
	circ_data += MBOX_DATA_SIZE;

	/* copy the remaining part of the mailbox data */
	memcpy(circ_data, data, MBOX_MSG_SIZE - MBOX_DATA_SIZE);

	/* announce the new data on the circular buffer by increasing the head */
	circ->head++;

	/* wake up the read process */
	wake_up_interruptible(&mbox_dev->info->queue_rx_data);

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
 * @name: device name
 *
 * @info: Private data of the sub-device. This argument is NULL for the main device
 * (/dev/mbox).
 */
static int mbox_dev_create(char *name, struct mbox_dev_sub_info_t *info)
{
	struct mbox_dev_t *inst_new;		/* new instance */
	struct mbox_dev_t **inst_ref;		/* instance reference */
	int ret;

	if (!mbox) {
		pr_err("mbox: Basic mailbox module doesn't exists.\n");
		return -EFAULT;
	}

	/* allocate the device structure */
	inst_new = kzalloc(sizeof(*inst_new), GFP_ATOMIC);
	if (!inst_new)
		return -ENOMEM;

	/* initialize the content of the structure */
	inst_new->minor = mbox->list_entries;
	inst_new->info = info;

	/* allocate the char device of the base class */
	inst_new->char_dev = cdev_alloc();
	if (!inst_new->char_dev) {
		pr_err("mbox: Unable to allocate basic char device.\n");
		ret = -EFAULT;
		goto done;
	}

	/* initialize and add the base class */
	cdev_init(inst_new->char_dev, &mbox_fops);
	inst_new->char_dev->owner = THIS_MODULE;
	ret = cdev_add(inst_new->char_dev, MKDEV(mbox->major, inst_new->minor), 1);
	if (ret) {
		pr_err("mbox: Failed to add base mailbox char device.\n");
		goto done;
	}

	/* create the sys device */
	inst_new->sys_dev = device_create(mbox->char_class, NULL,
					  MKDEV(mbox->major, inst_new->minor),
					  NULL, name);
	if (!inst_new->sys_dev) {
		pr_err("mbox: Device creation %s failed.\n", name);
		ret = -EFAULT;
		goto done;
	}

	/* add new instance */
	for (inst_ref = &mbox->list_mbox_dev; *inst_ref; inst_ref = &(*inst_ref)->next)
		;
	*inst_ref = inst_new;

	/* count the elements on the list */
	mbox->list_entries++;

	pr_info("mbox: Created device %s.\n", name);

	return 0;

done:
	mbox_dev_destroy(inst_new);
	return ret;
}

/**
 * This function creates and initializes a new mailbox sub-device.
 *
 * Returns 0 if successful,
 *         1 if the device already exists
 *         otherwise error code.
 *
 * @event_id: event ID of the new sub-device
 */
static int mbox_create(uint32_t event_id)
{
	char buffer[MBOX_DEV_FILE_SIZE];	/* buffer for device name */
	struct mbox_dev_sub_info_t *info;	/* private data of sub-device */
	int ret;

	/* check if the event ID is already used */
	ret = mbox_dev_get(NULL, MBOX_DEV_GET_BY_EVENT_ID, (int)event_id);
	if (!ret) {
		pr_err("mbox: The mailbox event ID %u is already in use.\n", event_id);
		return 1;
	}

	/* allocate the device structure */
	info = kzalloc(sizeof(*info), GFP_ATOMIC);
	if (!info)
		return -ENOMEM;

	/* initialize the device structure */
	info->data_wr = kzalloc(MBOX_MSG_SIZE, GFP_ATOMIC);
	if (!info->data_wr) {
		ret = -ENOMEM;
		goto done;
	}
	info->timeout_ms = MBOX_TIMEOUT_MS;
	info->event_id = event_id;

	/* initialize the circular buffer for receiving messages */
	info->data_rd.head = 0;
	info->data_rd.tail = 0;
	info->data_rd.buf = kzalloc(MBOX_MSG_SIZE * MBOX_FIFO_MAX, GFP_ATOMIC);
	if (!info->data_rd.buf) {
		ret = -ENOMEM;
		goto done;
	}

	/* initialize the mutex */
	mutex_init(&info->mtx_wr);

	 /* initialize the wait queue for the channel */
	init_waitqueue_head(&info->queue_rx_data);

	/* register the new sub-device on the list of devices */
	memset(buffer, 0, MBOX_DEV_FILE_SIZE);
	sprintf(buffer, MBOX_DEV_FILE, event_id);
	ret = mbox_dev_create(buffer, info);
	if (ret) {
		pr_err("mbox: Creation of ident structure failed.\n");
		goto done;
	}

	return 0;

done:
	mbox_destroy(info);
	return ret;
}

/**
 * This function handles the io control commands of the basic mailbox device.
 *
 * Returns 0 if successful, otherwise an error code.
 *
 * @file: file handler
 *
 * @cmd: io control command
 *
 * @arg: user data argument
 */
static long mbox_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mbox_dev_t *mbox_dev = NULL;         /* mailbox device */
	int ret;
	int id;

	id = MINOR(file->f_inode->i_rdev);
	ret = mbox_dev_get(&mbox_dev, MBOX_DEV_GET_BY_MINOR_ID, id);
	if (ret) {
		pr_err("mbox: Minor device %d is unknown.\n", id);
		return ret;
	}

	/* check the PID of the write process */
	if (!mbox_dev->pid) {
		pr_err("mbox: device is not open.\n");
		return -EACCES;
	}

	switch (cmd) {
	case MBOX_IOCTL_CREATE:
		/*
		 * Creating of new mailbox sub-device is only allowed by the
		 * master device. Its minor ID is 0.
		 */
		if (id != 0)
			return -EACCES;

		/*
		 * Creating of new mailbox sub-device is only allowed by the
		 * process who opens the master device.
		 */
		if (current->pid != mbox_dev->pid) {
			pr_err("mbox: ioctl access by invalid process.\n");
			return -EACCES;
		}

		ret = mbox_create((uint32_t __user)arg);
		break;

	case MBOX_IOCTL_TIMEOUT_SET:
		/* Setting the timeout is only allowed on mailbox sub-devices. */
		if (id == 0 || !mbox_dev->info)
			return -EACCES;

		/* set the mailbox send timeout */
		mbox_dev->info->timeout_ms =  *(unsigned int __user *)arg;
		break;

	case MBOX_IOCTL_VER_GET:
		ret = put_user(((MBOX_VER_MAJOR << 16) | MBOX_VER_MINOR),
			       (unsigned int __user *)arg);
		break;

	default:
		ret = -EPERM;
	}

	return ret;
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
static void mbox_cleanup(void)
{
	struct mbox_dev_t *idx;	/* mailbox device index */

	/* unregister the notifier */
	pl320_ipc_unregister_notifier(&mbox_nb);

	if (mbox) {
		/* destroy all sub-devices */
		for (idx = mbox->list_mbox_dev; idx; idx = idx->next)
			mbox_dev_destroy(idx);

		/*  destroy the class */
		if (mbox->char_class)
			class_destroy(mbox->char_class);

		/* Unregister region */
		if (mbox->dev)
			unregister_chrdev_region(mbox->dev, MBOX_DEVS_MAX + 1);

		kfree(mbox);
	}

	pr_info("mbox: Mailbox module cleaned up.\n");
}

/**
 * This function initialize the mailbox module on Linux by creating the master
 * device and setup the PL320 driver.
 *
 * Returns 0 if successful, otherwise error an code.
 */
static int __init mbox_init(void)
{
	int ret;

	/* allocate the device structure */
	mbox = kzalloc(sizeof(*mbox), GFP_ATOMIC);
	if (!mbox)
		return -ENOMEM;

	/* Alloc char dev regions. MBOX_DEVS_MAX doesn't include the base class */
	ret = alloc_chrdev_region(&mbox->dev, 0, MBOX_DEVS_MAX + 1, MBOX_DEV_NAME);
	if (ret) {
		pr_err("mbox: Failed to allocate char devices.\n");
		goto done;
	}

	mbox->major = MAJOR(mbox->dev);

	/* Create class in sysfs */
	mbox->char_class = class_create(THIS_MODULE, MBOX_DEV_NAME);
	if (!mbox->char_class) {
		pr_err("mbox: Failed to create mailbox class.\n");
		ret = -EFAULT;
		goto done;
	}

	/* create the device */
	ret = mbox_dev_create(MBOX_DEV_NAME, (struct mbox_dev_sub_info_t *)NULL);
	if (ret) {
		pr_err("mbox: failed create device %s\n", MBOX_DEV_NAME);
		goto done;
	}

	/* initialize the pl320 driver */
	ret = pl320_ipc_register_notifier(&mbox_nb);
	if (ret) {
		pr_err("mbox: failed initialize pl320 driver\n");
		goto done;
	}

	pr_info("mbox: module version %u.%u initialized\n", MBOX_VER_MAJOR, MBOX_VER_MINOR);

	return 0;

done:
	mbox_cleanup();
	return ret;
}

module_init(mbox_init);
module_exit(mbox_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GOAL");
MODULE_DESCRIPTION("Mailbox driver");
