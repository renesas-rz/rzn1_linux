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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/of_address.h>
#include <linux/pl320-ipc.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "goal_target_ctc.h"
#include "goal_ctc_util.h"
#include "goal_ctc_kernel.h"


/****************************************************************************/
/* not editable defines */
/****************************************************************************/
/**< version */
#define CTC_KERNEL_VERSION_MAJOR 4              /**< major version of the kernel */
#define CTC_KERNEL_VERSION_MINOR 1              /**< minor version of the kernel */

#define CTC_WORK_QUEUE_NAME "CTC_FIFO_WQUEUE"   /**< Work queue */
#define CTC_KERNEL_VALIDATION_TIMEOUT_MS 3000   /**< timeout till validation error */


/****************************************************************************/
/* Makros */
/****************************************************************************/
#define CTC_GET_TIME_MS(hdlTime) ((hdlTime.tv_sec * 1000) + (hdlTime.tv_nsec / 1000000)) /**< read time in ms */


/****************************************************************************/
/* Local structs/enums */
/****************************************************************************/
/**< FIFO data entry for received data */
struct ctcDataEntry {
    uint8_t *pBuf;                              /**< pointer to data on shared memory */
    uint16_t size;                              /**< size of data */
};

/**< Device structure */
struct ctcDeviceData {
    struct cdev chrdev;                         /**< character device */
    int pureChanId;                             /**< pure channel ID */
    struct device *sysDev;                      /**< sysfs device */
    int idxWrite;                               /**< write index */
    int idxRead;                                /**< read index */
    DECLARE_KFIFO(fifoRead, struct ctcDataEntry *, CTC_FIFO_COUNT); /**< read fifo */
    pid_t pid;                                  /**< PID of the user space process */
    wait_queue_head_t ctcRxWait;                /**< wait queue for read fifo */
    struct mutex mtxCtcDev;                     /**< mutex for ctc device */
    GOAL_BOOL_T flgCyclic;                      /**< flag for cyclic data */
    GOAL_BOOL_T flgUserUsed;                    /**< flag for used by user space */
};

typedef struct {
    dev_t ctcDev;                               /**< core to core device */
    int ctcMajor;                               /**< core to core major number */
    struct class *ctcCharClass;                 /**< core to core char class */
    GOAL_CTC_UTIL_T util;                       /**< utensils */
    struct ctcDeviceData ctcDevices[GOAL_TGT_CTC_CHN_MAX]; /**< core to core devices */
    struct workqueue_struct *pFifo_workqueue;   /**< fifo queue for work */
    unsigned int ctcMboxTimeoutMs;              /**< mailbox waiting time */
    GOAL_CTC_PRECIS_MSG_T *pCtcMboxDataTx;      /**< transmit data for mailbox */
    GOAL_CTC_PRECIS_MSG_T *pCtcMboxDataRx;      /**< receive data for mailbox */
    struct task_struct *pIdStatusThread;        /**< thread ID of the status validation */
} GOAL_CTC_DEVICE_T;


/****************************************************************************/
/* Local prototypes */
/****************************************************************************/
static GOAL_STATUS_T goal_ctcMemCreate(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    unsigned int lenMin,                        /**< minimum length of memory */
    GOAL_CTC_CREATE_PRECIS_T *pCreatePrecis     /**< create precis information */
);

static GOAL_STATUS_T goal_ctcPureChnInit(
    void *pArg                                  /**< function argument */
);

static int goal_ctcStatusValidation(
    void *pArg                                  /**< function argument */
);

static GOAL_STATUS_T goal_ctcMboxMsgNew(
    uint32_t channelId,                         /**< pure channel ID */
    uint32_t offset,                            /**< data offset */
    uint16_t len                                /**< data length */
);

static int ctc_open(
    struct inode *pInode,                       /**< pointer to inode */
    struct file *pFile                          /**< pointer to file */
);

static int ctc_release(
    struct inode *pInode,                       /**< pointer to inode */
    struct file *pFile                          /**< pointer to file */
);

static ssize_t ctc_read(
    struct file *pFile,                         /**< file handler */
    char *pBuf,                                 /**< buffer */
    size_t len,                                 /**< length */
    loff_t *pOffset                             /**< offset */
);

static ssize_t ctc_write(
    struct file *pFile,                         /**< file handler */
    const char *pBuf,                           /**< buffer */
    size_t len,                                 /**< length */
    loff_t *pOffset                             /**< offset */
);

long ctc_ioctl(
    struct file *pFile,                         /**< file handler */
    unsigned int cmd,                           /**< io control command */
    unsigned long arg                           /**< pointer to user data as ulong */
);

static void ctc_cleanup(
    void
);

static int ctc_create_devices(
    void
);

static int ctc_init_mbox(
    void
);

static int ctc_notifier(
    struct notifier_block *pNb,                 /**< notifier block */
    unsigned long page,                         /**< first element */
    void *pData                                 /**< pointer to second element */
);

static GOAL_STATUS_T goal_ctcMboxTx(
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis           /**< precis message */
);


/****************************************************************************/
/* Local variables */
/****************************************************************************/
static GOAL_CTC_DEVICE_T *pCtcDevice = NULL;    /**< core to core device structure */
static uint8_t ctcVersion[GOAL_CTC_VERSION_SIZE] = {CTC_KERNEL_VERSION_MAJOR, CTC_KERNEL_VERSION_MINOR}; /**< core to core version */
static resource_size_t ctcOffsetMem = 0;        /**< offset in memory */

/**< File operations for char devices */
static struct file_operations ctc_fops =
{
    .open = ctc_open,                           /**< open operation */
    .read = ctc_read,                           /**< read operation */
    .write = ctc_write,                         /**< write operation */
    .release = ctc_release,                     /**< release operation */
    .unlocked_ioctl = ctc_ioctl,                /**< io control operation */
};

/**< Core to core notifier */
static struct notifier_block ctc_nb = {
    .notifier_call = ctc_notifier,              /**< IRQ notifier */
};


/****************************************************************************/
/** Initializes the core to core module
 *
 * This function initialize the core to core module on Linux. A requirement
 * next to the char devices is the mailbox driver.
 *
 * @return 0 if successful, otherwise error code
 */
static int __init ctc_init(
    void
)
{
    int res;                                    /* response */

    goal_logInfo("CTC: Start initialization of CTC module version %u.%u.\n", CTC_KERNEL_VERSION_MAJOR, CTC_KERNEL_VERSION_MINOR);

    /* allocate the device structure */
    pCtcDevice = kmalloc(sizeof(GOAL_CTC_DEVICE_T), GFP_ATOMIC);
    if (!pCtcDevice) {
        /* error during allocation */
        goal_logErr("Could not allocate memory for ctc device structure.\n");
        return -EFAULT;
    }
    /* clear the memory section */
    GOAL_MEMSET(pCtcDevice, 0, sizeof(GOAL_CTC_DEVICE_T));

    /* allocate the config structure */
    pCtcDevice->util.pConfig = kmalloc(sizeof(struct GOAL_CTC_CONFIG_T), GFP_ATOMIC);
    if (!pCtcDevice->util.pConfig) {
        /* error during allocation */
        goal_logErr("Could not allocate memory for ctc config structure.\n");
        return -EFAULT;
    }
    /* clear the memory section */
    GOAL_MEMSET(pCtcDevice->util.pConfig, 0, sizeof(struct GOAL_CTC_CONFIG_T));

    /* Alloc char dev regions */
    res = alloc_chrdev_region(&pCtcDevice->ctcDev, 0, GOAL_TGT_CTC_CHN_MAX, CTC_DEVICE_NAME);
    if (res) {
        goal_logErr("Failed to alloc char devs.\n");
        return -EFAULT;
    }
    pCtcDevice->ctcMajor = MAJOR(pCtcDevice->ctcDev);

    /* Create class in sysfs */
    pCtcDevice->ctcCharClass = class_create(THIS_MODULE, CTC_DEVICE_NAME);
    goal_logInfo("Created ctc class.\n");

    /* Create char devices */
    res = ctc_create_devices();
    if (res) {
        ctc_cleanup();
        return res;
    }

    /* Init mailbox and interrupt handler */
    res = ctc_init_mbox();
    if (res) {
        ctc_cleanup();
        return res;
    }

    /* Create the workqueue for signaling */
    pCtcDevice->pFifo_workqueue = create_workqueue(CTC_WORK_QUEUE_NAME);

    goal_logInfo("CTC module version %u.%u initialized.\n", CTC_KERNEL_VERSION_MAJOR, CTC_KERNEL_VERSION_MINOR);

    return 0;
}


/****************************************************************************/
/** Create the shared memory
 *
 * This function initialize the shared memory for linux usage. The device
 * tree node c2c_sram contains the physical start address and the length of
 * the whole shared memory. It is remaped to Kernel space, thus the module is
 * able to read received messages.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
static GOAL_STATUS_T goal_ctcMemCreate(
    struct GOAL_CTC_CONFIG_T *pConfig,          /**< configuration */
    unsigned int lenMin,                        /**< minimum length of memory */
    GOAL_CTC_CREATE_PRECIS_T *pCreatePrecis     /**< create precis information */
)
{
    struct device_node *pDp;                    /* device tree node for shared memory register */
    struct resource *pRes;                      /* resource */
    int ret;                                    /* return value */
    unsigned int ramSize;                       /* size of RAM */
    void *pVirtKernMem;                         /* data buffer as virtual kernel memory */

    /* create only the write section */
    *pCreatePrecis = GOAL_CTC_CREATE_PRECIS_WRITE;

    if (NULL != pConfig->pMem) {
        goal_logErr("Memory section is already defined.\n");
        return GOAL_ERROR;
    }

    pRes = (struct resource *) kmalloc(sizeof(struct resource), GFP_ATOMIC);
    if (!pRes) {
        /* Error */
        goal_logErr("Unable to allocate resource for shared memory initialization.\n");
        return GOAL_ERROR;
    }

    /* find the SRAM node */
    pDp = of_find_compatible_node(NULL, NULL, "mmio-sram");
    if (!pDp) {
        /* Error */
        goal_logErr("Unable to find node c2c_sram.\n");
        /* free the allocated memory */
        kfree(pRes);
        return GOAL_ERROR;
    }

    /* read the memory size */
    ret = of_address_to_resource(pDp, 0, pRes);
    if (0 > ret) {
        goal_logErr("Could not get address for node %s.\n", pDp->full_name);
        of_node_put(pDp);
        /* free the allocated memory */
        kfree(pRes);
        return GOAL_ERROR;
    }

    ramSize = resource_size(pRes);
    if (lenMin > ramSize) {
        goal_logErr("Shared memory is too small: 0x%X byte availible 0x%X byte needed.\n", ramSize, lenMin);
        of_node_put(pDp);
        /* free the allocated memory */
        kfree(pRes);
        return GOAL_ERROR;
    }

    ctcOffsetMem = pRes->start;

    if (!request_mem_region(pRes->start, resource_size(pRes), "ctc")) {
        goal_logErr("Unable to request memory region\n");
        of_node_put(pDp);
        /* free the allocated memory */
        kfree(pRes);
        return GOAL_ERROR;
    }

    /* map the SRAM */
    pVirtKernMem = of_iomap(pDp, 0);
    if (NULL == pVirtKernMem) {
        goal_logErr("Unable to remap 0x%X byte\n", ramSize);
        of_node_put(pDp);
        /* free the allocated memory */
        kfree(pRes);
        return GOAL_ERROR;
    }

    /* return the memory reference */
    pConfig->pMem = (uint8_t *) pVirtKernMem;
    pConfig->len = ramSize;

    of_node_put(pDp);

    /* free the allocated memory */
    kfree(pRes);

    return GOAL_OK;
}


/****************************************************************************/
/** Initializes the char devices for the different channels
 *
 * All devices are created here. The shared memory region of reading and
 * writing is initialized at the beginning to ensure available of the needed
 * RAM.
 * Right after, the individual char devices and sysfs devices are initialized.
 *
 * @return 0 if successful, otherwise error code
 */
static int ctc_create_devices(
    void
)
{
    GOAL_STATUS_T res;                          /* result */
    GOAL_CTC_CREATE_PRECIS_T flgCreatePrecis = GOAL_CTC_CREATE_PRECIS_NONE; /* flag for creating the precis section */

    /* create the target ctc config */
    pCtcDevice->util.pConfig->pureChnCnt = GOAL_TGT_CTC_CHN_MAX;
    pCtcDevice->util.pConfig->pageCnt = GOAL_TGT_CTC_PAGE_COUNT;
    pCtcDevice->util.pConfig->pageSize = GOAL_TGT_CTC_PAGE_SIZE;

    pCtcDevice->ctcMboxTimeoutMs = GOAL_TGT_CTC_MBOX_TIME_MS;
    /* initialize the shared memory */
    res = goal_ctcUtilMemInit(&(pCtcDevice->util), ctcVersion, &flgCreatePrecis, goal_ctcMemCreate);
    if (GOAL_RES_ERR(res)) {
        return -EFAULT;
    }

    /* initialize the pure channels */
    res = goal_ctcUtilPureChnInit(&(pCtcDevice->util), flgCreatePrecis, goal_ctcPureChnInit, NULL);
    if (GOAL_RES_ERR(res)) {
        return -EFAULT;
    }

    /* start the kernel thread for status validation */
    pCtcDevice->pIdStatusThread = kthread_create(goal_ctcStatusValidation, NULL, "goal_ctcStatusValidation");
    if (IS_ERR(pCtcDevice->pIdStatusThread)) {
        goal_logErr("Unable to start ctc validation thread.\n");
        return PTR_ERR(pCtcDevice->pIdStatusThread);
    }
    wake_up_process(pCtcDevice->pIdStatusThread);
    return 0;
}


/****************************************************************************/
/** Initialize the pure channel structure
 *
 * This function creates and initializes the structure of the  pure channels
 * for ctc usage.
 *
 * @retval GOAL_OK successful
 * @retval other failed
 */
static GOAL_STATUS_T goal_ctcPureChnInit(
    void *pArg                                  /**< function argument */
)
{
    int idx;                                    /* index */
    int retVal;                                 /* return value */
    char buffer[CTC_DEVICE_FILE_SIZE];          /* buffer for device name */

    UNUSEDARG(pArg);

    /* Init individual char devs and sysfs devices */
    for (idx = 0; idx < pCtcDevice->util.pConfig->pureChnCnt; idx++) {
        cdev_init(&(pCtcDevice->ctcDevices[idx].chrdev), &ctc_fops);
        pCtcDevice->ctcDevices[idx].chrdev.owner = THIS_MODULE;
        pCtcDevice->ctcDevices[idx].chrdev.ops = &ctc_fops;
        pCtcDevice->ctcDevices[idx].pureChanId = idx;
        retVal = cdev_add(&(pCtcDevice->ctcDevices[idx].chrdev), MKDEV(pCtcDevice->ctcMajor, idx), 1);
        if (retVal) {
            goal_logErr("Failed to alloc char devs.\n");
            return GOAL_ERROR;
        } else {
            goal_logInfo("Created device "CTC_DEVICE_FILE".\n", idx);
        }
        sprintf(buffer, CTC_DEVICE_FILE, idx);
        pCtcDevice->ctcDevices[idx].sysDev = device_create(pCtcDevice->ctcCharClass, NULL, MKDEV(pCtcDevice->ctcMajor, idx), NULL, buffer);

        /* Init the fifo for the mailbox messages */
        INIT_KFIFO(pCtcDevice->ctcDevices[idx].fifoRead);

        pCtcDevice->ctcDevices[idx].idxWrite = 0;
        pCtcDevice->ctcDevices[idx].idxRead = 0;
        pCtcDevice->ctcDevices[idx].pid = 0;
        pCtcDevice->ctcDevices[idx].flgCyclic = false;
        pCtcDevice->ctcDevices[idx].flgUserUsed = false;

        /* init the wait queue for the channel */
        init_waitqueue_head(&(pCtcDevice->ctcDevices[idx].ctcRxWait));

        /* init the mutex */
        mutex_init(&(pCtcDevice->ctcDevices[idx].mtxCtcDev));
    }
    return GOAL_OK;
}


/****************************************************************************/
/** Initializes the mailboxes incl. interrupts here
 *
 * The mailbox driver PL320 is used for the communication. This function
 * registers the notifier.
 *
 * @return 0 if successful, otherwise error code
 */
static int ctc_init_mbox(
    void
)
{
    /* allocate the mailbox buffers */
    pCtcDevice->pCtcMboxDataTx = (GOAL_CTC_PRECIS_MSG_T *) kmalloc(sizeof(GOAL_CTC_PRECIS_MSG_T), GFP_ATOMIC);
    if (!pCtcDevice->pCtcMboxDataTx) {
        /* Error */
        goal_logErr("Unable to allocate memory for transmitting mailbox buffer.\n");
        return -EFAULT;
    }
    pCtcDevice->pCtcMboxDataRx = (GOAL_CTC_PRECIS_MSG_T *) kmalloc(sizeof(GOAL_CTC_PRECIS_MSG_T), GFP_ATOMIC);
    if (!pCtcDevice->pCtcMboxDataRx) {
        /* Error */
        goal_logErr("Unable to allocate memory for receiving mailbox buffer.\n");
        return -EFAULT;
    }

    /* initialize the pl320 driver */
    pl320_ipc_register_notifier(&ctc_nb);
    return 0;
}


/****************************************************************************/
/** Status validation thread
 *
 * This thread validates the pure channel status every second until the
 * foreigner core has been found.
 *
 * @return 0 if successful, otherwise error code
 */
static int goal_ctcStatusValidation(
    void *pArg                                  /**< function argument */
)
{
    GOAL_STATUS_T res;                          /* result */
    struct timespec timeoutValidation;          /* validation timeout */
    struct timespec tsValidation;               /* elapsed validation time */

    /* get the current time stamp */
    getnstimeofday(&timeoutValidation);
    do {
        msleep(1);
        res = goal_ctcUtilPureStatus(&pCtcDevice->util, 0);
        if (GOAL_RES_OK(res)) {
            return 0;
        }
        getnstimeofday(&tsValidation);
    } while ((CTC_GET_TIME_MS(timeoutValidation) + CTC_KERNEL_VALIDATION_TIMEOUT_MS) > CTC_GET_TIME_MS(tsValidation));
    goal_logWarn("CTC validation expired.\n");
    return 0;
}


/****************************************************************************/
/** Core to Core notifier
 *
 * This notifier is called, after a new mailbox message received.
 *
 * @return 0 if successful, otherwise error code
 */
static int ctc_notifier(
    struct notifier_block *pNb,                 /**< notifier block */
    unsigned long page,                         /**< first element */
    void *pData                                 /**< pointer to second element */
)
{
    GOAL_STATUS_T res;                          /* result */
    GOAL_CTC_PRECIS_T *pPrecis = NULL;          /* precis reference */
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis;          /* pointer to precis message */
    uint32_t channelId = 0;                     /* pure channel ID */
    uint32_t offset = 0;                        /* data offset */
    uint16_t len = 0;                           /* data length */
    GOAL_CTC_PURE_FLG_T flgMsg = GOAL_CTC_PURE_FLG_INV; /* message flag */

    /* The precis message data is splitted into the notifier page and pData.
     * pData points the the second u32 element. */
    pMsgPrecis = (GOAL_CTC_PRECIS_MSG_T *) (((uint32_t *) pData) - 1);

    /* validate the pure status */
    res = goal_ctcUtilPureStatus(&pCtcDevice->util, 0);
    if (GOAL_RES_ERR(res)) {
        goal_logDbg("CTC is not ready.\n");
        return 0;
    }

    /* decode the mailbox message */
    goal_ctcUtilPrecisMsgGet(pMsgPrecis, &channelId, &offset, &len, NULL, &flgMsg);

    /* check if the channel is open */
    if (pCtcDevice->ctcDevices[channelId].pid) {
        switch (flgMsg) {
            case GOAL_CTC_PURE_FLG_REQ:
                /* mailbox message with a new request received */
                res = goal_ctcMboxMsgNew(channelId, offset, len);
                if (GOAL_RES_OK(res)) {
                    return 0;
                }
                goal_logErr("A new message could not be handled.\n");
            break;

            default:
                goal_logErr("Unknown ctc message ID.\n");
                break;
        }
    }

    /* if an error occurs, increase the read index to avoid a full buffer. */
    res = goal_ctcUtilPrecisGet(&pPrecis, pCtcDevice->util.pRead, channelId);
    if (GOAL_RES_ERR(res)) {
        goal_logErr("Unable to get read precis of pure channel %u.\n", channelId);
        return 0;
    }
    goal_ctcUtilFifoReadInc(pPrecis, (GOAL_be32toh(pPrecis->idxRd_be32) + 1) % CTC_FIFO_COUNT, GOAL_CTC_CHN_PURE_STATUS_NOT_CYCLIC);

    return 0;
}


/****************************************************************************/
/** Handle new Job
 *
 * This function is called, if a new job was received on the mailbox. The data
 * are stored to the mailbox fifo and the read process is waked up.
 *
 * @retval GOAL_OK success
 * @retval GOAL_ERROR failure
 */
static GOAL_STATUS_T goal_ctcMboxMsgNew(
    uint32_t channelId,                         /**< pure channel ID */
    uint32_t offset,                            /**< data offset */
    uint16_t len                                /**< data length */
)
{
    struct ctcDataEntry *pEntry;                /* fifo entry */
    struct kfifo *pFifo;                        /* dido of the channel */
    unsigned int ret;                           /* return value */

    if (pCtcDevice->util.pConfig->len <= offset) {
        goal_logErr("The received offset is false: %u, allowed %u\n", offset, pCtcDevice->util.pConfig->len);
        return GOAL_ERROR;
    }

    if (pCtcDevice->util.pConfig->pageSize * pCtcDevice->util.pConfig->pageCnt <= len) {
        goal_logErr("Invalid data length: %u byte, allowed %u byte\n", len, pCtcDevice->util.pConfig->pageSize * pCtcDevice->util.pConfig->pageCnt);
        return GOAL_ERROR;
    }

    /* alloc a new fifo entry */
    pEntry = kmalloc(sizeof(struct ctcDataEntry), GFP_ATOMIC);
    if (!pEntry) {
        /* error during allocation */
        goal_logErr("Could not allocate memory for fifo entry.\n");
        return GOAL_ERROR;
    }

    /* store the data on the fifo */
    pEntry->size = len;
    pEntry->pBuf = pCtcDevice->util.pConfig->pMem + offset;

    /* Get the according channel fifo */
    pFifo = (struct kfifo *) &(pCtcDevice->ctcDevices[channelId].fifoRead);

    if (!kfifo_is_full(pFifo)) {
        /* add entry */
        ret = kfifo_in(pFifo, &pEntry, 1);
        if (1 != ret) {
            goal_logErr("Could not get data from FIFO.\n");
            /* free the allocated memory */
            kfree(pEntry);
            return GOAL_ERROR;
        }

        /* wake up the read process */
        wake_up_interruptible(&(pCtcDevice->ctcDevices[channelId].ctcRxWait));
        return GOAL_OK;

    } else {
        /* Error handler */
        goal_logErr("CTC mailbox FIFO is full.\n");
    }

    /* free the allocated memory */
    kfree(pEntry);
    return GOAL_ERROR;
}


/****************************************************************************/
/** Deinitializes the core to core module
 *
 * This function cleanups the core to core module.
 */
static void ctc_cleanup(
    void
)
{
    int idx;                                    /* index */

    /* unregister the notifier */
    pl320_ipc_unregister_notifier(&ctc_nb);

    /* free the allocated mailbox buffer */
    kfree(pCtcDevice->pCtcMboxDataRx);
    kfree(pCtcDevice->pCtcMboxDataTx);

    /* Deinit individual char devs and devices */
    for (idx = 0; idx < pCtcDevice->util.pConfig->pureChnCnt; idx++) {
        cdev_del(&(pCtcDevice->ctcDevices[idx].chrdev));
        device_destroy(pCtcDevice->ctcCharClass, MKDEV(pCtcDevice->ctcMajor, idx));
        goal_logInfo("Removed device ctc_chan%u.\n", idx);
    }

    /* Unregister region */
    if (0 != pCtcDevice->ctcDev) {
        unregister_chrdev_region(pCtcDevice->ctcDev, pCtcDevice->util.pConfig->pureChnCnt);
    }

    /* Destroy class */
    if (pCtcDevice->ctcCharClass) {
        class_destroy(pCtcDevice->ctcCharClass);
        goal_logInfo("Removed ctc class.\n");
    }

    goal_logInfo("CTC module released.\n");
}


/****************************************************************************/
/** Opens a core to core char device for reading/ writing
 *
 * Open a char device for core to core usage. This is only possible, if the
 * nonblocking option is disabled.
 *
 * @return 0 if successful, otherwise error code
 */
static int ctc_open(
    struct inode *pInode,                       /**< pointer to inode */
    struct file *pFile                          /**< pointer to file */
)
{
    int pureChanId;                             /* pure channel ID */

    pureChanId = MINOR(pFile->f_inode->i_rdev);

    /* Check for non blocking option */
    if (O_NONBLOCK == (O_NONBLOCK & (pFile->f_flags))) {
        return -EACCES;
    }

    /* check if the file is already open */
    if (pCtcDevice->ctcDevices[pureChanId].pid) {
        return -EACCES;
    }

    /* save the pid */
    pCtcDevice->ctcDevices[pureChanId].pid = current->pid;

    return 0;
}


/****************************************************************************/
/** Closes a core to core char device
 *
 * This function closes a core to core char device. This is only possible by
 * the task, which opened it.
 *
 * @param pInode, pointer to inode
 * @param pFile, pointer to file
 *
 * @return 0 if successful, otherwise error code
 */
static int ctc_release(
    struct inode *pInode,                       /**< pointer to inode */
    struct file *pFile                          /**< pointer to file */
)
{
    int pureChanId;                             /* pure channel ID */

    pureChanId = MINOR(pFile->f_inode->i_rdev);

    /* remove the pid */
    pCtcDevice->ctcDevices[pureChanId].pid = 0;
    return 0;
}


/****************************************************************************/
/** Reads up to len bytes from a core to core char device
 *
 * This function provides a blocking reading of new messages on the mailbox
 * FIFO.
 *
 * @return data length if successful, otherwise error code or 0 if signal
 * occurs
 */
static ssize_t ctc_read(
    struct file *pFile,                         /**< file handler */
    char *pBuf,                                 /**< buffer */
    size_t len,                                 /**< length */
    loff_t *pOffset                             /**< offset */
)
{
    int pureChanId;                             /* pure channel ID */
    struct kfifo *pFifo;                        /* fifo of the channel */
    struct ctcDataEntry *pEntry;                /* selected fifo entry */
    size_t fifoDataLen;                         /* length of the fido entry */
    ssize_t retVal = 0;                         /* return value */
    GOAL_CTC_PRECIS_T *pPrecis = NULL;          /* precis reference */
    GOAL_STATUS_T res;                          /* GOAL result */

    if (NULL == pCtcDevice) {
        goal_logErr("CTC device is not initialized.\n");
        return -EACCES;
    }

    pureChanId = MINOR(pFile->f_inode->i_rdev);

    /* check if the device is open */
    if (0 == pCtcDevice->ctcDevices[pureChanId].pid) {
        goal_logErr("Pure channel %u is not opened\n", pureChanId);
        return -EACCES;
    }

    if (pCtcDevice->util.pConfig->pureChnCnt <= pureChanId) {
        goal_logErr(CTC_DEVICE_FILE" does not exist.\n", pureChanId);
        return -EACCES;
    }

    /* validate the pure status */
    res = goal_ctcUtilPureStatus(&pCtcDevice->util, 0);
    if (GOAL_RES_ERR(res)) {
        goal_logErr("CTC is not ready.\n");
        return -EAGAIN;
    }

    /* Get the according channel fifo */
    pFifo = (struct kfifo *) &(pCtcDevice->ctcDevices[pureChanId].fifoRead);

    /* set module to sleep until there is an entry on the mailbox fifo */
    retVal = wait_event_interruptible(pCtcDevice->ctcDevices[pureChanId].ctcRxWait, !kfifo_is_empty(pFifo));
    if (0 == retVal) {
        /* get a fifo entry */
        fifoDataLen = kfifo_out(pFifo, &pEntry, 1);

        /* check if the reading was successfull */
        if (1 == fifoDataLen) {

            /* check the length */
            if ((uint16_t) len >= pEntry->size) {
                /* Copy the data to user space if pBuf is defined.
                 * retVal is 0 by wait_event_interruptible(). Thus, the size will be returned
                 * if no data has been copied. */
                if (NULL != pBuf) {
                    retVal = copy_to_user(pBuf, pEntry->pBuf, pEntry->size);
                }

                if (!retVal) {
                    /* Data has been read successful. Update the result length */
                    retVal = pEntry->size;
                }
                else {
                    /* Free the buffer. */
                    kfree(pEntry);
                    goal_logErr("Unable to copy data to user space.\n");
                    return (0 > retVal) ? (retVal) : (-EMSGSIZE);
                }
            }

            /* acknowledge the message at kernel space if pure channel is no goal priority
             * or buffer pointer is NULL */
            if ((NULL != pBuf) && (!pCtcDevice->ctcDevices[pureChanId].flgUserUsed)) {
                /* get the pure channel precis for confirming reading */
                res = goal_ctcUtilPrecisGet(&pPrecis, pCtcDevice->util.pRead, pureChanId);
                if (GOAL_RES_ERR(res)) {
                    goal_logErr("Unable to get read precis of pure channel %u.\n", pureChanId);
                    return -EFAULT;
                }
                goal_ctcUtilFifoReadInc(pPrecis, (GOAL_be32toh(pPrecis->idxRd_be32) + 1) % CTC_FIFO_COUNT, GOAL_CTC_CHN_PURE_STATUS_NOT_CYCLIC);
            }

            /* Free the buffer. */
            kfree(pEntry);
        }
    }

    return retVal;
}


/****************************************************************************/
/** Writes len bytes to a core to core char device
 *
 * This function writes the len bytes of the buffer to the core to core char
 * device. The write process will fail, if there are not enough pages
 * available.
 *
 * @return data length if successful, otherwise 0
 */
static ssize_t ctc_write(
    struct file *pFile,                         /**< file handler */
    const char *pBuf,                           /**< buffer */
    size_t len,                                 /**< length */
    loff_t *pOffset                             /**< offset */
)
{
    int pureChanId;                             /* pure channel ID */
    char *pDst;                                 /* destination on kernel memory */
    GOAL_CTC_DATA_ENTRY_T dataAlloc;            /* alloc data for write */
    GOAL_CTC_PRECIS_T *pPrecis = NULL;          /* precis reference */
    GOAL_STATUS_T res;                          /* result */

    if (NULL == pCtcDevice) {
        goal_logErr("CTC device is not initialized.\n");
        return -EACCES;
    }

    pureChanId = MINOR(pFile->f_inode->i_rdev);

    /* check if the channel is open */
    if (0 == pCtcDevice->ctcDevices[pureChanId].pid) {
        goal_logErr("Pure channel %u is not opened\n", pureChanId);
        return -EACCES;
    }

    /* evaluate the pure channel ID */
    if (pCtcDevice->util.pConfig->pureChnCnt <= pureChanId) {
        goal_logErr(CTC_DEVICE_FILE" does not exist.\n", pureChanId);
        return -EACCES;
    }

    /* validate the pure status */
    res = goal_ctcUtilPureStatus(&pCtcDevice->util, 0);
    if (GOAL_RES_ERR(res)) {
        goal_logErr("CTC is not ready.\n");
        return -EAGAIN;
    }

    /* get the pure channel precis */
    res = goal_ctcUtilPrecisGet(&pPrecis, pCtcDevice->util.pWrite, pureChanId);
    if (GOAL_RES_ERR(res)) {
        goal_logErr("Unable to get precis of pure channel %u.\n", pureChanId);
        return -EACCES;
    }

    mutex_lock(&(pCtcDevice->ctcDevices[pureChanId].mtxCtcDev));

    /* allocate the data buffer on */
    res = goal_ctcUtilAlloc(&dataAlloc, pCtcDevice->util.pConfig, pPrecis, (uint16_t) len, GOAL_CTC_CHN_PURE_STATUS_NOT_CYCLIC);
    if (GOAL_RES_ERR(res)) {
        /* ERROR */
        mutex_unlock(&(pCtcDevice->ctcDevices[pureChanId].mtxCtcDev));
        goal_logErr("Unable to allocate %u bytes on pure channel %u.\n", (uint16_t) len, pureChanId);
        return -EACCES;
    }

    /* determine the destination page for writing data */
    pDst = ((uint8_t *) &pPrecis->data) + (GOAL_be32toh(dataAlloc.page) * pCtcDevice->util.pConfig->pageSize);
    /* Copy the data from user space. */
    if (copy_from_user(pDst, pBuf, len)) {
        /* ERROR */
        mutex_unlock(&(pCtcDevice->ctcDevices[pureChanId].mtxCtcDev));
        goal_logErr("CTC: %s - Unable to copy user data.\n", __func__);
        return -EACCES;
    }

    /* add the fifo entry */
    goal_ctcUtilFifoWriteAdd(pPrecis, &dataAlloc, GOAL_CTC_CHN_PURE_STATUS_NOT_CYCLIC);

    /* prepare the mailbox message for the PL320 driver */
    goal_ctcUtilPrecisMsgSet(pCtcDevice->pCtcMboxDataTx,
                            pureChanId,
                            (uint32_t) ((uint8_t *) pDst - (uint8_t *) pCtcDevice->util.pConfig->pMem),
                            dataAlloc.len,
                            &dataAlloc,
                            GOAL_CTC_PURE_FLG_REQ);

    /* send the mailbox message */
    res = goal_ctcMboxTx(pCtcDevice->pCtcMboxDataTx);
    if (GOAL_RES_ERR(res)) {
        goal_logErr("Unable to send a mailbox message request [Err: 0x%x].\n", res);
        /* unable to send the message, avoid a buffer gab by increasing the read index */
        goal_ctcUtilFifoReadInc(pPrecis, (GOAL_be32toh(pPrecis->idxRd_be32) + 1) % CTC_FIFO_COUNT, GOAL_CTC_CHN_PURE_STATUS_NOT_CYCLIC);
        mutex_unlock(&(pCtcDevice->ctcDevices[pureChanId].mtxCtcDev));
        return -EACCES;
    }

    mutex_unlock(&(pCtcDevice->ctcDevices[pureChanId].mtxCtcDev));
    return (ssize_t) len;
}


/****************************************************************************/
/** Send a mailbox message
 *
 * This function triggers the transmission of a mailbox message.
 *
 * @retval GOAL_OK success
 * @retval other failure
 */
static GOAL_STATUS_T goal_ctcMboxTx(
    GOAL_CTC_PRECIS_MSG_T *pMsgPrecis           /**< precis message */
)
{
    struct timespec mboxTimeout;                /* mailbox timeout */
    struct timespec mboxTime;                   /* elapsed mailbox time */
    int resMbox;                                /* mailbox result */

    /* the fifo entry on the write section is added by the destination cores read function */

    /* get the current time stamp */
    getnstimeofday(&mboxTimeout);
    do {
        /* send the mailbox message */
        resMbox = pl320_ipc_transmit((uint32_t *) pMsgPrecis);
        getnstimeofday(&mboxTime);
        if (0 == resMbox) {
            return GOAL_OK;
        }
    } while ((CTC_GET_TIME_MS(mboxTimeout) + pCtcDevice->ctcMboxTimeoutMs) > CTC_GET_TIME_MS(mboxTime));

    return GOAL_ERROR;
}


/****************************************************************************/
/** Io control for core to core char device
 *
 * This function handles the io control commandos of the core to core device.
 *
 * @return 0 if successful, otherwise error code
 */
long ctc_ioctl(
    struct file *pFile,                         /**< file handler */
    unsigned int cmd,                           /**< io control command */
    unsigned long arg                           /**< pointer to user data as ulong */
)
{
    int ret = 0;                                /* return value */
    int pureChanId;                             /* pure channel ID */
    GOAL_STATUS_T res;                          /* result */
    pureChanId = MINOR(pFile->f_inode->i_rdev);

    if (NULL == pCtcDevice) {
        goal_logErr("CTC device is not initialized.\n");
        return 0;
    }

    if (pCtcDevice->util.pConfig->pureChnCnt <= pureChanId) {
        goal_logErr(CTC_DEVICE_FILE" does not exist.\n", pureChanId);
        return 0;
    }

    /* check if the channel is open */
    if (0 == pCtcDevice->ctcDevices[pureChanId].pid) {
        goal_logErr("Pure channel %u is not opened\n", pureChanId);
        return -EACCES;
    }

    switch (cmd) {
        case CTC_IOCTL_DEFAULT_CONF:
                /* return the number of pure channels */
                ret = put_user(pCtcDevice->util.pConfig->pureChnCnt, &((GOAL_CTC_IOCTL_DEF_CONF_T __user *) arg)->pConfig->pureChnCnt);
                if (!ret) {
                    /* return the memory size */
                    ret = put_user(pCtcDevice->util.pConfig->len, &((GOAL_CTC_IOCTL_DEF_CONF_T __user *) arg)->pConfig->len);
                }
                if (!ret) {
                    /* return the number of pages */
                    ret = put_user(pCtcDevice->util.pConfig->pageCnt, &((GOAL_CTC_IOCTL_DEF_CONF_T __user *) arg)->pConfig->pageCnt);
                }
                if (!ret) {
                    /* return the size of pages */
                    ret = put_user(pCtcDevice->util.pConfig->pageSize, &((GOAL_CTC_IOCTL_DEF_CONF_T __user *) arg)->pConfig->pageSize);
                }
                if (!ret) {
                    /* return the offset */
                    ret = put_user(ctcOffsetMem, &((GOAL_CTC_IOCTL_DEF_CONF_T __user *) arg)->offset);
                }
                if (!ret) {
                    /* get the number of priorities */
                    ret = get_user(pCtcDevice->util.pConfig->prioCnt, &((GOAL_CTC_IOCTL_DEF_CONF_T __user *) arg)->pConfig->prioCnt);
                    if (!ret) {
                        /* evaluate the number of priorities */
                        if (pCtcDevice->util.pConfig->prioCnt > pCtcDevice->util.pConfig->pureChnCnt) {
                            goal_logErr("Invalid number of ctc priorities.\n");
                            ret = -EFAULT;
                        }
                    }
                }
            break;

        case CTC_IOCTL_SET_MBOX_MS:
            /* set the mailbox send timeout */
            pCtcDevice->ctcMboxTimeoutMs = *(unsigned int __user *) arg;
            break;

        case CTC_IOCTL_MBOX_SEND:
            /* send a mailbox message */
            res = goal_ctcMboxTx((GOAL_CTC_PRECIS_MSG_T __user *) arg);
            if (GOAL_RES_ERR(res)) {
                ret = -EFAULT;
            }
            break;

        case CTC_IOCTL_USED_BY_USER:
            /* configure the precis of the pure channel to be handled in
             * User space (GOAL_TRUE)
             * or
             * Kernel space (GOAL_FALSE) */
            pCtcDevice->ctcDevices[pureChanId].flgUserUsed = (GOAL_BOOL_T __user) arg;
            break;

        default:
            ret = -EFAULT;
            break;
    }

    return ret;
}


module_init(ctc_init);
module_exit(ctc_cleanup);

MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("GOAL");
MODULE_DESCRIPTION("Core2Core driver");
