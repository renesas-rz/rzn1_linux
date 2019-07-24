/*-----------------------------------------------------------------------------
 * atemsys.h
 * Copyright (c) 2009 - 2019 acontis technologies GmbH, Weingarten, Germany
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Response                  Paul Bussmann
 * Description               atemsys.ko headerfile
 * Note: This header is also included by userspace!

 *  Changes:
 *
 *  V1.0.00 - Inital, PCI/PCIe only.
 *  V1.1.00 - PowerPC tweaks.
 *           Support for SoC devices (no PCI, i.e. Freescale eTSEC).
 *           Support for current linux kernel's (3.0). Removed deprecated code.
 *  V1.2.00 - 64 bit support. Compat IOCTL's for 32-Bit usermode apps.
 *  V1.2.01 - request_irq() sometimes failed -> Map irq to virq under powerpc.
 *  V1.2.02 - Support for current Linux kernel (3.8.0)
 *  V1.2.03 - Support for current Linux kernel (3.8.13) on armv7l (beaglebone)
 *  V1.2.04 - Use dma_alloc_coherent for arm, because of DMA memory corruption on
 *           Xilinx Zynq.
 *  V1.2.05 - OF Device Tree support for Xilinx Zynq (VIRQ mapping)
 *  V1.2.06 - Wrong major version.
 *  V1.2.07 - Tolerate closing, e.g. due to system()-calls.
 *  V1.2.08 - Add VM_DONTCOPY to prevent crash on system()-calls
 *  V1.2.09 - Apply second controller name change in dts (standard GEM driver for Xilinx Zynq) to avoid default driver loading.
 *  V1.2.10 - Removed IO address alignment to support R6040
 *  V1.2.11 - Fixed lockup in device_read (tLinkOsIst if NIC in interrupt mode) on dev_int_disconnect
 *  V1.2.12 - Fixed underflow in dev_disable_irq() when more than one interrupts pending because of disable_irq_nosync usage
 *  V1.2.13 - Fixed usage of x64 PCI physical addresses
 *  V1.2.14 - Changes for using with kernel beginnig from 2.6.18
 *  V1.2.15 - Add udev auto-loading support via DTB
 *  V1.2.16 - Add interrupt mode support for Xenomai 3 (Cobalt)
 *  V1.3.01 - Add IOCTL_MOD_GETVERSION
 *  V1.3.02 - Add support for kernel >= 4.11.00
 *  V1.3.03 - Fixed IOCTL_MOD_GETVERSION
 *  V1.3.04 - Fixed interrupt deadlock in Xenomai 2
 *  V1.3.05 - Use correct PCI domain
 *  V1.3.06 - Use rtdm_printk for Cobalt, add check if dev_int_disconnect was successful
 *  V1.3.07 - Removed IOCTL_PCI_RELEASE_DEVICE warnings due to untracked IOCTL_PCI_CONF_DEVICE
 *  V1.3.08 - Add support for kernel >= 4.13.00
 *  V1.3.09 - Add support for PRU ICSS in Device Tree
 *  V1.3.10 - Fix compilation on Ubuntu 18.04, Kernel 4.9.90, Xenomai 3.0.6 x64 Cobalt
 *  V1.3.11 - Add enable access to ARM cycle count register(CCNT)
 *  V1.3.12 - Add atemsys API version selection
 *  V1.3.13 - Add ARM64 support
 *----------------------------------------------------------------------------*/

#ifndef ATEMSYS_H
#define ATEMSYS_H

#include <linux/ioctl.h>
#include <linux/types.h>

#ifndef EC_MAKEVERSION
#define EC_MAKEVERSION(a,b,c,d) (((a)<<24)+((b)<<16)+((c)<<8))
#endif

#define ATEMSYS_VERSION_STR "1.3.13"
#define ATEMSYS_VERSION_NUM  1,3,13
#if (defined ATEMSYS_C)
#define USE_ATEMSYS_API_VERSION EC_MAKEVERSION(1,3,13,0)
#endif

/* support selection */
#if USE_ATEMSYS_API_VERSION >= EC_MAKEVERSION(1,3,4,0)
#define INCLUDE_ATEMSYS_PCI_DOMAIN
#endif

#define DRIVER_SUCCESS  0

/*
 * The major device number. We can't rely on dynamic
 * registration any more, because ioctls need to know
 * it.
 */
#define MAJOR_NUM 101

#if (defined INCLUDE_ATEMSYS_PCI_DOMAIN)
#define IOCTL_PCI_FIND_DEVICE           _IOWR(MAJOR_NUM, 0, PCI_SELECT_DESC)
#define IOCTL_PCI_CONF_DEVICE           _IOWR(MAJOR_NUM, 1, PCI_SELECT_DESC)
#endif
#define IOCTL_PCI_RELEASE_DEVICE        _IO(MAJOR_NUM, 2)
#define IOCTL_INT_CONNECT               _IOW(MAJOR_NUM, 3, __u32)
#define IOCTL_INT_DISCONNECT            _IOW(MAJOR_NUM, 4, __u32)
#define IOCTL_INT_INFO                  _IOR(MAJOR_NUM, 5, INT_INFO)
#define IOCTL_MOD_GETVERSION            _IOR(MAJOR_NUM, 6, __u32)
#define IOCTL_CPU_ENABLE_CYCLE_COUNT    _IOW(MAJOR_NUM, 7, __u32)

/*
 * The name of the device driver
 */
#define ATEMSYS_DEVICE_NAME "atemsys"

/* CONFIG_XENO_COBALT/CONFIG_XENO_MERCURY defined in xeno_config.h (may not be available when building atemsys.ko) */
#if (!defined CONFIG_XENO_COBALT) && (!defined CONFIG_XENO_MERCURY) && (defined CONFIG_XENO_VERSION_MAJOR) && (CONFIG_XENO_VERSION_MAJOR >= 3)
#define CONFIG_XENO_COBALT
#endif

/*
 * The name of the device file
 */
#ifdef CONFIG_XENO_COBALT
#define ATEMSYS_FILE_NAME "/dev/rtdm/" ATEMSYS_DEVICE_NAME
#else
#define ATEMSYS_FILE_NAME "/dev/" ATEMSYS_DEVICE_NAME
#endif /* CONFIG_XENO_COBALT */

#define PCI_MAXBAR (6)
#define USE_PCI_INT (0xFFFFFFFF) /* Query the selected PCI device for the assigned IRQ number */

typedef struct
{
   __u32  dwIOMem;           /* [out] IO Memory of PCI card (physical address) */
   __u32  dwIOLen;           /* [out] Length of the IO Memory area*/
} __attribute__((packed)) PCI_MEMBAR;

typedef struct
{
   __s32        nVendID;          /* [in] vendor ID */
   __s32        nDevID;           /* [in] device ID */
   __s32        nInstance;        /* [in] instance to look for (0 is the first instance) */
   __s32        nPciBus;          /* [in/out] bus */
   __s32        nPciDev;          /* [in/out] device */
   __s32        nPciFun;          /* [in/out] function */
   __s32        nBarCnt;          /* [out] Number of entries in aBar */
   __u32        dwIrq;            /* [out] IRQ or USE_PCI_INT */
   PCI_MEMBAR   aBar[PCI_MAXBAR]; /* [out] IO memory */
   __s32        nPciDomain;       /* [in/out] domain */
} __attribute__((packed)) PCI_SELECT_DESC;

typedef struct
{
   __u32        dwInterrupt;
} __attribute__((packed)) INT_INFO;


/* Defines and declarations for IO controls in v1_3_04 and earliear*/

#define IOCTL_PCI_FIND_DEVICE_v1_3_04    _IOWR(MAJOR_NUM, 0, PCI_SELECT_DESC_v1_3_04)
#define IOCTL_PCI_CONF_DEVICE_v1_3_04    _IOWR(MAJOR_NUM, 1, PCI_SELECT_DESC_v1_3_04)

typedef struct
{
   __s32        nVendID;          /* [in] vendor ID */
   __s32        nDevID;           /* [in] device ID */
   __s32        nInstance;        /* [in] instance to look for (0 is the first instance) */
   __s32        nPciBus;          /* [in/out] bus */
   __s32        nPciDev;          /* [in/out] device */
   __s32        nPciFun;          /* [in/out] function */
   __s32        nBarCnt;          /* [out] Number of entries in aBar */
   __u32        dwIrq;            /* [out] IRQ or USE_PCI_INT */
   PCI_MEMBAR   aBar[PCI_MAXBAR]; /* [out] IO memory */
} __attribute__((packed)) PCI_SELECT_DESC_v1_3_04;

#if (!defined INCLUDE_ATEMSYS_PCI_DOMAIN)
#define IOCTL_PCI_FIND_DEVICE           IOCTL_PCI_FIND_DEVICE_v1_3_04
#define IOCTL_PCI_CONF_DEVICE           IOCTL_PCI_CONF_DEVICE_v1_3_04
#endif

#endif  /* ATEMSYS_H */
