// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <dt-bindings/soc/rzn1-sysctrl.h>
#include <dt-bindings/soc/rzn1-memory-map.h>

#define GIC_BASE 	(RZN1_GIC_BASE + 0x2000)
#define BOOTADDR	(RZN1_SYSTEM_CTRL_BASE + RZN1_SYSCTRL_REG_BOOTADDR)
#define BOOTADDR2		CONFIG_RZN1_SMP_ALT_BOOTADDR
#define BOOTADDR2_CANARY	0x525a4e31
/*
 * RZ/N1 specific entry point for secondary CPUs. The second CPU enables
 * interrupts for itself (in masked state), then loops on a wfi for the
 * kernel to wake it up, have a look at the BOOTADDR for an address,
 * picks it up (and clears BOOTADDR) then just branches out.
 */
ENTRY(rzn1_secondary_startup)

	ldr	pc, =_reset 		@ reset vector
	ldr	pc, =_undf		@ undef
	ldr	pc, =_undf		@ svc
	ldr	pc, =_undf		@ abort
	ldr	pc, =_undf		@ irq
	nop
fiq:	b	fiq
_undf:	b	_undf

_reset:
	mrc	p15, 0, r0, c0, c0, 5	@ get processor id
	and	r0, r0, #15		@ What CPU am I?
	bne	_cpu1

	b 	park
_cpu1:	@ they both go in park anyway
park:
	ldr 	r2, =BOOTADDR2_CANARY
	ldr	r0, =BOOTADDR2		@ Check for canary in PEN2+4 location
	add	r0, #4
	ldr     r1, [r0]
	cmp	r1, r2
	ldr	r0, =BOOTADDR2
	ldrne	r0, =BOOTADDR

	mov	r3, #0
	ldr 	r2, =GIC_BASE

	mov 	r1, #1
	str 	r1, [r2] 		@ set GICC_CTLR.Enable
	mov 	r1, #255
	str 	r1, [r2, #4] 		@ set GIC_PMR.Priority to 0xff
	dsb

pen:	wfi
	ldr     r1, [r0]
	tst     r1, r1
	beq	pen
	str	r3, [r0]		@ Clear BOOTADDR
	bx	r1
	.ltorg
ENDPROC(rzn1_secondary_startup)
