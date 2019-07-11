#
# Makefile for the linux kernel.
#

ccflags-$(CONFIG_ARCH_MULTIPLATFORM) := -I$(srctree)/arch/arm/mach-rzn1/include

obj-$(CONFIG_ARCH_RZN1)		+= setup-rzn1.o sysctrl-rzn1.o
obj-$(CONFIG_RZN1_CM3)		+= rzn1-cm3.o
obj-$(CONFIG_RZN1_A5PSW_WORKAROUND)	+= rzn1-a5psw-workaround.o
obj-$(CONFIG_SMP) += platsmp.o
