#
# Makefile for the Cavium Octeon specific kernel interface routines
# under Linux.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 2005-2012 Cavium, Inc.
#

CFLAGS_octeon-platform.o = -I$(src)/../../../scripts/dtc/libfdt
CFLAGS_setup.o = -I$(src)/../../../scripts/dtc/libfdt

obj-y := setup.o serial.o octeon-platform.o octeon-irq.o csrc-octeon.o \
	 octeon-hw-status.o octeon_info.o cavium.o resource-mgr.o
obj-y += dma-octeon.o
obj-y += csrc-octeon-ptp.o
obj-y += octeon-pci-console.o
obj-y += octeon-memcpy.o
obj-y += executive/

obj-$(CONFIG_SMP)			+= smp.o
obj-$(CONFIG_NUMA)			+= octeon-numa.o
obj-$(CONFIG_NUMA)			+= csrc-fpa-clk.o
obj-$(CONFIG_SYSFS)			+= octeon-power-throttle.o
obj-$(CONFIG_SYSFS)                     += cacheinfo.o
obj-$(CONFIG_MTD)			+= flash_setup.o
obj-$(CONFIG_OCTEON_ILM)		+= oct_ilm.o
obj-$(CONFIG_CAVIUM_OCTEON_NAND)	+= octeon-nand.o
obj-$(CONFIG_CAVIUM_OCTEON_ERROR_TREE)	+= octeon-error-tree.o octeon-78xx-errors.o \
	octeon-73xx-errors.o octeon-7xxx-errors.o octeon-75xx-errors.o
obj-$(CONFIG_CAVIUM_OCTEON_KERNEL_CRYPTO) += octeon-crypto.o
obj-$(CONFIG_OCTEON_ERROR_INJECTOR)	+= octeon-error-injector.o
obj-$(CONFIG_CAVIUM_OCTEON_RAPIDIO)	+= octeon-rapidio.o
obj-$(CONFIG_OCTEON_FPA3)		+= octeon-fpa3.o

obj-$(CONFIG_PERF_EVENTS)		+= perf_counters.o
obj-$(CONFIG_CAVIUM_OCTEON_PERF)	+= perf_uncore.o

ifdef CONFIG_SMP
ifdef CONFIG_SYSFS
ifdef CONFIG_HOTPLUG_CPU
obj-y += unplug-cpu.o unplug-cpu-ll.o
endif
endif
endif

DTS_FILES = octeon_3xxx.dts octeon_68xx.dts
DTB_FILES = $(patsubst %.dts, %.dtb, $(DTS_FILES))

obj-y += $(patsubst %.dts, %.dtb.o, $(DTS_FILES))

# Let's keep the .dtb files around in case we want to look at them.
.SECONDARY:  $(addprefix $(obj)/, $(DTB_FILES))

clean-files += $(DTB_FILES) $(patsubst %.dtb, %.dtb.S, $(DTB_FILES))
