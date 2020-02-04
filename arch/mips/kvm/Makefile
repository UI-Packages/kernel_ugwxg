# Makefile for KVM support for MIPS
#

common-objs = $(addprefix ../../../virt/kvm/, kvm_main.o)

EXTRA_CFLAGS += -Ivirt/kvm -Iarch/mips/kvm

kvm_mipste-objs		:= kvm_mips_emul.o kvm_locore.o kvm_mips_int.o \
			   kvm_mips_stats.o kvm_mips_commpage.o \
			   kvm_mips_dyntrans.o kvm_trap_emul.o kvm_cb.o \
			   kvm_tlb.o \
			   $(addprefix ../../../virt/kvm/, coalesced_mmio.o)

obj-$(CONFIG_KVM)		+= $(common-objs) kvm_mips.o
obj-$(CONFIG_KVM_MIPS_TE)	+= kvm_mipste.o
obj-$(CONFIG_KVM_MIPS_VZ)	+= kvm_mipsvz.o kvm_mipsvz_guest.o \
				$(addprefix ../../../virt/kvm/, irqchip.o eventfd.o)
