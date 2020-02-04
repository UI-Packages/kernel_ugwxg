#include "kvm/irq.h"
#include "kvm/kvm.h"

#include <stdlib.h>
#include <linux/byteorder.h>

/* see arch/mips/include/asm/mach-paravirt/irq.h */
#define IRQ_MAX_GSI			128
#define MIPS_IRQ_MBOX_MAX		48
#define IRQCHIP_MASTER			0


struct kvm_irq_routing *irq_routing;
static int irq__add_routing(u32 gsi, u32 type, u32 irqchip, u32 pin)
{
	if (gsi >= IRQ_MAX_GSI)
		return -ENOSPC;

	irq_routing->entries[irq_routing->nr++] =
		(struct kvm_irq_routing_entry) {
			.gsi = gsi,
			.type = type,
			.u.irqchip.irqchip = irqchip,
			.u.irqchip.pin = pin,
		};

	return 0;
}

int irq__init(struct kvm *kvm)
{
	int i, r;

	irq_routing = calloc(sizeof(struct kvm_irq_routing) +
			IRQ_MAX_GSI * sizeof(struct kvm_irq_routing_entry), 1);
	if (irq_routing == NULL)
		return -ENOMEM;

	/* Add routing for first 48 GSIs (non-MSI) to master IRQCHIP */
	for (i = 1; i <= MIPS_IRQ_MBOX_MAX; i++)
		irq__add_routing(i, KVM_IRQ_ROUTING_IRQCHIP, IRQCHIP_MASTER, i);

	r = ioctl(kvm->vm_fd, KVM_SET_GSI_ROUTING, irq_routing);
	if (r) {
		free(irq_routing);
		return errno;
	}

	return 0;
}
dev_base_init(irq__init);

int irq__exit(struct kvm *kvm)
{
	free(irq_routing);
	return 0;
}
dev_base_exit(irq__exit);

int irq__add_msix_route(struct kvm *kvm, struct msi_msg *msg)
{
	int r;
	u32 gsi = le32_to_cpu(msg->data);

	irq_routing->entries[irq_routing->nr++] =
		(struct kvm_irq_routing_entry) {
			.gsi = gsi,
			.type = KVM_IRQ_ROUTING_MSI,
			.u.msi.address_hi = msg->address_hi,
			.u.msi.address_lo = msg->address_lo,
			.u.msi.data = msg->data,
	};

	r = ioctl(kvm->vm_fd, KVM_SET_GSI_ROUTING, irq_routing);
	if (r)
		return r;

	return gsi;
}
