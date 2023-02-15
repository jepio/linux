/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * KVM L1 hypervisor optimizations on Hyper-V.
 */

#ifndef __ARCH_X86_KVM_KVM_ONHYPERV_H__
#define __ARCH_X86_KVM_KVM_ONHYPERV_H__

#if IS_ENABLED(CONFIG_HYPERV)
int hv_remote_flush_tlb_with_range(struct kvm *kvm,
		struct kvm_tlb_range *range);
int hv_remote_flush_tlb(struct kvm *kvm);
void hv_track_root_tdp(struct kvm_vcpu *vcpu, hpa_t root_tdp);
void hv_flush_tlb_current(struct kvm_vcpu *vcpu);
#else /* !CONFIG_HYPERV */
static inline void hv_track_root_tdp(struct kvm_vcpu *vcpu, hpa_t root_tdp)
{
}
static inline void hv_flush_tlb_current(struct kvm_vcpu *vcpu)
{
}
#endif /* !CONFIG_HYPERV */

#endif
