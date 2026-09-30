#pragma once
#include "NodalTrialIdentity.h"

namespace tl::fea::native_physical_coefficients {
// Admission of immutable native M/J coefficients, not an inverse of the
// constrained system. The owner retains kUnspecified for rigid groups; scalar
// translation PSD proofs must continue to reject that borrowed mass view.
#if defined(__CUDACC__)
#define TL_NATIVE_COEFFICIENT_HD __host__ __device__
#else
#define TL_NATIVE_COEFFICIENT_HD
#endif
TL_NATIVE_COEFFICIENT_HD inline bool Empty(NodalRigidGroupInfo g) noexcept {
  return !g.source_instance_id&&!g.group_count&&!g.member_count&&!g.part_group_count&&!g.plain_source_instance_id;
}
TL_NATIVE_COEFFICIENT_HD inline bool ValidScope(NodalRigidGroupInfo g,std::size_t nodes) noexcept {
  // Existing shell/connector participants admit plain groups only. The complete
  // mapped owner has a separate future participant admission boundary.
  return !g.part_group_count&&!g.plain_source_instance_id&&
    (Empty(g)||(g.source_instance_id&&g.group_count&&g.member_count<=nodes&&g.group_count<=g.member_count/2));
}
TL_NATIVE_COEFFICIENT_HD inline bool SameScope(NodalRigidGroupInfo a,NodalRigidGroupInfo b) noexcept {
  return a.source_instance_id==b.source_instance_id&&a.group_count==b.group_count&&a.member_count==b.member_count&&
    a.part_group_count==b.part_group_count&&a.plain_source_instance_id==b.plain_source_instance_id;
}
TL_NATIVE_COEFFICIENT_HD inline bool Admitted(NodalRigidGroupInfo configured,NodalRigidGroupInfo actual,
    tlfea::contact::TranslationMassModel model,std::size_t nodes) noexcept {
  using Mass=tlfea::contact::TranslationMassModel;
  return ValidScope(configured,nodes)&&SameScope(configured,actual)&&
    model==(Empty(configured)?Mass::kIsotropicLumped:Mass::kUnspecified);
}
#undef TL_NATIVE_COEFFICIENT_HD

// These source/token checks authenticate pointers without dereferencing the
// borrowed arrays or publishing outputs. A matching count or caller-authored
// descriptor alone never authorizes constrained contributor execution.
inline bool SameOwnerScope(const NodalStamp& configured,const NodalStamp& actual) noexcept {
  return configured.owner_id==actual.owner_id&&configured.node_count==actual.node_count&&
    configured.fixed_dt==actual.fixed_dt&&configured.has_rotations==actual.has_rotations&&
    configured.temporal_scheme==actual.temporal_scheme&&SameScope(configured.rigid_groups,actual.rigid_groups);
}
inline NodalReport AuthenticateAccepted(FENodalState& owner,const NodalStamp& expected,
                                        const NodalAssemblyView& view) noexcept {
  if(!trial_identity::SameStamp(owner.accepted(),expected))
    return {NodalStatus::StaleTrial,"Contributor does not match the live accepted owner scope"};
  return owner.ValidateAcceptedAssemblySources(view);
}
inline NodalReport AuthenticatePrepared(FENodalState& owner,const NodalTrialToken& token,
                                        const NodalStamp& expected,const NodalPreparedView& view) noexcept {
  if(!trial_identity::SameStamp(owner.accepted(),expected))
    return {NodalStatus::StaleTrial,"Contributor does not match the live accepted owner scope"};
  NodalPreparedView actual;
  const auto report=owner.BorrowPrepared(token,&actual);
  if(report.status!=NodalStatus::Ok) return report;
  if(!trial_identity::SamePrepared(actual,view))
    return {NodalStatus::StaleTrial,"Contributor candidate is not the owner's common prepared token"};
  return {NodalStatus::Ok,"OK"};
}
} // namespace tl::fea::native_physical_coefficients
