#include "FENodalStateStorage.h"
#include "NodalRigidGroupStorage.h"
#include "NodalForceStageCaptureLayout.h"
#include "NodalForceStageSnapshotRanges.h"
#include <cmath>
#include <cstring>

namespace tl::fea {
NodalReport FENodalState::CopyPreparedForceStage(const NodalTrialToken& token,
    NodalForceStageSnapshotBuffer out,NodalPreparedView* prepared) {
  if(!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  auto& s=*impl_;
  if(!s.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if(!s.config.capture_force_stage_accelerations||!s.rigid_groups)
    return {NodalStatus::WrongPhase,"Force-stage capture was not enabled at startup"};
  const auto& groups=*s.rigid_groups;
  const nodal_detail::ForceStageCaptureLayout layout{s.config.node_count,groups.info.group_count};
  auto report=nodal_detail::ValidateForceStageOutputs(out,layout.nodes,layout.groups,prepared,token);
  if(report.status!=NodalStatus::Ok) return report;
  NodalPreparedView next;
  report=BorrowPrepared(token,&next);if(report.status!=NodalStatus::Ok) return report;
  // One transfer into existing owner-private staging. No group rows or caller
  // arrays can change until the last node/group value has passed finite checks.
  report=s.Check(cudaMemcpyAsync(s.staging.data(),s.scratch+layout.scratch_offset(),layout.values()*sizeof(double),
                               cudaMemcpyDeviceToHost,s.stream));
  if(report.status!=NodalStatus::Ok) return report;
  report=s.Check(cudaStreamSynchronize(s.stream));if(report.status!=NodalStatus::Ok) return report;
  for(std::size_t i=0;i<layout.values();++i)
    if(!std::isfinite(s.staging[i])) return s.Reject(NodalStatus::InvalidOutput,"Force-stage acceleration readback is nonfinite");
  const auto* data=s.staging.data();
  std::memcpy(out.acceleration_xyz,data,3*layout.nodes*sizeof(double));
  std::memcpy(out.angular_acceleration_xyz,data+3*layout.nodes,3*layout.nodes*sizeof(double));
  for(std::size_t g=0;g<layout.groups;++g) {
    const auto& source=groups.properties[g];
    const auto* a=data+6*layout.nodes+3*g;
    const auto* ar=data+6*layout.nodes+3*layout.groups+3*g;
    out.groups[g]={source.source_id,source.source_node_set_id,source.member_count,{a[0],a[1],a[2]},{ar[0],ar[1],ar[2]},source.source_kind};
  }
  *prepared=next;
  return {NodalStatus::Ok,"Prepared force-stage accelerations copied"};
}
} // namespace tl::fea
