#include "FENodalStateStorage.h"
#include "NodalSnapshotRanges.h"
#include "NodalRotation.h"
#include <cmath>

namespace tl::fea {
NodalReport FENodalState::Impl::StageNodalSnapshot(const double* state,bool prepared) {
  auto report=Check(cudaMemcpyAsync(staging.data(),state,state_values*sizeof(double),cudaMemcpyDeviceToHost,stream));
  if(report.status!=NodalStatus::Ok) return report;
  report=Check(cudaStreamSynchronize(stream)); if(report.status!=NodalStatus::Ok) return report;
  for(std::size_t i=0;i<state_values;++i)
    if(!std::isfinite(staging[i])) return Reject(NodalStatus::InvalidOutput,
        prepared?"Prepared readback is nonfinite":"Accepted readback is nonfinite");
  if(has_rotations)
    for(std::size_t i=0;i<config.node_count;++i)
      if(!nodal_detail::UnitQuaternion(nodal_detail::ReadQuaternion(staging.data()+9*config.node_count+4*i)))
        return Reject(NodalStatus::InvalidOutput,
            prepared?"Prepared quaternion readback is not unit":"Accepted quaternion readback is not unit",
            static_cast<std::uint32_t>(i));
  return {NodalStatus::Ok,"OK"};
}

NodalReport FENodalState::CopyAccepted(NodalSnapshotBuffer out,NodalStamp* stamp) {
  if(!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  auto& s=*impl_;
  if(!s.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  const nodal_detail::SnapshotRanges ranges(out,s.config.node_count,stamp,sizeof(*stamp));
  auto report=ranges.Validate(out,s.config.node_count,s.has_rotations);
  if(report.status!=NodalStatus::Ok) return report;
  report=s.StageNodalSnapshot(s.accepted,false); if(report.status!=NodalStatus::Ok) return report;
  ranges.PublishFields(s.staging.data()); *stamp=s.stamp;
  return {NodalStatus::Ok,"OK"};
}

NodalReport FENodalState::CopyPrepared(const NodalTrialToken& token,NodalSnapshotBuffer out,NodalPreparedView* prepared) {
  if(!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  auto& s=*impl_;
  if(!s.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  const nodal_detail::SnapshotRanges ranges(out,s.config.node_count,prepared,sizeof(*prepared));
  auto report=ranges.Validate(out,s.config.node_count,s.has_rotations,&token,sizeof(token));
  if(report.status!=NodalStatus::Ok) return report;
  // Borrow into a local value: its legacy error path clears the supplied view.
  // Authenticity and completed phase are established before trial device reads.
  NodalPreparedView next;
  report=BorrowPrepared(token,&next); if(report.status!=NodalStatus::Ok) return report;
  report=s.StageNodalSnapshot(s.trial,true); if(report.status!=NodalStatus::Ok) return report;
  ranges.PublishFields(s.staging.data()); *prepared=next;
  return {NodalStatus::Ok,"OK"};
}
} // namespace tl::fea
