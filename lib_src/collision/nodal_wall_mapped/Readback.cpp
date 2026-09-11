// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../NodalWallContactResultIO.h"
#include "../NodalWallContactDiagnosticIdentity.h"
#include "lib_src/elements/ShellPhysicalOutputRanges.h"
#include "lib_src/elements/ShellExecutionOutputRanges.h"
#include <algorithm>
namespace tlfea::contact::nodal_wall_mapped {
bool SameCertificate(Q4CertifiedIntegral a,Q4CertifiedIntegral b) noexcept {
  using fe::shell_startup_detail::SameBits;
  return SameBits(a.value,b.value) && SameBits(a.lower,b.lower) &&
      SameBits(a.upper,b.upper) && SameBits(a.error,b.error);
}
bool SameDiagnostics(const NodalWallMappedDiagnostics& a,const NodalWallMappedDiagnostics& b) noexcept {
  return a.valid && b.valid && d::SameDiagnostics(a.contact,b.contact) &&
      SameCertificate(a.removed_potential,b.removed_potential) &&
      a.current_response_rate_upper==b.current_response_rate_upper &&
      a.accepted_active_parents==b.accepted_active_parents && a.proposed_active_parents==b.proposed_active_parents &&
      a.prepared_activity_available==b.prepared_activity_available &&
      a.interval_tree_used==b.interval_tree_used;
}
} // namespace tlfea::contact::nodal_wall_mapped
namespace tlfea::contact {
namespace m=nodal_wall_mapped;
namespace d=nodal_wall_device_detail;
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
bool NodalWallMappedContact::Impl::OutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  using fe::trial_identity::Disjoint;
  return output && publication->PhysicalOutputDisjoint(output,bytes) &&
      Disjoint(output,bytes,this,sizeof(*this)) &&
      Disjoint(output,bytes,prepared.data(),prepared.layout().bytes) &&
      Disjoint(output,bytes,host.data(),host.bytes()) &&
      Disjoint(output,bytes,parents.data(),parents.size()*sizeof(m::Parent)) &&
      Disjoint(output,bytes,activity.data(),activity.size()) &&
      Disjoint(output,bytes,snapshots.data(),snapshots.size()*sizeof(fe::NodalRigidGroupSnapshot)) &&
      fe::shell_physical_owner::OutputDisjoint(physical,output,bytes) &&
      fe::shell_execution_detail::OutputDisjoint(rigid,output,bytes);
}
NodalWallDeviceReport NodalWallMappedContact::Impl::ReadDiagnostics(bool candidate,NodalWallMappedDiagnostics& output) {
  NodalWallMappedDiagnostics next;
  m::Summary summary;
  const auto* diagnostics=reinterpret_cast<const unsigned char*>(device)+offsetof(d::Storage,result)+
      offsetof(d::ActiveResults,diagnostics);
  auto report=Check(cudaMemcpyAsync(&next.contact,diagnostics,sizeof(next.contact),cudaMemcpyDeviceToHost,stream));
  if(report.status!=Code::Ok) return report;
  report=Check(cudaMemcpyAsync(&summary,remote.summary,sizeof(summary),cudaMemcpyDeviceToHost,stream));
  if(report.status!=Code::Ok) return report;
  report=Check(cudaStreamSynchronize(stream));
  if(report.status!=Code::Ok) return report;
  if(!next.contact.valid || !IsFinite(summary.rate) || summary.rate<0 ||
      !nodal_wall_detail::Certificate(summary.removed_potential))
    return {Code::NonFiniteArithmetic,"Mapped contact diagnostic validation failed"};
  next.removed_potential=summary.removed_potential;
  next.current_response_rate_upper=summary.rate;
  next.accepted_active_parents=accepted_active;
  next.proposed_active_parents=candidate?proposed_active:accepted_active;
  next.prepared_activity_available=candidate;
  next.interval_tree_used=candidate && summary.interval_tree_used;
  next.valid=true;
  output=next;
  return {Code::Ok,"Mapped diagnostics staged"};
}
NodalWallDeviceReport NodalWallMappedContact::Impl::ReadResults(const NodalWallMappedDiagnostics& expected) {
  if(!usable) return {Code::DeviceFailure,"Mapped wall is poisoned"};
  if(!has_results || !m::SameDiagnostics(expected,available))
    return {Code::StaleAttempt,"Stale mapped contact results"};
  auto& staging=prepared.storage().result;
  const auto& regions=prepared.layout().result;
  auto report=Check(cudaGetLastError());
  if(report.status!=Code::Ok) return report;
  report=Check(cudaMemcpyAsync(staging.parents,shadow.result.parents,regions.parents.bytes,cudaMemcpyDeviceToHost,stream));
  if(report.status!=Code::Ok) return report;
  report=Check(cudaMemcpyAsync(staging.nodes,shadow.result.nodes,regions.nodes.bytes,cudaMemcpyDeviceToHost,stream));
  if(report.status!=Code::Ok) return report;
  report=Check(cudaMemcpyAsync(staging.wall_face,shadow.result.wall_face,regions.wall_face.bytes,cudaMemcpyDeviceToHost,stream));
  if(report.status!=Code::Ok) return report;
  NodalWallMappedDiagnostics actual;
  report=ReadDiagnostics(expected.prepared_activity_available,actual);
  if(report.status!=Code::Ok) return report;
  if(!m::SameDiagnostics(expected,actual)) return {Code::StaleAttempt,"Device mapped result identity mismatch"};
  staging.diagnostics=actual.contact;
  for(std::size_t p=0;p<parents.size();++p) {
    const auto& value=staging.parents[p];
    if(!value.valid || value.parent_element_id!=parents[p].source_id ||
        !nodal_wall_detail::Certificate(value.resultant) || !nodal_wall_detail::Certificate(value.potential))
      return {Code::NonFiniteArithmetic,"Mapped parent result is invalid",UINT32_MAX,static_cast<std::uint32_t>(p)};
  }
  for(std::size_t i=0;i<prepared.layout().node_count;++i) {
    const auto& value=staging.nodes[i];
    if(!value.valid || value.node!=prepared.model().nodes[i].node || value.row.valid ||
        value.local_velocity_first_timestep!=0 || value.fixed ||
        !nodal_wall_detail::Certificate(value.force) || !nodal_wall_detail::Certificate(value.potential) ||
        !nodal_wall_detail::Certificate(value.stiffness) || !IsFinite(value.force_world) ||
        !IsFinite(value.wall_moment) || !IsFinite(value.surface_power))
      return {Code::NonFiniteArithmetic,"Mapped node result is invalid",value.node};
  }
  return {Code::Ok,"Mapped results staged before common publication"};
}
NodalWallDeviceReport NodalWallMappedContact::CopyResults(const NodalWallMappedDiagnostics& expected,
    const NodalWallDeviceResultView& output) {
  if(!impl_) return {Code::NotInitialized,"Mapped wall is not initialized"};
  auto& state=*impl_;
  const auto& layout=state.prepared.layout();
  if(!d::ValidResultView(output,layout.parent_count,layout.node_count,&expected.contact,state.config.limits.profile))
    return {Code::InvalidInput,"Mapped result capacities or aliases invalid"};
  const void* pointers[]{output.diagnostics,output.parents,output.nodes,output.wall_face};
  const std::size_t bytes[]{sizeof(*output.diagnostics),layout.result.parents.bytes,
      layout.result.nodes.bytes,layout.result.wall_face.bytes};
  for(unsigned i=0;i<4;++i) if(!state.OutputDisjoint(pointers[i],bytes[i]) ||
      !fe::trial_identity::Disjoint(pointers[i],bytes[i],&expected,sizeof(expected)) ||
      !fe::trial_identity::Disjoint(pointers[i],bytes[i],this,sizeof(*this)))
    return {Code::InvalidInput,"Mapped result output aliases retained source or identity"};
  const auto report=state.ReadResults(expected);
  if(report.status!=Code::Ok) {
    state.has_results=false;
    state.has_base=false;
    return report;
  }
  const auto& staged=state.prepared.storage().result;
  std::copy_n(staged.parents,layout.parent_count,output.parents);
  std::copy_n(staged.nodes,layout.node_count,output.nodes);
  std::copy_n(staged.wall_face,layout.node_count,output.wall_face);
  *output.diagnostics=staged.diagnostics;
  return report;
}
} // namespace tlfea::contact
