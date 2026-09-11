#include "NodalWallContactState.h"
#include "NodalWallContactResultIO.h"
#include "NodalWallContactDiagnosticIdentity.h"
#include <algorithm>

namespace tlfea::contact {
namespace {
using Code=NodalWallDeviceStatus;
namespace d=nodal_wall_device_detail;
using tl::fea::trial_identity::Disjoint;
using nodal_wall_device_detail::SameDiagnostics;
} // namespace
NodalWallDeviceReport NodalWallContactDevice::Impl::ReadResults(const NodalWallDiagnostics& expected) {
  if(!usable) return {Code::DeviceFailure,"Nodal wall CUDA storage is poisoned"};
  if(!has_results || !SameDiagnostics(expected,available)) return {Code::StaleAttempt,"Stale contact result identity"};
  auto& staging=prepared.storage().result; const auto& remote=device_shadow.result;
  const auto& regions=prepared.layout().result;
  auto r=Check(cudaGetLastError()); if(r.status!=Code::Ok) return r;
  r=Check(cudaMemcpyAsync(&staging.diagnostics,DeviceDiagnostics(),sizeof(staging.diagnostics),cudaMemcpyDeviceToHost,stream));
  if(r.status!=Code::Ok) return r;
  r=Check(cudaMemcpyAsync(staging.parents,remote.parents,regions.parents.bytes,cudaMemcpyDeviceToHost,stream));
  if(r.status!=Code::Ok) return r;
  r=Check(cudaMemcpyAsync(staging.nodes,remote.nodes,regions.nodes.bytes,cudaMemcpyDeviceToHost,stream));
  if(r.status!=Code::Ok) return r;
  r=Check(cudaMemcpyAsync(staging.wall_face,remote.wall_face,regions.wall_face.bytes,cudaMemcpyDeviceToHost,stream));
  if(r.status!=Code::Ok) return r;
  r=Check(cudaStreamSynchronize(stream)); if(r.status!=Code::Ok) return r;
  if(!SameDiagnostics(staging.diagnostics,expected)) return {Code::StaleAttempt,"Device contact result identity mismatch"};
  return {Code::Ok,"Contact results staged; caller must await owner/material commit"};
}
NodalWallDeviceReport NodalWallContactDevice::CopyResults(const NodalWallDiagnostics& expected,
    NodalWallDeviceResults* output) {
  if(!impl_) return {Code::NotInitialized,"Nodal wall is not initialized"};
  auto& s=*impl_; const auto& layout=s.prepared.layout();
  // Fixed ABI cannot name large parents, compact slots or a larger owner. Reject
  // before touching a potentially undersized caller buffer or expected record.
  if(layout.parent_count>MaxNodalWallDeviceParents || layout.node_count>MaxNodalWallDeviceNodes ||
      layout.global_count>MaxNodalWallDeviceNodes)
    return {Code::ResourceLimit,"Active contact results require the exact-capacity result view"};
  if(!Disjoint(output,sizeof(*output),&expected,sizeof(expected)))
    return {Code::InvalidInput,"Missing/overlapping contact readback"};
  const auto r=s.ReadResults(expected); if(r.status!=Code::Ok) return r;
  const auto& staged=s.prepared.storage().result;
  NodalWallDeviceResults next; next.diagnostics=staged.diagnostics;
  std::copy_n(staged.parents,layout.parent_count,next.parents);
  std::copy_n(staged.nodes,layout.node_count,next.nodes);
  std::copy_n(staged.wall_face,layout.node_count,next.wall_face);
  *output=next; return r;
}
NodalWallDeviceReport NodalWallContactDevice::CopyResults(const NodalWallDiagnostics& expected,
    const NodalWallDeviceResultView& output) {
  if(!impl_) return {Code::NotInitialized,"Nodal wall is not initialized"};
  auto& s=*impl_; const auto& layout=s.prepared.layout();
  if(!d::ValidResultView(output,layout.parent_count,layout.node_count,&expected,s.config.limits.profile))
    return {Code::InvalidInput,"Contact output capacities or disjoint ranges are invalid"};
  const auto r=s.ReadResults(expected); if(r.status!=Code::Ok) return r;
  const auto& staged=s.prepared.storage().result;
  std::copy_n(staged.parents,layout.parent_count,output.parents);
  std::copy_n(staged.nodes,layout.node_count,output.nodes);
  std::copy_n(staged.wall_face,layout.node_count,output.wall_face);
  *output.diagnostics=staged.diagnostics; return r;
}
} // namespace tlfea::contact
