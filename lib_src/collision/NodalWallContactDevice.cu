#include "NodalWallContactKernels.cuh"
#include "NodalWallContactState.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cmath>
#include <cstring>

namespace tlfea::contact {
namespace {
namespace d=nodal_wall_device_detail;
namespace fea=tl::fea;
using Code=NodalWallDeviceStatus;
__global__ void MarkFailure(fea::NodalAssemblyView v,unsigned node) { fea::RecordNodalAssemblyFailure(v,Status::kInvalidArgument,node); }
__global__ void Assemble(d::Storage* storage,fea::NodalAssemblyView v,NodalWallDiagnostics identity) {
  auto& s=*storage;
  d::ResetResult(s.base,s.model.parent_count,s.model.node_count);
  if (threadIdx.x==0) { s.control={}; d::ValidateAssembly(s,v); }
  __syncthreads();
  d::Evaluate(s,v.accepted,identity);
  if (threadIdx.x==0) {
    if (s.control.status==Code::Ok && d::Scatter(s,v)) {
      s.result.diagnostics.valid=true;
    }
    if (s.control.status!=Code::Ok) fea::RecordNodalAssemblyFailure(v,Status::kInvalidArgument,s.control.node);
  }
  __syncthreads();
  if (s.control.status==Code::Ok) d::CopyBase(s);
}
__global__ void Candidate(d::Storage* storage,fea::NodalPreparedView v,NodalWallDiagnostics identity) {
  auto& s=*storage;
  if (threadIdx.x==0) {
    s.control={}; const auto& b=s.base.diagnostics;
    if (!b.valid || b.owner_id!=v.owner_id || b.base_epoch!=v.kinematics.base_epoch || b.attempt!=v.attempt ||
        b.configuration_id!=identity.configuration_id || b.qualification_id!=identity.qualification_id ||
        b.wall_binding_id!=identity.wall_binding_id || b.time!=v.base_time || b.velocity_time!=v.base_velocity_time)
      d::Fail(s.control,Code::StaleAttempt);
  }
  __syncthreads();
  d::Evaluate(s,v.kinematics,identity);
  if (threadIdx.x==0 && s.control.status==Code::Ok && d::MeasureInterval(s,v)) s.result.diagnostics.valid=true;
}
bool Kinematics(const fea::DeviceNodalKinematicsView& v,std::size_t n,std::uint64_t epoch) {
  return v.position_xyz && v.velocity_xyz && v.node_count==n && v.base_epoch==epoch;
}
bool SameView(const fea::DeviceNodalKinematicsView& a,const fea::DeviceNodalKinematicsView& b) {
  return a.position_xyz==b.position_xyz && a.velocity_xyz==b.velocity_xyz &&
      a.angular_velocity_xyz==b.angular_velocity_xyz && a.orientation_wxyz==b.orientation_wxyz &&
      a.node_count==b.node_count && a.base_epoch==b.base_epoch;
}
using fea::trial_identity::Disjoint;
NodalWallDiagnostics Identity(const NodalWallDeviceConfig& c,const fea::NodalAssemblyView& v) {
  NodalWallDiagnostics d; d.owner_id=v.owner_id; d.configuration_id=c.configuration_id;
  d.qualification_id=c.qualification_id; d.wall_binding_id=c.wall_binding_id;
  d.base_epoch=v.accepted.base_epoch; d.attempt=v.attempt; d.phase=NodalWallDevicePhase::AcceptedBase;
  d.scheme=v.temporal_scheme; d.velocity_phase=v.velocity_phase;
  d.time=d.base_time=v.position_time; d.velocity_time=d.base_velocity_time=v.velocity_time;
  return d;
}
bool OutputView(const fea::NodalAssemblyView& v,std::size_t n) {
  const double* components[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                             v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for (unsigned i=0;i<6;++i) {
    if (!components[i]) return false;
    for (unsigned j=0;j<i;++j) if (!Disjoint(components[i],n*sizeof(double),components[j],n*sizeof(double))) return false;
  }
  return true;
}
} // namespace

NodalWallDeviceReport NodalWallContactDevice::Impl::Check(cudaError_t code) {
  if (code==cudaSuccess) return {Code::Ok,"OK"};
  usable=false; has_base=false; has_results=false;
  return {Code::DeviceFailure,cudaGetErrorString(code)};
}
NodalWallDeviceReport NodalWallContactDevice::Impl::ReadControl(cudaStream_t borrowed) {
  auto r=Check(cudaGetLastError()); if (r.status!=Code::Ok) return r;
  r=Check(cudaMemcpyAsync(&control,DeviceControl(),sizeof(control),cudaMemcpyDeviceToHost,borrowed));
  if (r.status!=Code::Ok) return r;
  r=Check(cudaStreamSynchronize(borrowed)); if (r.status!=Code::Ok) return r;
  return {control.status,control.status==Code::Ok?"OK":"Nodal wall operation rejected",
          control.node,control.parent,control.point};
}
NodalWallDeviceReport NodalWallContactDevice::Impl::FailAssembly(const fea::NodalAssemblyView& v,NodalWallDeviceReport r) {
  has_base=false; has_results=false;
  if (v.result && v.bounds) {
    MarkFailure<<<1,1,0,v.stream>>>(v,r.node);
    auto failure=Check(cudaGetLastError()); if (failure.status!=Code::Ok) return failure;
    failure=Check(cudaStreamSynchronize(v.stream)); if (failure.status!=Code::Ok) return failure;
  }
  return r;
}
NodalWallDeviceReport NodalWallContactDevice::AssembleAccepted(const fea::NodalAssemblyView& v,NodalWallDiagnostics* output) {
  if (!impl_) { Impl empty; return empty.FailAssembly(v,{Code::NotInitialized,"Nodal wall is not initialized"}); }
  auto& s=*impl_; s.has_base=false; s.has_results=false;
  auto fail=[&](NodalWallDeviceReport r) { return s.FailAssembly(v,r); };
  if (!s.usable) return fail({Code::DeviceFailure,"Nodal wall CUDA storage is poisoned"});
  const auto n=s.config.owner.node_count; const auto epoch=v.accepted.base_epoch;
  if (v.owner_id!=s.config.owner.owner_id) return fail({Code::WrongOwner,"Wrong nodal owner"});
  if (!Disjoint(output,sizeof(*output),&v,sizeof(v)) || !Kinematics(v.accepted,n,epoch) || v.forces.node_count!=n || v.mass.node_count!=n ||
      v.forces.base_epoch!=epoch || v.mass.base_epoch!=epoch || !v.mass.inverse_mass || !v.mass.fixed ||
      !v.translation_fixed_bits || !v.result || !v.bounds || !OutputView(v,n) ||
      v.temporal_scheme!=fea::NodalTemporalScheme::StaggeredHalfKickStart ||
      !std::isfinite(v.position_time) || !std::isfinite(v.velocity_time) || v.position_time<0 || v.velocity_time<0 ||
      (epoch==0 ? (v.velocity_phase!=fea::NodalVelocityPhase::Collocated || v.position_time!=0 || v.velocity_time!=0) :
                  v.velocity_phase!=fea::NodalVelocityPhase::PreviousMidpoint))
    return fail({Code::InvalidInput,"Invalid contact assembly, output or temporal phase"});
  if (!v.attempt || v.attempt<=s.last_attempt || (!s.last_attempt && epoch!=0) ||
      epoch<s.last_epoch || epoch-s.last_epoch>1 ||
      (s.has_stream && v.stream!=s.stream) ||
      (s.last_attempt && (epoch==s.last_epoch ?
        (v.position_time!=s.base_view.position_time || v.velocity_time!=s.base_view.velocity_time) :
        (v.position_time!=s.base_view.position_time+s.config.owner.fixed_dt ||
         v.velocity_time!=s.base_view.position_time+.5*s.config.owner.fixed_dt))))
    return fail({Code::StaleAttempt,"Duplicate, skipped or stale contact assembly"});
  auto r=s.Check(cudaGetLastError()); if (r.status!=Code::Ok) return fail(r);
  s.last_epoch=epoch; s.last_attempt=v.attempt; s.stream=v.stream; s.has_stream=true; s.base_view=v;
  const auto identity=Identity(s.config,v);
  Assemble<<<1,d::Workers,0,v.stream>>>(s.device,v,identity);
  r=s.ReadControl(v.stream); if (r.status!=Code::Ok) return fail(r);
  NodalWallDiagnostics next;
  r=s.Check(cudaMemcpyAsync(&next,s.DeviceDiagnostics(),sizeof(next),cudaMemcpyDeviceToHost,v.stream));
  if (r.status!=Code::Ok) return fail(r);
  r=s.Check(cudaStreamSynchronize(v.stream)); if (r.status!=Code::Ok) return fail(r);
  s.has_base=true; s.has_results=true; s.available=next; *output=next;
  return {Code::Ok,"Accepted contact force assembled"};
}
NodalWallDeviceReport NodalWallContactDevice::EvaluateCandidate(const fea::NodalPreparedView& v,NodalWallDiagnostics* output) {
  if (!impl_) return {Code::NotInitialized,"Nodal wall is not initialized"};
  auto& s=*impl_; s.has_results=false;
  if (!s.usable) return {Code::DeviceFailure,"Nodal wall CUDA storage is poisoned"};
  const auto& b=s.base_view; const double h=s.config.owner.fixed_dt;
  if (v.owner_id!=s.config.owner.owner_id) return {Code::WrongOwner,"Wrong candidate owner"};
  if (!Disjoint(output,sizeof(*output),&v,sizeof(v)) || !s.has_base || !SameView(v.base_kinematics,b.accepted) ||
      !Kinematics(v.kinematics,s.config.owner.node_count,s.last_epoch) || v.attempt!=s.last_attempt ||
      v.attempt<=s.last_candidate_attempt || v.stream!=s.stream ||
      v.temporal_scheme!=fea::NodalTemporalScheme::StaggeredHalfKickStart ||
      v.base_velocity_phase!=b.velocity_phase || v.velocity_phase!=fea::NodalVelocityPhase::PreviousMidpoint ||
      v.base_time!=b.position_time || v.base_velocity_time!=b.velocity_time ||
      v.proposed_time!=b.position_time+h || !std::isfinite(v.proposed_time) || v.proposed_time<=b.position_time ||
      v.velocity_time!=b.position_time+.5*h || v.kick_dt!=(s.last_epoch?h:.5*h))
    return {Code::StaleAttempt,"Candidate does not match this attempt's accepted force and time samples"};
  auto r=s.Check(cudaGetLastError()); if (r.status!=Code::Ok) return r;
  s.last_candidate_attempt=v.attempt;
  auto identity=Identity(s.config,b); identity.phase=NodalWallDevicePhase::PreparedCandidate;
  identity.time=v.proposed_time; identity.velocity_time=v.velocity_time;
  identity.velocity_phase=v.velocity_phase; identity.kick_dt=v.kick_dt;
  Candidate<<<1,d::Workers,0,v.stream>>>(s.device,v,identity);
  r=s.ReadControl(v.stream); if (r.status!=Code::Ok) return r;
  NodalWallDiagnostics next;
  r=s.Check(cudaMemcpyAsync(&next,s.DeviceDiagnostics(),sizeof(next),cudaMemcpyDeviceToHost,v.stream));
  if (r.status!=Code::Ok) return r;
  r=s.Check(cudaStreamSynchronize(v.stream)); if (r.status!=Code::Ok) return r;
  s.has_results=true; s.available=next; *output=next; return {Code::Ok,"Candidate contact evaluated without publication"};
}
} // namespace tlfea::contact
