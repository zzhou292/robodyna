// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Evaluation.cuh"
#include "Scatter.cuh"
#include "Kernels.cuh"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace tlfea::contact {
namespace m=nodal_wall_mapped;
namespace d=nodal_wall_device_detail;
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
namespace {
__global__ void BeginAssembly(d::Storage* pointer,m::Sidecar side,fe::NodalAssemblyView view) {
  auto& storage=*pointer;
  storage.control={};
  *side.summary={};
  side.summary->points_admitted=m::ValidateAssembly(storage,side,view);
}
__global__ void CheckResponse(d::Storage* pointer,m::Sidecar side,fe::NodalAssemblyView view) {
  auto& storage=*pointer;
  if(storage.control.status==Code::Ok && m::Response(storage,side,view.accepted)) {
    double step=0,frequency=0;
    if(!mass_detail::Upper(::sqrt(side.summary->rate),&frequency) ||
        !mass_detail::UpperProduct(storage.model.config.owner.fixed_dt,frequency,&step) || step>=1.6)
      d::Fail(storage.control,Code::StepTooLarge);
  }
}
__global__ void FinishAssembly(d::Storage* pointer,fe::NodalAssemblyView view) {
  auto& storage=*pointer;
  if(storage.control.status==Code::Ok) storage.result.diagnostics.valid=true;
  if(storage.control.status!=Code::Ok)
    fe::RecordNodalAssemblyFailure(view,Status::kInvalidArgument,storage.control.node);
}
__global__ void CopyAcceptedBase(d::Storage* pointer) {
  auto& storage=*pointer;
  if(storage.control.status==Code::Ok)
    d::CopyBase(storage,blockIdx.x*blockDim.x+threadIdx.x,gridDim.x*blockDim.x);
}
__global__ void BeginCandidate(d::Storage* pointer,m::Sidecar side,fe::NodalPreparedView view) {
  auto& storage=*pointer;
  storage.control={};
  side.summary->parent_failure=~0ull;
  side.summary->points_admitted=false;
  if(!storage.base.diagnostics.valid || storage.base.diagnostics.attempt!=view.attempt ||
      storage.base.diagnostics.base_epoch!=view.kinematics.base_epoch)
    d::Fail(storage.control,Code::StaleAttempt);
  else side.summary->points_admitted=true;
}
__global__ void FinishCandidate(d::Storage* pointer,m::Sidecar side,fe::NodalPreparedView view) {
  auto& storage=*pointer;
  if(storage.control.status==Code::Ok &&
      d::MeasureInterval(storage,view) && m::RemovedPotential(storage,side)) {
    storage.result.diagnostics.stiffness_rate_bound=side.summary->rate;
    storage.result.diagnostics.valid=true;
  }
}
NodalWallDiagnostics Identity(const NodalWallDeviceConfig& config,const fe::NodalAssemblyView& view) {
  NodalWallDiagnostics result;
  result.owner_id=view.owner_id;
  result.configuration_id=config.configuration_id;
  result.qualification_id=config.qualification_id;
  result.wall_binding_id=config.wall_binding_id;
  result.base_epoch=view.accepted.base_epoch;
  result.attempt=view.attempt;
  result.phase=NodalWallDevicePhase::AcceptedBase;
  result.scheme=view.temporal_scheme;
  result.velocity_phase=view.velocity_phase;
  result.time=result.base_time=view.position_time;
  result.velocity_time=result.base_velocity_time=view.velocity_time;
  return result;
}
}
NodalWallDeviceReport NodalWallMappedContact::Impl::Check(cudaError_t code) {
  if(code==cudaSuccess) return {Code::Ok,"OK"};
  usable=false;
  has_base=false;
  has_results=false;
  return {Code::DeviceFailure,cudaGetErrorString(code)};
}
NodalWallDeviceReport NodalWallMappedContact::Impl::ReadControl() {
  auto report=Check(cudaGetLastError());
  if(report.status!=Code::Ok) return report;
  const auto* pointer=reinterpret_cast<const unsigned char*>(device)+offsetof(d::Storage,control);
  report=Check(cudaMemcpyAsync(&control,pointer,sizeof(control),cudaMemcpyDeviceToHost,stream));
  if(report.status!=Code::Ok) return report;
  report=Check(cudaStreamSynchronize(stream));
  if(report.status!=Code::Ok) return report;
  return {control.status,"Mapped wall numerical stage completed",control.node,control.parent,control.point};
}
NodalWallDeviceReport NodalWallMappedContact::AssembleAccepted(fe::FENodalState& owner,
    const fe::NodalTrialToken& token,const fe::NodalAssemblyView& view,NodalWallMappedDiagnostics* output) {
  if(!impl_) return {Code::NotInitialized,"Mapped wall is not initialized"};
  auto& state=*impl_;
  state.has_base=false;
  state.has_results=false;
  if(!state.usable) return {Code::DeviceFailure,"Mapped wall is poisoned"};
  auto report=state.Authenticate(owner);
  if(report.status!=Code::Ok) return report;
  using fe::trial_identity::Disjoint;
  if(!state.OutputDisjoint(output,sizeof(*output)) || !Disjoint(output,sizeof(*output),this,sizeof(*this)) ||
      !Disjoint(output,sizeof(*output),&token,sizeof(token)) || !Disjoint(output,sizeof(*output),&view,sizeof(view)))
    return {Code::InvalidInput,"Mapped wall output overlaps source, owner or attempt"};
  const auto stamp=owner.accepted();
  if(!fe::native_physical_coefficients::SameOwnerScope(state.config.owner,stamp) ||
      view.attempt<=state.last_attempt || (!state.last_attempt && stamp.epoch!=0) ||
      (state.last_attempt && (stamp.epoch<state.base_stamp.epoch || stamp.epoch-state.base_stamp.epoch>1)))
    return {Code::StaleAttempt,"Mapped wall base is skipped or already consumed"};
  fe::NodalCinAssemblyView cin;
  const auto borrowed=fe::shell_physical_owner::BorrowAssembly(owner,token,stamp,view,state.witnesses,&cin);
  if(borrowed.status!=fe::NodalStatus::Ok)
    return {borrowed.status==fe::NodalStatus::DeviceFailure?Code::DeviceFailure:Code::StaleAttempt,borrowed.message};
  state.stream=view.stream;
  report=state.CaptureActivity(nullptr,nullptr);
  if(report.status!=Code::Ok) return report;
  report=state.CaptureBodies();
  if(report.status!=Code::Ok) return report;
  state.base=view;
  state.base_stamp=stamp;
  state.last_attempt=view.attempt;
  const auto nodes=state.shadow.model.node_count,parents=state.shadow.model.parent_count;
  BeginAssembly<<<1,1,0,state.stream>>>(state.device,state.remote,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  m::parallel::Evaluate(state.device,state.remote,view.accepted,Identity(state.config,view),
      nodes,parents,state.stream,true);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  CheckResponse<<<1,1,0,state.stream>>>(state.device,state.remote,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  m::parallel::Scatter(state.device,state.remote,view,cin,nodes,state.stream);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  FinishAssembly<<<1,1,0,state.stream>>>(state.device,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  CopyAcceptedBase<<<m::parallel::Blocks(nodes>parents?nodes:parents),d::Workers,0,state.stream>>>(state.device);
  report=state.ReadControl();
  if(report.status!=Code::Ok) return report;
  NodalWallMappedDiagnostics next;
  report=state.ReadDiagnostics(false,next);
  if(report.status!=Code::Ok) return report;
  state.has_base=true;
  state.has_results=true;
  state.available=next;
  *output=next;
  return {Code::Ok,"Actual mapped force and CIN stiffness assembled once"};
}
NodalWallDeviceReport NodalWallMappedContact::EvaluateCandidate(fe::FENodalState& owner,
    const fe::NodalTrialToken& token,const fe::NodalPreparedView& view,
    const fe::ShellPhysicalDiagnostics& materials,NodalWallMappedDiagnostics* output) {
  if(!impl_) return {Code::NotInitialized,"Mapped wall is not initialized"};
  auto& state=*impl_;
  state.has_results=false;
  if(!state.usable) return {Code::DeviceFailure,"Mapped wall is poisoned"};
  auto report=state.Authenticate(owner);
  if(report.status!=Code::Ok) return report;
  using fe::trial_identity::Disjoint;
  if(!state.OutputDisjoint(output,sizeof(*output)) || !Disjoint(output,sizeof(*output),this,sizeof(*this)) ||
      !Disjoint(output,sizeof(*output),&token,sizeof(token)) || !Disjoint(output,sizeof(*output),&view,sizeof(view)) ||
      !Disjoint(output,sizeof(*output),&materials,sizeof(materials)))
    return {Code::InvalidInput,"Mapped candidate output overlaps input/source"};
  if(!state.has_base || view.attempt!=state.last_attempt || view.attempt<=state.last_candidate ||
      view.stream!=state.stream || view.proposed_time!=state.base_stamp.time+state.config.owner.fixed_dt ||
      view.kick_dt!=(state.base_stamp.epoch?state.config.owner.fixed_dt:.5*state.config.owner.fixed_dt))
    return {Code::StaleAttempt,"Missing same-attempt mapped accepted force"};
  const auto authenticated=fe::native_physical_coefficients::AuthenticatePrepared(owner,token,state.base_stamp,view);
  if(authenticated.status!=fe::NodalStatus::Ok)
    return {authenticated.status==fe::NodalStatus::DeviceFailure?Code::DeviceFailure:Code::StaleAttempt,authenticated.message};
  state.last_candidate=view.attempt;
  report=state.CaptureActivity(&token,&materials);
  if(report.status!=Code::Ok) return report;
  auto identity=Identity(state.config,state.base);
  identity.phase=NodalWallDevicePhase::PreparedCandidate;
  identity.time=view.proposed_time;
  identity.velocity_time=view.velocity_time;
  identity.velocity_phase=view.velocity_phase;
  identity.kick_dt=view.kick_dt;
  BeginCandidate<<<1,1,0,state.stream>>>(state.device,state.remote,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  m::parallel::Evaluate(state.device,state.remote,view.kinematics,identity,
      state.shadow.model.node_count,state.shadow.model.parent_count,state.stream,false);
  if(cudaPeekAtLastError()!=cudaSuccess) return state.ReadControl();
  FinishCandidate<<<1,1,0,state.stream>>>(state.device,state.remote,view);
  report=state.ReadControl();
  if(report.status!=Code::Ok) return report;
  NodalWallMappedDiagnostics next;
  report=state.ReadDiagnostics(true,next);
  if(report.status!=Code::Ok) return report;
  state.has_results=true;
  state.available=next;
  *output=next;
  return {Code::Ok,"Same-mask candidate work and separate removal potential staged"};
}
} // namespace tlfea::contact
