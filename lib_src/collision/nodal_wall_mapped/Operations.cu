// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Kernels.cuh"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
namespace tlfea::contact {
namespace m=nodal_wall_mapped;
namespace d=nodal_wall_device_detail;
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
namespace {
__global__ void Assemble(d::Storage* pointer,m::Sidecar side,fe::NodalAssemblyView view,
    fe::NodalCinAssemblyView cin,NodalWallDiagnostics identity) {
  auto& storage=*pointer;
  d::ResetResult(storage.base,storage.model.parent_count,storage.model.node_count);
  if(threadIdx.x==0) {
    storage.control={};
    *side.summary={};
    m::ValidateAssembly(storage,side,view);
  }
  __syncthreads();
  d::Evaluate<true>(storage,view.accepted,identity,side.accepted);
  if(threadIdx.x==0) {
    if(storage.control.status==Code::Ok && m::Response(storage,side,view.accepted)) {
      double step=0,frequency=0;
      if(!mass_detail::Upper(::sqrt(side.summary->rate),&frequency) ||
          !mass_detail::UpperProduct(storage.model.config.owner.fixed_dt,frequency,&step) || step>=1.6)
        d::Fail(storage.control,Code::StepTooLarge);
    }
    if(storage.control.status==Code::Ok && m::StageStiffness(storage,side,cin) &&
        d::Scatter(storage,view,false)) {
      d::PublishScatter(storage,view);
      for(unsigned i=0;i<storage.model.node_count;++i)
        cin.translational_stiffness[storage.model.nodes[i].node]=side.stiffness[i];
      storage.result.diagnostics.valid=true;
    }
    if(storage.control.status!=Code::Ok)
      fe::RecordNodalAssemblyFailure(view,Status::kInvalidArgument,storage.control.node);
  }
  __syncthreads();
  if(storage.control.status==Code::Ok) d::CopyBase(storage);
}
__global__ void Candidate(d::Storage* pointer,m::Sidecar side,fe::NodalPreparedView view,
    NodalWallDiagnostics identity) {
  auto& storage=*pointer;
  if(threadIdx.x==0) {
    storage.control={};
    if(!storage.base.diagnostics.valid || storage.base.diagnostics.attempt!=view.attempt ||
        storage.base.diagnostics.base_epoch!=view.kinematics.base_epoch)
      d::Fail(storage.control,Code::StaleAttempt);
  }
  __syncthreads();
  d::Evaluate<true>(storage,view.kinematics,identity,side.accepted);
  if(threadIdx.x==0 && storage.control.status==Code::Ok &&
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
  Assemble<<<1,d::Workers,0,state.stream>>>(state.device,state.remote,view,cin,Identity(state.config,view));
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
  Candidate<<<1,d::Workers,0,state.stream>>>(state.device,state.remote,view,identity);
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
