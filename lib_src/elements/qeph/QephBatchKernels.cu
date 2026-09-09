#include "QephBatchDiagnostics.h"
#include "../../solvers/NodalForceAssembly.h"

namespace tl::fea::qeph::batch_detail {
namespace {
namespace sc=tlfea::contact;
__global__ void MarkFailure(NodalAssemblyView view) { RecordNodalAssemblyFailure(view,sc::Status::kInvalidArgument); }

__device__ bool ValidateNodes(Storage& s,NodalAssemblyView v,bool initial) {
  if(v.mass.model!=sc::TranslationMassModel::kIsotropicLumped) { s.control.status=BatchStatus::InvalidMass; return false; }
  for(unsigned n=0;n<s.model.config.owner.node_count;++n) {
    const double im=v.mass.inverse_mass[n],ij=v.inverse_inertia[n];
    if(v.mass.fixed[n]||v.translation_fixed_bits[n]||v.rotation_fixed[n]||
       !detail::Positive(im)||!detail::Positive(ij)||
       ::fabs(im*s.model.mass[n]-1)>1e-12||::fabs(ij*s.model.inertia[n]-1)>1e-12) {
      s.control.status=BatchStatus::InvalidMass; s.control.node=n; return false;
    }
    const auto x=ReadVector(v.accepted.position_xyz,n),velocity=ReadVector(v.accepted.velocity_xyz,n);
    const auto omega=ReadVector(v.accepted.angular_velocity_xyz,n); const auto* q=v.accepted.orientation_wxyz+4*n;
    if(!FiniteVector(x)||!FiniteVector(velocity)||!FiniteVector(omega)||!tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]})) {
      s.control.status=BatchStatus::InvalidInput; s.control.node=n; return false;
    }
    const auto ref=s.model.initial_position[n];
    if(initial&&(!detail::SameHistoryBits(x.x,ref.x)||!detail::SameHistoryBits(x.y,ref.y)||
       !detail::SameHistoryBits(x.z,ref.z)||velocity.x!=0||velocity.y!=0||velocity.z!=0||
       omega.x!=0||omega.y!=0||omega.z!=0)) {
      s.control.status=BatchStatus::InvalidInput; s.control.node=n; return false;
    }
  }
  return true;
}
__global__ void Assemble(Storage* storage,const Slab* accepted,NodalAssemblyView v,bool initial) {
  auto& s=*storage; s.control={};
  if(v.result->base_epoch!=v.accepted.base_epoch||v.result->attempt!=v.attempt||
     v.bounds->base_epoch!=v.accepted.base_epoch||v.bounds->attempt!=v.attempt||
     !v.bounds->initialized||!v.bounds->valid||v.bounds->sealed||v.result->status!=sc::Status::kOk)
    s.control.status=BatchStatus::AssemblyFailure;
  else if(ValidateNodes(s,v,initial)) {
    for(unsigned e=0;e<s.model.config.element_count;++e) {
      const auto& result=accepted->element[e]; const auto& h=result.proposed_history;
      if(!h.matches_reference(s.model.element[e].reference)||h.stamp().time!=v.position_time||
         h.stamp().sample_index!=v.accepted.base_epoch) {
        s.control.status=BatchStatus::StaleTrial; s.control.element=e; break;
      }
      if(AccumulateNodalForces<4>(s.model.element[e].nodes,result.internal_force,result.internal_couple,v.forces,-1)
         !=NodalForceAssemblyStatus::Success) { s.control.status=BatchStatus::AssemblyFailure; s.control.element=e; break; }
    }
  }
  if(s.control.status!=BatchStatus::Success) RecordNodalAssemblyFailure(v,sc::Status::kInvalidArgument,s.control.node);
}
__global__ void Candidate(Storage* storage,const Slab* accepted,Slab* trial,NodalPreparedView v,BatchDiagnostics identity) {
  auto& s=*storage; s.control={}; s.control.diagnostics=identity;
  for(unsigned e=0;e<s.model.config.element_count;++e) {
    PrescribedInterval interval; interval.base_time=v.base_time; interval.dt=s.model.config.owner.fixed_dt;
    interval.sample_index=v.kinematics.base_epoch+1;
    for(unsigned i=0;i<4;++i) {
      const auto n=s.model.element[e].nodes[i];
      interval.position_endpoint[i]=ReadVector(v.kinematics.position_xyz,n);
      interval.velocity_midpoint[i]=ReadVector(v.kinematics.velocity_xyz,n);
      interval.omega_midpoint[i]=ReadVector(v.kinematics.angular_velocity_xyz,n);
    }
    const auto status=EvaluateForce(s.model.element[e].reference,accepted->element[e].proposed_history,interval,trial->element[e]);
    if(status!=Status::kSuccess) {
      s.control.status=BatchStatus::ElementFailure; s.control.element=e; s.control.element_status=status; return;
    }
  }
  if(!Measure(s.model,*accepted,*trial,v,s.control)) { s.control.status=BatchStatus::NonfiniteResult; return; }
  s.control.diagnostics.valid=true;
}
}
void LaunchAssembly(Storage* s,const Slab* a,NodalAssemblyView v,bool initial) { Assemble<<<1,1,0,v.stream>>>(s,a,v,initial); }
void LaunchCandidate(Storage* s,const Slab* a,Slab* b,NodalPreparedView v,BatchDiagnostics d) { Candidate<<<1,1,0,v.stream>>>(s,a,b,v,d); }
void LaunchFailure(NodalAssemblyView v) { MarkFailure<<<1,1,0,v.stream>>>(v); }
} // namespace tl::fea::qeph::batch_detail
