#include "T3BatchDiagnostics.h"
#include "mapped/Result.h"
#include "mapped/SerialDiagnostics.h"
#include "T3LayeredJ2.h"
#include "T3BatchFailureSection.h"
#include "T3BatchOnePointSection.h"
#include "../../solvers/NodalForceAssembly.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::t3::batch_detail {
namespace {
namespace sc=tlfea::contact;
__global__ void MarkFailure(NodalAssemblyView view) { RecordNodalAssemblyFailure(view,sc::Status::kInvalidArgument); }

__device__ bool ValidateNodes(Storage& s,NodalAssemblyView v,bool initial) {
  if(!native_physical_coefficients::Admitted(s.model.config.owner.rigid_groups,v.rigid_groups,
      v.mass.model,s.model.config.owner.node_count)) { s.control.status=BatchStatus::InvalidMass; return false; }
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
    if(initial&&!shell_startup_detail::MatchesInitialNode(s.model.config.startup,x,ref,velocity,omega,q)) {
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
      if(AccumulateNodalForces<3>(s.model.element[e].nodes,result.internal_force,result.internal_couple,v.forces,-1)
         !=NodalForceAssemblyStatus::Success) { s.control.status=BatchStatus::AssemblyFailure; s.control.element=e; break; }
    }
    if(s.control.status==BatchStatus::Success&&initial&&!s.model.joined&&
       s.model.config.startup.kind==BatchStartupKind::ReferenceUniformTranslation) {
      double kinetic=0;
      for(unsigned n=0;n<s.model.config.owner.node_count;++n)
        if(!shell_startup_detail::AddInitialTranslationKinetic(s.model.mass[n],ReadVector(v.accepted.velocity_xyz,n),kinetic)) {
          s.control.status=BatchStatus::NonfiniteResult; s.control.node=n; break;
        }
      if(s.control.status==BatchStatus::Success) {
        s.control.diagnostics.kinetic_translation=kinetic;
        s.control.diagnostics.valid=true;
      }
    }
  }
  if(s.control.status!=BatchStatus::Success) RecordNodalAssemblyFailure(v,sc::Status::kInvalidArgument,s.control.node);
}
__global__ void CandidateElements(Storage* storage,const Slab* accepted,Slab* trial,NodalPreparedView v,
    shell_batch_plasticity_detail::DeviceStorage* plasticity,unsigned accepted_slab,
    shell_batch_plasticity_detail::MixedDeviceStorage* mixed,
    shell_batch_plasticity_detail::FailureDeviceStorage* failure,
    shell_batch_plasticity_detail::OnePointDeviceStorage* one_point) {
  auto& s=*storage;
  auto* element_status=s.candidate_status;
  // Parent work is independent: each thread owns its result, section history
  // and status. No control or shared-node force reduction is written here.
  const unsigned first=blockIdx.x*blockDim.x+threadIdx.x;
  const unsigned stride=gridDim.x*blockDim.x;
  for(unsigned e=first;e<s.model.config.element_count;e+=stride) {
    element_status[e]=Status::kSuccess;
    PrescribedInterval interval; interval.base_time=v.base_time; interval.dt=s.model.config.owner.fixed_dt;
    interval.sample_index=v.kinematics.base_epoch+1;
    shell_batch_fields::Gather(s.model.element[e].nodes,v.kinematics,
      interval.position,interval.velocity,interval.angular_velocity);
    if(s.model.mapped&&mixed&&mixed->law[e]==ShellSectionLaw::RigidSkin) {
      element_status[e]=mapped::AdvanceSkin(s.model.element[e].reference,accepted->element[e],interval,trial->element[e]);
      if(element_status[e]==Status::kSuccess&&failure) {
        failure->state[1u-accepted_slab][e]=ShellBatchFailureState{};
      }
    } else if(mixed&&mixed->law[e]==ShellSectionLaw::Law44Nip1)
      element_status[e]=failure&&one_point?
        EvaluateOnePointSection(s.model.element[e].reference,accepted->element[e].proposed_history,
          interval,*mixed,*failure,*one_point,accepted_slab,e,trial->element[e]):Status::kInvalidInput;
    else if(failure&&mixed)
      element_status[e]=EvaluateFailureSection(s.model.element[e].reference,accepted->element[e].proposed_history,
        interval,*mixed,*failure,accepted_slab,e,trial->element[e]);
    else if(mixed)
      element_status[e]=EvaluateMixedSection(s.model.element[e].reference,accepted->element[e].proposed_history,
        interval,*mixed,accepted_slab,e,trial->element[e]);
    else if(!plasticity)
      element_status[e]=EvaluateForce(s.model.element[e].reference,accepted->element[e].proposed_history,interval,trial->element[e]);
    else {
      const auto& old_section=plasticity->section[accepted_slab][e];
      const LayeredJ2History base{accepted->element[e].proposed_history,old_section.history};
      LayeredJ2ForceTrial candidate;
      element_status[e]=EvaluateLayeredJ2Force(s.model.element[e].reference,plasticity->parameters[e],base,interval,candidate);
      if(element_status[e]==Status::kSuccess) {
        ShellBatchSectionState section;
        section.history=candidate.proposed_section; section.diagnostics=candidate.section_diagnostics;
        section.cumulative_plastic_work_J=old_section.cumulative_plastic_work_J+
            candidate.section_diagnostics.plastic_work_density_increment*
            base.shell.data().thickness*candidate.force.kinematics.area;
        if(!tl::math::Finite(section.cumulative_plastic_work_J)) element_status[e]=Status::kNonfiniteResult;
        else { trial->element[e]=candidate.force; plasticity->section[1u-accepted_slab][e]=section; }
      }
    }
  }
}
__global__ void FinalizeCandidate(Storage* storage,const Slab* accepted,const Slab* trial,
                                   NodalPreparedView v,BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  mapped::FinalizeSerial(*storage, *accepted, *trial, v, identity, mixed, storage->control);
}
}
void LaunchAssembly(Storage* s,const Slab* a,NodalAssemblyView v,bool initial) { Assemble<<<1,1,0,v.stream>>>(s,a,v,initial); }
void LaunchCandidate(Storage* s,const Slab* a,Slab* b,NodalPreparedView v,BatchDiagnostics d,
    shell_batch_plasticity_detail::DeviceStorage* plasticity,unsigned accepted_slab,std::size_t element_count,
    shell_batch_plasticity_detail::MixedDeviceStorage* mixed,
    shell_batch_plasticity_detail::FailureDeviceStorage* failure,
    shell_batch_plasticity_detail::OnePointDeviceStorage* one_point, bool mapped) {
  // The private caller supplies its immutable startup-admitted active count,
  // never a device-header dereference or a new independent capacity setting.
  constexpr unsigned threads=64;
  const unsigned blocks=1u+static_cast<unsigned>((element_count-1)/threads);
  CandidateElements<<<blocks,threads,0,v.stream>>>(s,a,b,v,plasticity,accepted_slab,mixed,failure,one_point);
  // Preserve the first launch error for ReadControl and never finalize stale
  // parent slots after a rejected launch. Stream execution errors are checked
  // by the existing control readback/synchronization before any publication.
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  if (mapped) LaunchMappedObserverDiagnostics(s, a, b, v, d, mixed);
  else FinalizeCandidate<<<1,1,0,v.stream>>>(s,a,b,v,d,mixed);
}
void LaunchFailure(NodalAssemblyView v) { MarkFailure<<<1,1,0,v.stream>>>(v); }
} // namespace tl::fea::t3::batch_detail
