// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchMeasurement.h"
#include "Type25BatchStorage.h"
#include "Type25Math.h"
#include "../../solvers/NodalForceAssembly.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::type25::batch_detail {
namespace {
namespace sc=tlfea::contact;
__global__ void MarkFailure(NodalAssemblyView view) { RecordNodalAssemblyFailure(view,sc::Status::kInvalidArgument); }
__device__ bool ValidateNodes(Storage& s,const NodalAssemblyView& v,bool initial) {
  if(!native_physical_coefficients::Admitted(s.model.config.owner.rigid_groups,v.rigid_groups,v.mass.model,s.model.config.owner.node_count)) {
    s.control.status=BatchStatus::InvalidMass;return false;
  }
  for(std::size_t n=0;n<s.model.config.owner.node_count;++n) {
    const double im=v.mass.inverse_mass[n],ij=v.inverse_inertia[n];const auto& node=s.model.nodes[n];
    if(v.mass.fixed[n]||v.translation_fixed_bits[n]||v.rotation_fixed[n]||!detail::Positive(im)||!detail::Positive(ij)||
       ::fabs(im*node.mass-1)>1e-12||::fabs(ij*node.inertia-1)>1e-12) {
      s.control.status=BatchStatus::InvalidMass;s.control.node=static_cast<std::uint32_t>(n);return false;
    }
    const auto x=shell_batch_fields::ReadVector(v.accepted.position_xyz,n),velocity=shell_batch_fields::ReadVector(v.accepted.velocity_xyz,n);
    const auto omega=shell_batch_fields::ReadVector(v.accepted.angular_velocity_xyz,n);const auto* q=v.accepted.orientation_wxyz+4*n;
    if(!tl::math::fixed3::Finite(x)||!tl::math::fixed3::Finite(velocity)||!tl::math::fixed3::Finite(omega)||
       !tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]})||
       (initial&&!shell_startup_detail::MatchesInitialNode(s.model.config.startup,x,node.reference,velocity,omega,q))) {
      s.control.status=BatchStatus::InvalidInput;s.control.node=static_cast<std::uint32_t>(n);return false;
    }
  }
  return true;
}
__global__ void Assemble(Storage* storage,const Slab* accepted,NodalAssemblyView view,bool initial) {
  auto& s=*storage;s.control={};
  if(view.result->base_epoch!=view.accepted.base_epoch||view.result->attempt!=view.attempt||
     view.bounds->base_epoch!=view.accepted.base_epoch||view.bounds->attempt!=view.attempt||
     !view.bounds->initialized||!view.bounds->valid||view.bounds->sealed||view.result->status!=sc::Status::kOk)
    s.control.status=BatchStatus::AssemblyFailure;
  else if(ValidateNodes(s,view,initial)) {
    // One writer, connection then endpoint order. Shared endpoint contributions
    // retain source order and exactly the same scalar accumulation as the host.
    for(std::size_t e=0;e<s.model.config.element_count;++e) {
      const auto* rhs=accepted->element[e].endpoints;
      const Vec3 force[2]={rhs[0].force_N,rhs[1].force_N},couple[2]={rhs[0].couple_Nm,rhs[1].couple_Nm};
      if(AccumulateNodalForces<2>(s.model.elements[e].nodes,force,couple,view.forces,+1)!=NodalForceAssemblyStatus::Success) {
        s.control.status=BatchStatus::AssemblyFailure;s.control.element=static_cast<std::uint32_t>(e);break;
      }
    }
  }
  if(s.control.status!=BatchStatus::Success)RecordNodalAssemblyFailure(view,sc::Status::kInvalidArgument,s.control.node);
}
__global__ void CandidateElements(Storage* storage,const Slab* accepted,Slab* trial,NodalPreparedView view) {
  auto& s=*storage;const std::size_t first=blockIdx.x*blockDim.x+threadIdx.x,stride=gridDim.x*blockDim.x;
  for(std::size_t e=first;e<s.model.config.element_count;e+=stride) {
    const auto& element=s.model.elements[e];EndpointKinematics nodes[2];
    for(unsigned i=0;i<2;++i) {
      const auto n=element.nodes[i];nodes[i]={shell_batch_fields::ReadVector(view.kinematics.position_xyz,n),
        shell_batch_fields::ReadVector(view.kinematics.velocity_xyz,n),shell_batch_fields::ReadVector(view.kinematics.angular_velocity_xyz,n)};
    }
    s.candidate_status[e]=Evaluate(s.model.units,s.model.properties[element.property_index],element.reference,
                                 accepted->element[e].history,nodes,s.model.config.owner.fixed_dt,trial->element[e]);
  }
}
__global__ void FinalizeCandidate(Storage* storage,const Slab* accepted,const Slab* trial,
                                  NodalPreparedView view,BatchDiagnostics identity) {
  auto& s=*storage;Control next;next.diagnostics=identity;
  for(std::size_t e=0;e<s.model.config.element_count;++e)if(s.candidate_status[e]!=Status::Success) {
    next.status=BatchStatus::ElementFailure;next.element=static_cast<std::uint32_t>(e);
    next.element_status=s.candidate_status[e];break;
  }
  if(next.status==BatchStatus::Success) {
    if(!MeasurePrepared(s.model,s.measurement,next.diagnostics)) next.status=BatchStatus::NonfiniteResult;
    else next.diagnostics.valid=true;
  }
  s.control=next;
}
}
void LaunchAssembly(Storage* s,const Slab* accepted,NodalAssemblyView view,bool initial) {Assemble<<<1,1,0,view.stream>>>(s,accepted,view,initial);}
void LaunchCandidate(Storage* s,const Slab* accepted,Slab* trial,NodalPreparedView view,BatchDiagnostics diagnostics,std::size_t count) {
  constexpr unsigned threads=64;const unsigned blocks=1u+static_cast<unsigned>((count-1)/threads);
  CandidateElements<<<blocks,threads,0,view.stream>>>(s,accepted,trial,view);
  if(cudaPeekAtLastError()!=cudaSuccess)return;
  LaunchMeasurement(s,accepted,trial,view,count);
  if(cudaPeekAtLastError()!=cudaSuccess)return;
  FinalizeCandidate<<<1,1,0,view.stream>>>(s,accepted,trial,view,diagnostics);
}
void LaunchFailure(NodalAssemblyView view) {MarkFailure<<<1,1,0,view.stream>>>(view);}
} // namespace tl::fea::type25::batch_detail
