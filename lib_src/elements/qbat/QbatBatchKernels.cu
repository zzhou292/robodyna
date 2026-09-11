// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchStorage.h"
#include "QbatBatchAdvance.h"
#include "QbatBatchMeasure.h"
#include "../../solvers/NodalForceAssembly.h"
#include "../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::qbat::batch_detail {
namespace {
namespace contact=tlfea::contact;
__global__ void MarkFailure(NodalAssemblyView view) {
  RecordNodalAssemblyFailure(view,contact::Status::kInvalidArgument);
}
__device__ bool ValidateNodes(Storage& storage,const NodalAssemblyView& view,bool initial) {
  auto& control=storage.control;
  const auto& model=storage.model;
  if(!native_physical_coefficients::Admitted(model.config.owner.rigid_groups,view.rigid_groups,
      view.mass.model,model.config.owner.node_count)) {
    control.status=BatchStatus::InvalidMass;
    return false;
  }
  for(std::size_t node=0;node<model.config.owner.node_count;++node) {
    const double inverse_mass=view.mass.inverse_mass[node];
    const double inverse_inertia=view.inverse_inertia[node];
    if(view.mass.fixed[node]||view.translation_fixed_bits[node]||view.rotation_fixed[node]||
        !detail::Positive(inverse_mass)||!detail::Positive(inverse_inertia)||
        ::fabs(inverse_mass*model.mass[node]-1)>1e-12||::fabs(inverse_inertia*model.inertia[node]-1)>1e-12) {
      control.status=BatchStatus::InvalidMass;
      control.node=static_cast<std::uint32_t>(node);
      return false;
    }
    const auto x=shell_batch_fields::ReadVector(view.accepted.position_xyz,node);
    const auto velocity=shell_batch_fields::ReadVector(view.accepted.velocity_xyz,node);
    const auto omega=shell_batch_fields::ReadVector(view.accepted.angular_velocity_xyz,node);
    const auto* quaternion=view.accepted.orientation_wxyz+4*node;
    if(!detail::Finite(x)||!detail::Finite(velocity)||!detail::Finite(omega)||
        !tl::math::UnitQuaternion({quaternion[0],quaternion[1],quaternion[2],quaternion[3]})||
        (initial&&!shell_startup_detail::MatchesInitialNode(model.config.startup,x,
            model.initial_position[node],velocity,omega,quaternion))) {
      control.status=BatchStatus::InvalidInput;
      control.node=static_cast<std::uint32_t>(node);
      return false;
    }
  }
  return true;
}
__global__ void Assemble(Storage* storage,const Slab* accepted,NodalAssemblyView view,bool initial) {
  auto& s=*storage;
  s.control={};
  if(view.result->base_epoch!=view.accepted.base_epoch||view.result->attempt!=view.attempt||
      view.bounds->base_epoch!=view.accepted.base_epoch||view.bounds->attempt!=view.attempt||
      !view.bounds->initialized||!view.bounds->valid||view.bounds->sealed||view.result->status!=contact::Status::kOk) {
    s.control.status=BatchStatus::AssemblyFailure;
  } else if(ValidateNodes(s,view,initial)) {
    // One writer preserves native parent/local order for shared physical nodes.
    for(std::size_t parent=0;parent<s.model.config.element_count;++parent) {
      const auto& result=accepted->element[parent];
      if(!ValidResult(result,s.model.element[parent].material,view.position_time,view.accepted.base_epoch)) {
        s.control.status=BatchStatus::NonfiniteResult;
        s.control.element=static_cast<std::uint32_t>(parent);
        break;
      }
      if(AccumulateNodalForces<4>(s.model.element[parent].nodes,result.internal_force_n,
          result.internal_couple_nm,view.forces,-1)!=NodalForceAssemblyStatus::Success) {
        s.control.status=BatchStatus::AssemblyFailure;
        s.control.element=static_cast<std::uint32_t>(parent);
        break;
      }
    }
  }
  if(s.control.status!=BatchStatus::Success) {
    RecordNodalAssemblyFailure(view,contact::Status::kInvalidArgument,s.control.node);
  }
}
__global__ void CandidateElements(Storage* storage,const Slab* accepted,Slab* trial,NodalPreparedView view) {
  auto& s=*storage;
  const std::size_t first=blockIdx.x*blockDim.x+threadIdx.x;
  const std::size_t stride=gridDim.x*blockDim.x;
  for(std::size_t parent=first;parent<s.model.config.element_count;parent+=stride) {
    PrescribedInterval interval;
    interval.base_time=view.base_time;
    interval.dt=s.model.config.owner.fixed_dt;
    interval.sample_index=view.kinematics.base_epoch+1;
    shell_batch_fields::Gather(s.model.element[parent].nodes,view.kinematics,
        interval.position_endpoint,interval.velocity_midpoint,interval.omega_midpoint);
    s.candidate_status[parent]=Advance(s.model.element[parent],accepted->element[parent],interval,trial->element[parent]);
  }
}
__global__ void FinalizeCandidate(Storage* storage,const Slab* accepted,const Slab* trial,
    NodalPreparedView view,BatchDiagnostics identity) {
  auto& s=*storage;
  s.control={};
  s.control.diagnostics=identity;
  for(std::size_t parent=0;parent<s.model.config.element_count;++parent) {
    if(s.candidate_status[parent]==Status::kSuccess) continue;
    s.control.status=BatchStatus::ElementFailure;
    s.control.element=static_cast<std::uint32_t>(parent);
    s.control.element_status=s.candidate_status[parent];
    return;
  }
  if(!Measure(s.model,*accepted,*trial,view,s.control.diagnostics)) {
    s.control.status=BatchStatus::NonfiniteResult;
    return;
  }
  s.control.diagnostics.valid=true;
}
} // namespace
void LaunchAssembly(Storage* storage,const Slab* accepted,NodalAssemblyView view,bool initial) {
  Assemble<<<1,1,0,view.stream>>>(storage,accepted,view,initial);
}
void LaunchCandidate(Storage* storage,const Slab* accepted,Slab* trial,NodalPreparedView view,
    BatchDiagnostics diagnostics,std::size_t count) {
  constexpr unsigned threads=64;
  const unsigned blocks=1u+static_cast<unsigned>((count-1)/threads);
  CandidateElements<<<blocks,threads,0,view.stream>>>(storage,accepted,trial,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  FinalizeCandidate<<<1,1,0,view.stream>>>(storage,accepted,trial,view,diagnostics);
}
void LaunchFailure(NodalAssemblyView view) { MarkFailure<<<1,1,0,view.stream>>>(view); }
} // namespace tl::fea::qbat::batch_detail
