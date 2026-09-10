#pragma once
#include "NodalWallContactDiagnostics.h"
#include "lib_src/solvers/NodalForceAssembly.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"

namespace tlfea::contact::nodal_wall_device_detail {
namespace fea=tl::fea;
// Avoid whole-result temporaries on one CUDA lane as the collection grows.
// All worker lanes call these helpers; arithmetic fields retain their owning
// default initialization and each record has exactly one writer.
__device__ inline void ResetResult(ActiveResults& result,unsigned parents,unsigned nodes) {
  const unsigned lane=threadIdx.x;
  if (lane==0) result.diagnostics={};
  for (unsigned p=lane;p<parents;p+=Workers) result.parents[p]={};
  for (unsigned n=lane;n<nodes;n+=Workers) {
    result.nodes[n]={}; result.wall_face[n]=0;
  }
}
__device__ inline void CopyBase(Storage& s) {
  const unsigned lane=threadIdx.x;
  if (lane==0) s.base.diagnostics=s.result.diagnostics;
  for (unsigned p=lane;p<s.model.parent_count;p+=Workers) s.base.parents[p]=s.result.parents[p];
  for (unsigned n=lane;n<s.model.node_count;n+=Workers) {
    s.base.nodes[n]=s.result.nodes[n]; s.base.wall_face[n]=s.result.wall_face[n];
  }
}
__device__ inline bool ValidateAssembly(Storage& s,const fea::NodalAssemblyView& v) {
  if (v.result->base_epoch!=v.accepted.base_epoch || v.result->attempt!=v.attempt ||
      v.bounds->base_epoch!=v.accepted.base_epoch || v.bounds->attempt!=v.attempt ||
      !v.bounds->initialized || !v.bounds->valid || v.bounds->sealed || v.result->status!=Status::kOk)
    return Fail(s.control,Code::AssemblyFailure);
  if (!fea::native_physical_coefficients::Admitted(s.model.config.owner.rigid_groups,v.rigid_groups,
      v.mass.model,s.model.config.owner.node_count)) return Fail(s.control,Code::InvalidMass);
  for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node;
    if (v.mass.inverse_mass[n]!=s.model.inverse_mass[n] || v.mass.fixed[n]!=s.model.fixed[n] ||
        v.translation_fixed_bits[n]!=(s.model.fixed[n]?7:0)) return Fail(s.control,Code::InvalidMass,n);
    if (v.accepted.base_epoch==0) {
      const auto initial=s.model.initial_position[n]; const auto* x=v.accepted.position_xyz+3*n;
      if (x[0]!=initial.x || x[1]!=initial.y || x[2]!=initial.z) return Fail(s.control,Code::GeometryFailure,n);
    }
  }
  return true;
}
// All lanes enter each barrier. Compact slots are selected through immutable
// native shares; no parent-count constants or source fixture arrays are copied.
__device__ inline void Evaluate(Storage& s,const fea::DeviceNodalKinematicsView& k,
                               const NodalWallDiagnostics& identity) {
  const auto lane=threadIdx.x;
  ResetResult(s.result,s.model.parent_count,s.model.node_count);
  if (lane==0) s.result.diagnostics=identity;
  for (unsigned i=lane;i<s.model.node_count;i+=Workers) s.node_status[i]={};
  __syncthreads();
  if (s.control.status!=Code::Ok) return;
  const auto count=static_cast<std::uint32_t>(k.node_count); // Active host binding checked the complete owner extent.
  const VectorView x{k.position_xyz,count,3,1},v{k.velocity_xyz,count,3,1};
  // Evaluate the local law with native physical coefficients. For a rigid
  // group its rate is diagnostic, not the response of the coupled system.
  const LumpedTranslationMassView mass{s.model.inverse_mass,s.model.fixed,count,k.base_epoch,
                                      TranslationMassModel::kIsotropicLumped};
  for (unsigned compact=lane;compact<s.model.node_count;compact+=Workers) {
    const unsigned n=s.model.nodes[compact].node; const auto position=x.at(n),velocity=v.at(n);
    auto& status=s.node_status[compact]; const auto initial=s.model.initial_position[n];
    if (!Inside(position,s.model.coverage.physical) || !IsFinite(velocity)) Fail(status,Code::GeometryFailure,n);
    else if (s.model.fixed[n] && (position.x!=initial.x || position.y!=initial.y || position.z!=initial.z))
      Fail(status,Code::GeometryFailure,n);
    else {
      unsigned face=UINT32_MAX; TrianglePointGeometry point;
      if (s.model.query.FindOwner({s.model.config.law.wall_x,position.y,position.z},&face,&point)!=Status::kOk ||
          face==UINT32_MAX) Fail(status,Code::GeometryFailure,n);
      else {
        s.result.wall_face[compact]=s.model.face_ids[face]; NodalWallPointResult node;
        // Immutable source incidence avoids scanning all parents per node.
        // Stored slots retain the original p/l order and omit T3 padding.
        for (unsigned i=s.model.incident_offsets[compact];i<s.model.incident_offsets[compact+1];++i) {
          if (status.status!=Code::Ok) break;
          const unsigned slot=s.model.incident_slots[i],p=slot/4,l=slot%4;
          NodalWallPointResult share;
          const auto code=EvaluateNodalWallPoint({n,s.model.parents[p].share},position,velocity,
                                                mass,s.model.config.law,identity.attempt,&share);
          if (code.status!=NodalWallStatus::Ok) { status.point=code; Fail(status,Code::PointFailure,n,p); }
          else if (!nodal_wall_reduction::AddShare(node,share)) Fail(status,Code::NonFiniteArithmetic,n,p);
          else s.shares[4*p+l]=share;
        }
        if (status.status==Code::Ok) {
          node.force_world={-node.force.value,0,0}; node.wall_reaction={node.force.value,0,0};
          node.wall_moment=geometry_detail::Cross(node.wall_point,node.wall_reaction);
          node.surface_power=Dot(node.force_world,velocity); node.local_velocity_first_timestep=0;
          if (!node.valid || !IsFinite(node.wall_moment) || !IsFinite(node.surface_power)) Fail(status,Code::NonFiniteArithmetic,n);
          else s.result.nodes[compact]=node;
        }
      }
    }
  }
  __syncthreads();
  if (lane==0) {
    for (unsigned i=0;i<s.model.node_count;++i) if (s.node_status[i].status!=Code::Ok) {
      s.control=s.node_status[i]; break;
    }
    if (s.control.status==Code::Ok) for (unsigned p=0;p<s.model.parent_count;++p) {
      auto& out=s.result.parents[p]; const auto& parent=s.model.parents[p];
      out.parent_element_id=parent.parent_element_id; out.parent_face_id=parent.parent_face_id;
      out.feature_id=parent.feature_id; out.family=parent.family; out.arity=parent.arity;
      // Stride four preserves the existing storage layout. A T3's fourth
      // share is never read; its unused force output remains zero from reset.
      for (unsigned l=0;l<parent.arity;++l) {
        const auto& share=s.shares[4*p+l]; out.force[l]=share.force;
        if (!nodal_wall_reduction::Sum(out.resultant,share.force) ||
            !nodal_wall_reduction::Sum(out.potential,share.potential)) { Fail(s.control,Code::NonFiniteArithmetic,UINT32_MAX,p); break; }
      }
      if (s.control.status!=Code::Ok) break;
      bool accurate=out.resultant.error<=s.model.config.law.parent_force_error &&
                    out.potential.error<=s.model.config.law.parent_energy_error;
      for (unsigned l=0;l<parent.arity;++l) accurate=accurate && out.force[l].error<=s.model.config.law.parent_force_error;
      if (!accurate) { Fail(s.control,Code::Accuracy,UINT32_MAX,p); break; }
      out.valid=true;
    }
    auto& d=s.result.diagnostics;
    if (s.control.status==Code::Ok) for (unsigned i=0;i<s.model.node_count;++i) {
      const auto& node=s.result.nodes[i];
      if (!nodal_wall_reduction::Sum(d.resultant,node.force) || !nodal_wall_reduction::Sum(d.potential,node.potential)) {
        Fail(s.control,Code::NonFiniteArithmetic,node.node); break;
      }
      d.wall_reaction=Add(d.wall_reaction,node.wall_reaction); d.wall_moment=Add(d.wall_moment,node.wall_moment);
      d.surface_power+=node.surface_power;
      const double penetration=x.at(node.node).x-s.model.config.law.wall_x;
      if (penetration>d.maximum_penetration) d.maximum_penetration=penetration;
      if (!IsFinite(d.wall_reaction) || !IsFinite(d.wall_moment) || !IsFinite(d.surface_power)) {
        Fail(s.control,Code::NonFiniteArithmetic,node.node); break;
      }
    }
    d.node_count=s.model.node_count; d.parent_count=s.model.parent_count; d.stiffness_rate_bound=s.model.rate;
  }
  __syncthreads();
}
__device__ inline bool Scatter(Storage& s,const fea::NodalAssemblyView& v) {
  const auto owner_nodes=s.model.config.owner.node_count;
  double* actual[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                   v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for (unsigned c=0;c<6;++c) for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node;
    s.staged_force[c*owner_nodes+n]=actual[c][n];
  }
  const fea::DeviceNodalForceView staged{s.staged_force,s.staged_force+owner_nodes,s.staged_force+2*owner_nodes,
      s.staged_force+3*owner_nodes,s.staged_force+4*owner_nodes,s.staged_force+5*owner_nodes,
      v.forces.node_count,v.forces.base_epoch};
  for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node; const std::size_t index=n;
    const tl::math::Vec3 force{s.result.nodes[i].force_world.x,0,0},couple{};
    Q4IntegralInterval sum;
    if (fea::AccumulateNodalForces<1>(&index,&force,&couple,staged)!=fea::NodalForceAssemblyStatus::Success ||
        !q4_bounds::Add({actual[0][n],actual[0][n]},{force.x,force.x},&sum) ||
        !Radius(s.staged_force[n],sum,&s.addition_error[i])) return Fail(s.control,Code::AssemblyFailure,n);
  }
  for (unsigned c=0;c<6;++c) for (unsigned i=0;i<s.model.node_count;++i) {
    const auto n=s.model.nodes[i].node; actual[c][n]=s.staged_force[c*owner_nodes+n];
  }
  return true; // Legacy timestep rows are deliberately untouched.
}
} // namespace tlfea::contact::nodal_wall_device_detail
