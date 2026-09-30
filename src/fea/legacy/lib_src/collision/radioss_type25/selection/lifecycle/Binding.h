// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "NormalAccess.h"
#include "../Retained.h"
#include "../NewImpact.h"
#include "../Continuation.h"
#include "../../coefficients/Pair.h"
#include <climits>
namespace tlfea::contact::radioss_type25::selection::lifecycle::detail {
namespace values=selection::detail;
namespace g=geometry_detail;
TL_MATH_HOST_DEVICE inline bool Factors(const Kinematics& current,units_detail::Factors& out) {
  if(current.units==KinematicsUnits::Native) {out={1,1,1,1,1,1,1,1};return true;}
  return current.units==KinematicsUnits::Si&&units_detail::Make(current.native_units,out);
}
TL_MATH_HOST_DEVICE inline Vector Position(const Input& input,std::uint32_t node,
    const units_detail::Factors& units) {
  const auto value=input.current.positions.at(node);
  return input.current.units==KinematicsUnits::Native?Vector{value.x,value.y,value.z}:
      Vector{value.x/units.length,value.y/units.length,value.z/units.length};
}
TL_MATH_HOST_DEVICE inline Vector Velocity(const Input& input,std::uint32_t node,
    const units_detail::Factors& units) {
  const auto value=input.current.velocities.at(node);
  return input.current.units==KinematicsUnits::Native?Vector{value.x,value.y,value.z}:
      Vector{value.x/units.velocity,value.y/units.velocity,value.z/units.velocity};
}
// Called only after complete source/range admission. These concrete factories
// create value packets, not owner leases or candidate-completeness authority.
TL_MATH_HOST_DEVICE inline NativePairInput Pair(const Input& input,std::size_t row,
    int local_main,std::size_t ordinal,const units_detail::Factors& units) {
  const auto& scene=input.source;const auto& secondary=scene.secondary[row];
  const auto& main=scene.mains[local_main-1];NativePairInput pair;
  pair.key={scene.nodes[secondary.node].source_id,scene.generation,row,main.global_id};
  pair.local_main=local_main;pair.occurrence=ordinal;
  pair.secondary=Position(input,secondary.node,units);
  pair.segment_type=main.segment_type;pair.secondary_gap=secondary.gap;
  pair.main_gap_max=main.maximum_gap;pair.main_coefficient=main.coefficient;
  pair.secondary_coefficient=secondary.coefficient;pair.initial_contact_flag=secondary.initial_contact_flag;
  for(unsigned i=0;i<4;++i) {
    const auto node=main.nodes[i];pair.main_node_ids[i]=scene.nodes[node].source_id;
    pair.main_vertices[i]=Position(input,node,units);pair.normal_slot[i]=FaceNormal(input,std::size_t(local_main-1),i);
    pair.neighbors[i]=main.neighbors[i];pair.main_gap[i]=main.gap[i];
    const int reference=main.normal_reference[i];const auto& normal=ReferenceNormal(input,std::size_t(reference-1));
    if(normal.boundary!=0) {
      pair.boundary_ids[i]=std::uint64_t(reference);
      for(unsigned j=0;j<2;++j)pair.vertex_bisector[i][j]=normal.bisector[j];
    }
    // Unbound VTX storage is unconsumed native scratch. Leave its value packet
    // zeros without reading/normalizing a nonexistent boundary normal.
  }
  return pair;
}
TL_MATH_HOST_DEVICE inline NativeContinuationInput Continuation(const Input& input,
    std::size_t row,int local_main,std::size_t ordinal,const int* sliding,
    const units_detail::Factors& units) {
  NativeContinuationInput out;out.pair=Pair(input,row,local_main,ordinal,units);
  out.segment_count=int(input.source.main_count);
  const auto& main=input.source.mains[local_main-1];
  const auto& secondary=input.source.nodes[input.source.secondary[row].node];
  out.secondary_constraint=secondary.constraint;out.secondary_skew=secondary.skew;
  for(unsigned i=0;i<4;++i) {
    out.normal_reference[i]=main.normal_reference[i];out.sliding_reference[i]=sliding[i];
    const auto& node=input.source.nodes[main.nodes[i]];
    out.main_constraint[i]=node.constraint;out.main_skew[i]=node.skew;
  }
  return out;
}
TL_MATH_HOST_DEVICE inline NativeNewImpactInput NewImpact(const Input& input,
    std::size_t row,int local_main,std::size_t ordinal,const units_detail::Factors& units) {
  NativeNewImpactInput out;out.pair=Pair(input,row,local_main,ordinal,units);
  out.segment_count=int(input.source.main_count);out.previous_dt=input.step.previous_dt;
  const auto& main=input.source.mains[local_main-1];
  out.secondary_velocity=Velocity(input,input.source.secondary[row].node,units);
  for(unsigned i=0;i<4;++i)out.main_velocity[i]=Velocity(input,main.nodes[i],units);
  const int partner=values::OppositeLocal(out);
  if(partner>0) {
    const auto& opposite=input.source.mains[partner-1];
    out.opposite.local_main=partner;out.opposite.global_main=opposite.global_id;
    for(unsigned i=0;i<4;++i) {
      out.opposite.main_node_ids[i]=input.source.nodes[opposite.nodes[i]].source_id;
      out.opposite.normal_slot[i]=FaceNormal(input,std::size_t(partner-1),i);out.opposite.neighbors[i]=opposite.neighbors[i];
      const int reference=opposite.normal_reference[i];const auto& normal=ReferenceNormal(input,std::size_t(reference-1));
      if(normal.boundary!=0) {
        out.opposite.boundary_ids[i]=std::uint64_t(reference);
        for(unsigned j=0;j<2;++j)out.opposite.vertex_bisector[i][j]=normal.bisector[j];
      }
    }
  }
  return out;
}
TL_MATH_HOST_DEVICE inline bool SameKey(GeometryRowKey a,GeometryRowKey b) {
  return a.secondary_source_id==b.secondary_source_id&&a.generation==b.generation&&
      a.history_index==b.history_index&&a.main_segment==b.main_segment;
}
TL_MATH_HOST_DEVICE inline Status SelectGeometry(const Input& input,std::size_t row,
    const NativeGeometryHistory& history,Occurrence& occurrence) {
  if(occurrence.secondary<=0)return Status::Ok;
  const auto& scene=input.source;const auto& main=scene.mains[occurrence.local_main-1];
  const auto& secondary=scene.secondary[row];
  const GeometryRowKey key{scene.nodes[secondary.node].source_id,scene.generation,row,main.global_id};
  if(!SameKey(key,occurrence.cache.key)||occurrence.cache.local_main!=occurrence.local_main||
     history.row.irtlm[0]==INT_MIN||(std::int64_t(history.row.irtlm[0])!=main.global_id&&
       -std::int64_t(history.row.irtlm[0])!=main.global_id))return Status::InvalidInput;
  int sector=history.row.irtlm[1]%5;if(sector<0)sector=-sector;
  if(sector<1||sector>4)return Status::UndefinedNativeInput;
  const auto& cache=occurrence.cache.sector[sector-1];
  if(!(cache.defined&ClampedBarycentricDefined))return Status::UndefinedNativeInput;
  NativeScalarCoefficient coefficient;
  const auto status=EvaluateNativePairCoefficient(input.profile.coefficient,
      {main.coefficient,secondary.coefficient,input.profile.minimum_coefficient,input.profile.maximum_coefficient},
      &coefficient);
  if(status!=CoefficientStatus::Ok)return status==CoefficientStatus::UnsupportedProfile?
      Status::UnsupportedProfile:status==CoefficientStatus::NonfiniteResult?Status::NonfiniteResult:Status::InvalidInput;
  occurrence.selected={true,key,occurrence.local_main,sector,history.row.irtlm[1],cache.lb,cache.lc,coefficient.value};
  return Status::Ok;
}
TL_MATH_HOST_DEVICE inline NativeGeometryInput Geometry(const Input& input,
    const Occurrence& occurrence,const units_detail::Factors& units) {
  const auto row=occurrence.selected.key.history_index;
  const auto pair=Pair(input,row,occurrence.selected.local_main,occurrence.cache.occurrence,units);
  NativeGeometryInput out;out.key=pair.key;out.secondary=pair.secondary;
  out.selection_code=occurrence.selected.selection_code;out.lb=occurrence.selected.lb;out.lc=occurrence.selected.lc;
  out.incoming_stiffness=occurrence.selected.incoming_stiffness;
  out.secondary_gap=pair.secondary_gap;out.segment_type=pair.segment_type;
  for(unsigned i=0;i<4;++i) {
    out.main_node_ids[i]=pair.main_node_ids[i];out.main_vertices[i]=pair.main_vertices[i];
    out.corner_normal[i]=pair.normal_slot[i];out.main_gap[i]=pair.main_gap[i];
    out.neighbors[i]=pair.neighbors[i];out.boundary_ids[i]=pair.boundary_ids[i];
    for(unsigned j=0;j<2;++j)out.vertex_bisector[i][j]=pair.vertex_bisector[i][j];
  }
  return out;
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle::detail
