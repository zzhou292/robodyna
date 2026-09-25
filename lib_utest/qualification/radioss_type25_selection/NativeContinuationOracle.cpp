// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <stdexcept>
#include <vector>
namespace type25_selection_test {
extern "C" void rd_selection_continuation(const double*, const double*, const float*, const float*,
    const int*, const int*, const int*, const int*, const int*, const int*, const int*, const int*,
    const double*, double*, int*, int*, double*, int*, int*);
namespace {
void Pack(double* out, n::Vector value) { out[0]=value.x; out[1]=value.y; out[2]=value.z; }
void Pack(float* out, n::StoredNormal value) { out[0]=value.x; out[1]=value.y; out[2]=value.z; }
bool Same(n::Vector a, n::Vector b) {
  const double x[]{a.x,a.y,a.z},y[]{b.x,b.y,b.z}; return std::memcmp(x,y,sizeof(x))==0;
}
bool Same(n::StoredNormal a,n::StoredNormal b) {
  const float x[]{a.x,a.y,a.z},y[]{b.x,b.y,b.z}; return std::memcmp(x,y,sizeof(x))==0;
}
void Finite(double value) {
  if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite continuation oracle operand/output");
}
} // namespace
s::NativeContinuationResult OracleContinuation(const s::Profile& profile,
    const s::NativeContinuationInput& input,const n::NativeGeometryHistory& prior,
    double seed,RetainedScratchObservation* observation) {
  const auto& pair=input.pair;
  if(profile.gap_mode!=1||profile.initial_penetration!=5||profile.local_processor!=1||
      profile.foreign_rows||profile.thermal||profile.gap_loading||
      pair.radiation_range!=0||pair.applied_gap!=0)
    throw std::invalid_argument("Unselected continuation reference profile");
  if(!pair.key.secondary_source_id||pair.key.secondary_source_id!=prior.secondary_source_id||
      pair.key.generation!=prior.generation||pair.key.main_segment<=0||pair.local_main<=0||
      input.segment_count<pair.local_main||
      std::int64_t(pair.segment_type)<-2*std::int64_t(input.segment_count)||
      std::int64_t(pair.segment_type)>2*std::int64_t(input.segment_count))
    throw std::invalid_argument("Inconsistent continuation identity/domain");
  // Direct native indexing, with a bounded qualification-only reference arena.
  // This cap does not constrain the production value API or reassign source IDs.
  constexpr int maximum_references=1<<20;
  int references=0;
  for(unsigned i=0;i<4;++i) {
    if(input.normal_reference[i]<=0||input.normal_reference[i]>maximum_references||
        input.sliding_reference[i]<0||input.main_constraint[i]<0||input.main_skew[i]<0||
        (pair.boundary_ids[i]&&pair.boundary_ids[i]!=std::uint64_t(input.normal_reference[i])))
      throw std::invalid_argument("Unsupported reference-library normal domain");
    references=std::max(references,input.normal_reference[i]);
    for(unsigned j=0;j<i;++j) {
      if(input.normal_reference[i]==input.normal_reference[j]&&
          bool(pair.boundary_ids[i])!=bool(pair.boundary_ids[j]))
        throw std::invalid_argument("Inconsistent native LBOUND alias");
      if(pair.main_node_ids[i]==pair.main_node_ids[j]&&
          (input.main_constraint[i]!=input.main_constraint[j]||input.main_skew[i]!=input.main_skew[j]))
        throw std::invalid_argument("One physical node has inconsistent constraint operands");
    }
  }
  if(input.secondary_constraint<0||input.secondary_skew<0)
    throw std::invalid_argument("Invalid secondary constraint operands");
  std::vector<int> bounds(std::size_t(references),0);
  std::vector<float> bisectors(std::size_t(references)*6,0.f);
  double coordinates[15],scalars[12]{pair.main_coefficient,pair.secondary_coefficient,
      pair.secondary_gap,pair.main_gap_max};
  float normals[12]; int flags[19]{};
  int constraints[5]{},skews[5]{};
  Finite(seed);
  for(unsigned i=0;i<4;++i) {
    if(!pair.main_node_ids[i]) throw std::invalid_argument("Zero main-node identity");
    Pack(coordinates+3*i,pair.main_vertices[i]); Pack(normals+3*i,pair.normal_slot[i]);
    flags[i]=int(i+1);
    for(unsigned j=0;j<i;++j) if(pair.main_node_ids[i]==pair.main_node_ids[j]) {
      if(!Same(pair.main_vertices[i],pair.main_vertices[j]))
        throw std::invalid_argument("One native node has conflicting coordinate bits");
      flags[i]=flags[j];
    }
    flags[4+i]=pair.neighbors[i]; flags[14+i]=prior.row.irtlm[i];
    scalars[4+i]=pair.main_gap[i];
    constraints[i]=input.main_constraint[i]; skews[i]=input.main_skew[i];
    if(pair.boundary_ids[i]) {
      for(unsigned j=0;j<i;++j) if(pair.boundary_ids[i]==pair.boundary_ids[j])
        for(unsigned k=0;k<2;++k) if(!Same(pair.vertex_bisector[i][k],pair.vertex_bisector[j][k]))
          throw std::invalid_argument("One native boundary has conflicting float storage");
      const auto slot=std::size_t(input.normal_reference[i]-1);
      bounds[slot]=1;
      for(unsigned j=0;j<2;++j) Pack(bisectors.data()+6*slot+3*j,pair.vertex_bisector[i][j]);
    }
  }
  Pack(coordinates+12,pair.secondary);
  flags[12]=pair.segment_type; flags[13]=pair.key.main_segment; flags[18]=pair.initial_contact_flag;
  scalars[8]=pair.radiation_range; scalars[9]=pair.applied_gap;
  scalars[10]=prior.row.selection_metric[0]; scalars[11]=prior.row.selection_metric[1];
  constraints[4]=input.secondary_constraint; skews[4]=input.secondary_skew;
  for(double value:coordinates) Finite(value);
  for(double value:scalars) Finite(value);
  for(float value:normals) Finite(value);
  for(float value:bisectors) Finite(value);
  double values[36],row_values[4]; int sector_flags[24],markers[4],axes[3],decision[2];
  rd_selection_continuation(coordinates,scalars,normals,bisectors.data(),flags,
      input.normal_reference,input.sliding_reference,bounds.data(),&references,&input.segment_count,
      constraints,skews,&seed,values,sector_flags,markers,row_values,axes,decision);
  s::NativeContinuationResult result;
  result.history=prior;
  for(unsigned i=0;i<4;++i) result.history.row.irtlm[i]=markers[i];
  result.row_replaced=decision[1]!=0;
  // The original winner writes private CAND_E=1. Translate only that observed
  // write back to its real supplied local-main identity; untouched rows retain
  // their original raw local index even if its bytes happen to equal this slot.
  if(result.row_replaced) result.history.row.irtlm[2]=pair.local_main;
  result.history.row.selection_metric[0]=row_values[2];
  result.history.row.selection_metric[1]=row_values[3];
  for(double value:row_values) Finite(value);
  result.classification_product=row_values[0]; result.distance_squared=row_values[1];
  result.active=row_values[0]>0;
  result.selected_subtriangle=result.active?decision[0]:0;
  result.cache.key=pair.key; result.cache.occurrence=pair.occurrence; result.cache.local_main=pair.local_main;
  for(unsigned i=0;i<3;++i) {
    if(axes[i]!=0&&axes[i]!=1) throw std::runtime_error("Unexpected native axis flag");
    if(axes[i]) result.constrained_axis_mask|=1u<<i;
  }
  RetainedScratchObservation scratch;
  for(unsigned i=0;i<4;++i) {
    const auto* value=values+9*i; const auto* flag=sector_flags+6*i;
    auto& sector=result.sector[i]; auto& cache=result.cache.sector[i];
    sector.far=flag[0]; sector.penetration=value[4]; sector.distance_squared=value[8];
    sector.defined=s::FarDefined|s::PenetrationDefined|s::DistanceSquaredDefined;
    cache.far=flag[1]; cache.penetration=value[5]; cache.defined=s::FarDefined|s::PenetrationDefined;
    Finite(value[4]); Finite(value[5]); Finite(value[8]);
    if(flag[2]) {
      Finite(value[0]); Finite(value[1]); sector.raw_lb=value[0]; sector.raw_lc=value[1];
      sector.defined|=s::RawBarycentricDefined;
    }
    if(flag[3]) {
      for(unsigned j:{2u,3u,6u,7u}) Finite(value[j]);
      sector.clamped_lb=value[2]; sector.clamped_lc=value[3]; sector.defined|=s::ClampedBarycentricDefined;
      cache.lb=value[6]; cache.lc=value[7]; cache.defined|=s::ClampedBarycentricDefined;
    }
    result.sliding_match[i]=flag[4]; result.cylindrical_gap[i]=flag[5];
    scratch.raw_lb[i]=value[0]; scratch.raw_lc[i]=value[1];
    scratch.clamped_lb[i]=value[2]; scratch.clamped_lc[i]=value[3];
    scratch.cache_lb[i]=value[6]; scratch.cache_lc[i]=value[7];
  }
  if(observation) *observation=scratch;
  return result;
}
} // namespace type25_selection_test
