// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <limits>
#include <vector>
namespace type25_selection_test::impact_reference {
inline void Finite(double value) {
  if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite new-impact oracle operand/output");
}
inline void Pack(double* out,n::Vector value) { out[0]=value.x;out[1]=value.y;out[2]=value.z; }
inline void Pack(float* out,n::StoredNormal value) { out[0]=value.x;out[1]=value.y;out[2]=value.z; }
inline bool Same(n::Vector a,n::Vector b) {
  const double x[]{a.x,a.y,a.z},y[]{b.x,b.y,b.z};return std::memcmp(x,y,sizeof(x))==0;
}
inline bool Same(n::StoredNormal a,n::StoredNormal b) {
  const float x[]{a.x,a.y,a.z},y[]{b.x,b.y,b.z};return std::memcmp(x,y,sizeof(x))==0;
}
struct Packet {
  int counts[4]{};
  double kinematics[30]{},control[3]{},metrics[2]{};
  int markers[4]{};
  std::vector<int> source_int,bounds;
  std::vector<double> source_real;
  std::vector<float> normals,bisectors;
  NewImpactOracleStorage storage;
  static constexpr int Capacity=1<<20;

  Packet(const s::Profile& profile,const s::NativeNewImpactInput& input,
      const n::NativeGeometryHistory& prior) {
    const auto& pair=input.pair;
    if(profile.gap_mode!=1||profile.initial_penetration!=5||profile.local_processor!=1||
        profile.foreign_rows||profile.thermal||profile.gap_loading||
        pair.radiation_range!=0||pair.applied_gap!=0)
      throw std::invalid_argument("Unselected new-impact oracle profile");
    if(input.segment_count<1||input.segment_count>Capacity||pair.local_main<1||
        pair.local_main>input.segment_count||pair.key.main_segment<=0||
        !pair.key.secondary_source_id||pair.key.secondary_source_id!=prior.secondary_source_id||
        pair.key.generation!=prior.generation||
        std::int64_t(pair.segment_type)<-2*std::int64_t(input.segment_count)||
        std::int64_t(pair.segment_type)>2*std::int64_t(input.segment_count))
      throw std::invalid_argument("Invalid or oversized native main-index domain");
    if(prior.row.irtlm[0]==std::numeric_limits<int>::min())
      throw std::invalid_argument("Native negative-main comparison is not representable");
    const int partner=pair.segment_type>0 ?
        (pair.segment_type>input.segment_count ? pair.segment_type-input.segment_count : pair.segment_type) : 0;
    if(input.opposite.local_main!=partner||
        (partner&&(partner==pair.local_main||input.opposite.global_main<=0)))
      throw std::invalid_argument("Opposite descriptor does not match native MSEGTYP");
    if(partner) {
      constexpr unsigned reverse[]{1,0,3,2};
      for(unsigned i=0;i<4;++i)
        if(input.opposite.main_node_ids[i]!=pair.main_node_ids[reverse[i]])
          throw std::invalid_argument("Supplied opposite-node order contradicts native SH2SURF25");
      // Packet consistency only: this does not authenticate an external source
      // roster, nor populate the native partner IRECT that these routines do not read.
    }
    if(input.previous_dt<0) throw std::invalid_argument("Negative native DT1");
    for(unsigned i=0;i<4;++i) {
      if(!pair.main_node_ids[i]||(partner&&!input.opposite.main_node_ids[i]))
        throw std::invalid_argument("Missing source node identity");
      for(unsigned j=0;j<i;++j) if(pair.main_node_ids[i]==pair.main_node_ids[j]&&
          (!Same(pair.main_vertices[i],pair.main_vertices[j])||!Same(input.main_velocity[i],input.main_velocity[j])))
        throw std::invalid_argument("One native node has conflicting kinematic bits");
    }
    std::uint64_t maximum=0;std::size_t unbound=0;
    const std::uint64_t* ids[]{pair.boundary_ids,input.opposite.boundary_ids};
    const unsigned sides=partner?2u:1u;
    for(unsigned side=0;side<sides;++side)for(unsigned i=0;i<4;++i) {
      if(ids[side][i]) maximum=std::max(maximum,ids[side][i]);else ++unbound;
    }
    if(maximum>Capacity||unbound>std::size_t(Capacity-maximum))
      throw std::invalid_argument("Native normal-reference arena exceeds qualification cap");
    const auto references=std::max<std::size_t>(1,std::size_t(maximum)+unbound);
    counts[0]=input.segment_count;counts[1]=int(references);counts[2]=pair.local_main;
    counts[3]=pair.initial_contact_flag;
    storage.main_slots=std::size_t(input.segment_count);storage.reference_slots=references;
    // Per main: C++14ints+6doubles+12floats; Fortran14ints+7doubles.
    // Per reference:4-byte LBOUND +6binary32 components. Includes both layers.
    storage.table_bytes=264*storage.main_slots+28*storage.reference_slots;
    source_int.assign(14*storage.main_slots,0);source_real.assign(6*storage.main_slots,0.);
    normals.assign(12*storage.main_slots,0.f);bounds.assign(references,0);bisectors.assign(6*references,0.f);
    auto* main=source_int.data()+14*std::size_t(pair.local_main-1);
    for(unsigned i=0;i<4;++i) {
      main[i]=int(i+1);
      for(unsigned j=0;j<i;++j)if(pair.main_node_ids[i]==pair.main_node_ids[j])main[i]=main[j];
      Pack(kinematics+3*i,pair.main_vertices[i]);Pack(kinematics+15+3*i,input.main_velocity[i]);
      markers[i]=prior.row.irtlm[i];
    }
    Pack(kinematics+12,pair.secondary);Pack(kinematics+27,input.secondary_velocity);
    main[4]=pair.segment_type;main[5]=pair.key.main_segment;
    auto* scalar=source_real.data()+6*std::size_t(pair.local_main-1);
    scalar[0]=pair.main_coefficient;scalar[1]=pair.main_gap_max;
    for(unsigned i=0;i<4;++i)scalar[2+i]=pair.main_gap[i];
    control[0]=pair.secondary_coefficient;control[1]=pair.secondary_gap;control[2]=input.previous_dt;
    metrics[0]=prior.row.selection_metric[0];metrics[1]=prior.row.selection_metric[1];
    // Actual bound IDs are preserved; fresh zero-LBOUND references represent
    // unbound corners. COR22 uses those references only for this lookup.
    std::size_t fresh=std::size_t(maximum);
    std::uint64_t seen[8]{};n::StoredNormal seen_values[8][2]{};unsigned seen_count=0;
    for(unsigned side=0;side<sides;++side) {
      const int local=side?partner:pair.local_main;
      const auto* slots=side?input.opposite.normal_slot:pair.normal_slot;
      const auto* neighbors=side?input.opposite.neighbors:pair.neighbors;
      const auto* vectors=side?input.opposite.vertex_bisector:pair.vertex_bisector;
      auto* integer=source_int.data()+14*std::size_t(local-1);
      if(side) {
        integer[5]=input.opposite.global_main;
        // COR22/DST22/GLOB22 do not read partner IRECT, coefficients or gaps.
        // They remain unused storage, not manufactured partner geometry.
      }
      for(unsigned i=0;i<4;++i) {
        Pack(normals.data()+12*std::size_t(local-1)+3*i,slots[i]);integer[6+i]=neighbors[i];
        const auto id=ids[side][i];
        const auto reference=id?std::size_t(id):++fresh;integer[10+i]=int(reference);
        if(!id)continue;
        for(unsigned j=0;j<seen_count;++j)if(seen[j]==id)
          for(unsigned k=0;k<2;++k)if(!Same(seen_values[j][k],vectors[i][k]))
            throw std::invalid_argument("One native boundary ID has inconsistent float32 storage");
        seen[seen_count]=id;
        for(unsigned k=0;k<2;++k) {
          seen_values[seen_count][k]=vectors[i][k];
          Pack(bisectors.data()+6*(reference-1)+3*k,vectors[i][k]);
        }
        ++seen_count;bounds[reference-1]=1;
      }
    }
    for(double value:kinematics)Finite(value);
    for(double value:control)Finite(value);
    for(double value:metrics)Finite(value);
    for(double value:source_real)Finite(value);
    for(float value:normals)Finite(value);
    for(float value:bisectors)Finite(value);
  }
};
} // namespace type25_selection_test::impact_reference
