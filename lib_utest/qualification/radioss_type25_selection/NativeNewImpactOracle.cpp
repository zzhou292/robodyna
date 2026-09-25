// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "native/NewImpactPacket.h"
#include <stdexcept>
namespace type25_selection_test {
extern "C" void rd_selection_new_impact(const int*,const double*,const int*,const double*,const float*,
    const int*,const float*,const double*,const int*,const double*,const double*,double*,double*,int*,
    double*,int*,int*,double*,double*,int*);
s::NativeNewImpactResult OracleNewImpact(const s::Profile& profile,const s::NativeNewImpactInput& input,
    const n::NativeGeometryHistory& prior,double seed,NewImpactScratchObservation* observation,
    NewImpactOracleStorage* storage) {
  using impact_reference::Finite;
  Finite(seed);impact_reference::Packet packet(profile,input,prior);
  double projection[20],side_values[8],cache_values[12],metrics[2],scalars[4];
  int side_flags[16],cache_far[4],markers[4],decision[10];
  rd_selection_new_impact(packet.counts,packet.kinematics,packet.source_int.data(),packet.source_real.data(),
      packet.normals.data(),packet.bounds.data(),packet.bisectors.data(),packet.control,packet.markers,
      packet.metrics,&seed,projection,side_values,side_flags,cache_values,cache_far,markers,metrics,scalars,decision);
  s::NativeNewImpactResult result;result.history=prior;
  result.source_key=input.pair.key;result.source_local_main=input.pair.local_main;
  result.cache.key=input.pair.key;result.cache.occurrence=input.pair.occurrence;
  result.cache.local_main=decision[2];
  if(decision[2]==input.pair.local_main)result.cache.key.main_segment=input.pair.key.main_segment;
  else if(input.opposite.local_main&&decision[2]==input.opposite.local_main)
    result.cache.key.main_segment=input.opposite.global_main;
  else throw std::runtime_error("Native cache returned an unsupplied main identity");
  for(unsigned i=0;i<4;++i)result.history.row.irtlm[i]=markers[i];
  for(unsigned i=0;i<2;++i){Finite(metrics[i]);result.history.row.selection_metric[i]=metrics[i];}
  Finite(scalars[0]);result.classification_product=scalars[0];result.active=scalars[0]>0;
  result.selected_subtriangle=decision[0];
  if(decision[3]<0||decision[3]>2)throw std::runtime_error("Unexpected native side observation");
  result.selected_side=static_cast<s::ImpactSide>(decision[3]);result.row_replaced=decision[4]!=0;
  result.primary.subtriangle=decision[5];result.opposite.subtriangle=decision[6];
  result.primary.intersection=decision[7];result.opposite.intersection=decision[8];
  result.recontact_intersection=decision[9];
  if(result.active) {
    Finite(scalars[1]);result.penetration=scalars[1];result.scalar_defined|=s::ImpactPenetrationDefined;
  }
  if(result.selected_side!=s::ImpactSide::None) {
    Finite(scalars[2]);Finite(scalars[3]);result.lb=scalars[2];result.lc=scalars[3];
    result.far=decision[1];result.scalar_defined|=s::ImpactWeightsAndFarDefined;
  }
  for(unsigned i=0;i<4;++i) {
    const auto* p=projection+5*i;auto& out=result.projection[i];
    for(unsigned j=0;j<5;++j)Finite(p[j]);
    out.raw_lb=p[0];out.raw_lc=p[1];out.clamped_lb=p[2];out.clamped_lc=p[3];out.distance_squared=p[4];
    out.defined=s::DistanceSquaredDefined|s::RawBarycentricDefined|s::ClampedBarycentricDefined;
    Finite(side_values[2*i]);Finite(side_values[2*i+1]);
    result.primary.penetration[i]=side_values[2*i];result.opposite.penetration[i]=side_values[2*i+1];
    result.primary.far[i]=side_flags[4*i];result.opposite.far[i]=side_flags[4*i+1];
    result.primary.cylindrical_gap[i]=side_flags[4*i+2];result.opposite.cylindrical_gap[i]=side_flags[4*i+3];
    auto& cache=result.cache.sector[i];const auto* c=cache_values+3*i;
    for(unsigned j=0;j<3;++j)Finite(c[j]);
    cache.far=cache_far[i];cache.penetration=c[0];cache.lb=c[1];cache.lc=c[2];
    cache.defined=s::FarDefined|s::PenetrationDefined|s::ClampedBarycentricDefined;
  }
  if(observation)*observation={scalars[1],scalars[2],scalars[3],decision[1]};
  if(storage)*storage=packet.storage;
  return result;
}
} // namespace type25_selection_test
