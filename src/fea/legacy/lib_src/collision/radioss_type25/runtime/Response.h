// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "MassOperands.h"
#include "../IsotropicResponse.h"
#include "../geometry/HistoryRow.h"
#include "../assembly/Endpoints.h"
#include "../selection/lifecycle/Binding.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
// Per-occurrence FOR3 arguments. Only a complete, selected native geometry may
// enter this factory. Raw masses are finite physical coefficients, including
// fully constrained nodes. Subtract each main velocity in native source order.
TL_MATH_HOST_DEVICE inline bool ForceInput(const lifecycle::Input& input,
    const lifecycle::Occurrence& occurrence,const NativeGeometryFinalResult& geometry,
    MassOperands mass,const units_detail::Factors& units,double previous_kick,NativeFrictionInput& result) {
  NativeFrictionInput out;
  const auto row=occurrence.selected.key.history_index;
  const auto& secondary=input.source.secondary[row];
  const auto& main=input.source.mains[occurrence.selected.local_main-1];
  out.normal.penetration=geometry.penetration;
  out.normal.stiffness=geometry.geometry.incoming_stiffness;
  out.normal.dt=input.step.previous_dt;out.normal.time=input.step.time;out.dt12=previous_kick;
  if(!NativeMass(mass,secondary.node,units,out.normal.secondary_mass))return false;
  out.normal_axis=geometry.geometry.normal;
  auto velocity=lifecycle::detail::Velocity(input,secondary.node,units);
  for(unsigned slot=0;slot<4;++slot) {
    const auto node=main.nodes[slot];const auto h=geometry.geometry.weights[slot];
    out.normal.weights[slot]=h;
    if(!NativeMass(mass,node,units,out.normal.main_mass[slot]))return false;
    out.main_vertices[slot]=lifecycle::detail::Position(input,node,units);
    const auto main_velocity=lifecycle::detail::Velocity(input,node,units);
    velocity.x=velocity.x-h*main_velocity.x;
    velocity.y=velocity.y-h*main_velocity.y;
    velocity.z=velocity.z-h*main_velocity.z;
  }
  out.relative_velocity=velocity;
  out.normal.normal_velocity=tl::math::fixed3::Dot(out.normal_axis,velocity);
  result=out;return true;
}
// Source order within one original NVSIZ cohort, restricted to a single history
// writer. Other rows cannot alter these offsets or response histories. Cohort
// boundaries are supplied by complete positive-occurrence compaction, never
// inferred from thread/block IDs or penetration-active filtering.
TL_MATH_HOST_DEVICE inline TransactionReport RespondRowCohort(
    const TransactionConfig& config,const lifecycle::Input& input,
    const lifecycle::Occurrence* occurrences,const NativeRawGeometryResult* raw,
    std::size_t first,std::size_t last,MassOperands mass,
    const units_detail::Factors& units,double previous_kick,
    NativeContactRow& history,NativeGeometryFinalResult* finalized,
    NativeFrictionResult* responses,assembly::SiEndpoints* packets) {
  for(std::size_t i=first;i<last;++i)if(occurrences[i].selected.enabled)
    geometry_detail::InitialOffset(input.step.time,raw[i],history);
  for(std::size_t i=first;i<last;++i)if(occurrences[i].selected.enabled) {
    finalized[i]={raw[i],geometry_detail::ShiftOffset(raw[i],history)};
    if(!normal_detail::Nonnegative(finalized[i].penetration)||!history_detail::Valid(history))
      return {TransactionStatus::NumericalFailure,"Nonfinite native geometry offset",SIZE_MAX,i};
  }
  for(std::size_t i=first;i<last;++i)if(occurrences[i].selected.enabled)
    geometry_detail::StageStiffness(finalized[i],history);
  for(std::size_t i=first;i<last;++i) {
    responses[i]={};packets[i]={};
    if(!occurrences[i].selected.enabled)continue;
    NativeFrictionInput arguments;
    if(!ForceInput(input,occurrences[i],finalized[i],mass,units,previous_kick,arguments))
      return {TransactionStatus::NumericalFailure,"Native response mass is not representable",SIZE_MAX,i};
    const auto status=EvaluateNativeFriction(config.normal,config.friction,
        config.friction_coefficients,arguments,history.history,&responses[i]);
    if(status!=NormalStatus::Ok)
      return {TransactionStatus::NumericalFailure,"Native normal/friction response rejected",SIZE_MAX,i};
    history.history=responses[i].history;
    assembly::NativeEndpoints endpoints;
    if(assembly::PrepareNativeEndpoints(config.assembly,responses[i],&endpoints)!=assembly::Status::Ok||
       assembly::EndpointsToSi(config.units,endpoints,&packets[i])!=assembly::Status::Ok)
      return {TransactionStatus::NumericalFailure,"Native endpoint preparation rejected",SIZE_MAX,i};
  }
  return {TransactionStatus::Ok,"OK"};
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
