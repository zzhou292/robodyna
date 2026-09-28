// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "MeasurementTypes.h"
#include "../QbatBatchMeasure.h"

namespace tl::fea::qbat::mapped {
// AccumulateInternalWork invokes exactly four subtractions per output. Saving
// their operands preserves its native expressions without summing from zero.
struct WorkOperands {
  double* values;
  unsigned next=0;
  TL_QBAT_HD void operator-=(double value) noexcept { values[next++]=value; }
};

TL_QBAT_HD inline MeasurementParent PrepareMeasurementParent(
    const batch_detail::Storage& state, const batch_detail::Slab& accepted,
    const batch_detail::Slab& trial, const NodalPreparedView& view,
    const BatchDiagnostics& identity, std::size_t parent) noexcept {
  MeasurementParent out;
  // A failed Advance leaves the old trial slot untouched. Never inspect it.
  if (state.candidate_status[parent]!=Status::kSuccess) return out;
  const auto& element=state.model.element[parent];
  const auto& now=trial.element[parent];
  if (!batch_detail::ValidResult(now,element.material,identity.time,identity.epoch)) return out;
  const auto& old=accepted.element[parent];
  out.active=now.history.element_active ? 1 : 0;
  out.newly_removed=old.history.element_active && !now.history.element_active ? 1 : 0;
  out.area_ratio=now.kinematics.geometry.area_m2/element.reference.quadrilateral().area;
  out.thickness_ratio=now.history.thickness_m/element.reference.input().quadrilateral.thickness;
  out.native_dt=now.diagnostics.unscaled_element_dt_s;
  for (unsigned channel=0; channel<2; ++channel) {
    out.internal_work[channel]=now.history.internal_work_j[channel];
    out.internal_increment[channel]=now.history.internal_work_j[channel]-old.history.internal_work_j[channel];
  }
  out.plastic_work=now.history.plastic_work_j;
  out.plastic_increment=now.history.plastic_work_j-old.history.plastic_work_j;
  out.viscous_work=now.history.numerical_viscous_work_j;
  out.viscous_increment=now.history.numerical_viscous_work_j-old.history.numerical_viscous_work_j;
  for (unsigned point=0; point<4; ++point) {
    for (double value : now.history.point[point].strain) {
      const double magnitude=::fabs(value);
      if (magnitude>out.maximum_strain) out.maximum_strain=magnitude;
    }
  }
  if (state.model.config.usage==BatchUsage::CoupledForces) {
    WorkOperands kick{out.kick_operand}, drift{out.drift_operand};
    shell_batch_fields::AccumulateInternalWork(element.nodes,old.internal_force_n,
        old.internal_couple_nm,view,state.model.config.owner.fixed_dt,kick,drift);
  }
  // Derived overflow is deliberately not an early error. The original caller
  // folds every admitted parent before displacement and aggregate finiteness.
  out.valid=1;
  return out;
}

TL_QBAT_HD inline void AccumulateMeasurementParent(const batch_detail::Model& model,
    const MeasurementParent& now,std::size_t parent,BatchDiagnostics& d) noexcept {
    if (now.active) ++d.active_count;
    if (now.newly_removed) ++d.newly_removed_count;
    if (!parent || now.area_ratio<d.minimum_area_ratio) d.minimum_area_ratio=now.area_ratio;
    if (!parent || now.thickness_ratio<d.minimum_thickness_ratio) d.minimum_thickness_ratio=now.thickness_ratio;
    if (!parent || now.native_dt<d.minimum_native_dt) d.minimum_native_dt=now.native_dt;
    for (unsigned channel=0; channel<2; ++channel) {
      d.internal_work_j[channel]+=now.internal_work[channel];
      d.internal_work_increment_j[channel]+=now.internal_increment[channel];
    }
    d.plastic_work_j+=now.plastic_work;
    d.plastic_work_increment_j+=now.plastic_increment;
    d.numerical_viscous_work_j+=now.viscous_work;
    d.numerical_viscous_work_increment_j+=now.viscous_increment;
    if (now.maximum_strain>d.maximum_absolute_strain) d.maximum_absolute_strain=now.maximum_strain;
    if (model.config.usage==BatchUsage::CoupledForces) {
      for (unsigned slot=0; slot<4; ++slot) {
        d.internal_kick_work-=now.kick_operand[slot];
        d.internal_drift_work-=now.drift_operand[slot];
      }
    }
}

TL_QBAT_HD inline bool MeasureStagedParents(const batch_detail::Model& model,
    const MeasurementParent* values, BatchDiagnostics& d) noexcept {
  d.element_count=model.config.element_count;
  for (std::size_t parent=0; parent<model.config.element_count; ++parent) {
    const auto& now=values[parent];
    if (now.valid!=1) return false;
    AccumulateMeasurementParent(model,now,parent,d);
  }
  return true;
}
} // namespace tl::fea::qbat::mapped
