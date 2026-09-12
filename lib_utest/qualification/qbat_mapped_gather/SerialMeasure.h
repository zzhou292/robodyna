// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/qbat/QbatBatchResultChecks.h"
#include "lib_src/elements/ShellBatchFields.h"

namespace qbat_gather_test::serial {
using namespace tl::fea;
using namespace tl::fea::qbat;
using namespace tl::fea::qbat::batch_detail;
TL_QBAT_HD inline bool Measure(const Model& model,const Slab& accepted,const Slab& trial,
    const NodalPreparedView& view,BatchDiagnostics& diagnostics) noexcept {
  auto& d=diagnostics;
  d.element_count=model.config.element_count;
  for(std::size_t parent=0;parent<model.config.element_count;++parent) {
    const auto& old=accepted.element[parent];
    const auto& now=trial.element[parent];
    const auto& element=model.element[parent];
    if(!ValidResult(now,element.material,d.time,d.epoch)) return false;
    if(now.history.element_active) ++d.active_count;
    if(old.history.element_active&&!now.history.element_active) ++d.newly_removed_count;
    const double area=now.kinematics.geometry.area_m2/element.reference.quadrilateral().area;
    const double thickness=now.history.thickness_m/element.reference.input().quadrilateral.thickness;
    if(!parent||area<d.minimum_area_ratio) d.minimum_area_ratio=area;
    if(!parent||thickness<d.minimum_thickness_ratio) d.minimum_thickness_ratio=thickness;
    const double dt=now.diagnostics.unscaled_element_dt_s;
    if(!parent||dt<d.minimum_native_dt) d.minimum_native_dt=dt;
    for(unsigned channel=0;channel<2;++channel) {
      d.internal_work_j[channel]+=now.history.internal_work_j[channel];
      d.internal_work_increment_j[channel]+=now.history.internal_work_j[channel]-old.history.internal_work_j[channel];
    }
    d.plastic_work_j+=now.history.plastic_work_j;
    d.plastic_work_increment_j+=now.history.plastic_work_j-old.history.plastic_work_j;
    d.numerical_viscous_work_j+=now.history.numerical_viscous_work_j;
    d.numerical_viscous_work_increment_j+=now.history.numerical_viscous_work_j-old.history.numerical_viscous_work_j;
    for(unsigned point=0;point<4;++point) {
      for(double value:now.history.point[point].strain) {
        const double magnitude=::fabs(value);
        if(magnitude>d.maximum_absolute_strain) d.maximum_absolute_strain=magnitude;
      }
    }
    if(model.config.usage==BatchUsage::CoupledForces) {
      shell_batch_fields::AccumulateInternalWork(element.nodes,old.internal_force_n,old.internal_couple_nm,
          view,model.config.owner.fixed_dt,d.internal_kick_work,d.internal_drift_work);
    }
  }
  for(std::size_t node=0;node<model.config.owner.node_count;++node) {
    const auto delta=shell_batch_fields::Difference(
        shell_batch_fields::ReadVector(view.kinematics.position_xyz,node),model.initial_position[node]);
    const double magnitude=::sqrt(shell_batch_fields::Dot(delta,delta));
    if(!tl::math::Finite(magnitude)) return false;
    if(magnitude>d.maximum_displacement) d.maximum_displacement=magnitude;
  }
  const double values[]{d.plastic_work_j,d.plastic_work_increment_j,d.numerical_viscous_work_j,
      d.numerical_viscous_work_increment_j,d.internal_kick_work,d.internal_drift_work,
      d.minimum_area_ratio,d.minimum_thickness_ratio,d.maximum_displacement,d.maximum_absolute_strain};
  return detail::FiniteValues(values)&&detail::FiniteValues(d.internal_work_j)&&
      detail::FiniteValues(d.internal_work_increment_j)&&detail::Positive(d.minimum_native_dt);
}
} // namespace qbat_gather_test::serial
