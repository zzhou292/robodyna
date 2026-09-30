// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/ShellBatchFailure.h"
#include <cstring>
#include <vector>

namespace qt_mapped_test {
using StateBits=std::vector<std::uint64_t>;
inline void Add(StateBits& out,double value) {
  std::uint64_t bits;
  std::memcpy(&bits,&value,sizeof(bits));
  out.push_back(bits);
}
template<std::size_t N> void Add(StateBits& out,const double (&values)[N]) {
  for (double value:values) Add(out,value);
}
inline void Add(StateBits& out,const tl::material::TabulatedShellPlasticityHistory& h) {
  Add(out,h.stress);
  Add(out,h.plastic_strain);
  Add(out,h.filtered_rate_per_s);
}
inline void Add(StateBits& out,const tl::material::failure::ConstantPlasticFailureHistory& h) {
  Add(out,h.damage);
  Add(out,h.failure_time_s);
  out.push_back(h.point_active);
}
inline void Add(StateBits& out,const tl::fea::ShellBatchLayeredSection& section) {
  out.push_back(static_cast<unsigned>(section.law()));
  if (const auto* plastic=section.plastic()) {
    for (const auto& point:plastic->history.point) Add(out,point);
    const auto& d=plastic->diagnostics;
    const double values[]{d.plastic_work_density_increment,d.maximum_plastic_strain,d.mean_plastic_strain,
        d.minimum_tangent_ratio,d.mean_tangent_ratio,d.mean_yield_before_pa,d.last_point_yield_before_pa,
        plastic->cumulative_plastic_work_J};
    Add(out,values);
  } else if (const auto* point=section.one_point()) {
    Add(out,point->point.saved);
    const auto& current=point->point.current;
    Add(out,current.history);
    const double values[]{current.plastic_increment,current.tangent_ratio,current.elastic_thickness_strain,
        current.plastic_thickness_strain,current.yield_before_pa,current.equivalent_stress_pa,
        current.plastic_work_density,point->point.reported_thickness_m,
        point->point.plastic_work_increment_j,point->cumulative_plastic_work_J};
    Add(out,values);
    Add(out,point->point.failure.history);
    out.push_back(point->point.failure.failed_now);
  } else if (const auto* elastic=section.elastic()) {
    for (const auto& point:elastic->point) Add(out,point.stress);
  }
}
inline void Add(StateBits& out,const tl::fea::ShellBatchFailureState& state) {
  out.push_back(static_cast<unsigned>(state.policy()));
  out.push_back(state.active);
  for (const auto& point:state.current_force_point) Add(out,point.stress);
  if (const auto* points=state.constant_points()) {
    for (unsigned p=0;p<3;++p) Add(out,points[p]);
  } else if (const auto* points=state.tab1_points()) {
    for (unsigned p=0;p<3;++p) {
      Add(out,points[p].damage);
      Add(out,points[p].maximum_damage);
      Add(out,points[p].failure_time_s);
      out.push_back(points[p].table_segment);
      out.push_back(points[p].point_active);
    }
  }
}
} // namespace qt_mapped_test
