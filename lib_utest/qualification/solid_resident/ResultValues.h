// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solids/resident/Results.h"
#include "../solid_law36_point/TestSupport.h"
#include "../solid_law42_caller/Values.h"
#include <vector>

namespace solid_resident_test {
namespace s=tl::fea::solids;
inline void Vector(std::vector<double>& out,tl::math::Vec3 x) {
  out.insert(out.end(),{x.x,x.y,x.z});
}
inline std::vector<double> Values(const s::Result18& result) {
  std::vector<double> out;
  for (const auto& point:result.history.point) {
    const auto p=law36_test::Values(point.material.point);
    out.insert(out.end(),p.begin(),p.end());
    out.insert(out.end(),{point.material.internal_energy_density_j_m3,point.material.plastic_work_j,
        point.density_kg_m3,point.storage_volume_m3,point.initial_volume_m3,point.bulk_pressure_pa});
  }
  const auto& g=result.history.global;
  out.insert(out.end(),std::begin(g.stress_pa),std::end(g.stress_pa));
  out.insert(out.end(),{g.plastic_strain,g.density_kg_m3,g.internal_energy_density_j_m3,
      g.plastic_work_j,g.bulk_pressure_pa});
  for (auto x:result.history.saved_local_position_m) Vector(out,x);
  for (auto f:result.cache.rhs_force_n) Vector(out,f);
  const auto& d=result.cache.diagnostics;
  out.insert(out.end(),{d.selection_factor,d.selective_poisson_ratio,double(d.selected_point),
      d.minimum_unscaled_dt_s,d.raw_stiffness_n_m,d.internal_work_increment_j,d.plastic_work_increment_j,
      result.cache.stiffness.translation_n_m,result.cache.stiffness.rotation_nm});
  return out;
}
template<class Modes> inline std::vector<double> RubberHistory(
    const tl::material::law42::CallerHistory& history,const Modes& modes) {
  std::vector<double> out(std::begin(history.stress_pa),std::end(history.stress_pa));
  out.insert(out.end(),{history.density_kg_m3,history.internal_energy_density_j_m3,history.bulk_pressure_pa});
  for (const auto& row:modes) for (double x:row) out.push_back(x);
  return out;
}
inline std::vector<double> Values(const s::Result24& result) {
  auto out=RubberHistory(result.history.material,result.history.physical_hourglass);
  for (auto f:result.cache.rhs_force_n) Vector(out,f);
  const auto& d=result.cache.diagnostics;
  const auto material=law42_caller_test::Values(d.material);
  out.insert(out.end(),material.begin(),material.end());
  out.insert(out.end(),{d.stabilization_work_j,d.stabilization_modulus_pa,d.stabilization_viscosity_kg_m_s,
      result.cache.stiffness.translation_n_m,result.cache.stiffness.rotation_nm});
  return out;
}
inline std::vector<double> Values(const s::Result6z& result) {
  auto out=RubberHistory(result.history.material,result.history.hourglass_stress_pa);
  for (auto f:result.cache.rhs_force_n) Vector(out,f);
  const auto material=law42_caller_test::Values(result.cache.material);
  out.insert(out.end(),material.begin(),material.end());
  const auto& d=result.cache.stabilization;
  for (const auto& row:d.modal_velocity_m_s) for (double x:row) out.push_back(x);
  for (const auto& row:d.modal_force_n) for (double x:row) out.push_back(x);
  out.insert(out.end(),{d.effective_shear_modulus_pa,d.damping_kg_m_s,d.first_work_j,d.second_work_j,
      result.cache.total_internal_work_increment_j,
      result.cache.stiffness.translation_n_m,result.cache.stiffness.rotation_nm});
  return out;
}
struct Results {
  s::Result18 a;
  s::Result24 b;
  s::Result6z c;
  s::ResultBuffers Buffers() {return {&a,1,&b,1,&c,1};}
};
template<class Result> inline void Exact(const Result& a,const Result& b) {
  const auto x=Values(a),y=Values(b);
  ASSERT_EQ(x.size(),y.size());
  for (std::size_t k=0;k<x.size();++k) EXPECT_EQ(std::memcmp(&x[k],&y[k],sizeof(double)),0)<<k;
  EXPECT_EQ(a.stamp.sample_index,b.stamp.sample_index);
  EXPECT_EQ(a.stamp.time_s,b.stamp.time_s);
}
inline void Exact(const Results& a,const Results& b) {Exact(a.a,b.a);Exact(a.b,b.b);Exact(a.c,b.c);}
} // namespace solid_resident_test
