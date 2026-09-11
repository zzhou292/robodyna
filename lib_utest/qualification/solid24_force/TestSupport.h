// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid24_reference/JacobianSupport.h"
#include "lib_src/elements/solid24/Solid24Force.h"
#include <vector>
#include "../solid_law42_caller/Values.h"

namespace heph_test {
namespace s=tl::fea::solid24;
namespace b=tl::fea::solid_common;
inline s::Material Material(double density=1980) {
  s::Material p;
  if (tl::material::law42::Prepare(24e6,.463,density,1e26,p)!=tl::material::law42::Status::Ok)
    throw std::runtime_error("test material preparation failed");
  return p;
}
inline s::Reference Reference(s::ReferenceInput input=solid24_test::Brick()) {
  input.profile.reference_strain=s::ReferenceStrain::TotalLagrangian10;
  s::Reference r;
  if (s::InitializeReference(input,r)!=s::Status::Success) throw std::runtime_error("test reference failed");
  return r;
}
inline s::PrescribedInterval Interval(const s::Reference& r,const s::History& h,double dt=1e-6) {
  s::PrescribedInterval i;
  i.base_time_s=h.stamp().time_s; i.dt_s=dt; i.sample_index=h.stamp().sample_index+1;
  for (unsigned n=0;n<8;++n) i.position_m[n]=r.input().position_m[n];
  return i;
}
inline double ForceNorm(const s::ForceTrial& result) {
  double sum=0;
  for(const auto& f:result.rhs_force_n) sum+=b::Dot(f,f);
  return std::sqrt(sum);
}
inline std::vector<double> Values(const s::ForceTrial& t) {
  std::vector<double> out;
  const auto put=[&](double x){out.push_back(x);};
  const auto& h=t.proposed_history.values();
  for(double x:h.material.stress_pa)put(x);
  put(h.material.density_kg_m3);put(h.material.internal_energy_density_j_m3);put(h.material.bulk_pressure_pa);
  for(const auto& row:h.physical_hourglass)for(double x:row)put(x);
  put(t.proposed_history.stamp().time_s);
  for(const auto& f:t.rhs_force_n){put(f.x);put(f.y);put(f.z);}
  for(double x:t.geometry.current.frame.v)put(x);
  for(const auto& x:t.geometry.current.local_position_m){put(x.x);put(x.y);put(x.z);}
  for(const auto& x:t.geometry.local_velocity_m_s){put(x.x);put(x.y);put(x.z);}
  put(t.geometry.current.volume_m3);put(t.geometry.current.characteristic_length_m);
  for(const auto& p:t.geometry.derivative_per_m)for(double x:p)put(x);
  for(const auto& p:t.geometry.hourglass_projection)for(double x:p)put(x);
  for(double x:t.geometry.jacobian_diagonal_m)put(x);
  for(double x:t.geometry.material_displacement_gradient)put(x);
  for(double x:t.geometry.engineering_rate_per_s)put(x);
  const auto& m=t.diagnostics.material;
  const auto material_values=law42_caller_test::Values(m);
  out.insert(out.end(),material_values.begin(),material_values.end());
  put(t.diagnostics.stabilization_work_j);put(t.diagnostics.stabilization_modulus_pa);
  put(t.diagnostics.stabilization_viscosity_kg_m_s);
  return out;
}
inline bool Same(const s::ForceTrial& a,const s::ForceTrial& b) {
  const auto x=Values(a),y=Values(b);
  return x.size()==y.size() && std::memcmp(x.data(),y.data(),x.size()*sizeof(double))==0 &&
      a.proposed_history.stamp().sample_index==b.proposed_history.stamp().sample_index &&
      a.proposed_history.initialized()==b.proposed_history.initialized() &&
      s::force_detail::SameReference(a.proposed_history.reference(),b.proposed_history.reference()) &&
      s::force_detail::SameMaterial(a.proposed_history.material(),b.proposed_history.material());
}
} // namespace heph_test
