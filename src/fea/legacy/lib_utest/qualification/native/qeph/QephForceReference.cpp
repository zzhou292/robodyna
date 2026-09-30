#include "QephForceReference.h"
#include "NativeQephBridge.h"
#include "QephGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tl::qualification::qeph {
namespace {
template<std::size_t N> void Pack(double*& p,const std::array<double,N>& a) noexcept {
  std::copy(a.begin(),a.end(),p); p+=N;
}
template<std::size_t N> void Read(const double*& p,std::array<double,N>& a) noexcept {
  std::copy_n(p,N,a.begin()); p+=N;
}
std::array<double,detail::kHistoryValues> PackHistory(const HistoryValues& h) noexcept {
  std::array<double,detail::kHistoryValues> values{};
  double* p=values.data();
  Pack(p,h.stress); Pack(p,h.material_stress); Pack(p,h.bending_stress);
  Pack(p,h.stabilization); Pack(p,h.strain_curvature); *p++=h.thickness;
  Pack(p,h.internal_work); *p++=h.hourglass_viscous_work; *p++=h.active;
  return values;
}
HistoryValues ReadHistory(const double*& p) noexcept {
  HistoryValues h;
  Read(p,h.stress); Read(p,h.material_stress); Read(p,h.bending_stress);
  Read(p,h.stabilization); Read(p,h.strain_curvature); h.thickness=*p++;
  Read(p,h.internal_work); h.hourglass_viscous_work=*p++; h.active=*p++;
  return h;
}
void ReadNodes(const double*& p,std::array<Vec3,4>& nodes) noexcept {
  for (auto& n:nodes) { n={p[0],p[1],p[2]}; p+=3; }
}
bool Positive(double x) noexcept { return std::isfinite(x)&&x>0; }
}

Status EvaluateForce(const Reference& reference,const History& base,
                     const PrescribedInterval& interval,ForceTrial& output) noexcept {
  if (!reference.prepared()||!base.matches_reference(reference)) return Status::kInvalidReference;
  const auto stamp=base.stamp();
  const double end=interval.base_time+interval.dt;
  if (interval.base_time!=stamp.time||!Positive(interval.dt)||!std::isfinite(end)||
      !(end>stamp.time)||stamp.sample_index==std::numeric_limits<std::uint64_t>::max()||
      interval.sample_index!=stamp.sample_index+1||
      !detail::ValidGeometry(interval.position_endpoint)||
      !detail::Finite(interval.velocity_midpoint)||!detail::Finite(interval.omega_midpoint))
    return Status::kInvalidInput;
  const auto x=detail::PackNodes(interval.position_endpoint);
  const auto v=detail::PackNodes(interval.velocity_midpoint);
  const auto omega=detail::PackNodes(interval.omega_midpoint);
  const auto history=PackHistory(base.data());
  const auto& input=reference.data().input;
  const double material[]{input.density,input.young_modulus,input.poisson_ratio,input.thickness,
                          reference.data().nodal_mass[0]};
  std::array<double,detail::kForceValues> values{};
  int planar=-1,status=-1;
  try {
    const std::lock_guard<std::mutex> lock(detail::NativeContext());
    detail::qeph_q2_force(x.data(),v.data(),omega.data(),material,history.data(),
                          &interval.dt,values.data(),&planar,&status);
  } catch (...) { return Status::kNativeFailure; }
  if (status!=0) return Status::kNativeFailure;
  for (double value:values) if (!std::isfinite(value)) return Status::kNonfiniteResult;
  ForceTrial candidate;
  const auto geometry=detail::UnpackKinematics(values.data(),planar,interval,candidate.kinematics);
  if (geometry!=Status::kSuccess) return geometry;
  const double* p=values.data()+detail::kKinematicValues;
  const auto proposed=ReadHistory(p);
  const auto preparation=PreparePrescribedHistory(reference,proposed,{end,interval.sample_index},
                                                   candidate.proposed_history);
  if (preparation!=Status::kSuccess) return Status::kNonfiniteResult;
  ReadNodes(p,candidate.internal_force); ReadNodes(p,candidate.internal_couple);
  auto& diagnostics=candidate.diagnostics;
  diagnostics.effective_thickness=*p++; diagnostics.native_sound_speed=*p++;
  diagnostics.membrane_viscosity=*p++; diagnostics.stabilization_viscosity=*p++;
  diagnostics.translational_stiffness=*p++; diagnostics.rotational_stiffness=*p++;
  diagnostics.unscaled_element_dt=*p++; Read(p,diagnostics.internal_work_increment);
  diagnostics.hourglass_viscous_work_increment=*p++;
  if (!Positive(diagnostics.effective_thickness)||!Positive(diagnostics.native_sound_speed)||
      !Positive(diagnostics.translational_stiffness)||!Positive(diagnostics.rotational_stiffness)||
      !Positive(diagnostics.unscaled_element_dt)||diagnostics.membrane_viscosity<0||
      diagnostics.stabilization_viscosity<0) return Status::kNonfiniteResult;
  output=candidate;
  return Status::kSuccess;
}
}  // namespace tl::qualification::qeph
