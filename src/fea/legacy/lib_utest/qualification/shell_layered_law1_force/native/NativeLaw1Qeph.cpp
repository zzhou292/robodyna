#include "NativeLaw1Input.h"
#include "../../native/qeph/NativeQephBridge.h"
#include "../../native/qeph/QephGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tl::qualification::layered_law1_native {
using namespace qeph;
namespace qdetail=qeph::detail;
extern "C" void l1_qeph_force(const double*,const double*,const double*,const double*,
 const double*,const double*,double*,double*,int*,int*);
namespace {
template<std::size_t N> void Pack(double*& p,const std::array<double,N>& a) noexcept {
  std::copy(a.begin(),a.end(),p); p+=N;
}
template<std::size_t N> void Read(const double*& p,std::array<double,N>& a) noexcept {
  std::copy_n(p,N,a.begin()); p+=N;
}
std::array<double,qdetail::kHistoryValues> PackHistory(const HistoryValues& h) noexcept {
  std::array<double,qdetail::kHistoryValues> values{};
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

Status Evaluate(const Reference& reference,const QephHistory& accepted,
                const PrescribedInterval& interval,QephTrial& output) noexcept {
  if(!detail::OutputDisjoint(reference,accepted,interval,output)||
     !detail::Finite(accepted.points)) return Status::kInvalidInput;
  const auto& base=accepted.shell;
  if (!reference.prepared()||!base.matches_reference(reference)) return Status::kInvalidReference;
  const auto stamp=base.stamp();
  const double end=interval.base_time+interval.dt;
  if (interval.base_time!=stamp.time||!Positive(interval.dt)||!std::isfinite(end)||
      !(end>stamp.time)||stamp.sample_index==std::numeric_limits<std::uint64_t>::max()||
      interval.sample_index!=stamp.sample_index+1||
      !qdetail::ValidGeometry(interval.position_endpoint)||
      !qdetail::Finite(interval.velocity_midpoint)||!qdetail::Finite(interval.omega_midpoint))
    return Status::kInvalidInput;
  const auto x=qdetail::PackNodes(interval.position_endpoint);
  const auto v=qdetail::PackNodes(interval.velocity_midpoint);
  const auto omega=qdetail::PackNodes(interval.omega_midpoint);
  const auto history=PackHistory(base.data());
  const auto& input=reference.data().input;
  const double material[]{input.density,input.young_modulus,input.poisson_ratio,input.thickness,
                          reference.data().nodal_mass[0]};
  std::array<double,qdetail::kForceValues> values{};
  int planar=-1,status=-1;
  auto points=accepted.points;
  try {
    const std::scoped_lock lock(qdetail::NativeContext(),detail::SectionContext());
    l1_qeph_force(x.data(),v.data(),omega.data(),material,history.data(),
                  &interval.dt,points[0].data(),values.data(),&planar,&status);
  } catch (...) { return Status::kNativeFailure; }
  if (status!=0) return Status::kNativeFailure;
  for (double value:values) if (!std::isfinite(value)) return Status::kNonfiniteResult;
  QephTrial result;
  result.points=points;
  if(!detail::Finite(points)) return Status::kNonfiniteResult;
  auto& candidate=result.shell;
  const auto geometry=qdetail::UnpackKinematics(values.data(),planar,interval,candidate.kinematics);
  if (geometry!=Status::kSuccess) return geometry;
  const double* p=values.data()+qdetail::kKinematicValues;
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
  output=result;
  return Status::kSuccess;
}
}  // namespace tl::qualification::layered_law1_native
