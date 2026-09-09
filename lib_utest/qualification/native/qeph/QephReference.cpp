#include "QephReference.h"
#include "NativeQephBridge.h"
#include "QephGeometry.h"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace tl::qualification::qeph {
std::mutex& detail::NativeContext() noexcept {
  static std::mutex context;
  return context;
}
namespace {
using Packet = std::array<double,12>;
Packet Pack(const std::array<Vec3,4>& x) {
  Packet packed{};
  for (unsigned i=0;i<4;++i) {
    packed[3*i]=x[i].x; packed[3*i+1]=x[i].y; packed[3*i+2]=x[i].z;
  }
  return packed;
}
bool Positive(double value) { return std::isfinite(value)&&value>0; }
template<std::size_t N> bool Finite(const std::array<double,N>& a) {
  return std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);});
}
template<std::size_t N> void Read(const double*& p,std::array<double,N>& a) {
  std::copy_n(p,N,a.begin()); p+=N;
}
void Read(const double*& p,Matrix3& a) { std::copy_n(p,9,a.v); p+=9; }
void Read(const double*& p,std::array<Vec3,4>& a) {
  for (auto& x:a) { x={p[0],p[1],p[2]}; p+=3; }
}
}

Status Initialize(const ReferenceInput& input,Reference& output) noexcept {
  if (!Positive(input.density)||!Positive(input.young_modulus)||!Positive(input.thickness)||
      !std::isfinite(input.poisson_ratio)||input.poisson_ratio<0||input.poisson_ratio>=.5||
      !detail::ValidGeometry(input.position)) return Status::kInvalidInput;
  for (unsigned i=0;i<4;++i) for (unsigned j=i+1;j<4;++j)
    if (input.node_ids[i]==input.node_ids[j]) return Status::kInvalidInput;
  const auto x=Pack(input.position);
  const double material[]{input.density,input.thickness,input.young_modulus};
  std::array<double,detail::kStartupValues> values{};
  try {
    const std::lock_guard<std::mutex> lock(detail::NativeContext());
    detail::qeph_q1_startup(x.data(),material,values.data());
  } catch (...) { return Status::kNativeFailure; }
  if (!Finite(values)) return Status::kNonfiniteResult;
  Reference candidate;
  auto& data=candidate.data_;
  data.input=input;
  const double* p=values.data();
  Read(p,data.frame); data.area=*p++;
  Read(p,data.derivative_x); Read(p,data.derivative_y); Read(p,data.local_position);
  if (!detail::ProperFrame(data.frame)||!Positive(data.area)) return Status::kNativeFailure;
  for (unsigned i=0;i<4;++i) {
    if (!Positive(p[i])) return Status::kNonfiniteResult;
  }
  data.nodal_mass.fill(p[0]); data.physical_inertia.fill(p[1]);
  data.added_inertia.fill(p[2]); data.isotropic_inertia.fill(p[3]);
  candidate.prepared_=true;
  output=candidate;
  return Status::kSuccess;
}

Status EvaluatePrescribed(const Reference& reference,const PrescribedInterval& interval,
                          Kinematics& output) noexcept {
  if (!reference.prepared()) return Status::kInvalidReference;
  if (!std::isfinite(interval.base_time)||interval.base_time<0||!Positive(interval.dt)||
      !std::isfinite(interval.base_time+interval.dt)||
      !detail::ValidGeometry(interval.position_endpoint)||
      !detail::Finite(interval.velocity_midpoint)||!detail::Finite(interval.omega_midpoint))
    return Status::kInvalidInput;
  const auto x=Pack(interval.position_endpoint),v=Pack(interval.velocity_midpoint),
             omega=Pack(interval.omega_midpoint);
  std::array<double,detail::kKinematicValues> values{};
  int planar=-1,status=-1;
  try {
    const std::lock_guard<std::mutex> lock(detail::NativeContext());
    detail::qeph_q1_kinematics(x.data(),v.data(),omega.data(),
        &reference.data().input.thickness,&interval.dt,values.data(),&planar,&status);
  } catch (...) { return Status::kNativeFailure; }
  if (status!=0||(planar!=0&&planar!=1)) return Status::kNativeFailure;
  if (!Finite(values)) return Status::kNonfiniteResult;
  Kinematics candidate;
  const double* p=values.data();
  Read(p,candidate.frame);
  candidate.area=*p++; candidate.reciprocal_area=*p++; candidate.characteristic_length=*p++;
  Read(p,candidate.nodal_factors);
  candidate.raw_warpage_abs=*p++; candidate.effective_warpage=*p++;
  candidate.planar=planar!=0;
  Read(p,candidate.local_position); Read(p,candidate.local_normals);
  Read(p,candidate.projection_inverse); Read(p,candidate.projection_columns);
  Read(p,candidate.projected_omega); Read(p,candidate.regular_rate); Read(p,candidate.hourglass_rate);
  if (!detail::ProperFrame(candidate.frame)||!Positive(candidate.area)||
      !Positive(candidate.reciprocal_area)||!Positive(candidate.characteristic_length)||
      !Positive(candidate.nodal_factors[0])||!Positive(candidate.nodal_factors[1]))
    return Status::kNativeFailure;
  candidate.base_time=interval.base_time; candidate.dt=interval.dt;
  candidate.sample_index=interval.sample_index;
  output=candidate;
  return Status::kSuccess;
}
}  // namespace tl::qualification::qeph
