#include "T3Reference.h"
#include "NativeT3Bridge.h"
#include "T3Geometry.h"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace tl::qualification::t3 {
namespace {
bool Positive(double value) { return std::isfinite(value) && value > 0; }
template<std::size_t N> bool Finite(const std::array<double, N>& values) {
  return std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); });
}

// GNU Fortran can retain local fixed-size work arrays in static storage. The
// context lock covers both native calls; it shares no COMMON with QEPH.
std::mutex& NativeContext() { static std::mutex context; return context; }
}  // namespace

Status Initialize(const ReferenceInput& input, Reference& output) noexcept {
  if (!Positive(input.density) || !Positive(input.thickness) || !Positive(input.young_modulus) ||
      !std::isfinite(input.poisson_ratio) || input.poisson_ratio<0 || input.poisson_ratio>=.5)
    return Status::kInvalidInput;
  for (unsigned i=0; i<3; ++i) {
    const auto& x=input.position[i];
    for (double value : {x.x,x.y,x.z})
      if (!std::isfinite(value) || std::abs(value)>kMaximumCoordinate) return Status::kInvalidInput;
    for (unsigned j=0; j<i; ++j)
      if (input.node_ids[i]==input.node_ids[j]) return Status::kInvalidInput;
  }
  if (!detail::SupportedGeometry(input.position)) return Status::kUnsupportedGeometry;

  std::array<double,9> position{};
  for (unsigned i=0; i<3; ++i) {
    position[3*i]=input.position[i].x;
    position[3*i+1]=input.position[i].y;
    position[3*i+2]=input.position[i].z;
  }
  std::array<double,detail::kFrameValues> geometry{};
  std::array<double,detail::kMassValues> mass{};
  const double material[]{input.density,input.thickness};
  const double acos_limit=1-kAcosBoundaryMargin;
  int status=-1;
  try {
    const std::lock_guard<std::mutex> lock(NativeContext());
    detail::t3_r1_frame(position.data(),geometry.data());
    if (!Finite(geometry) || !Positive(geometry[9]) || !Positive(geometry[10]) ||
        !Positive(geometry[12])) return Status::kNonfiniteResult;
    detail::t3_r1_selected_mass(geometry.data()+10,geometry.data()+9,material,
                              &acos_limit,mass.data(),&status);
  } catch (...) { return Status::kNativeFailure; }
  if (status==1) return Status::kUnsupportedGeometry;
  if (status!=0) return Status::kNativeFailure;
  if (!Finite(mass)) return Status::kNonfiniteResult;
  for (unsigned i=3; i<23; ++i) if (!Positive(mass[i])) return Status::kNonfiniteResult;
  for (unsigned i=23; i<26; ++i) if (mass[i]!=0) return Status::kNativeFailure;

  Reference candidate;
  auto& data=candidate.data_;
  data.input=input;
  std::copy_n(geometry.begin(),9,data.frame.v);
  if (!detail::ProperFrame(data.frame)) return Status::kNativeFailure;
  data.area=geometry[9];
  data.local_position={{{0,0,0},{geometry[10],0,0},{geometry[11],geometry[12],0}}};
  std::copy_n(mass.begin(),3,data.angle_cosine.begin());
  std::copy_n(mass.begin()+3,3,data.angle_weight.begin());
  std::copy_n(mass.begin()+6,3,data.nodal_mass.begin());
  std::copy_n(mass.begin()+9,3,data.physical_inertia.begin());
  std::copy_n(mass.begin()+12,3,data.added_inertia.begin());
  std::copy_n(mass.begin()+15,3,data.isotropic_inertia.begin());
  data.element_mass=mass[18]; data.element_isotropic_inertia=mass[19];
  data.element_physical_inertia=mass[20]; data.element_added_inertia=mass[21];
  data.characteristic_length=mass[22];
  std::copy_n(mass.begin()+23,3,data.startup_derivative.begin());
  candidate.prepared_=true;
  output=candidate;
  return Status::kSuccess;
}
}  // namespace tl::qualification::t3
