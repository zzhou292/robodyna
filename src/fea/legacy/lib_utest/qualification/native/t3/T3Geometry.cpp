#include "T3Geometry.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::t3::detail {
// A bounded geometric preflight, not replacement native geometry. Long-double
// norm/area comparisons use the already rounded world-edge differences that
// C3EVEC3 consumes. These bounds keep its ordinary squared norms far from both
// binary64 overflow and underflow; starter C3EVEC3 has no normalization floor.
bool SupportedGeometry(const std::array<Vec3, 3>& x) {
  std::array<std::array<long double, 3>, 3> edge{};
  std::array<long double, 3> length{};
  for (unsigned i=0; i<3; ++i) {
    const auto& a=x[i]; const auto& b=x[(i+1)%3];
    edge[i]={b.x-a.x,b.y-a.y,b.z-a.z};
    length[i]=std::hypot(edge[i][0],edge[i][1],edge[i][2]);
    if (!std::isfinite(length[i]) || length[i]<kMinimumEdge || length[i]>kMaximumEdge) return false;
  }
  const auto shortest=*std::min_element(length.begin(),length.end());
  const auto longest=*std::max_element(length.begin(),length.end());
  if (shortest/longest<kMinimumEdgeRatio) return false;
  const auto& a=edge[0]; const auto& b=edge[1];
  const long double twice_area=std::hypot(a[1]*b[2]-a[2]*b[1],
      a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
  return std::isfinite(twice_area) &&
      twice_area/(longest*longest)>=kMinimumNormalizedTwiceArea;
}

bool ProperFrame(const Matrix3& f) {
  constexpr double tolerance=5e-13;
  for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) {
    double dot=0;
    for (unsigned k=0; k<3; ++k) dot+=f.v[3*k+i]*f.v[3*k+j];
    if (std::abs(dot-(i==j?1.:0.))>tolerance) return false;
  }
  const double determinant=f.v[0]*(f.v[4]*f.v[8]-f.v[5]*f.v[7])
      -f.v[1]*(f.v[3]*f.v[8]-f.v[5]*f.v[6])+f.v[2]*(f.v[3]*f.v[7]-f.v[4]*f.v[6]);
  return std::abs(determinant-1)<=tolerance;
}

}  // namespace tl::qualification::t3::detail
