#include "QephGeometry.h"
#include "lib_src/elements/ReissnerRotation.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace tl::qualification::qeph::detail {
namespace {
namespace vector = tl::fea::reissner::detail;
double Length(Vec3 x) { return std::hypot(x.x,x.y,x.z); }
}
bool Finite(const std::array<Vec3,4>& value) noexcept {
  for (const auto& x:value)
    if (!std::isfinite(x.x)||!std::isfinite(x.y)||!std::isfinite(x.z)) return false;
  return true;
}
bool ValidGeometry(const std::array<Vec3,4>& x) noexcept {
  if (!Finite(x)) return false;
  std::array<Vec3,4> p{};
  double scale=0;
  for (unsigned i=1;i<4;++i) {
    p[i]=vector::Subtract(x[i],x[0]);
    scale=std::max(scale,Length(p[i]));
  }
  if (!(scale>0)||!std::isfinite(scale)) return false;
  for (auto& point:p) point=vector::Scale(point,1/scale);
  const Vec3 r=vector::Subtract(vector::Add(p[1],p[2]),p[3]);
  const Vec3 s=vector::Subtract(vector::Add(p[2],p[3]),p[1]);
  const Vec3 normal=vector::Cross(r,s);
  const double length=Length(normal);
  constexpr double conditioning=128*std::numeric_limits<double>::epsilon();
  // Sufficient conditioned convex projected-domain check, not a new geometry
  // formula: corner turns must all point along the native mean normal.
  if (!(length>conditioning)) return false;
  const Vec3 unit=vector::Scale(normal,1/length);
  for (unsigned i=0;i<4;++i) {
    const auto a=vector::Subtract(p[(i+1)%4],p[i]);
    const auto b=vector::Subtract(p[(i+2)%4],p[(i+1)%4]);
    if (!(vector::Dot(vector::Cross(a,b),unit)>conditioning)) return false;
  }
  // Below this native determinant scale CLSKEW3 clamps/deactivates the cell.
  const long double determinant=static_cast<long double>(length)*scale*scale;
  return determinant>1e-20L && determinant<std::numeric_limits<double>::max();
}
bool ProperFrame(const Matrix3& frame) noexcept {
  Vec3 columns[3];
  for (unsigned j=0;j<3;++j) columns[j]={frame.v[j],frame.v[3+j],frame.v[6+j]};
  for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) {
    const double value=vector::Dot(columns[i],columns[j]);
    if (!std::isfinite(value)||std::abs(value-(i==j?1.:0.))>1e-10) return false;
  }
  return vector::Dot(vector::Cross(columns[0],columns[1]),columns[2])>0;
}
}  // namespace tl::qualification::qeph::detail
