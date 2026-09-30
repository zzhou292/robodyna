// SPDX-License-Identifier: AGPL-3.0-or-later
// I25XSAVE's search extent field; this is a length, not surface curvature.
#include "Values.h"
namespace tlfea::contact::radioss_type25::search_startup::detail {
Report Extent(const Input& in, const Vector* x, double* primary, double& maximum) noexcept {
  maximum = 0;
  for (std::size_t i = 0; i < in.topology.primary_count; ++i) {
    double value = 0;
    if (in.profile.curvature != 0) {
      const auto box = Bounds(x,in.topology.mains[i]);
      const double xx = box.maximum.x-box.minimum.x;
      const double yy = box.maximum.y-box.minimum.y;
      const double zz = box.maximum.z-box.minimum.z;
      value = .5*std::max(std::max(xx,yy),zz);
      if (!std::isfinite(value)) return {Status::NonfiniteResult, i};
    }
    primary[i] = value;
    maximum = std::max(maximum,value);
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::search_startup::detail
