// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SurfaceContactGeometry.h"
#include <algorithm>
#include <array>

namespace tlfea::contact::self_contact_transaction {
// Search directions only. The caller must verify every candidate against all
// exact represented arm trajectories; no direction or iterator grants authority.
// Stream singleton/edge/triangle supporting directions with fixed small ray
// storage, without materializing the quadratic/cubic candidate family.
template <unsigned Count>
class ConeDirectionStream {
 public:
  static_assert(Count >= 3 && Count <= 12, "bounded cone search storage");
  static constexpr unsigned RayCount = Count;
  static constexpr unsigned MaximumDirections =
      Count + Count * (Count - 1) / 2 + Count * (Count - 1) * (Count - 2) / 6;
  explicit ConeDirectionStream(const Vec3 (&rays)[RayCount]) noexcept {
    std::copy_n(rays, RayCount, rays_.begin());
  }
  bool Next(Vec3* output) noexcept {
    if (!output || emitted_ == MaximumDirections) return false;
    if (phase_ == 1) {
      *output = rays_[first_++];
      if (first_ == RayCount) {
        phase_ = 2; first_ = 0; second_ = 1;
      }
    } else if (phase_ == 2) {
      const auto edge = Subtract(rays_[second_], rays_[first_]);
      // |edge|^2 times the closest point on the supporting line. Division
      // and barycentric admission are unnecessary: the final verifier decides.
      *output = geometry_detail::Cross(
          edge, geometry_detail::Cross(rays_[first_], edge));
      if (++second_ == RayCount) {
        ++first_; second_ = first_ + 1;
      }
      if (first_ == RayCount - 1) {
        phase_ = 3; first_ = 0; second_ = 1; third_ = 2;
      }
    } else {
      *output = geometry_detail::Cross(
          Subtract(rays_[second_], rays_[first_]),
          Subtract(rays_[third_], rays_[first_]));
      if (++third_ == RayCount) {
        ++second_; third_ = second_ + 1;
        if (second_ == RayCount - 1) {
          ++first_; second_ = first_ + 1; third_ = second_ + 1;
        }
      }
    }
    ++emitted_;
    return true;
  }
  unsigned emitted() const noexcept { return emitted_; }
 private:
  std::array<Vec3, RayCount> rays_;
  unsigned phase_ = 1, first_ = 0, second_ = 1, third_ = 2, emitted_ = 0;
};
using ConeDirections = ConeDirectionStream<8>;
using CurvedConeDirections = ConeDirectionStream<12>;
static_assert(ConeDirections::MaximumDirections == 92);
static_assert(CurvedConeDirections::MaximumDirections == 298);
}  // namespace tlfea::contact::self_contact_transaction
