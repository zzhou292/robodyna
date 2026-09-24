// SPDX-License-Identifier: AGPL-3.0-or-later
#include "RootIntervalFilter.h"
#include "../SurfaceMaterialBounds.h"
#include "../SurfaceContactGeometry.h"
#include <cerrno>
#include <cfloat>
#include <cfenv>
#if defined(__x86_64__) && defined(__SSE2__)
#include <xmmintrin.h>
#endif

namespace tlfea::contact::represented_interval_crossing {
namespace {
using I = Q4IntegralInterval;
using V = material_detail::CrossBounds;

bool Regular(const Vec3 (&vertices)[2][3]) noexcept {
  V e[2], f[2], control[3], mixed[2];
  for (unsigned endpoint=0; endpoint<2; ++endpoint)
    if (!material_detail::Edge(vertices[endpoint][1], vertices[endpoint][0], 1, &e[endpoint]) ||
        !material_detail::Edge(vertices[endpoint][2], vertices[endpoint][0], 1, &f[endpoint])) return false;
  if (!material_detail::Cross(e[0], f[0], &control[0]) ||
      !material_detail::Cross(e[1], f[1], &control[2]) ||
      !material_detail::Cross(e[0], f[1], &mixed[0]) ||
      !material_detail::Cross(e[1], f[0], &mixed[1])) return false;
  // Affine edges have a quadratic cross product. Its exact middle Bernstein
  // control is (e0 x f1 + e1 x f0)/2. A strictly signed component of all three
  // controls excludes every zero normal, including between saved endpoints.
  for (unsigned c=0; c<3; ++c) {
    I sum;
    if (!q4_bounds::Add(mixed[0].component[c], mixed[1].component[c], &sum) ||
        !q4_bounds::Scale(sum, .5, &control[1].component[c])) return false;
  }
  for (unsigned c=0; c<3; ++c)
    if ((control[0].component[c].lower>0 && control[1].component[c].lower>0 && control[2].component[c].lower>0) ||
        (control[0].component[c].upper<0 && control[1].component[c].upper<0 && control[2].component[c].upper<0)) return true;
  return false;
}

bool RelativeHull(const RootIntervalInput& input, unsigned side, Vec3 axis, I* output) noexcept {
  I hull{DBL_MAX,-DBL_MAX};
  for (unsigned endpoint=0; endpoint<2; ++endpoint)
    for (const auto point : input.vertices[side][endpoint]) {
      V relative; I projection;
      if (!material_detail::Edge(point,input.vertices[0][endpoint][0],1,&relative) ||
          !material_detail::Dot(relative,axis,&projection)) return false;
      if (projection.lower<hull.lower) hull.lower=projection.lower;
      if (projection.upper>hull.upper) hull.upper=projection.upper;
    }
  *output=hull; return true;
}

bool Separates(const RootIntervalInput& input, Vec3 axis) noexcept {
  if (!IsFinite(axis) || (axis.x==0 && axis.y==0 && axis.z==0)) return false;
  I first,second;
  return RelativeHull(input,0,axis,&first) && RelativeHull(input,1,axis,&second) &&
      (first.upper<second.lower || second.upper<first.lower);
}
RootIntervalResult Prove(const RootIntervalInput& input) noexcept {
  for (const auto& triangle : input.vertices)
    for (const auto& endpoint : triangle)
      for (const auto point : endpoint) if (!IsFinite(point)) return {};
  if (!Regular(input.vertices[0]) || !Regular(input.vertices[1])) return {};
  constexpr Vec3 coordinates[]{{1,0,0},{0,1,0},{0,0,1}};
  for (const auto axis : coordinates) if (Separates(input,axis)) return {true,axis};
  for (unsigned side=0; side<2; ++side) {
    const auto* triangle=input.vertices[side][0];
    // Approximate normals select directions only. A finite represented axis
    // need not be the exact face normal: the outward projection gap is proof.
    const auto axis=geometry_detail::Cross(Subtract(triangle[1],triangle[0]),
                                           Subtract(triangle[2],triangle[0]));
    if (Separates(input,axis)) return {true,axis};
  }
  return {};
}

bool SupportedEnvironment() noexcept {
  // Exact native fallback is independent of ambient floating-point rounding.
  // Enable the interval proof only where its RN/no-flush contract is observable.
#if defined(__x86_64__) && defined(__SSE2__) && FLT_EVAL_METHOD == 0 && !defined(__FAST_MATH__)
  constexpr unsigned flush_and_rounding=0x8040u|0x6000u;
  return std::fegetround()==FE_TONEAREST && (_mm_getcsr()&flush_and_rounding)==0;
#else
  return false;
#endif
}
} // namespace

RootIntervalResult ProveRootIntervalSeparation(const RootIntervalInput& input) noexcept {
  if (!SupportedEnvironment()) return {};
  // Standard thread-local non-stop arithmetic prevents a new floating trap on
  // an inconclusive overflow/underflow. Restore flags, masks, rounding and errno
  // before either returning a certificate or entering the unchanged exact path.
  const int previous_errno=errno;
  std::fenv_t previous;
  if (std::feholdexcept(&previous)!=0) { errno=previous_errno;return {}; }
  const auto result=Prove(input);
  const int restored=std::fesetenv(&previous);
  errno=previous_errno;
  return restored==0?result:RootIntervalResult{};
}
} // namespace tlfea::contact::represented_interval_crossing
