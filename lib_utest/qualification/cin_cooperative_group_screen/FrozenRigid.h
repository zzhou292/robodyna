// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_timestep/Ordinary.h"
#include "lib_src/collision/RigidNormalResponse.h"

namespace tl::fea::cooperative_test::frozen {
using namespace cin_timestep;
// Reuses the qualified actual force-frame response and outward-rounded positive
// arithmetic. The body operator is the rigid projection of diag(kN I3,kR I3).
// Its trace majorizes its largest eigenvalue; this is an analytical surrogate
// screen, not parity with the inconsistent native RBYM timestep branch.
TL_SURFACE_HD inline bool AddRigidMemberTrace(const tlfea::contact::RigidContactBody& body,
    tl::math::Vec3 position, double translation, double rotation, double& trace) noexcept {
  namespace contact = tlfea::contact;
  namespace arithmetic = contact::mass_detail;
  if (!std::isfinite(translation) || translation < 0 ||
      !std::isfinite(rotation) || rotation < 0 || !std::isfinite(trace) || trace < 0)
    return false;
  double next = trace;
  const tl::math::Vec3 axes[]{{1,0,0}, {0,1,0}, {0,0,1}};
  for (const auto axis : axes) {
    contact::RigidNormalResponse response;
    if (contact::EvaluateRigidNormalResponse(body, position, axis, response) != contact::Status::kOk ||
        contact::AccumulateRigidContactTrace(translation, response, next) != contact::Status::kOk)
      return false;
  }
  const auto inertia = body.current_frame.inertia;
  const double principal[]{inertia.x, inertia.y, inertia.z};
  for (const auto value : principal) {
    double inverse = 0;
    double term = 0;
    if (!arithmetic::UpperQuotient(1, value, &inverse) ||
        !arithmetic::UpperProduct(rotation, inverse, &term) ||
        !arithmetic::UpperSum(next, term, &next)) return false;
  }
  trace = next;
  return true;
}
TL_SURFACE_HD inline bool RigidTraceLimit(double trace, double factor, ScalarLimit& output) noexcept {
  if (!std::isfinite(trace) || trace < 0 || !std::isfinite(factor) || factor <= 0 || factor > 1)
    return false;
  ScalarLimit next;
  if (trace > 0) {
    // Round the final positive bound down. Reject overflow/underflow rather than
    // turn an unrepresentable response into an unbounded admitted step.
    double square = 2/trace;
    if (!std::isfinite(square) || square <= 0) return false;
    square = ::nextafter(square, 0.);
    const double root = ::nextafter(::sqrt(square), 0.);
    next.dt = ::nextafter(factor*root, 0.);
    if (!std::isfinite(next.dt) || next.dt <= 0) return false;
    next.bounded = true;
  }
  output = next;
  return true;
}
} // namespace tl::fea::cin_timestep
