// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_timestep/Rigid.h"
#include <type_traits>

namespace tl::fea::cin_advance::group_screen {
// Private, one-tile values. Trivial records give CUDA shared storage an explicit
// lifetime without constructors. No response survives its owning screen launch.
struct ResponseValues {
  double inverse_effective_mass, inverse_upper;
  bool valid;
};
struct MemberResponses {
  ResponseValues axis[3];
};
static_assert(std::is_trivial<MemberResponses>::value);
static_assert(sizeof(MemberResponses) == 72);

TL_SURFACE_HD inline MemberResponses PrepareResponses(
    const tlfea::contact::RigidContactBody& body, tl::math::Vec3 position,
    double translation, double rotation) noexcept {
  MemberResponses next{};
  // A bad coefficient precedes any response in the original member call.
  if (!std::isfinite(translation) || translation < 0 ||
      !std::isfinite(rotation) || rotation < 0) return next;
  const tl::math::Vec3 axes[]{{1,0,0}, {0,1,0}, {0,0,1}};
  for (unsigned axis = 0; axis < 3; ++axis) {
    tlfea::contact::RigidNormalResponse response;
    if (tlfea::contact::EvaluateRigidNormalResponse(body, position, axes[axis], response) !=
        tlfea::contact::Status::kOk) break;
    next.axis[axis] = {response.inverse_effective_mass, response.inverse_upper, response.valid};
  }
  return next;
}
struct ReadPreparedResponse {
  const MemberResponses& values;
  unsigned next_axis = 0;
  TL_SURFACE_HD bool operator()(const tlfea::contact::RigidContactBody&,
      tl::math::Vec3, tl::math::Vec3, tlfea::contact::RigidNormalResponse& output) noexcept {
    // The shared original member loop invokes this exactly in x/y/z order.
    const auto& value = values.axis[next_axis++];
    if (!value.valid) return false;
    output = {value.inverse_effective_mass, value.inverse_upper, value.valid};
    return true;
  }
};
} // namespace tl::fea::cin_advance::group_screen
