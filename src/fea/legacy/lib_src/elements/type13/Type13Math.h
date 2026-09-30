// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13Deformation.h"
#include "Type13Response.h"
#include "Type13Stability.h"
#include "../spring/SpringFrame.h"
#include "../spring/SpringScatter.h"

namespace tl::fea::type13 {
namespace detail {
TL_TYPE13_HD inline Status ConvertSIValues(const Property& property,
                                          Evaluation& next) {
  const auto* channels = next.native_history.channels;
  const Vec3 force{channels[0].force, channels[1].force, channels[2].force};
  const Vec3 couple{channels[3].force, channels[4].force, channels[5].force};
  spring::WrenchValues native_wrenches[2];
  if (!spring::Scatter(next.native_frame.axes, next.native_frame.length,
                       force, couple, native_wrenches)) {
    return Status::NonfiniteResult;
  }
  UnitFactors units;
  if (!ResolveUnits(property.units(), units)) {
    return Status::InvalidInput;
  }
  const double moment = units.force * property.units().length_to_m;
  if (!Positive(moment)) {
    return Status::NonfiniteResult;
  }
  next.local_force_N = tl::math::fixed3::Scale(force, units.force);
  next.local_couple_Nm = tl::math::fixed3::Scale(couple, moment);
  for (unsigned i = 0; i < 2; ++i) {
    next.endpoints[i] = {
        tl::math::fixed3::Scale(native_wrenches[i].force, units.force),
        tl::math::fixed3::Scale(native_wrenches[i].couple, moment)};
    if (!tl::math::fixed3::Finite(next.endpoints[i].force_N) ||
        !tl::math::fixed3::Finite(next.endpoints[i].couple_Nm)) {
      return Status::NonfiniteResult;
    }
  }
  const double total = channels[0].signed_work + channels[1].signed_work +
      channels[2].signed_work + channels[3].signed_work +
      channels[4].signed_work + channels[5].signed_work;
  next.total_signed_work_J = total * moment;
  for (unsigned i = 0; i < ChannelCount; ++i) {
    next.signed_work_J[i] = channels[i].signed_work * moment;
    if (!tl::math::fixed3::Finite(next.signed_work_J[i])) {
      return Status::NonfiniteResult;
    }
  }
  if (!tl::math::fixed3::Finite(next.total_signed_work_J) ||
      !tl::math::fixed3::Finite(next.local_force_N) ||
      !tl::math::fixed3::Finite(next.local_couple_Nm)) {
    return Status::NonfiniteResult;
  }
  return Status::Success;
}

TL_TYPE13_HD inline Status ForcePacket(
    const Property& property, const Reference& reference,
    const NativeHistory& accepted, const NativeEndpointKinematics (&nodes)[2],
    double dt, Evaluation& output) {
  Evaluation next;
  next.native_history = accepted;
  spring::FrameValues frame;
  const auto frame_status = spring::AdvanceFrame(
      accepted.transverse_axis, nodes, dt, 1e-15, frame);
  if (frame_status == spring::Status::NonfiniteResult) {
    return Status::NonfiniteResult;
  }
  if (frame_status != spring::Status::Success) {
    return Status::DegenerateGeometry;
  }
  next.native_frame = {frame.axes, frame.midpoint_axes,
                       frame.length, frame.midpoint_length};
  auto status = Deformation(reference, accepted, next.native_frame,
                            nodes, dt, next.native_history);
  if (status != Status::Success) {
    return status;
  }
  status = Response(property, reference, accepted, dt, next.native_history);
  if (status != Status::Success) {
    return status;
  }
  status = ConvertSIValues(property, next);
  if (status != Status::Success) {
    return status;
  }
  status = CriticalStep(property, reference, frame.length, next.stability);
  if (status != Status::Success) {
    return status;
  }
  next.newly_failed = accepted.active && !next.native_history.active;
  output = next;
  return Status::Success;
}
} // namespace detail

// Exact fresh TT=0, DT1=0 force packet, no INISPRI/preload, original positions,
// uniform translation and zero spin. Even dt=0 passes through native frame
// normalization; the result is the completed initial force cache/history.
TL_TYPE13_HD inline Status InitializeForce(
    const Property& property, const Reference& reference,
    const NativeEndpointKinematics (&nodes)[2], Evaluation& output) {
  if (!detail::ValidReference(property, reference) || !detail::ValidNodes(nodes)) {
    return Status::InvalidInput;
  }
  for (unsigned i = 0; i < 2; ++i) {
    const auto si = tl::math::fixed3::Scale(nodes[i].position,
                                           property.units().length_to_m);
    const auto& expected = reference.position_m[i];
    if (si.x != expected.x || si.y != expected.y || si.z != expected.z ||
        nodes[i].angular_velocity.x != 0 ||
        nodes[i].angular_velocity.y != 0 ||
        nodes[i].angular_velocity.z != 0) {
      return Status::UnsupportedScope;
    }
  }
  if (nodes[0].velocity.x != nodes[1].velocity.x ||
      nodes[0].velocity.y != nodes[1].velocity.y ||
      nodes[0].velocity.z != nodes[1].velocity.z) {
    return Status::UnsupportedScope;
  }
  NativeHistory virgin;
  virgin.transverse_axis = tl::math::fixed3::Column(reference.axes, 1);
  return detail::ForcePacket(property, reference, virgin, nodes, 0, output);
}

// TT>0 packet: native working units, positive DT1, actual endpoint x and interval
// midpoint v/spin. No clock/owner/token is manufactured or advanced here. The
// caller authenticates phase and chooses whether to publish the staged result.
// Accepted input and output may alias; failures leave the entire output intact.
TL_TYPE13_HD inline Status Evaluate(
    const Property& property, const Reference& reference,
    const NativeHistory& accepted, const NativeEndpointKinematics (&nodes)[2],
    double native_dt, Evaluation& output) {
  if (!detail::ValidReference(property, reference) ||
      !detail::ValidHistory(accepted) || !detail::ValidNodes(nodes) ||
      !detail::Positive(native_dt)) {
    return Status::InvalidInput;
  }
  return detail::ForcePacket(property, reference, accepted, nodes,
                             native_dt, output);
}
} // namespace tl::fea::type13
