// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceSection.h"
#include "ForceDamping.h"
#include "ForceStiffness.h"
#include "ForceAssembly.h"

namespace tl::fea::beam18 {
struct ForceHistoryWriter {
  TL_BEAM18_HD static void Store(const Reference& reference, const Material& material,
      const HistoryValues& values, ForceStamp stamp, ForceHistory& output) noexcept {
    output.reference_ = reference; output.material_ = material;
    output.values_ = values; output.stamp_ = stamp; output.prepared_ = true;
  }
};
namespace force_detail {
TL_BEAM18_HD inline Status Calculate(const Reference& reference, const Material& material,
    const HistoryValues& accepted, const PrescribedInterval& interval, ForceTrial& output) noexcept {
  ForceTrial next;
  HistoryValues history;
  const auto section = SectionSI(reference);
  if (!Geometry(accepted,interval,next.geometry)) return Status::DegenerateGeometry;
  if (!Stiffness(reference,section,material,next.geometry.length_m,next.diagnostics) ||
      !Rates(next.geometry,interval,next.rate) ||
      !SectionResponse(section,material,accepted,next.rate,interval.dt_s,
          next.geometry.length_m,next,history) ||
      !Damping(section,material,history,next.rate,interval.dt_s,next.geometry.length_m,next.diagnostics) ||
      !EndpointForces(next)) return Status::NonfiniteResult;
  history.section_seed = next.geometry.axis[1];
  ForceHistoryWriter::Store(reference,material,history,
      {interval.base_time_s+interval.dt_s,interval.sample_index},next.proposed_history);
  output = next;
  return Status::Success;
}
} // namespace force_detail
// Native virgin OFF1, zero point/global history and source PEVECI seed. A
// constructor at sample zero has no completed interval or finite-step update.
TL_BEAM18_HD inline Status InitializeForce(const Reference& reference, const Material& material,
    Vec3 uniform_velocity_m_s, ForceTrial& output) noexcept {
  if (!force_detail::MaterialValid(reference,material) ||
      !tl::math::fixed3::Finite(uniform_velocity_m_s)) return Status::InvalidInput;
  HistoryValues virgin;
  virgin.section_seed = reference.geometry().orientation_seed;
  PrescribedInterval initial;
  for (unsigned n = 0; n < 2; ++n) {
    initial.position_endpoint_m[n] = reference.geometry().endpoint_m[n];
    initial.velocity_midpoint_m_s[n] = uniform_velocity_m_s;
  }
  return force_detail::Calculate(reference,material,virgin,initial,output);
}
TL_BEAM18_HD inline Status EvaluateForce(const Reference& reference, const Material& material,
    const ForceHistory& accepted, const PrescribedInterval& interval, ForceTrial& output) noexcept {
  if (!force_detail::MaterialValid(reference,material) ||
      !force_detail::SameReference(reference,accepted.reference()) ||
      !force_detail::SameMaterial(material,accepted.material()) ||
      !force_detail::HistoryValid(material,accepted.values()) ||
      !force_detail::IntervalValid(accepted,interval)) return Status::InvalidInput;
  return force_detail::Calculate(reference,material,accepted.values(),interval,output);
}
} // namespace tl::fea::beam18
