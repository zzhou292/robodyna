// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens. See source manifest.
#pragma once
#include "QephCurrentFrame.h"
#include "QephVelocityCorrection.h"
#include "QephProjectionUnits.h"
#include "QephRates.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline bool SaneReference(const ReferenceData& r) {
  // Caller preserves the startup record immutably. These bounded checks catch
  // malformed PODs; they are not provenance authentication and do not repeat
  // startup frame/mass assembly on every prescribed evaluation.
  const auto& in=r.input;
  if(!r.prepared||!ValidProjectionLength(in.projection_working_length_m)||!ValidShellReferencePlacement(r.input.placement)||!Positive(in.density)||!Positive(in.young_modulus)||!Positive(in.thickness)||
     !tl::math::Finite(in.poisson_ratio)||in.poisson_ratio<0||in.poisson_ratio>=.5||
     !Positive(r.area)||!Proper(r.frame)) return false;
  for(unsigned n=0;n<4;++n) {
    if(!Finite(in.position[n])||!Finite(r.local_position[n])||r.local_position[n].z!=0||
       !tl::math::Finite(r.derivative_x[n])||!tl::math::Finite(r.derivative_y[n])||
       !Positive(r.nodal_mass[n])||!Positive(r.physical_inertia[n])||
       !Positive(r.added_inertia[n])||!Positive(r.isotropic_inertia[n])) return false;
    for(unsigned j=0;j<n;++j) if(in.node_ids[n]==in.node_ids[j]) return false;
  }
  return true;
}
TL_QEPH_HD inline bool FiniteKinematics(const Kinematics& o) {
  if(!ValidProjectionLength(o.projection_metric.working_length_m)||
     !Proper(o.frame)||!Positive(o.area)||!Positive(o.reciprocal_area)||!Positive(o.characteristic_length)||
     !Positive(o.nodal_factors[0])||!Positive(o.nodal_factors[1])||
     !tl::math::Finite(o.raw_warpage_abs)||!tl::math::Finite(o.effective_warpage)) return false;
  for(unsigned n=0;n<4;++n)
    if(!Finite(o.local_position[n])||!Finite(o.local_normals[n])||!Finite(o.projection_columns[n])) return false;
  for(double value:o.projection_inverse) if(!tl::math::Finite(value)) return false;
  for(double value:o.projected_omega) if(!tl::math::Finite(value)) return false;
  for(double value:o.regular_rate) if(!tl::math::Finite(value)) return false;
  for(double value:o.hourglass_rate) if(!tl::math::Finite(value)) return false;
  return true;
}
TL_QEPH_HD inline bool FiniteWork(const GeometryWork& g) {
  const double data[]{g.x13,g.x24,g.y13,g.y24,g.mx13,g.mx23,g.mx34,g.my13,g.my23,g.my34,
                       g.l13,g.l24,g.lm,g.py1,g.px2,g.py2};
  for(double value:data) if(!tl::math::Finite(value)) return false;
  return Finite(g.v13)&&Finite(g.v24)&&Finite(g.vhi)&&FiniteKinematics(g.values);
}

// Private staged work entry also serves the later coherent material/force
// chain. No stage consumes already packed public diagnostics as a substitute
// for these source intermediates. Failure leaves the supplied work untouched.
TL_QEPH_HD inline Status PrepareGeometry(const ReferenceData& reference,
                                        const PrescribedInterval& interval,GeometryWork& output) {
  if(!SaneReference(reference)) return Status::kInvalidReference;
  if(!tl::math::Finite(interval.base_time)||interval.base_time<0||!Positive(interval.dt)||
     !tl::math::Finite(interval.base_time+interval.dt)||
     !(interval.base_time+interval.dt>interval.base_time)) return Status::kInvalidInput;
  for(unsigned n=0;n<4;++n)
    if(!Finite(interval.position_endpoint[n])||!Finite(interval.velocity_midpoint[n])||
       !Finite(interval.omega_midpoint[n])) return Status::kInvalidInput;
  GeometryWork candidate;
  auto status=CurrentGeometry(interval,candidate);
  if(status!=Status::kSuccess) return status;
  GatherRates(interval,candidate);
  CorrectMidpointVelocity(interval.dt,candidate);
  status=ProjectWarpedRatesInWorkingLength(interval,reference.input.projection_working_length_m,candidate);
  if(status!=Status::kSuccess) return status;
  NormalizeRates(candidate);
  ComputeRates(candidate);
  auto& o=candidate.values;
  for(unsigned n=0;n<4;++n) o.local_position[n].z=o.effective_warpage*(n%2?-1.:1.);
  if(!FiniteWork(candidate)) return Status::kNonfiniteResult;
  o.base_time=interval.base_time; o.dt=interval.dt; o.sample_index=interval.sample_index;
  output=candidate;
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::detail

namespace tl::fea::qeph {
// Current one-cell prescribed geometry/rates, fixed native explicit QEPH path:
// IREP0, IRESP2, ISMSTR-1, IMPL_S0, IVECTOR0, NPT0, IDRIL0, IXFEM0.
// ReferenceData must be the immutable successful startup result; public POD
// sanity checks cannot authenticate arbitrary coordinated finite mutations.
// Current geometry uses the declared Q3a determinant/convexity thresholds,
// but the actual ENGINE frame arithmetic. Every observable must be finite.
// Native planar switching and its O(h) general-rigid shear-rate residual are
// preserved. This is not dynamics admission or an initial-force operation.
// All output bytes are unchanged on failure; no owner/clock/history is mutated.
TL_QEPH_HD inline Status EvaluatePrescribed(const ReferenceData& reference,
                                           const PrescribedInterval& interval,Kinematics& output) {
  detail::GeometryWork candidate;
  const auto status=detail::PrepareGeometry(reference,interval,candidate);
  if(status!=Status::kSuccess) return status;
  output=candidate.values;
  return Status::kSuccess;
}
} // namespace tl::fea::qeph
