// Q4 shape values adapted from Project Chrono ChElementShellReissner4.cpp,
// L1/L2/L3/L4 and ShapeFunctions. Copyright (c) 2014 projectchrono.org.
// Source SHA256: 3d4d13296ab2bf5c8c1b459ae889e3125994c3eedc2949fad41c6e57dff061b5.
// Changes: checked parent/domain values, existing TL vector views, staged
// allocation-free host/device interpolation and transpose force projection.
//
// Copyright (c) 2016, Project Chrono Development Team. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// - Neither the name of the nor the names of its contributors may be used to
//   endorse or promote products derived from this software without specific
//   prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include "SurfaceContactTypes.h"

namespace tlfea::contact {

// Physical bilinear midsurface parent, never a display triangle. ID meanings
// match SurfaceTriangle: stable feature, owning FE parent and parent face.
// The caller retains asset/instance/source-ID mapping in its model binding.
// Natural node order is (+,+),(-,+),(-,-),(+,-), as in Chrono Reissner Q4.
struct SurfaceQ4 {
  std::uint32_t nodes[4]{};
  std::uint64_t feature_id = 0, parent_element_id = 0;
  std::uint32_t parent_face_id = 0;
  double half_thickness = 0;  // Only zero is admitted by this midsurface map.
};
struct Q4Point {
  std::uint32_t parent_index = 0;
  double u = 0, v = 0;
};
struct Q4SurfaceView {
  VectorView positions;
  VectorView velocities;  // Independently strided, required even for static data.
  const SurfaceQ4* parents = nullptr;
  std::uint32_t parent_count = 0;
};
struct Q4PointKinematics {
  Vec3 position, velocity;
  double shape[4]{};
};
struct Q4NodalForces {
  std::uint32_t nodes[4]{};
  Vec3 forces[4], couples[4];  // Direct WORLD couples are identically zero.
};

// No clamping, renormalization or extrapolation. Failure preserves all output
// entries. Inputs/output must not overlap; all values use one memory space.
TL_SURFACE_HD inline Status EvaluateQ4Shape(double u, double v, double* output) {
  if (!output || !IsFinite(u) || !IsFinite(v)) return Status::kInvalidArgument;
  if (::fabs(u) > 1 || ::fabs(v) > 1) return Status::kOutOfRange;
  const double shape[4] = {.25*(1+u)*(1+v), .25*(1-u)*(1+v),
                           .25*(1-u)*(1-v), .25*(1+u)*(1-v)};
  double sum = 0;
  for (double weight : shape) {
    if (!IsFinite(weight) || weight < 0 || weight > 1) return Status::kNonFiniteResult;
    sum += weight;
  }
  if (::fabs(sum - 1) > 1e-12) return Status::kNonFiniteResult;
  for (unsigned n = 0; n < 4; ++n) output[n] = shape[n];
  return Status::kOk;
}

namespace q4_detail {
TL_SURFACE_HD inline bool SameParent(const SurfaceQ4& a,const SurfaceQ4& b) {
  if (a.feature_id != b.feature_id || a.parent_element_id != b.parent_element_id ||
      a.parent_face_id != b.parent_face_id || a.half_thickness != b.half_thickness) return false;
  for (unsigned n=0;n<4;++n) if (a.nodes[n] != b.nodes[n]) return false;
  return true;
}
TL_SURFACE_HD inline Status ValidateParent(const SurfaceQ4& parent, std::uint32_t node_count) {
  if (!node_count || !parent.feature_id || !parent.parent_element_id || !IsFinite(parent.half_thickness))
    return Status::kInvalidArgument;
  if (parent.half_thickness != 0) return Status::kUnsupportedInterpolation;
  for (unsigned n = 0; n < 4; ++n) {
    if (parent.nodes[n] >= node_count) return Status::kOutOfRange;
    for (unsigned previous = 0; previous < n; ++previous)
      if (parent.nodes[n] == parent.nodes[previous]) return Status::kInvalidArgument;
  }
  return Status::kOk;
}
TL_SURFACE_HD inline Status ValidatePoint(const Q4SurfaceView& surface, const Q4Point& point) {
  if (!surface.positions.valid() || !surface.velocities.valid() ||
      surface.positions.node_count != surface.velocities.node_count || !surface.parents || !surface.parent_count)
    return Status::kInvalidArgument;
  if (point.parent_index >= surface.parent_count) return Status::kOutOfRange;
  return ValidateParent(surface.parents[point.parent_index], surface.positions.node_count);
}
}  // namespace q4_detail

// Same Q4 map for position and velocity; rotations are not inputs because this
// zero-offset midsurface has zero rotational contact Jacobian. No physical
// buffers, retained state or clock are owned or mutated. This checks mapping
// arithmetic, not rectangular footprint, reference admissibility or inversion;
// those geometry and owner/epoch checks belong to the later contact batch.
// Pointee lengths and disjoint input/output ranges are caller-owned contracts.
// All four nodal inputs are checked, including nodes with zero shape weight.
TL_SURFACE_HD inline Status EvaluateQ4Point(const Q4SurfaceView& surface, const Q4Point& point,
                                           Q4PointKinematics* output) {
  if (!output) return Status::kInvalidArgument;
  auto status = q4_detail::ValidatePoint(surface, point);
  if (status != Status::kOk) return status;
  Q4PointKinematics candidate;
  status = EvaluateQ4Shape(point.u, point.v, candidate.shape);
  if (status != Status::kOk) return status;
  const auto& parent = surface.parents[point.parent_index];
  for (unsigned n = 0; n < 4; ++n) {
    const Vec3 position = surface.positions.at(parent.nodes[n]), velocity = surface.velocities.at(parent.nodes[n]);
    if (!IsFinite(position) || !IsFinite(velocity)) return Status::kInvalidArgument;
    candidate.position = Add(candidate.position, Scale(position, candidate.shape[n]));
    candidate.velocity = Add(candidate.velocity, Scale(velocity, candidate.shape[n]));
  }
  if (!IsFinite(candidate.position) || !IsFinite(candidate.velocity)) return Status::kNonFiniteResult;
  *output = candidate;
  return Status::kOk;
}

// Return J^T*f in natural parent order; do not write shared physical buffers.
// The later batch performs checked additive assembly once per contribution.
// Arbitrary force directions are admitted by this algebraic map for independent
// moment/work tests; the bounded wall law separately enforces normal-only force.
// Force projection needs valid topology/views, not the values of x/v. Failure
// preserves the complete caller result, as in EvaluateQ4Point.
TL_SURFACE_HD inline Status ProjectQ4PointForce(const Q4SurfaceView& surface, const Q4Point& point,
                                               Vec3 force, Q4NodalForces* output) {
  if (!output || !IsFinite(force)) return Status::kInvalidArgument;
  auto status = q4_detail::ValidatePoint(surface, point);
  if (status != Status::kOk) return status;
  double shape[4];
  status = EvaluateQ4Shape(point.u, point.v, shape);
  if (status != Status::kOk) return status;
  Q4NodalForces candidate;
  for (unsigned n = 0; n < 4; ++n) {
    candidate.nodes[n] = surface.parents[point.parent_index].nodes[n];
    candidate.forces[n] = Scale(force, shape[n]);
    if (!IsFinite(candidate.forces[n])) return Status::kNonFiniteResult;
  }
  *output = candidate;
  return Status::kOk;
}

}  // namespace tlfea::contact
