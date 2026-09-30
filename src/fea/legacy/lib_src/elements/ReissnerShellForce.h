#pragma once

// Complete prescribed elastic Q4 force operation, adapted from the qualified
// Project Chrono ChElementShellReissner4.cpp at ab261d4baa (BSD-3-Clause; notice
// in ReissnerFrame.h). ANS row interpolation and -integral(B^T C epsilon)
// preserve the owning implementation. No tangent, dynamics, mass, material
// history, damping or contact is supplied by this operation.
#include "ReissnerShellKinematics.h"

#if defined(__CUDACC__)
#define TL_SHELL_HD __host__ __device__
#else
#define TL_SHELL_HD
#endif

namespace tl::fea::reissner {

enum class ShellStatus {
  kSuccess, kInvalidReference, kInvalidSection, kInvalidConfiguration,
  kOutsideChart, kNonfiniteResult
};

namespace shell_detail {
TL_SHELL_HD inline bool ValidPoint(const ShellPointReference& point, bool gauss) {
  if (!Finite(point.frame_offset) || !Finite(point.area_weight) ||
      (gauss ? point.area_weight <= 0 : point.area_weight != 0)) return false;
  for (unsigned axis = 0; axis < 2; ++axis) {
    if (!Finite(point.natural[axis]) || ::fabs(point.natural[axis]) > 1 ||
        !Finite(point.strain0[axis]) || !Finite(point.curvature0[axis])) return false;
    for (unsigned n = 0; n < 4; ++n)
      if (!Finite(point.gradient[n][axis])) return false;
  }
  for (unsigned n = 0; n < 4; ++n)
    if (!Finite(point.shape[n]) || point.shape[n] < 0 || point.shape[n] > 1) return false;
  return true;
}
TL_SHELL_HD inline ShellStatus ConvertStatus(Status status) {
  if (status == Status::kOutsideChart) return ShellStatus::kOutsideChart;
  if (status == Status::kNonfiniteResult) return ShellStatus::kNonfiniteResult;
  return ShellStatus::kInvalidConfiguration;
}
}  // namespace shell_detail

// Caller supplies immutable, adapter-prepared reference/section values in the
// same memory space. `prepared` catches default data; it is not authentication
// of edited tables or a substitute for model admission. The single supported
// reference is a planar rectangle, centered isotropic elasticity, four Gauss
// points and ANS. Current finite deformations must stay in the qualified mean
// chart. A successful force evaluation does not certify a dynamics step.
//
// All output fields are staged; any failure leaves output unchanged. Inputs
// and output must not overlap. No allocations, time changes, global assembly,
// atomic writes or hidden mutable caches occur. The coordinator adds returned
// world forces/couples into its one shared node space only after all required
// element operations pass. Calls may run independently on distinct outputs.
TL_SHELL_HD inline ShellStatus ComputeShellForce(const ShellReference& reference,
                                                const ElasticSection& section,
                                                const ShellConfiguration& configuration,
                                                ShellResult& output) {
  using namespace shell_detail;
  if (!reference.prepared) return ShellStatus::kInvalidReference;
  for (unsigned p = 0; p < 4; ++p)
    if (!ValidPoint(reference.gauss[p], true) || !ValidPoint(reference.ans[p], false))
      return ShellStatus::kInvalidReference;
  if (!section.prepared || !Finite(section.thickness) || section.thickness <= 0 ||
      !Finite(section.density) || section.density <= 0) return ShellStatus::kInvalidSection;
  for (unsigned i = 0; i < 144; ++i)
    if (!Finite(section.stiffness[i])) return ShellStatus::kInvalidSection;
  Kinematics state;
  auto status = PrepareKinematics(reference, configuration, state);
  if (status != Status::kSuccess) return ConvertStatus(status);

  // Four ANS samples, retaining only the two transverse rows. This avoids
  // persistent 12x24 B, 15x24 D and 15x15 G caches at every integration point.
  double ans_strain[4][2]{};
  double ans_derivative[4][2][24]{};
  for (unsigned p = 0; p < 4; ++p) {
    PointResponse sample;
    status = EvaluatePoint(state, configuration, reference.ans[p], false, sample);
    if (status != Status::kSuccess) return ConvertStatus(status);
    for (unsigned axis = 0; axis < 2; ++axis) {
      const unsigned row = 3 * axis + 2;
      ans_strain[p][axis] = sample.strain[row];
      for (unsigned c = 0; c < 24; ++c) ans_derivative[p][axis][c] = sample.derivative[row][c];
    }
  }

  ShellResult candidate;
  double generalized_force[24]{};
  for (unsigned p = 0; p < 4; ++p) {
    const auto& point = reference.gauss[p];
    PointResponse sample;
    status = EvaluatePoint(state, configuration, point, true, sample);
    if (status != Status::kSuccess) return ConvertStatus(status);
    const double ans_weight[2][4] = {{(1 + point.natural[1]) * .5, 0, (1 - point.natural[1]) * .5, 0},
                                      {0, (1 - point.natural[0]) * .5, 0, (1 + point.natural[0]) * .5}};
    for (unsigned axis = 0; axis < 2; ++axis) {
      const unsigned row = 3 * axis + 2;
      sample.strain[row] = 0;
      for (unsigned c = 0; c < 24; ++c) sample.derivative[row][c] = 0;
      for (unsigned a = 0; a < 4; ++a) {
        sample.strain[row] += ans_weight[axis][a] * ans_strain[a][axis];
        for (unsigned c = 0; c < 24; ++c)
          sample.derivative[row][c] += ans_weight[axis][a] * ans_derivative[a][axis][c];
      }
    }
    for (unsigned row = 0; row < 12; ++row) {
      candidate.strain[p][row] = sample.strain[row];
      for (unsigned column = 0; column < 12; ++column)
        candidate.resultant[p][row] += section.stiffness[12 * row + column] * sample.strain[column];
      const double energy = .5 * point.area_weight * sample.strain[row] * candidate.resultant[p][row];
      candidate.energy += energy;
      if (row >= 6) candidate.bending_energy += energy;
      if (!Finite(candidate.resultant[p][row])) return ShellStatus::kNonfiniteResult;
      for (unsigned column = 0; column < 24; ++column)
        generalized_force[column] -= point.area_weight * sample.derivative[row][column] * candidate.resultant[p][row];
    }
  }
  if (!Finite(candidate.energy) || !Finite(candidate.bending_energy)) return ShellStatus::kNonfiniteResult;
  for (unsigned i = 0; i < 24; ++i)
    if (!Finite(generalized_force[i])) return ShellStatus::kNonfiniteResult;
  for (unsigned n = 0; n < 4; ++n) {
    candidate.force[n] = {generalized_force[6*n], generalized_force[6*n+1], generalized_force[6*n+2]};
    candidate.couple[n] = {generalized_force[6*n+3], generalized_force[6*n+4], generalized_force[6*n+5]};
  }
  output = candidate;
  return ShellStatus::kSuccess;
}

}  // namespace tl::fea::reissner
#undef TL_SHELL_HD
