#pragma once

// P2 qualification only. Reuses stage-1 ANS and replaces only Gauss derivative
// storage/contraction. Adapted from qualified Project Chrono Reissner4 at
// ab261d4baa (2014, Alessandro Tasora and Radu Serban; BSD-3-Clause terms in
// ReissnerFrame.h). Production scalar force SHA256 remains
// 870bb72075f5a734f5dd0209d63162fee62a693c5ddc71b58397d95f587801ca.
#include "ReissnerShellForceAnsRows.h"
#include "ReissnerGaussContraction.h"

#if defined(__CUDACC__)
#define TL_GAUSS_FORCE_HD __host__ __device__
#else
#define TL_GAUSS_FORCE_HD
#endif

namespace tl::qualification::reissner_gauss_contract {

// Admitted planar rectangles, centered isotropic elastic section, full ANS and
// the same strict director chart as the scalar operation. Disjoint inputs and
// complete output are staged. No production dispatch, state, history or clock.
TL_GAUSS_FORCE_HD inline shell::ShellStatus ComputeShellForceGaussContract(
    const shell::ShellReference& reference, const shell::ElasticSection& section,
    const shell::ShellConfiguration& configuration, shell::ShellResult& output) {
  using namespace shell;
  using namespace shell::shell_detail;
  if (!reference.prepared) return ShellStatus::kInvalidReference;
  for (unsigned p = 0; p < 4; ++p)
    if (!ValidPoint(reference.gauss[p], true) || !ValidPoint(reference.ans[p], false)) return ShellStatus::kInvalidReference;
  if (!section.prepared || !Finite(section.thickness) || section.thickness <= 0 ||
      !Finite(section.density) || section.density <= 0) return ShellStatus::kInvalidSection;
  for (unsigned i = 0; i < 144; ++i)
    if (!Finite(section.stiffness[i])) return ShellStatus::kInvalidSection;
  Kinematics state;
  auto status = PrepareKinematics(reference, configuration, state);
  if (status != Status::kSuccess) return ConvertStatus(status);
  double ans_strain[4][2]{};
  double ans_derivative[4][2][24]{};
  for (unsigned p = 0; p < 4; ++p) {
    reissner_ans_rows::AnsPointResponse sample;
    status = reissner_ans_rows::EvaluateTransversePoint(state, configuration, reference.ans[p], sample);
    if (status != Status::kSuccess) return ConvertStatus(status);
    for (unsigned axis = 0; axis < 2; ++axis) {
      ans_strain[p][axis] = sample.strain[axis];
      for (unsigned c = 0; c < 24; ++c) ans_derivative[p][axis][c] = sample.derivative[axis][c];
    }
  }

  ShellResult candidate;
  double generalized_force[24]{};
  for (unsigned p = 0; p < 4; ++p) {
    const auto& point = reference.gauss[p];
    GaussPointKinematics sample;
    status = PrepareGaussPoint(state, configuration, point, sample);
    if (status != Status::kSuccess) return ConvertStatus(status);
    const double ans_weight[2][4] = {{(1 + point.natural[1]) * .5, 0, (1 - point.natural[1]) * .5, 0},
                                    {0, (1 - point.natural[0]) * .5, 0, (1 + point.natural[0]) * .5}};
    for (unsigned axis = 0; axis < 2; ++axis) {
      const unsigned row = 3 * axis + 2;
      sample.strain[row] = 0;
      for (unsigned a = 0; a < 4; ++a) sample.strain[row] += ans_weight[axis][a] * ans_strain[a][axis];
    }
    for (unsigned row = 0; row < 12; ++row) {
      candidate.strain[p][row] = sample.strain[row];
      for (unsigned column = 0; column < 12; ++column)
        candidate.resultant[p][row] += section.stiffness[12 * row + column] * sample.strain[column];
      const double energy = .5 * point.area_weight * sample.strain[row] * candidate.resultant[p][row];
      candidate.energy += energy;
      if (row >= 6) candidate.bending_energy += energy;
      if (!Finite(candidate.resultant[p][row])) return ShellStatus::kNonfiniteResult;
    }
    status = AccumulateGaussPointForce(state, point, sample, ans_derivative, candidate.resultant[p], generalized_force);
    if (status != Status::kSuccess) return ConvertStatus(status);
  }
  if (!Finite(candidate.energy) || !Finite(candidate.bending_energy)) return ShellStatus::kNonfiniteResult;
  for (unsigned n = 0; n < 4; ++n) {
    candidate.force[n] = {generalized_force[6 * n], generalized_force[6 * n + 1], generalized_force[6 * n + 2]};
    candidate.couple[n] = {generalized_force[6 * n + 3], generalized_force[6 * n + 4], generalized_force[6 * n + 5]};
  }
  output = candidate;
  return ShellStatus::kSuccess;
}

}  // namespace tl::qualification::reissner_gauss_contract
#undef TL_GAUSS_FORCE_HD
