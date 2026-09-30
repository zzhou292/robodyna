#pragma once

// P2 qualification only: direct transverse ANS rows, unchanged Gauss operation.
// Adapted from the qualified Project Chrono ChElementShellReissner4.cpp at
// ab261d4baa (2014, Alessandro Tasora and Radu Serban; BSD-3-Clause terms in
// the included ReissnerFrame.h). The frozen TL scalar orchestration is
// ReissnerShellForce.h, SHA256
// 870bb72075f5a734f5dd0209d63162fee62a693c5ddc71b58397d95f587801ca;
// point arithmetic is ReissnerShellKinematics.h, SHA256
// c196cde6ac916d6bfda0e42ee03af4889374c7374329ec8d6239aca19f7ca460.
// Production ComputeShellForce remains unchanged and is the separate oracle.
#include "lib_src/elements/ReissnerShellForce.h"

#if defined(__CUDACC__)
#define TL_ANS_ROWS_HD __host__ __device__
#else
#define TL_ANS_ROWS_HD
#endif

namespace tl::qualification::reissner_ans_rows {
namespace shell = tl::fea::reissner;

struct AnsPointResponse {
  // Full response rows 2 and 5: transverse components of eps1 and eps2.
  double strain[2]{};
  // d(strain)/d(x, WORLD spin), six columns per physical node.
  double derivative[2][24]{};
};
static_assert(sizeof(AnsPointResponse) == 400);

namespace detail {
// Row times matrix, using the owning dot operation in the same product/sum
// order as shell_detail::EvaluatePoint. No full three-row block is formed.
TL_ANS_ROWS_HD inline shell::Vec3 RightProduct(shell::Vec3 row, const shell::Matrix3& matrix) {
  using shell::detail::Dot;
  return {Dot(row, {matrix.v[0], matrix.v[3], matrix.v[6]}),
          Dot(row, {matrix.v[1], matrix.v[4], matrix.v[7]}),
          Dot(row, {matrix.v[2], matrix.v[5], matrix.v[8]})};
}
}  // namespace detail

// Lower-level point operation: state is from PrepareKinematics for this same
// configuration, and point is an admitted ANS reference. No curvature or
// unused membrane rows are constructed. Shape weights enter the frozen
// variation exactly once; zero shape weights still have mean-frame correction.
// All output is staged, and inputs/output must occupy disjoint storage.
TL_ANS_ROWS_HD inline shell::Status EvaluateTransversePoint(
    const shell::shell_detail::Kinematics& state,
    const shell::ShellConfiguration& configuration,
    const shell::ShellPointReference& point, AnsPointResponse& output) {
  using namespace shell;
  using namespace shell::shell_detail;
  const auto phi = Interpolate(state.relative_rotation, point.shape);
  Matrix3 rotation, gamma;
  auto status = ComputeRotationAndJacobian(phi, rotation, gamma);
  if (status != Status::kSuccess) return status;
  const auto frame = Product(Product(state.mean.frame, rotation), point.frame_offset);
  const auto frame_transpose = Transpose(frame);
  const Vec3 transverse{frame_transpose.v[6], frame_transpose.v[7], frame_transpose.v[8]};
  const auto world_gamma = Product(state.mean.frame, gamma);
  SpinJacobian weighted, orientation;
  for (unsigned n = 0; n < 4; ++n)
    weighted.node[n] = Scale(Product(world_gamma, state.inverse_rotation_jacobian_world[n]), point.shape[n]);
  status = CorrectOrientationVariation(weighted, state.mean.spin, orientation);
  if (status != Status::kSuccess) return status;

  AnsPointResponse candidate;
  for (unsigned axis = 0; axis < 2; ++axis) {
    const auto position_gradient = InterpolateGradient(configuration.position, point, axis);
    candidate.strain[axis] = Dot(transverse, position_gradient) - point.strain0[axis].z;
    const auto position_star_row = detail::RightProduct(transverse, Skew(position_gradient));
    for (unsigned n = 0; n < 4; ++n) {
      const auto translation_row = Scale(transverse, point.gradient[n][axis]);
      const auto rotation_row = detail::RightProduct(position_star_row, orientation.node[n]);
      for (unsigned c = 0; c < 3; ++c) {
        candidate.derivative[axis][6 * n + c] = Component(translation_row, c);
        candidate.derivative[axis][6 * n + 3 + c] = Component(rotation_row, c);
      }
    }
    if (!Finite(candidate.strain[axis])) return Status::kNonfiniteResult;
    for (unsigned c = 0; c < 24; ++c)
      if (!Finite(candidate.derivative[axis][c])) return Status::kNonfiniteResult;
  }
  output = candidate;
  return Status::kSuccess;
}

// Same prescribed planar-rectangle/centered-isotropic-elastic domain, complete
// result, chart checks and disjoint-input/staged-output contract as the scalar
// oracle. This experiment changes only ANS point construction. It retains the
// four-sample ANS table and full existing Gauss EvaluatePoint; no immediate
// Gauss contraction, dynamics, mass, history or production dispatch is added.
TL_ANS_ROWS_HD inline shell::ShellStatus ComputeShellForceAnsRows(
    const shell::ShellReference& reference, const shell::ElasticSection& section,
    const shell::ShellConfiguration& configuration, shell::ShellResult& output) {
  using namespace shell;
  using namespace shell::shell_detail;
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

  double ans_strain[4][2]{};
  double ans_derivative[4][2][24]{};
  for (unsigned p = 0; p < 4; ++p) {
    AnsPointResponse sample;
    status = EvaluateTransversePoint(state, configuration, reference.ans[p], sample);
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
    candidate.force[n] = {generalized_force[6 * n], generalized_force[6 * n + 1], generalized_force[6 * n + 2]};
    candidate.couple[n] = {generalized_force[6 * n + 3], generalized_force[6 * n + 4], generalized_force[6 * n + 5]};
  }
  output = candidate;
  return ShellStatus::kSuccess;
}

}  // namespace tl::qualification::reissner_ans_rows
#undef TL_ANS_ROWS_HD
