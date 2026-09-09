#pragma once

// P2 qualification only: unchanged Reissner point mathematics, immediate
// contraction of existing derivative blocks instead of a 12x24 B scratch.
// Adapted from Project Chrono ChElementShellReissner4.cpp, qualified branch
// ab261d4baa (2014, Alessandro Tasora and Radu Serban; BSD-3-Clause notice in
// ReissnerFrame.h). Owning point oracle: ReissnerShellKinematics.h SHA256
// c196cde6ac916d6bfda0e42ee03af4889374c7374329ec8d6239aca19f7ca460.
#include "lib_src/elements/ReissnerShellKinematics.h"

#if defined(__CUDACC__)
#define TL_GAUSS_CONTRACT_HD __host__ __device__
#else
#define TL_GAUSS_CONTRACT_HD
#endif

namespace tl::qualification::reissner_gauss_contract {
namespace shell = tl::fea::reissner;

struct GaussPointKinematics {
  double strain[12]{};
  shell::Matrix3 frame_transpose, world_gamma;
  shell::Vec3 phi, position_gradient[2], spatial_curvature[2];
};
static_assert(sizeof(GaussPointKinematics) == 360);

// The state and admitted point belong to this same configuration. Strains
// include the raw transverse rows, before the caller applies the existing ANS
// interpolation. No derivative array is built. Disjoint caller output is staged.
TL_GAUSS_CONTRACT_HD inline shell::Status PrepareGaussPoint(
    const shell::shell_detail::Kinematics& state,
    const shell::ShellConfiguration& configuration,
    const shell::ShellPointReference& point, GaussPointKinematics& output) {
  using namespace shell;
  using namespace shell::shell_detail;
  GaussPointKinematics candidate;
  candidate.phi = Interpolate(state.relative_rotation, point.shape);
  Matrix3 rotation, gamma;
  const auto status = ComputeRotationAndJacobian(candidate.phi, rotation, gamma);
  if (status != Status::kSuccess) return status;
  const auto frame = Product(Product(state.mean.frame, rotation), point.frame_offset);
  candidate.frame_transpose = Transpose(frame);
  candidate.world_gamma = Product(state.mean.frame, gamma);
  for (unsigned axis = 0; axis < 2; ++axis) {
    candidate.position_gradient[axis] = InterpolateGradient(configuration.position, point, axis);
    const auto strain = Subtract(Product(candidate.frame_transpose, candidate.position_gradient[axis]), point.strain0[axis]);
    const auto phi_gradient = InterpolateGradient(state.relative_rotation, point, axis);
    candidate.spatial_curvature[axis] = Product(candidate.world_gamma, phi_gradient);
    const auto curvature = Subtract(Product(candidate.frame_transpose, candidate.spatial_curvature[axis]), point.curvature0[axis]);
    for (unsigned c = 0; c < 3; ++c) {
      candidate.strain[3 * axis + c] = Component(strain, c);
      candidate.strain[6 + 3 * axis + c] = Component(curvature, c);
    }
    if (!Finite(candidate.position_gradient[axis]) || !Finite(candidate.spatial_curvature[axis]))
      return Status::kNonfiniteResult;
  }
  if (!Finite(candidate.frame_transpose) || !Finite(candidate.world_gamma)) return Status::kNonfiniteResult;
  for (const auto value : candidate.strain)
    if (!Finite(value)) return Status::kNonfiniteResult;
  output = candidate;
  return Status::kSuccess;
}

// Same point/state contract as PrepareGaussPoint. Resultants already include
// ANS strain interpolation. The retained four-point ANS table uses WORLD-spin
// columns. Each coordinate receives rows 0..11 in the scalar oracle's order;
// only independent nodes are reordered. All 24 accumulator entries are staged.
TL_GAUSS_CONTRACT_HD inline shell::Status AccumulateGaussPointForce(
    const shell::shell_detail::Kinematics& state,
    const shell::ShellPointReference& point, const GaussPointKinematics& sample,
    const double ans_derivative[4][2][24], const double resultant[12],
    double generalized_force[24]) {
  using namespace shell;
  using namespace shell::shell_detail;
  double candidate[24];
  for (unsigned c = 0; c < 24; ++c) {
    if (!Finite(generalized_force[c])) return Status::kNonfiniteInput;
    candidate[c] = generalized_force[c];
  }
  for (unsigned r = 0; r < 12; ++r)
    if (!Finite(resultant[r])) return Status::kNonfiniteInput;
  if (!Finite(point.area_weight) || point.area_weight <= 0) return Status::kNonfiniteInput;

  Matrix3 raw_orientation[4];
  SpinJacobian orientation;
  {
    SpinJacobian weighted;
    for (unsigned n = 0; n < 4; ++n) {
      raw_orientation[n] = Product(sample.world_gamma, state.inverse_rotation_jacobian_world[n]);
      weighted.node[n] = Scale(raw_orientation[n], point.shape[n]);
    }
    const auto status = CorrectOrientationVariation(weighted, state.mean.spin, orientation);
    if (status != Status::kSuccess) return status;
  }
  const double ans_weight[2][4] = {{(1 + point.natural[1]) * .5, 0, (1 - point.natural[1]) * .5, 0},
                                  {0, (1 - point.natural[0]) * .5, 0, (1 + point.natural[0]) * .5}};
  for (unsigned axis = 0; axis < 2; ++axis) {
    const auto position_star = Product(sample.frame_transpose, Skew(sample.position_gradient[axis]));
    for (unsigned n = 0; n < 4; ++n) {
      const auto translation = Scale(sample.frame_transpose, point.gradient[n][axis]);
      const auto rotation = Product(position_star, orientation.node[n]);
      // Raw transverse rows are checked even though ANS replaces them.
      if (!Finite(translation) || !Finite(rotation)) return Status::kNonfiniteResult;
      for (unsigned row = 0; row < 3; ++row) {
        for (unsigned c = 0; c < 3; ++c) {
          double dx = translation.v[3 * row + c], spin = rotation.v[3 * row + c];
          if (row == 2) {
            dx = 0; spin = 0;
            for (unsigned a = 0; a < 4; ++a) {
              dx += ans_weight[axis][a] * ans_derivative[a][axis][6 * n + c];
              spin += ans_weight[axis][a] * ans_derivative[a][axis][6 * n + 3 + c];
            }
          }
          candidate[6 * n + c] -= point.area_weight * dx * resultant[3 * axis + row];
          candidate[6 * n + 3 + c] -= point.area_weight * spin * resultant[3 * axis + row];
        }
      }
    }
  }
  for (unsigned axis = 0; axis < 2; ++axis) {
    const auto phi_gradient = InterpolateGradient(state.relative_rotation, point, axis);
    Matrix3 elle;
    auto status = ComputeRotationJacobianVariation(sample.phi, phi_gradient, elle);
    if (status != Status::kSuccess) return status;
    const auto world_elle = Product(state.mean.frame, elle);
    SpinJacobian frozen_curvature, complete_curvature;
    for (unsigned n = 0; n < 4; ++n)
      frozen_curvature.node[n] = Add(
          Scale(Product(world_elle, state.inverse_rotation_jacobian_world[n]), point.shape[n]),
          Scale(raw_orientation[n], point.gradient[n][axis]));
    status = CorrectCurvatureVariation(sample.spatial_curvature[axis], frozen_curvature,
                                       state.mean.spin, complete_curvature);
    if (status != Status::kSuccess) return status;
    const auto curvature_star = Skew(sample.spatial_curvature[axis]);
    for (unsigned n = 0; n < 4; ++n) {
      const auto block = Product(sample.frame_transpose,
          Add(Product(curvature_star, orientation.node[n]), complete_curvature.node[n]));
      if (!Finite(block)) return Status::kNonfiniteResult;
      for (unsigned row = 0; row < 3; ++row) {
        for (unsigned c = 0; c < 3; ++c) {
          candidate[6 * n + c] -= point.area_weight * 0.0 * resultant[6 + 3 * axis + row];
          candidate[6 * n + 3 + c] -= point.area_weight * block.v[3 * row + c] * resultant[6 + 3 * axis + row];
        }
      }
    }
  }
  for (const auto value : candidate)
    if (!Finite(value)) return Status::kNonfiniteResult;
  for (unsigned c = 0; c < 24; ++c) generalized_force[c] = candidate[c];
  return Status::kSuccess;
}

}  // namespace tl::qualification::reissner_gauss_contract
#undef TL_GAUSS_CONTRACT_HD
