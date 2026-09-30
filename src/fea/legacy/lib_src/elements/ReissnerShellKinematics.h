#pragma once

// Adapted from Project Chrono, ChElementShellReissner4.cpp (2014,
// Alessandro Tasora and Radu Serban), and the locally qualified coherent
// first-variation branch at ab261d4baa. BSD-3-Clause terms are reproduced in
// the included ReissnerFrame.h. Retains rotation/curvature/ANS mathematics;
// replaces Eigen caches with one fixed-size point scratch and WORLD spins.
#include "ReissnerShellData.h"
#include "ReissnerRotation.h"

#if defined(__CUDACC__)
#define TL_SHELL_HD __host__ __device__
#else
#define TL_SHELL_HD
#endif

namespace tl::fea::reissner {
namespace shell_detail {
using namespace detail;

struct Kinematics {
  MeanFrame mean;
  Vec3 relative_rotation[4]{};
  Matrix3 inverse_rotation_jacobian_world[4]{};
};

struct PointResponse {
  double strain[12]{};
  // d(strain)/d(x, WORLD spin), six columns per physical node.
  // Working scratch for one point only; not persistent per-element storage.
  double derivative[12][24]{};
};

TL_SHELL_HD inline Status PrepareKinematics(const ShellReference& reference,
                                          const ShellConfiguration& configuration,
                                          Kinematics& output) {
  Quaternion directors[4];
  for (unsigned n = 0; n < 4; ++n) {
    const auto q = configuration.rotation[n];
    const auto offset = reference.node_frame_offset[n];
    if (!Finite(configuration.position[n]) || !Finite(q) || !Finite(offset))
      return Status::kNonfiniteInput;
    if (::fabs(Dot(q, q) - 1) > 1e-12 || ::fabs(Dot(offset, offset) - 1) > 1e-12)
      return Status::kNonUnitQuaternion;
    directors[n] = Product(q, offset);
  }
  Kinematics candidate;
  auto status = ComputeMeanFrame(directors, candidate.mean);
  if (status != Status::kSuccess) return status;
  const auto mean_transpose = Transpose(candidate.mean.frame);
  for (unsigned n = 0; n < 4; ++n) {
    const auto relative = Product(mean_transpose, Product(Rotation(configuration.rotation[n]),
                                                         Rotation(reference.node_frame_offset[n])));
    status = ComputeRotationVector(relative, candidate.relative_rotation[n]);
    if (status != Status::kSuccess) return status;
    Matrix3 inverse;
    status = ComputeRotationJacobianInverse(candidate.relative_rotation[n], inverse);
    if (status != Status::kSuccess) return status;
    candidate.inverse_rotation_jacobian_world[n] = Product(inverse, mean_transpose);
  }
  output = candidate;
  return Status::kSuccess;
}

TL_SHELL_HD inline Vec3 Interpolate(const Vec3 values[4], const double weights[4]) {
  Vec3 result;
  for (unsigned n = 0; n < 4; ++n) result = Add(result, Scale(values[n], weights[n]));
  return result;
}

TL_SHELL_HD inline Vec3 InterpolateGradient(const Vec3 values[4],
                                           const ShellPointReference& point, unsigned axis) {
  Vec3 result;
  for (unsigned n = 0; n < 4; ++n) result = Add(result, Scale(values[n], point.gradient[n][axis]));
  return result;
}

TL_SHELL_HD inline void StoreBlock(double matrix[12][24], unsigned row,
                                  unsigned column, const Matrix3& block) {
  for (unsigned r = 0; r < 3; ++r)
    for (unsigned c = 0; c < 3; ++c) matrix[row + r][column + c] = block.v[3 * r + c];
}

// Shared by Gauss and ANS evaluations. Curvature is deliberately omitted at
// ANS points; the caller retains only their two transverse strain/B rows.
TL_SHELL_HD inline Status EvaluatePoint(const Kinematics& state,
                                       const ShellConfiguration& configuration,
                                       const ShellPointReference& point,
                                       bool curvature, PointResponse& output) {
  const auto phi = Interpolate(state.relative_rotation, point.shape);
  Matrix3 rotation, gamma;
  auto status = ComputeRotationAndJacobian(phi, rotation, gamma);
  if (status != Status::kSuccess) return status;
  const auto frame = Product(Product(state.mean.frame, rotation), point.frame_offset);
  const auto frame_transpose = Transpose(frame);
  const auto world_gamma = Product(state.mean.frame, gamma);
  Matrix3 raw_orientation[4];
  SpinJacobian weighted, orientation;
  for (unsigned n = 0; n < 4; ++n) {
    raw_orientation[n] = Product(world_gamma, state.inverse_rotation_jacobian_world[n]);
    weighted.node[n] = Scale(raw_orientation[n], point.shape[n]);
  }
  status = CorrectOrientationVariation(weighted, state.mean.spin, orientation);
  if (status != Status::kSuccess) return status;
  PointResponse candidate;
  for (unsigned axis = 0; axis < 2; ++axis) {
    const auto position_gradient = InterpolateGradient(configuration.position, point, axis);
    const auto strain = Subtract(Product(frame_transpose, position_gradient), point.strain0[axis]);
    for (unsigned c = 0; c < 3; ++c) candidate.strain[3 * axis + c] = Component(strain, c);
    const auto position_star = Product(frame_transpose, Skew(position_gradient));
    for (unsigned n = 0; n < 4; ++n) {
      StoreBlock(candidate.derivative, 3 * axis, 6 * n, Scale(frame_transpose, point.gradient[n][axis]));
      StoreBlock(candidate.derivative, 3 * axis, 6 * n + 3, Product(position_star, orientation.node[n]));
    }
    if (!curvature) continue;
    const auto phi_gradient = InterpolateGradient(state.relative_rotation, point, axis);
    const auto spatial_curvature = Product(world_gamma, phi_gradient);
    const auto material_curvature = Subtract(Product(frame_transpose, spatial_curvature), point.curvature0[axis]);
    for (unsigned c = 0; c < 3; ++c) candidate.strain[6 + 3 * axis + c] = Component(material_curvature, c);
    Matrix3 elle;
    status = ComputeRotationJacobianVariation(phi, phi_gradient, elle);
    if (status != Status::kSuccess) return status;
    const auto world_elle = Product(state.mean.frame, elle);
    SpinJacobian frozen_curvature, complete_curvature;
    for (unsigned n = 0; n < 4; ++n) {
      frozen_curvature.node[n] = Add(
          Scale(Product(world_elle, state.inverse_rotation_jacobian_world[n]), point.shape[n]),
          Scale(raw_orientation[n], point.gradient[n][axis]));
    }
    status = CorrectCurvatureVariation(spatial_curvature, frozen_curvature, state.mean.spin, complete_curvature);
    if (status != Status::kSuccess) return status;
    const auto curvature_star = Skew(spatial_curvature);
    for (unsigned n = 0; n < 4; ++n) {
      const auto derivative = Product(frame_transpose,
          Add(Product(curvature_star, orientation.node[n]), complete_curvature.node[n]));
      StoreBlock(candidate.derivative, 6 + 3 * axis, 6 * n + 3, derivative);
    }
  }
  for (unsigned r = 0; r < 12; ++r) {
    if (!Finite(candidate.strain[r])) return Status::kNonfiniteResult;
    for (unsigned c = 0; c < 24; ++c)
      if (!Finite(candidate.derivative[r][c])) return Status::kNonfiniteResult;
  }
  output = candidate;
  return Status::kSuccess;
}

}  // namespace shell_detail
}  // namespace tl::fea::reissner
#undef TL_SHELL_HD
