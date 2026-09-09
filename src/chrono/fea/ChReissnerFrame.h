// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
// =============================================================================

#ifndef CH_REISSNER_FRAME_H
#define CH_REISSNER_FRAME_H

#include <array>

#include "chrono/core/ChMatrix33.h"

namespace chrono {
namespace fea {

/// Checked stateless operations for a four-director Reissner interpolation.
/// These first variations do not qualify an element force, tangent, material,
/// or time integrator. All increments and output Jacobians use WORLD spins:
/// delta(R) = [delta(theta)]_x R. Operations allocate no storage and leave the
/// output unchanged on failure. Input and output arrays may alias.
enum class ChReissnerFrameStatus { Success, NonfiniteInput, NonUnitQuaternion, OutsideChart, NonfiniteResult };

using ChReissnerSpinJacobian = std::array<ChMatrix33<>, 4>;

struct ChReissnerMeanFrame {
    ChQuaterniond rotation{1, 0, 0, 0};
    ChMatrix33<> frame{1};
    /// delta(theta_mean) = sum_n spin[n] delta(theta_n).
    ChReissnerSpinJacobian spin{{ChMatrix33<>(0), ChMatrix33<>(0), ChMatrix33<>(0), ChMatrix33<>(0)}};
};

/// Equal-weight, sign-aligned normalized quaternion mean and its analytic first
/// variation. The mean follows the existing alternative in Reissner4's
/// UpdateNodalAndAveragePosAndOrientation; its variation uses Chrono's spatial
/// quaternion derivative, the normalization derivative, and quaternion product.
///
/// Every input quaternion must be finite with |q.q - 1| <= 1e-12. Invalid
/// quaternions are rejected, not replaced by an identity or silently repaired.
/// After alignment to node 0, EVERY pair must have dot > cos(pi/4), bounding
/// relative director rotations strictly below 90 degrees. This open chart
/// makes the signs unambiguous for all node permutations. Arbitrary common
/// rigid rotation is allowed; a wider relative-rotation chart is not admitted.
/// The quaternion output is sign-equivalent, not component-sign canonical.
ChApi ChReissnerFrameStatus ComputeReissnerMeanFrame(const std::array<ChQuaterniond, 4>& rotation,
                                                   ChReissnerMeanFrame& output);

/// Complete an orientation derivative that was calculated holding the mean
/// frame fixed. weighted_frozen[n] ALREADY includes its shape weight N_n:
/// W*_n = W_n + (I - sum_m W_m) B_n, where B_n = mean_spin[n].
/// Return full weighted derivatives, including contributions at zero ANS
/// shape weights. Never divide by N_n or multiply this result by N_n again.
/// Inputs must describe the same current configuration and WORLD convention.
ChApi ChReissnerFrameStatus CorrectReissnerOrientationVariation(const ChReissnerSpinJacobian& weighted_frozen,
                                                               const ChReissnerSpinJacobian& mean_spin,
                                                               ChReissnerSpinJacobian& output);

/// Complete the spatial curvature derivative for one reference direction:
/// C*_n = C_n + (-[k]_x - sum_m C_m) B_n.
/// spatial_curvature is the current WORLD k = A DRot(phi) phi_,alpha;
/// frozen[n] is its derivative holding A fixed. No material-frame curvature
/// conversion is performed here. The caller must include the completed
/// orientation variation when differentiating T^T k.
ChApi ChReissnerFrameStatus CorrectReissnerCurvatureVariation(const ChVector3d& spatial_curvature,
                                                            const ChReissnerSpinJacobian& frozen,
                                                            const ChReissnerSpinJacobian& mean_spin,
                                                            ChReissnerSpinJacobian& output);

}  // namespace fea
}  // namespace chrono

#endif
