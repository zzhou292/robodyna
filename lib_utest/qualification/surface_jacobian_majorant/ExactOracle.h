// SPDX-License-Identifier: MIT
#pragma once
#include "lib_src/collision/SurfaceJacobianMajorantValues.h"

namespace majorant_test {
namespace ct = tlfea::contact;
bool ExactBounds(const ct::SurfaceJacobianMajorant&);
bool ExactQuadratic(const ct::SurfaceJacobianMajorant&, const ct::Vec3* velocities);
// A represented node-to-generalized-coordinate matrix, row-major [3*count][columns].
bool ExactCongruence(const ct::SurfaceJacobianMajorant&, const double* transform,
    unsigned columns, const double* generalized);
bool OldWeightBoundFails(double represented_normal_x);
} // namespace majorant_test
