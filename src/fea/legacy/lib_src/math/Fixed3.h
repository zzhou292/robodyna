// =============================================================================
// Adapted from Project Chrono's locally qualified ChReissnerFrame.h/.cpp.
// Copyright (c) 2026 projectchrono.org. All rights reserved.
// Source paths: chrono/src/chrono/fea/ChReissnerFrame.{h,cpp}.
// Source SHA256 (header):
// 90e6769b67cdb6f287d0f38cdea0c74e631600cb2d951765608160d5a805323b
// Source SHA256 (implementation):
// 360f58c96fa24cb46dc044592adee848e777cc23a8c53d7ade859697b0834a30
// These identify the local qualified helper, not an upstream release.
// Quaternion product/world derivative and rotation-matrix arithmetic follow
// Chrono core ChQuaternion.h/.cpp and ChMatrix33.h. Changes here replace
// Eigen/Chrono objects with fixed-size binary64 values and host/device inline
// functions; the chart, first variations and failure order are retained.
// No element force, tangent, material history, mass or dynamics is implemented.
//
// The Chrono distribution license is reproduced for this adaptation:
// Copyright (c) 2016, Project Chrono Development Team
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//  - Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  - Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  - Neither the name of the nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
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
// =============================================================================

// Shared fixed-size records extracted without layout or arithmetic changes.
#pragma once
namespace tl::math {
struct Vec3 {
  double x = 0, y = 0, z = 0;
};
struct Matrix3 {
  double v[9]{};  // Row-major; default is the zero matrix.
};
}  // namespace tl::math
