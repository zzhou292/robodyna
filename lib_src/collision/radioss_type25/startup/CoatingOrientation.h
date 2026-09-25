// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 SEG_INS geometry after confirmed shell/solid membership.
#pragma once
#include "Types.h"

namespace tlfea::contact::radioss_type25::startup {
// Actual ordered native NDS coordinate packet, including repeated slots. The
// segment slots identify its first three corners inside that same packet.
// Source identity, incident-solid choice and coordinate-unit authority remain
// with the source binder. This value does not search or admit a contact surface.
struct NativeCoatingOrientationInput {
    unsigned node_count = 0;
    Vector positions[20]{};
    unsigned segment_slots[3]{};
};
enum class CoatingOrientation : int { Unspecified = 0, Forward = 1, Reversed = -1 };
struct NativeCoatingOrientationResult {
    CoatingOrientation orientation = CoatingOrientation::Unspecified;
    // Native VOL: the ordered determinant, not a physical volume divided by six.
    double center_triangle_determinant = 0;
};

namespace coating_detail {
TL_MATH_HOST_DEVICE inline bool Finite(Vector value) noexcept {
    return tl::math::Finite(value.x) && tl::math::Finite(value.y) && tl::math::Finite(value.z);
}
}

// Native low-order NDS uses eight slots even for repeated-node solids. Higher
// selected native packets use 10/16/20 slots. Never infer arity from unique nodes.
// All inputs are consumed before the single output publication. Rejection keeps
// output unchanged; unused position slots are not inspected. Zero determinant
// is Forward, exactly matching the source's strict VOL > ZERO test.
TL_MATH_HOST_DEVICE inline Status EvaluateNativeCoatingOrientation(
        const NativeCoatingOrientationInput& input,
        NativeCoatingOrientationResult* output) noexcept {
    if (!output) return Status::InvalidInput;
    const auto count = input.node_count;
    if (count != 8 && count != 10 && count != 16 && count != 20)
        return Status::UnsupportedProfile;
    for (unsigned corner = 0; corner < 3; ++corner)
        if (input.segment_slots[corner] >= count) return Status::InvalidInput;

    Vector center{};
    for (unsigned node = 0; node < count; ++node) {
        const auto point = input.positions[node];
        if (!coating_detail::Finite(point)) return Status::InvalidInput;
        center.x = center.x + point.x;
        center.y = center.y + point.y;
        center.z = center.z + point.z;
        if (!coating_detail::Finite(center)) return Status::NonfiniteResult;
    }
    center.x = center.x / count;
    center.y = center.y / count;
    center.z = center.z / count;

    const auto p1 = input.positions[input.segment_slots[0]];
    const auto p2 = input.positions[input.segment_slots[1]];
    const auto p3 = input.positions[input.segment_slots[2]];
    const Vector d41{p3.x - center.x, p3.y - center.y, p3.z - center.z};
    const Vector d42{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};
    const Vector d43{p3.x - p2.x, p3.y - p2.y, p3.z - p2.z};
    if (!coating_detail::Finite(d41) || !coating_detail::Finite(d42) || !coating_detail::Finite(d43))
        return Status::NonfiniteResult;

    const double nx = d43.y * d42.z - d42.y * d43.z;
    const double ny = d43.z * d42.x - d42.z * d43.x;
    const double nz = d43.x * d42.y - d42.x * d43.y;
    const double determinant = d41.x * nx + d41.y * ny + d41.z * nz;
    if (!tl::math::Finite(nx) || !tl::math::Finite(ny) || !tl::math::Finite(nz) ||
        !tl::math::Finite(determinant)) return Status::NonfiniteResult;

    NativeCoatingOrientationResult result;
    result.orientation = determinant > 0 ? CoatingOrientation::Reversed : CoatingOrientation::Forward;
    result.center_triangle_determinant = determinant;
    *output = result;
    return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::startup
