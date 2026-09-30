#include "FrameGeometryData.h"
#include <cmath>

namespace crash::visual::full_shell {
std::size_t FrameGeometryBudget(std::size_t n, std::size_t p, std::size_t t, const FrameGeometryOptions& o) {
    output::Require(o.geometry.valid() && n && p && t && n <= o.geometry.vertices &&
        p <= o.geometry.parents && t <= o.geometry.triangles && o.host_bytes &&
        o.host_bytes <= 512 * 1024 * 1024 && ReplayColorModeName(o.colors) &&
        std::isfinite(o.plastic_strain_maximum) && o.plastic_strain_maximum >= 0,
        "Invalid full-shell presentation counts/options");
    // Charge mesh/staging, integer/color/source-index arrays, parent fields and
    // transient decode/context/index construction. Shared input backing is not
    // reallocated here. Bounds deliberately include conservative working copies.
    std::size_t bytes = 128 * 1024;
    for (const auto term : {std::pair<std::size_t, std::size_t>{n, 128}, {t, 192}, {p, 384}}) {
        output::Require(bytes <= o.host_bytes && term.first <= (o.host_bytes - bytes) / term.second,
            "Full-shell presentation byte capacity exceeded");
        bytes += term.first * term.second;
    }
    if (o.parent_activity) {
        // Original topology/index plus complete-capacity visible staging. The
        // borrowed immutable ActivityRecord remains the caller's reservation.
        constexpr std::size_t width = 64;
        output::Require(bytes <= o.host_bytes && t <= (o.host_bytes - bytes) / width,
            "Full-shell activity topology exceeds host byte capacity");
        bytes += t * width;
    }
    return bytes;
}
namespace detail {
void CheckContext(const output::full_shell::source::PreparedSourceMapping& mapping,
        const output::full_shell::Context& context) {
    output::Require(context.nodes() == mapping.nodes() && context.parents().size() == mapping.parents().size(),
        "Frame context counts differ from source mapping");
    const auto expected = mapping.MakeFrameContext(context.identity(), context.fixed_dt(), context.limits());
    output::Require(output::full_shell::SameIdentity(expected.identity(), context.identity()) &&
        expected.point_layout_sha256() == context.point_layout_sha256() &&
        expected.points() == context.points() && expected.point_offsets() == context.point_offsets(),
        "Frame context source/native point identity differs from mapping");
}
} // namespace detail
} // namespace crash::visual::full_shell
