#pragma once
#include "output/AcceptedReplay.h"
#include "chrono/assets/ChColor.h"
#include <algorithm>
#include <cmath>

namespace crash::visual {
struct ReplayScalarLegend {
    std::size_t native = 0, not_applicable = 0, unavailable = 0;
    double maximum = 0;
};
inline bool ValidReplayScalar(const output::ReplayParentScalar& p) noexcept {
    switch (p.applicability) {
        case output::ReplayScalarApplicability::NativeValue: return std::isfinite(p.value) && p.value >= 0;
        case output::ReplayScalarApplicability::NotApplicable:
        case output::ReplayScalarApplicability::Unavailable: return p.value == 0;
    }
    return false;
}
inline const char* ReplayApplicabilityName(output::ReplayScalarApplicability a) noexcept {
    switch (a) {
        case output::ReplayScalarApplicability::NativeValue: return "native equivalent plastic strain";
        case output::ReplayScalarApplicability::NotApplicable: return "plastic strain not applicable";
        case output::ReplayScalarApplicability::Unavailable: return "plastic strain unavailable";
    }
    return nullptr;
}
// Requires a validated non-native tag; field staging rejects all other tags.
inline chrono::ChColor ReplayMissingScalarColor(output::ReplayScalarApplicability a) noexcept {
    return a == output::ReplayScalarApplicability::NotApplicable
        ? chrono::ChColor(.48f, .50f, .52f) : chrono::ChColor(.72f, .30f, .74f);
}

// A fixed blue/yellow/red ramp shared by mesh colors and the visible legend.
// Normalized values outside the declared range saturate at its endpoints.
inline chrono::ChColor ReplayScalarColor(double normalized) {
    const chrono::ChColor blue(.12f, .64f, .94f), yellow(.98f, .84f, .16f), red(.90f, .12f, .10f);
    const double t = std::clamp(normalized, 0., 1.);
    const auto& a = t <= .5 ? blue : yellow;
    const auto& b = t <= .5 ? yellow : red;
    return chrono::ChColor::Interp(a, b, t <= .5 ? 2*t : 2*t-1);
}

} // namespace crash::visual
