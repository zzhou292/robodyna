#pragma once

#include <string_view>

namespace crash::visual {
// Fixed presentation choices for source-wall replay. IncidentSide retains the
// original camera. WallSide views the same geometry from the opposite X side.
enum class ReplayView { IncidentSide, WallSide, ExplicitFixed };

constexpr const char* ReplayViewName(ReplayView view) noexcept {
    switch (view) {
        case ReplayView::IncidentSide: return "incident-side";
        case ReplayView::WallSide: return "wall-side";
        case ReplayView::ExplicitFixed: return "explicit-fixed";
    }
    return nullptr;
}

// Strict option spelling; failure leaves the caller's selection unchanged.
inline bool ParseReplayView(std::string_view text, ReplayView& view) noexcept {
    if (text == "incident-side") view = ReplayView::IncidentSide;
    else if (text == "wall-side") view = ReplayView::WallSide;
    else return false;
    return true;
}
}  // namespace crash::visual
