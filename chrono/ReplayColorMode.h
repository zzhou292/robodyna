#pragma once
#include <string_view>

namespace crash::visual {
// Automatic preserves the original plastic-strain/uniform presentation.
enum class ReplayColorMode { Automatic, Uniform, PartId, PlasticStrain };
constexpr const char* ReplayColorModeName(ReplayColorMode mode) noexcept {
    switch (mode) {
        case ReplayColorMode::Automatic: return "auto";
        case ReplayColorMode::Uniform: return "uniform";
        case ReplayColorMode::PartId: return "part-id";
        case ReplayColorMode::PlasticStrain: return "plastic-strain";
    }
    return nullptr;
}
inline bool ParseReplayColorMode(std::string_view text, ReplayColorMode& mode) noexcept {
    if (text == "auto") mode = ReplayColorMode::Automatic;
    else if (text == "uniform") mode = ReplayColorMode::Uniform;
    else if (text == "part-id") mode = ReplayColorMode::PartId;
    else if (text == "plastic-strain") mode = ReplayColorMode::PlasticStrain;
    else return false;
    return true;
}
} // namespace crash::visual
