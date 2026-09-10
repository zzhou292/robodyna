#include "ReplayPartColors.h"
#include <map>

namespace crash::visual {
chrono::ChColor ReplayPartColor(std::uint64_t part_id) noexcept {
    // Versioned seed advances the fixed Weyl offset; unsigned wrap is defined.
    std::uint64_t bits = part_id + (ReplayPartPaletteSeed+1)*UINT64_C(0x9e3779b97f4a7c15);
    bits = (bits ^ (bits >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    bits = (bits ^ (bits >> 27)) * UINT64_C(0x94d049bb133111eb);
    bits ^= bits >> 31;
    const double hue = double(bits >> 40) * (6. / 16777216.);
    const unsigned sector = static_cast<unsigned>(hue);
    const double fraction = hue - sector;
    const double saturation = .60 + .24 * double((bits >> 24) & 255) / 255.;
    const double value = .78 + .18 * double((bits >> 32) & 255) / 255.;
    const float high = static_cast<float>(value), low = static_cast<float>(value * (1-saturation));
    const float down = static_cast<float>(value * (1-saturation*fraction));
    const float up = static_cast<float>(value * (1-saturation*(1-fraction)));
    switch (sector) {
        case 0: return {high,up,low};
        case 1: return {down,high,low};
        case 2: return {low,high,up};
        case 3: return {low,down,high};
        case 4: return {up,low,high};
        default: return {high,low,down};
    }
}
bool ReplayPartColors::Initialize(const std::vector<std::uint64_t>& parts,
                                  std::vector<chrono::ChColor>& colors) {
    if (parts.empty() || parts.size() > ReplayPartColorTriangleLimit) return false;
    for (auto part : parts) if (!part) return false;
    std::map<std::uint64_t,chrono::ChColor> unique;
    std::vector<chrono::ChColor> next_colors;
    next_colors.reserve(parts.size());
    for (auto part : parts) {
        const auto color = ReplayPartColor(part);
        next_colors.push_back(color);
        unique.emplace(part,color);
    }
    std::vector<ReplayPartLegendEntry> next_legend;
    next_legend.reserve(unique.size());
    for (const auto& item : unique) next_legend.push_back({item.first,item.second});
    colors.swap(next_colors);
    legend_.swap(next_legend);
    return true;
}
} // namespace crash::visual
