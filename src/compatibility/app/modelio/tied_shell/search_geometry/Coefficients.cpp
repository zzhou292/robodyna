#include "Internal.h"
#include <cmath>

namespace crash::modelio::tied_shell::search_detail {
double ConsumedThickness(const RankedCoefficient& c) {
    Require(std::isfinite(c.geometry) && c.geometry > 0 && std::isfinite(c.part) && c.part >= 0 &&
            std::isfinite(c.element) && c.element >= 0, "Invalid tied native thickness coefficient");
    return c.part != 0 ? c.part : c.element != 0 ? c.element : c.geometry;
}
double ResolveEquivalent(const std::vector<RankedCoefficient>& values, std::vector<bool>& winners) {
    Require(!values.empty() && values.size() <= 32, "Tied native shell match extent is invalid");
    std::array<bool,32> next{};
    double thickness = 0, best_geometry = -1, best_modulus = -1;
    for (const auto& c : values) {
        ConsumedThickness(c);
        Require(c.family == values.front().family, "Tied cross-family native shell ranking needs ordered admission");
        Require(values.size() == 1 || (std::isfinite(c.modulus) && c.modulus > 0),
                "Tied ambiguous shell rank modulus is unavailable");
        if (c.geometry > best_geometry || (c.geometry == best_geometry && c.modulus > best_modulus)) {
            best_geometry = c.geometry;
            best_modulus = c.modulus;
        }
    }
    bool first = true;
    for (std::size_t i = 0; i < values.size(); ++i) {
        const auto& c = values[i];
        if (c.geometry != best_geometry || c.modulus != best_modulus) continue;
        const auto current = ConsumedThickness(c);
        if (first) thickness = current;
        else Same(thickness, current);
        first = false;
        next[i] = true;
    }
    Require(!first, "Tied native rank has no equivalent winner");
    winners.assign(next.begin(), next.begin()+values.size());
    return thickness;
}
void Resolve(SearchGeometryData& d) {
    std::vector<RankedCoefficient> coefficients;
    coefficients.reserve(32);
    std::vector<bool> winners;
    winners.reserve(32);
    for (auto& master : d.masters) {
        coefficients.clear();
        for (std::size_t i = 0; i < master.match_count; ++i) {
            const auto& match = d.matches.at(master.first_match+i);
            const auto& p = d.properties.at(match.property);
            Require(master.match_count == 1 || p.rank_modulus_available,
                    "Missing tied material coefficient for ambiguous native match");
            coefficients.push_back({p.geometry_thickness, p.rank_modulus, p.part_override,
                                    p.element_override, match.family});
        }
        const auto thickness = ResolveEquivalent(coefficients, winners);
        Require(coefficients.front().family == master.family,
                "Declared tied topology differs from consumed shell family");
        master.bounds_thickness = thickness;
        master.projection_thickness = thickness;
        for (std::size_t i = 0; i < winners.size(); ++i) {
            d.matches[master.first_match+i].equivalent_winner = winners[i];
            d.equivalent_winner_occurrences += winners[i];
        }
    }
}
} // namespace crash::modelio::tied_shell::search_detail
