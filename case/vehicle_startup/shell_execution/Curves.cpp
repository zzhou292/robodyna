#include "Packing.h"
#include "modelio/source_assembly/NativeDeclarationViews.h"
#include <algorithm>

namespace crash::cases::vehicle_startup::shell_execution::detail {
namespace {
using output::Require;
void Check(const modelio::assembly::Curve& curve) {
    Require(curve.id && curve.plastic_strain.size() == curve.stress_pa.size() &&
        curve.plastic_strain.size() >= 2 && curve.plastic_strain.size() <= fe::MaxShellPlasticityCurvePoints,
        "Supplied hardening curve has an invalid complete point range");
}
bool SameCurve(const modelio::assembly::Curve& a, const modelio::assembly::Curve& b) {
    if (a.plastic_strain.size() != b.plastic_strain.size()) return false;
    for (std::size_t i = 0; i < a.plastic_strain.size(); ++i) {
        if (output::Bits(a.plastic_strain[i]) != output::Bits(b.plastic_strain[i]) ||
            output::Bits(a.stress_pa[i]) != output::Bits(b.stress_pa[i])) return false;
    }
    return true;
}
}
void PackCurves(Packing& out, const std::vector<modelio::assembly::Curve>& original,
                const std::vector<modelio::assembly::Curve>& failure) {
    Require(out.curves.empty() && original.size() <= fe::MaxPlasticityCatalogDefinitions &&
        failure.size() <= fe::MaxPlasticityCatalogDefinitions, "Hardening curve source exceeds definition scope");
    std::size_t points = 0;
    for (const auto& material : out.materials) {
        const auto id = material.curve_id;
        if (!id) continue;
        if (std::any_of(out.curves.begin(), out.curves.end(), [&](const auto& curve) { return curve.curve_id == id; })) continue;
        const modelio::assembly::Curve* selected = nullptr;
        for (const auto* collection : {&original, &failure}) {
            for (const auto& curve : *collection) {
                if (curve.id != id) continue;
                Check(curve);
                Require(!selected || SameCurve(*selected, curve), "One supplied curve ID has conflicting point values");
                selected = &curve;
            }
        }
        Require(selected, "Resolved shell material references an absent supplied curve");
        Require(selected->plastic_strain.size() <= fe::MaxShellPlasticityCurvePoints - points,
                "Complete referenced shell hardening pool exceeds native capacity");
        points += selected->plastic_strain.size();
        out.curves.push_back(modelio::assembly::detail::NativeCurve(*selected));
    }
}
} // namespace crash::cases::vehicle_startup::shell_execution::detail
