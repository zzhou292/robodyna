#pragma once
#include "SourceAssemblyData.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::assembly::detail {
// Borrowed supplied values only. The caller retains the source through native
// catalog construction, which owns its final curve pool.
inline tl::fea::ShellPlasticityCurveInput NativeCurve(const Curve& curve) {
    output::Require(curve.plastic_strain.size() <= UINT32_MAX,
                    "Curve count cannot be represented by native material input");
    return {curve.id, {curve.plastic_strain.data(), curve.stress_pa.data(),
                      static_cast<std::uint32_t>(curve.plastic_strain.size())}};
}
inline tl::fea::ShellPlasticitySectionInput NativeSection(const Section& section) noexcept {
    return {section.id, section.thickness_m[0], section.through_thickness_points};
}
} // namespace crash::modelio::assembly::detail
