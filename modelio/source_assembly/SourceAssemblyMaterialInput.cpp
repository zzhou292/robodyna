#include "SourceAssemblyMaterialInput.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::assembly {
SourceAssemblyMaterialInput::SourceAssemblyMaterialInput(const SourceAssembly& source, MaterialRatePolicy policy)
    : source_(source), policy_(policy) {
    output::Require(policy == MaterialRatePolicy::OpenRadiossDirectImportDefault, "Unsupported explicit assembly material-rate policy");
    const auto& data = source_.data();
    for (const auto& curve : data.curves) {
        output::Require(curve.plastic_strain.size() <= UINT32_MAX, "Curve count cannot be represented by native material input");
        curves_.push_back({curve.id, {curve.plastic_strain.data(), curve.stress_pa.data(), static_cast<std::uint32_t>(curve.plastic_strain.size())}});
    }
    for (const auto& material : data.materials) {
        output::Require(material.source_rate_type == 0, "Assembly direct-import policy requires source VP=0");
        // Same pinned direct-import chain as SourcePartMaterial: absent Fcut
        // becomes zero in CPP_GET_FLOATV_FLOATD, then HM_READ_MAT44 with
        // ISMOOTH=1 resolves 10000/s. It is not a Radioss CFG re-read default.
        const tl::material::TabulatedShellPlasticityRate rate{true, material.rate_c_per_s, material.rate_p, 10000.};
        materials_.push_back({material.id, material.curve_id, material.young_pa, material.poisson_ratio, material.density_kg_m3, rate});
    }
    for (const auto& section : data.sections)
        sections_.push_back({section.id, section.thickness_m[0], section.through_thickness_points});
    for (const auto& parent : data.parents)
        parents_.push_back({parent.family == ShellFamily::Qeph ? tl::fea::ShellBindingFamily::Qeph : tl::fea::ShellBindingFamily::T3,
            parent.family_index, parent.source_id, parent.part_id, parent.material_id, parent.section_id});
}
tl::fea::ShellBatchPlasticityBindingInput SourceAssemblyMaterialInput::input() const noexcept {
    return {curves_.empty() ? nullptr : curves_.data(), materials_.empty() ? nullptr : materials_.data(),
        sections_.empty() ? nullptr : sections_.data(), parents_.empty() ? nullptr : parents_.data(),
        curves_.size(), materials_.size(), sections_.size(), parents_.size()};
}
}  // namespace crash::modelio::assembly
