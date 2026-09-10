#include "SourceAssemblyMaterialInput.h"
#include "NativeMaterialInput.h"
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
    for (const auto& material : data.materials)
        materials_.push_back(detail::NativeMaterial(material));
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
