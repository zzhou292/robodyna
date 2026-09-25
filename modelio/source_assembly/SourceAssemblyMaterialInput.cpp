#include "SourceAssemblyMaterialInput.h"
#include "NativeMaterialInput.h"
#include "SourceAssemblyShellInput.h"
#include "NativeDeclarationViews.h"
#include "output/ArtifactIO.h"

namespace crash::modelio::assembly {
SourceAssemblyMaterialInput::SourceAssemblyMaterialInput(const SourceAssembly& source, MaterialRatePolicy policy)
    : source_(source), policy_(policy), execution_(Law1ExecutionPolicy::Resolve(
        Law1ExecutionProfile::LegacyLayered, source.data().units,
        QephReferenceMetric::Resolve(QephMetricProfile::LegacyOneMetre, source.data().units))) {
    Pack();
}
SourceAssemblyMaterialInput::SourceAssemblyMaterialInput(const SourceAssemblyShellInput& shells,
    MaterialRatePolicy policy, Law1ExecutionProfile profile)
    : source_(shells.source()), policy_(policy), execution_(Law1ExecutionPolicy::Resolve(
        profile, source_.data().units, shells.qeph_metric())) {
    Pack();
}
void SourceAssemblyMaterialInput::Pack() {
    output::Require(policy_ == MaterialRatePolicy::OpenRadiossDirectImportDefault,
                    "Unsupported explicit assembly material-rate policy");
    const auto& data = source_.data();
    for (const auto& curve : data.curves) curves_.push_back(detail::NativeCurve(curve));
    for (const auto& material : data.materials) materials_.push_back(detail::NativeMaterial(material));
    for (const auto& section : data.sections) sections_.push_back(detail::NativeSection(section));
    for (const auto& parent : data.parents) {
        tl::fea::ShellPlasticityParentInput row{parent.family == ShellFamily::Qeph ?
            tl::fea::ShellBindingFamily::Qeph : tl::fea::ShellBindingFamily::T3,
            parent.family_index, parent.source_id, parent.part_id, parent.material_id, parent.section_id};
        const auto& material = data.materials.at(parent.material_index);
        const auto& section = data.sections.at(parent.section_index);
        row.execution = execution_.Parent(ResolveLaw1SourceDriver(material, section), row.family,
            detail::NativeSection(section).formulation);
        parents_.push_back(row);
    }
}
tl::fea::ShellBatchPlasticityBindingInput SourceAssemblyMaterialInput::input() const noexcept {
    return {curves_.empty() ? nullptr : curves_.data(), materials_.empty() ? nullptr : materials_.data(),
        sections_.empty() ? nullptr : sections_.data(), parents_.empty() ? nullptr : parents_.data(),
        curves_.size(), materials_.size(), sections_.size(), parents_.size()};
}
}  // namespace crash::modelio::assembly
