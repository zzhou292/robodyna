#include "Internal.h"
#include "modelio/source_assembly/NativeMaterialInput.h"

namespace crash::modelio::vehicle {
struct VehicleSectionResolution::Data {
    explicit Data(const VehicleSourcePlan& value) : source(value) {}
    VehicleSourcePlan source;
    assembly::ArtifactIdentity identity;
    resolution::Declarations declarations;
    std::vector<SectionParentResolution> parents;
    std::vector<tl::fea::ShellPlasticityMaterialInput> native_materials;
    std::vector<tl::fea::ShellFailureParentInput> native_parents;
    std::size_t budget = 0;
    const assembly::Material* Material(std::size_t p) const noexcept {
        if (p >= declarations.parts.size()) return nullptr;
        const auto& part = declarations.parts[p];
        if (part.status == SectionDisposition::Existing) return source.material(p);
        if (part.status == SectionDisposition::GlassTab1) return &declarations.glass[part.material_index].material;
        return part.status == SectionDisposition::ConstantFailure ?
            &declarations.failure.materials[part.material_index] : nullptr;
    }
    const assembly::Section* Section(std::size_t p) const noexcept {
        if (p >= declarations.parts.size()) return nullptr;
        const auto& part = declarations.parts[p];
        if (part.status == SectionDisposition::Existing) return source.section(p);
        if (part.status == SectionDisposition::GlassTab1) return &declarations.glass[part.section_index].section;
        return part.status == SectionDisposition::ConstantFailure ?
            &declarations.failure.sections[part.section_index] : nullptr;
    }
    void PrepareNativeValues() {
        native_materials.resize(declarations.parts.size());
        for (std::size_t p = 0; p < declarations.parts.size(); ++p) {
            const auto* material = Material(p);
            if (!material) continue;
            if (declarations.parts[p].status == SectionDisposition::GlassTab1) {
                native_materials[p] = resolution::NativeGlassMaterial(*material);
                continue;
            }
            auto native = assembly::detail::NativeMaterial(*material);
            if (declarations.parts[p].status == SectionDisposition::ConstantFailure && material->curve_id) {
                native.continuation = tl::material::ShellPlasticityCurveContinuation::NativeLastSegment;
            }
            native_materials[p] = native;
        }
        native_parents.resize(parents.size());
        for (std::size_t e = 0; e < parents.size(); ++e) {
            const auto& parent = parents[e];
            const auto& part = declarations.parts[parent.part_index];
            if (part.status == SectionDisposition::Unresolved) continue;
            const auto& original = source.parts()[parent.part_index];
            auto& native = native_parents[e];
            native.source = {parent.topology == SourceShellTopology::Q4 ?
                                 tl::fea::ShellBindingFamily::Qeph : tl::fea::ShellBindingFamily::T3,
                             parent.topology_index, parent.source_parent_id, original.part_id,
                             original.material_id, original.section_id};
            if (part.status == SectionDisposition::ConstantFailure) {
                native.policy = tl::fea::ShellFailurePolicy::ConstantAllPoints;
                native.constant.failure_strain = part.failure_strain;
            } else if (part.status == SectionDisposition::GlassTab1) {
                native.policy = tl::fea::ShellFailurePolicy::Tab1AnyPoint;
                native.tab1.table = {{-.3,0,.3},part.failure_strain};
                native.tab1.parent_policy = tl::fea::sections::ShellTab1ParentPolicy::AnyPoint;
            }
        }
    }
};

VehicleSectionResolution VehicleSectionResolution::Read(const VehicleSourcePlan& source,
    const std::filesystem::path& path, const assembly::ArtifactIdentity& identity, ResolutionLimits limits) {
    resolution::Preflight(source, identity, limits);
    return ReadBytes(source, output::ReadBounded(path, identity.bytes), identity, limits);
}
VehicleSectionResolution VehicleSectionResolution::ReadBytes(const VehicleSourcePlan& source,
    const std::string& bytes, const assembly::ArtifactIdentity& identity, ResolutionLimits limits) {
    const auto budget = resolution::Preflight(source, identity, limits);
    output::Require(bytes.size() == identity.bytes && output::Sha256(bytes) == identity.sha256,
                    "Vehicle section resolution content authentication failed");
    output::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag | rapidjson::kParseIterativeFlag |
              rapidjson::kParseValidateEncodingFlag>(bytes.data(), bytes.size());
    output::Require(!doc.HasParseError() && doc.IsObject(), "Invalid vehicle section resolution JSON");
    resolution::UniqueKeys(doc);
    resolution::CheckAuthority(source, doc);
    auto next = std::make_shared<Data>(source);
    next->identity = identity;
    next->budget = budget;
    next->declarations = resolution::ReadDeclarations(source, doc, limits);
    next->parents = resolution::ReadParents(source);
    next->PrepareNativeValues();
    return VehicleSectionResolution(std::move(next));
}
const VehicleSourcePlan& VehicleSectionResolution::source() const noexcept { return data_->source; }
const assembly::ArtifactIdentity& VehicleSectionResolution::identity() const noexcept { return data_->identity; }
const ResolutionCounts& VehicleSectionResolution::counts() const noexcept { return data_->declarations.counts; }
bool VehicleSectionResolution::includes_glass() const noexcept { return data_->declarations.includes_glass; }
const std::vector<SectionPartResolution>& VehicleSectionResolution::parts() const noexcept { return data_->declarations.parts; }
const std::vector<SectionParentResolution>& VehicleSectionResolution::parents() const noexcept { return data_->parents; }
const assembly::Material* VehicleSectionResolution::material(std::size_t part) const noexcept { return data_->Material(part); }
const assembly::Section* VehicleSectionResolution::section(std::size_t part) const noexcept { return data_->Section(part); }
const tl::fea::ShellPlasticityMaterialInput* VehicleSectionResolution::native_material(std::size_t part) const noexcept {
    return data_->Material(part) ? &data_->native_materials[part] : nullptr;
}
const tl::fea::ShellFailureParentInput* VehicleSectionResolution::native_parent(std::size_t parent) const noexcept {
    return parent < data_->parents.size() && data_->Material(data_->parents[parent].part_index) ?
        &data_->native_parents[parent] : nullptr;
}
const std::vector<assembly::Curve>& VehicleSectionResolution::failure_curves() const noexcept {
    return data_->declarations.failure.curves;
}
std::size_t VehicleSectionResolution::startup_budget_bytes() const noexcept { return data_->budget; }
} // namespace crash::modelio::vehicle
