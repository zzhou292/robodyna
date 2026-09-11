#include "ResolutionData.h"
#include "modelio/source_assembly/NativeMaterialInput.h"

namespace crash::modelio::vehicle {

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
    next->key.artifact = identity;
    next->budget = budget;
    next->declarations = resolution::ReadDeclarations(source, doc, limits);
    next->parents = resolution::ReadParents(source);
    next->PrepareNativeValues();
    return VehicleSectionResolution(std::move(next));
}
const VehicleSourcePlan& VehicleSectionResolution::source() const noexcept { return data_->source; }
const assembly::ArtifactIdentity& VehicleSectionResolution::identity() const noexcept { return data_->key.artifact; }
const ResolutionCounts& VehicleSectionResolution::counts() const noexcept { return data_->declarations.counts; }
bool VehicleSectionResolution::includes_glass() const noexcept { return data_->declarations.includes_glass; }
const std::vector<SectionPartResolution>& VehicleSectionResolution::parts() const noexcept { return data_->declarations.parts; }
const std::vector<SectionParentResolution>& VehicleSectionResolution::parents() const noexcept { return data_->Parents(); }
const assembly::Material* VehicleSectionResolution::material(std::size_t part) const noexcept { return data_->Material(part); }
const assembly::Section* VehicleSectionResolution::section(std::size_t part) const noexcept { return data_->Section(part); }
const tl::fea::ShellPlasticityMaterialInput* VehicleSectionResolution::native_material(std::size_t part) const noexcept {
    if (!data_->Material(part)) return nullptr;
    if (data_->base && part != data_->midlayer_part) return &data_->base->native_materials[part];
    return &data_->native_materials[data_->base ? 0 : part];
}
const tl::fea::ShellFailureParentInput* VehicleSectionResolution::native_parent(std::size_t parent) const noexcept {
    return parent < data_->Parents().size() && data_->Material(data_->Parents()[parent].part_index) ?
        &data_->native_parents[parent] : nullptr;
}
const std::vector<assembly::Curve>& VehicleSectionResolution::failure_curves() const noexcept {
    return data_->base ? data_->base->declarations.failure.curves : data_->declarations.failure.curves;
}
const ResolutionKey& VehicleSectionResolution::resolution_key() const noexcept { return data_->key; }
const NativeFormulationCounts& VehicleSectionResolution::native_counts() const noexcept { return data_->native_counts; }
const NativeParentMapping* VehicleSectionResolution::native_mapping(std::size_t parent) const noexcept {
    return parent < data_->mapping.size() ? &data_->mapping[parent] : nullptr;
}
tl::fea::ShellSectionFormulation VehicleSectionResolution::section_formulation(std::size_t part) const noexcept {
    return data_->base && part == data_->midlayer_part ? tl::fea::ShellSectionFormulation::OneThicknessPoint :
                                                      tl::fea::ShellSectionFormulation::LayeredNip3;
}
std::size_t VehicleSectionResolution::startup_budget_bytes() const noexcept { return data_->budget; }
} // namespace crash::modelio::vehicle
