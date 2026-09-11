#include "ResolutionData.h"
#include <algorithm>

namespace crash::modelio::vehicle {
namespace {
constexpr std::uint64_t MidlayerId = 2000524;
std::size_t SourcePart(const VehicleSectionResolution& base) {
    const auto& parts = base.source().parts();
    const auto found = std::find_if(parts.begin(),parts.end(),[](const auto& p) { return p.part_id == MidlayerId; });
    output::Require(found != parts.end(), "Original midlayer is missing");
    return std::size_t(found - parts.begin());
}
void CheckProfile(const VehicleSectionResolution& base, ResolutionProfile profile) {
    output::Require(profile == ResolutionProfile::OriginalMidlayerV1 &&
                    base.resolution_key().profile == ResolutionProfile::Artifact && base.includes_glass(),
                    "Original midlayer requires the unchanged V2 glass resolution");
    const auto& source = base.source();
    const auto& data = source.canonical().data();
    const auto& units = data.inputs.units;
    output::Require(units.mass == "t" && units.length == "mm" && units.time == "s" &&
                    units.mass_to_kg == 1000 && units.length_to_m == .001 && units.time_to_s == 1,
                    "Original midlayer source units changed");
    output::Require(data.inputs.source_member.sha256 ==
                    "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301" &&
                    data.inputs.source_member.bytes == 42846753 &&
                    data.inputs.canonical_manifest.sha256 ==
                    "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8" &&
                    data.inputs.tire_policy == "omit_original_tire_shells" &&
                    source.counts().parents == 349645 && source.parts().size() == 867,
                    "Original midlayer source/selection authority changed");
    constexpr std::uint64_t excluded[]{2000211,2000360,2000363,2000367,2000487,2000488,2000489,2000490};
    output::Require(data.excluded_parts.size() == std::size(excluded) &&
                    std::equal(data.excluded_parts.begin(),data.excluded_parts.end(),std::begin(excluded)),
                    "Original midlayer tire exclusions changed");
    const auto p = SourcePart(base);
    const auto& original = source.parts()[p];
    output::Require(base.parts()[p].status == SectionDisposition::Unresolved &&
                    original.material_id == MidlayerId && original.section_id == MidlayerId &&
                    original.shell_count == 4251, "Original midlayer association changed");
    constexpr const char* hashes[]{
        "2351979967841c0ea8f8e8818b86c3424d8c563fe09145325aa58420d64654fa",
        "240754beb432b884f7d28c2136a5cbe88a4c7f5ca090937dac072499f5dae78d",
        "a330ee9bc5cc7ded29025bebd993ab69b12ebe783ea30474e5df2cd426ee7f90"};
    for (unsigned i = 0; i < 3; ++i) {
        const auto& block = original.unresolved_sources[i];
        output::Require(block.sha256 == hashes[i] && output::Sha256(block.raw_text) == hashes[i],
                        "Original midlayer raw source changed");
    }
    const auto& counts = base.counts();
    output::Require(counts.shells == 349645 && counts.unresolved_shells == 9353 &&
                    counts.existing_shells + counts.failure_shells + counts.glass_shells == 340292 &&
                    counts.unresolved_parts > 0, "Original V2 resolution coverage changed");
}
}
std::size_t VehicleSectionResolution::ForecastOriginalMidlayer(const VehicleSectionResolution& base,
    ResolutionProfile profile, ResolutionLimits limits) {
    // Existing preflight validates every configurable cap before source traversal.
    resolution::Preflight(base.source(),base.identity(),limits);
    CheckProfile(base,profile);
    std::size_t total = 0;
    const auto add = [&](std::size_t count, std::size_t width) {
        output::Require(count <= (limits.host_bytes-total)/width, "Midlayer overlay exceeds host cap");
        total += count*width;
    };
    add(base.startup_budget_bytes(),1);
    add(sizeof(Data) + 2*sizeof(void*),1);
    add(base.parts().size(),sizeof(SectionPartResolution));
    add(base.parents().size(),sizeof(NativeParentMapping) + sizeof(tl::fea::ShellFailureParentInput));
    add(1,sizeof(resolution::MidlayerDeclaration) + sizeof(tl::fea::ShellPlasticityMaterialInput));
    // Raw strings/cards, temporary line parsing, hashing and one native material
    // preparation; the source block cap is independent of the large source key.
    add(32768,1);
    for (const auto& block : base.source().parts()[SourcePart(base)].unresolved_sources) {
        output::Require(block.raw_text.size() <= 64*1024, "Midlayer source block cap exceeded");
        add(block.raw_text.size()+1,8);
    }
    return total;
}
VehicleSectionResolution VehicleSectionResolution::ResolveOriginalMidlayer(const VehicleSectionResolution& base,
    ResolutionProfile profile, ResolutionLimits limits) {
    const auto budget = ForecastOriginalMidlayer(base,profile,limits);
    auto next = std::make_shared<Data>(base.source());
    next->base = base.data_;
    next->key = {base.identity(),profile};
    next->budget = budget;
    next->midlayer_part = SourcePart(base);
    const auto& units = base.source().canonical().data().inputs.units;
    next->midlayer.emplace(resolution::ReadMidlayer(base.source().parts()[next->midlayer_part],
        {units.mass_to_kg,units.length_to_m,units.time_to_s}));
    next->declarations.parts = base.parts();
    next->declarations.counts = base.counts();
    next->declarations.includes_glass = true;
    auto& part = next->declarations.parts[next->midlayer_part];
    part.status = SectionDisposition::Midlayer;
    part.failure_strain = next->midlayer->failure_strain;
    auto& counts = next->declarations.counts;
    ++counts.midlayer_parts;
    counts.midlayer_shells = 4251;
    --counts.unresolved_parts;
    counts.unresolved_shells -= counts.midlayer_shells;
    next->native_materials.push_back(assembly::detail::NativeMaterial(next->midlayer->material,
                                      assembly::detail::NativeLaw44Rate::FilteredZeroC));
    auto mapped = resolution::MapNativeParents(base.parents(),next->declarations.parts,base.source().parts());
    output::Require(mapped.counts.qbat == 4250 && mapped.counts.qeph + mapped.counts.t3 +
                    mapped.counts.qbat == 344543, "Midlayer native formulation coverage changed");
    next->mapping = std::move(mapped.mapping);
    next->native_parents = std::move(mapped.parents);
    next->native_counts = mapped.counts;
    return VehicleSectionResolution(std::move(next));
}
} // namespace crash::modelio::vehicle
