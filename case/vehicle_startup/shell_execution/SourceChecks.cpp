#include "Internal.h"

namespace crash::cases::vehicle_startup::shell_execution::detail {
namespace {
ReferenceFamily ReferenceFamilyOf(fe::ShellBindingFamily family) {
    switch (family) {
    case fe::ShellBindingFamily::Qeph: return ReferenceFamily::Qeph;
    case fe::ShellBindingFamily::T3: return ReferenceFamily::T3;
    case fe::ShellBindingFamily::Qbat: return ReferenceFamily::Qbat;
    default: return ReferenceFamily::None;
    }
}
std::uint64_t SourceId(const fe::ShellBatchBinding& binding, const source::NativeParentMapping& row) {
    switch (row.family) {
    case fe::ShellBindingFamily::Qeph: return binding.qeph_source_id(row.family_index);
    case fe::ShellBindingFamily::T3: return binding.t3_source_id(row.family_index);
    case fe::ShellBindingFamily::Qbat: return binding.qbat_source_id(row.family_index);
    default: return 0;
    }
}
}
const VehicleSectionResolution& CheckOriginalReferencePrefix(const VehicleShellReferences& references,
    const fe::ShellBatchBinding& binding) {
    const auto* resolution = references.resolution();
    Require(resolution && resolution->resolution_key().profile == source::ResolutionProfile::OriginalRigidPartsV1 &&
        resolution->includes_glass() && resolution->rigid_source(), "Complete original shell execution requires retained rigid/midlayer resolution");
    const auto& counts = resolution->counts();
    Require(counts.parts == 867 && counts.shells == 349645 && counts.unresolved_shells == 0 &&
        counts.rigid_parts == 22 && counts.rigid_shells == 5102 && counts.midlayer_shells == 4251 &&
        resolution->parents().size() == counts.shells && resolution->parts().size() == counts.parts &&
        resolution->source().parts().size() == counts.parts && references.rows().size() == counts.shells &&
        references.counts().succeeded == counts.shells && references.counts().rejected == 0 &&
        references.counts().unresolved == 0 && binding.qeph_count() >= 324094 &&
        binding.t3_count() >= 21301 && binding.qbat_count() >= 4250,
        "Complete original shell execution coverage changed");
    Require(&resolution->source().canonical().data() == &references.source().canonical().data(),
        "Execution resolution and original references do not share canonical authority");
    source::NativeFormulationCounts seen;
    for (std::size_t i = 0; i < counts.shells; ++i) {
        const auto& parent = resolution->parents()[i];
        Require(parent.part_index < counts.parts, "Execution parent part index exceeds source scope");
        const auto& part = resolution->source().parts()[parent.part_index];
        const auto* mapping = resolution->native_mapping(i);
        const auto* failure = resolution->native_parent(i);
        const auto& reference = references.rows()[i];
        Require(mapping && failure && mapping->family != fe::ShellBindingFamily::None &&
            reference.status == ReferenceStatus::Success && reference.family == ReferenceFamilyOf(mapping->family) &&
            reference.reference_index == mapping->family_index && reference.element_id == parent.source_parent_id &&
            reference.part_index == parent.part_index && reference.canonical_parent == parent.canonical_parent &&
            reference.part_id == part.part_id && reference.material_id == part.material_id &&
            reference.section_id == part.section_id && reference.role == resolution->role(parent.part_index) &&
            reference.rigid_root_index == resolution->rigid_root_index(parent.part_index),
            "Execution parent differs from original reference identity/role");
        const fe::ShellPlasticityParentInput expected{mapping->family, mapping->family_index,
            parent.source_parent_id, part.part_id, part.material_id, part.section_id};
        Require(Same(failure->source, expected), "Execution failure source differs from resolved native mapping");
        std::size_t* next = nullptr;
        switch (mapping->family) {
        case fe::ShellBindingFamily::Qeph: next = &seen.qeph; break;
        case fe::ShellBindingFamily::T3: next = &seen.t3; break;
        case fe::ShellBindingFamily::Qbat: next = &seen.qbat; break;
        default: Require(false, "Unknown complete execution formulation");
        }
        Require(mapping->family_index == (*next)++, "Execution native family index is not dense source order");
        Require(SourceId(binding, *mapping) == parent.source_parent_id,
                "Native binding family index names another original source EID");
    }
    const auto& expected = resolution->native_counts();
    Require(seen.qeph == expected.qeph && seen.t3 == expected.t3 && seen.qbat == expected.qbat,
        "Complete execution formulation totals differ from native binding");
    return *resolution;
}
const VehicleSectionResolution& CheckSource(const physical_model::VehiclePhysicalModel& model) {
    const auto& references = model.shell_source().references();
    const auto& binding = model.shell_source().shells();
    Require(binding.qeph_count() == 324094 && binding.t3_count() == 21301 && binding.qbat_count() == 4250,
        "Complete original shell execution coverage changed");
    const auto& resolved = CheckOriginalReferencePrefix(references, binding);
    const auto* resolution = &resolved;
    Require(&resolution->source().canonical().data() == &references.source().canonical().data() &&
        &references.source().canonical().data() == &model.source_domain().source().tied_source().canonical().data() &&
        model.coefficients().shells() && model.coefficients().shells()->Matches(binding, model.source_domain().domain()) &&
        model.rigid_assembly().coefficients()->Matches(model.coefficients()),
        "Execution source, native reference, domain and actual rigid ledger authorities differ");
    return resolved;
}
Law1ExecutionPolicy ResolvePolicy(const physical_model::VehiclePhysicalModel& model, Law1ExecutionProfile profile) {
    return ResolvePolicy(model.shell_source().references(), profile);
}
Law1ExecutionPolicy ResolvePolicy(const VehicleShellReferences& references, Law1ExecutionProfile profile) {
    const auto& units = references.source().canonical().data().inputs.units;
    const auto policy = Law1ExecutionPolicy::Resolve(profile,
        {units.mass_to_kg, units.length_to_m, units.time_to_s}, references.qeph_metric());
    if (policy.requires_ordinary_explicit_defaults()) {
        const auto* resolution = references.resolution();
        Require(resolution, "Native LAW1 requires resolved original sections");
        for (std::size_t i = 0; i < resolution->parents().size(); ++i) {
            const auto part = resolution->parents()[i].part_index;
            const auto* material = resolution->material(part);
            const auto* section = resolution->section(part);
            const auto* row = resolution->native_parent(i);
            Require(material && section && row, "Missing resolved LAW1 source row");
            const auto driver = material->source.keyword == "*MAT_ELASTIC" ? resolution->source().law1_driver(part) :
                modelio::assembly::ResolveLaw1SourceDriver(*material, *section);
            Require(driver.material_id() == row->source.material_id && driver.section_id() == row->source.section_id,
                    "Native LAW1 driver identity differs before allocation");
            (void)policy.Parent(driver, row->source.family, resolution->section_formulation(part));
        }
        for (std::size_t i = 0; i < references.rows().size(); ++i) {
            if (const auto* quad = references.qeph(i)) {
                Require(output::Bits(quad->input.projection_working_length_m) == output::Bits(units.length_to_m),
                        "Native LAW1 source is bound to another QEPH working metric");
            }
        }
    }
    return policy;
}

void PackSource(const physical_model::VehiclePhysicalModel& model, Packing& packed) {
    PackSource(model, packed, ResolvePolicy(model, Law1ExecutionProfile::LegacyLayered));
}
void PackSource(const physical_model::VehiclePhysicalModel& model, Packing& packed, const Law1ExecutionPolicy& policy) {
    PackOriginalReferenceSource(model.shell_source().references(), packed, policy);
}
void PackOriginalReferenceSource(const VehicleShellReferences& references, Packing& packed,
    const Law1ExecutionPolicy& policy) {
    const auto& resolution = *references.resolution();
    for (std::size_t part = 0; part < resolution.parts().size(); ++part) {
        const auto* material = resolution.material(part);
        const auto* section = resolution.section(part);
        const auto* native = resolution.native_material(part);
        Require(material && section && native, "Complete execution has an unavailable source declaration");
        AddPart(packed, resolution.source().parts()[part], *material, *section, *native,
                resolution.role(part), resolution.section_formulation(part));
    }
    PackCurves(packed, resolution.source().curves(), resolution.failure_curves());
    for (std::size_t i = 0; i < resolution.parents().size(); ++i) {
        auto row = *resolution.native_parent(i);
        const auto part = resolution.parents()[i].part_index;
        const auto& material = *resolution.material(part);
        const auto& section = *resolution.section(part);
        const auto driver = material.source.keyword == "*MAT_ELASTIC" ? resolution.source().law1_driver(part) :
            modelio::assembly::ResolveLaw1SourceDriver(material, section);
        if (policy.requires_ordinary_explicit_defaults()) {
            Require(driver.material_id() == row.source.material_id && driver.section_id() == row.source.section_id,
                    "Resolved LAW1 driver belongs to another source MID/SID");
        }
        row.source.execution = policy.Parent(driver, row.source.family, resolution.section_formulation(part));
        packed.parents.push_back(row.source);
        packed.failure.push_back(row);
    }
}
} // namespace crash::cases::vehicle_startup::shell_execution::detail
