#include "Internal.h"

namespace crash::cases::vehicle_wall::native::execution_detail {
const vehicle_startup::VehicleSectionResolution& Check(const EnvelopePhysicalSource& source) {
    const auto& refs = source.vehicle_references();
    const auto& native = source.shells();
    const auto& domain = source.domain();
    const auto& e = source.environment_parent();
    const auto& wall = source.wall();
    const auto& resolution = original::detail::CheckOriginalReferencePrefix(refs, native);
    Require(native.qeph_count() == refs.counts().qeph_succeeded+1 &&
        native.t3_count() == refs.counts().t3_succeeded && native.qbat_count() == refs.counts().qbat_succeeded &&
        e.qeph_index == refs.counts().qeph_succeeded && e.catalog_append_ordinal == refs.rows().size(),
        "Combined execution has another environment/family scope");
    Require(source.coefficients().domain()->SharesStorage(domain) && source.coefficients().shells() &&
        source.coefficients().shells()->Matches(native, domain) &&
        source.rigid_assembly().coefficients()->Matches(source.coefficients()),
        "Combined execution must retain its actual complete ledger and rigid graph");
    Require(e.element_id == wall.ids().shell && e.part_id == wall.ids().part &&
        e.material_id == wall.ids().material && e.section_id == wall.ids().section &&
        native.qeph_source_id(e.qeph_index) == e.element_id,
        "Environment execution identity differs from its declared source");
    const auto& reference = native.qeph_reference(e.qeph_index).input;
    const auto& material = wall.declaration().material;
    Require(output::Bits(reference.young_modulus) == output::Bits(material.young_pa) &&
        output::Bits(reference.poisson_ratio) == output::Bits(material.poisson) &&
        output::Bits(reference.density) == output::Bits(material.density_kg_m3) &&
        output::Bits(reference.thickness) == output::Bits(material.thickness_m) &&
        reference.placement == fe::ShellReferencePlacement::Centered &&
        output::Bits(reference.projection_working_length_m) == output::Bits(refs.qeph_metric().working_length_m()),
        "Environment execution material, placement or working length differs from its real reference");
    for (unsigned k=0; k<4; ++k) {
        Require(e.domain_nodes[k] == wall.vehicle_prefix().nodes+k &&
            native.qeph_nodes(e.qeph_index)[k] == e.shell_nodes[k] &&
            native.active_nodes()[e.shell_nodes[k]].source_id == wall.ids().nodes[k] &&
            domain.nodes()[e.domain_nodes[k]].source_id == wall.ids().nodes[k],
            "Environment execution uses a foreign node/source map");
    }
    return resolution;
}
void AppendWall(original::detail::Packing& packed, const EnvelopePhysicalSource& source,
    const modelio::assembly::Law1ExecutionPolicy& policy) {
    AppendDeclaredWall(packed,source.environment_parent(),source.wall().declaration().material,
        source.wall().geometry().native_working_length_m,policy);
}
void AppendDeclaredWall(original::detail::Packing& packed,const EnvironmentParent& e,const MaterialDeclaration& m,
    double working_length_m,const modelio::assembly::Law1ExecutionPolicy& policy) {
    Require(policy.profile() == modelio::assembly::Law1ExecutionProfile::NativeA62OrdinaryNpt0 &&
        output::Bits(policy.coefficient_working_length_m()) == output::Bits(working_length_m),
        "Declared native elastic wall requires the authenticated ordinary LAW1 execution policy");
    for (const auto& row : packed.materials) Require(row.material_id != e.material_id, "Environment MID collides with resolved source");
    for (const auto& row : packed.sections) Require(row.section_id != e.section_id, "Environment SID collides with resolved source");
    for (const auto& row : packed.parents)
        Require(row.source_parent_id != e.element_id && row.source_part_id != e.part_id,
            "Environment EID/PID collides with an original physical parent");
    fe::ShellPlasticityMaterialInput material;
    material.material_id=e.material_id; material.law=fe::ShellSectionLaw::LayeredLaw1Nip3;
    material.young_pa=m.young_pa; material.poisson_ratio=m.poisson; material.density_kg_m3=m.density_kg_m3;
    fe::ShellPlasticitySectionInput section;
    section.section_id=e.section_id; section.thickness_m=m.thickness_m;
    section.through_thickness_points=3; section.formulation=fe::ShellSectionFormulation::LayeredNip3;
    fe::ShellPlasticityParentInput parent{fe::ShellBindingFamily::Qeph,e.qeph_index,e.element_id,e.part_id,e.material_id,e.section_id};
    // This is the new native case declaration. No invented original MAT/SECTION
    // cards or SourceLaw1Driver are used to obtain the already-qualified policy.
    parent.execution.policy=fe::ShellParentExecutionPolicy::GlobalLaw1Npt0;
    parent.execution.global_law1={fe::ShellLaw1Thickness::Accepted,policy.coefficient_working_length_m()};
    fe::ShellFailureParentInput failure;
    failure.source=parent; failure.policy=fe::ShellFailurePolicy::None;
    packed.materials.push_back(material); packed.sections.push_back(section);
    packed.parents.push_back(parent); packed.failure.push_back(failure);
}
}
