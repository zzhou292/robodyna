#include "Internal.h"
#include "lib_src/math/ScalarBits.h"
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
Shape Check(const EnvelopeOwnerSource& owner,const VehicleSource& vehicle,Declaration declaration,Limits limits) {
    const auto& model=owner.execution_source().mechanical();
    const auto& wall=model.wall();
    const auto& embedding=model.embedding();
    const auto& original=vehicle.mixed().initial().context().pre_correction().physical();
    const auto& canonical=original.shell_source().references().source().canonical().data();
    const auto& physical=owner.physical();
    const Limits hard;
    if(!limits.nodes || limits.nodes>hard.nodes || !limits.shells || limits.shells>hard.shells ||
        !limits.own_bytes || limits.own_bytes>hard.own_bytes || !limits.coexistence_bytes ||
        limits.coexistence_bytes>hard.coexistence_bytes || !limits.metadata_bytes || limits.metadata_bytes>hard.metadata_bytes)
        Reject(Status::ResourceLimit,"Invalid finite-wall source limits");
    if(declaration.profile!=Profile::AllRetainedVehicleNodesToFixedMeshV1 ||
        !declaration.topology_generation || !declaration.source_generation)
        Reject(Status::UnsupportedProfile,"Explicit wall interface profile/generation is required");
    if(&canonical!=&model.vehicle_references().source().canonical().data() ||
        !embedding.original().Matches(original.source_domain().domain()) ||
        !physical.prepared() || !physical.domain()->SharesStorage(embedding.domain()) ||
        !physical.coefficients()->Matches(model.coefficients()) ||
        !physical.mapping() || !physical.mapping()->Matches(model.shells(),model.domain()))
        Reject(Status::SourceMismatch,"Wall and vehicle do not share the actual declared physical source");
    Shape shape;
    shape.nodes=model.domain().node_count();shape.vehicle_nodes=embedding.original().node_count();
    shape.shells=vehicle.gap_operands().shells().size()+1;
    shape.quads=vehicle.gap_operands().counts().quads;
    shape.units=vehicle.provenance().units;
    if(shape.quads>vehicle.gap_operands().shells().size() ||
        shape.quads+vehicle.gap_operands().counts().triangles!=vehicle.gap_operands().shells().size())
        Reject(Status::SourceMismatch,"Complete physical gap family census differs");
    n::units_detail::Factors units;
    if(!n::units_detail::Make(shape.units,units) ||
        !tl::math::SameScalarBits(shape.units.length_m,wall.geometry().native_working_length_m) ||
        shape.nodes!=shape.vehicle_nodes+4 || vehicle.startup_input().node_count!=shape.vehicle_nodes ||
        vehicle.gap_operands().counts().nodes!=shape.vehicle_nodes ||
        vehicle.gap_operands().corrected().coefficients().size()!=shape.vehicle_nodes ||
        owner.roles().node.size()!=shape.nodes)
        Reject(Status::SourceMismatch,"Finite-wall working units/domain/complete globalK source differ");
    if(shape.nodes>limits.nodes || shape.shells>limits.shells)
        Reject(Status::ResourceLimit,"Finite-wall source exceeds count caps");
    const auto& environment=model.environment_parent();
    if(environment.element_id!=wall.ids().shell || environment.part_id!=wall.ids().part ||
        environment.material_id!=wall.ids().material || environment.section_id!=wall.ids().section ||
        environment.qeph_index>=model.shells().qeph_count() ||
        model.shells().qeph_source_id(environment.qeph_index)!=wall.ids().shell ||
        wall.translation_fixed_bits().size()!=shape.nodes || wall.rotation_fixed().size()!=shape.nodes ||
        !wall.ids().interface)
        Reject(Status::SourceMismatch,"Declared wall is not the genuine appended QEPH parent");
    const auto& nodes=model.shells().qeph_nodes(environment.qeph_index);
    for(unsigned k=0;k<4;++k) {
        const auto node=environment.domain_nodes[k];
        if(node!=shape.vehicle_nodes+k || nodes[k]!=environment.shell_nodes[k] ||
            physical.mapping()->owner_index(nodes[k])!=node ||
            model.domain().nodes()[node].source_id!=wall.ids().nodes[k] ||
            wall.translation_fixed_bits()[node]!=7 || wall.rotation_fixed()[node]!=1 ||
            owner.roles().node[node]!=vehicle_runtime::SourceRole::Shell || model.rigid_assembly().FindMember(node))
            Reject(Status::SourceMismatch,"Wall node/fixed/rigid/CIN source role differs");
        // Actual shared physical incidence, never a contact mass substitute.
        const auto& row=model.coefficients().nodes()[node];
        const auto& count=row.occurrences;
        if(count.qeph!=1 || count.t3 || count.qbat || count.type25 || count.type13 || count.element_mass ||
            count.solid18 || count.solid24 || count.solid6z || count.solid18_law44 || count.solid18_law90 || count.beam18 ||
            !tl::math::SameScalarBits(row.coefficients.mass,wall.geometry().reference.nodal_mass[k]) ||
            !tl::math::SameScalarBits(row.coefficients.isotropic_inertia,wall.geometry().reference.isotropic_inertia[k]))
            Reject(Status::SourceMismatch,"Wall physical incidence/raw coefficients differ from its real component");
        shape.wall_nodes[k]=std::uint32_t(node);
    }
    return shape;
}
}
