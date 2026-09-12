#include "Internal.h"
#include "modelio/physical_domain/Policy.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_startup::physical_model {
Forecast VehiclePhysicalModel::Preflight(const modelio::physical_domain::VehiclePhysicalDomain& source,
                                         const VehicleShellBinding& shells, Limits limits) {
    using detail::Require;
    const bool supports = source.policy() == modelio::physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5;
    const bool extended = supports || source.policy() == modelio::physical_domain::Policy::RetainedShellAssembliesExtendedSolidsV4;
    const Limits hard = extended ? Limits::ExtendedSolids() : Limits{};
    const std::size_t requested[]{limits.host_bytes, limits.shell_map_bytes, limits.beam_bytes,
        limits.solid_bytes, limits.beam_contribution_bytes, limits.ledger_bytes, limits.part_bytes,
        limits.plain_bytes, limits.rigid_binding_bytes};
    const std::size_t maximum[]{hard.host_bytes, hard.shell_map_bytes, hard.beam_bytes,
        hard.solid_bytes, hard.beam_contribution_bytes, hard.ledger_bytes, hard.part_bytes,
        hard.plain_bytes, hard.rigid_binding_bytes};
    for (unsigned i = 0; i < std::size(requested); ++i)
        Require(requested[i] && requested[i] <= maximum[i], "Invalid vehicle physical model limit");
    Require(limits.structural_beam_bytes && limits.structural_beam_bytes <= hard.structural_beam_bytes &&
        limits.structural_contribution_bytes && limits.structural_contribution_bytes <= hard.structural_contribution_bytes,
        "Invalid structural beam model limit");
    Require(bool(source.source().structural_beam_source()) == supports,
        "Physical model structural beam authority and profile differ");
    const auto& canonical = source.source().tied_source().canonical().data();
    Require(&canonical == &shells.references().source().canonical().data(),
            "Vehicle shells and physical source do not share canonical authority");
    Require(source.source().solid_source().data().policy == modelio::physical_domain::detail::SolidPolicy(source.policy()),
            "Physical model source-domain profile differs from its solids");
    Require(shells.shells().qeph_count() == 324094 && shells.shells().t3_count() == 21301 &&
        shells.shells().qbat_count() == 4250 && (extended || source.domain().node_count() == 372435) &&
        source.source().type13_source().data().beams.size() == 4442 &&
        source.source().solid_source().data().rows.size() == (supports ? 4980u : extended ? 4900u : 2412u),
        "Complete retained vehicle mechanical source count changed");
    Forecast f;
    f.shell_source = shells.forecast().total_bytes;
    f.physical_source = source.forecast().total_bytes;
    // Overcharge the shared canonical/source backing here to bound the serial
    // source adapters without reproducing their private allocation layouts.
    const auto masses = modelio::point_mass::VehiclePointMassSource::Preflight(source.source(), source.domain());
    const auto welds = modelio::type25::VehicleType25Source::Preflight(source.source(), source.domain(),
                                                                    detail::WeldDeclaration());
    tl::util::BoundedArenaLayout native(limits.host_bytes), packing(limits.host_bytes), all(limits.host_bytes);
    tl::util::ArenaRegion ignored;
    for (unsigned i = 1; i < std::size(requested); ++i)
        Require(native.Append<unsigned char>(requested[i], ignored), "Native vehicle model reservation exceeds cap");
    if (supports) Require(native.Append<unsigned char>(limits.structural_beam_bytes, ignored) &&
        native.Append<unsigned char>(limits.structural_contribution_bytes, ignored),
        "Structural beam native reservations exceed cap");
    f.native_reservation = native.bytes();
    const auto& beam = source.source().type13_source().data();
    const auto& solid = source.source().solid_source().data();
    Require(packing.Append<tl::fea::type13::ModelNode>(beam.nodes.size(), ignored) &&
        packing.Append<tl::fea::type13::ModelConnection>(beam.beams.size(), ignored) &&
        packing.Append<tl::fea::solids::Input18>(solid.solid18.size(), ignored) &&
        packing.Append<tl::fea::solids::Input24>(solid.solid24.size(), ignored) &&
        packing.Append<tl::fea::solids::Input6z>(solid.solid6z.size(), ignored) &&
        packing.Append<tl::fea::solids::Input18Law44>(solid.solid18_law44.size(), ignored) &&
        packing.Append<tl::fea::solids::Input18Law90>(solid.solid18_law90.size(), ignored) &&
        packing.Append<tl::fea::NodalRigidGroupMember>(source.counts().plain_members, ignored) &&
        packing.Append<tl::fea::NodalRigidGroupInput>(source.plain_groups().size(), ignored),
        "Vehicle mechanics input packing exceeds cap");
    if (supports) {
        const auto& structural = source.source().structural_beam_source()->data();
        Require(structural.rows.size() == 142 &&
            packing.Append<tl::fea::beam18::ParentInput>(structural.rows.size(), ignored),
            "Vehicle support structural beam census or packing exceeds scope");
    }
    f.packing_bytes = packing.bytes();
    Require(all.Append<unsigned char>(sizeof(VehiclePhysicalModel) + 4096, ignored) &&
        all.Append<unsigned char>(f.shell_source, ignored) && all.Append<unsigned char>(f.physical_source, ignored) &&
        all.Append<unsigned char>(masses.total_bytes, ignored) && all.Append<unsigned char>(welds.total_bytes, ignored) &&
        all.Append<unsigned char>(f.native_reservation, ignored) && all.Append<unsigned char>(f.packing_bytes, ignored),
        "Complete vehicle physical model construction exceeds cap");
    f.producer_source = masses.total_bytes + welds.total_bytes;
    f.total_bytes = all.bytes();
    return f;
}
} // namespace crash::cases::vehicle_startup::physical_model
