#include "Storage.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_native_contact {
std::uint64_t VehicleContactStartup::Data::TopologyGeneration(Role role) const {
    // Immutable source generation is the declared initial topology version for
    // self; the additional wall retains its separately declared version.
    return role == Role::Self ? self.snapshot().source_generation : wall.declaration().topology_generation;
}
n::initial_source::Input VehicleContactStartup::Data::InitialInput(Role role) const {
    const auto index = detail::RoleIndex(role);
    output::Require(model && fields[index], "Case source fields are not prepared");
    auto input = model->input(sources(), role, *fields[index]);
    input.stamp.runtime_topology = TopologyGeneration(role);
    input.engine_handoff = n::initial_source::EngineHandoff::SourceProvedFreshSerialSearchAtZero;
    return input;
}
n::TransactionConfig VehicleContactStartup::Data::ContactConfig(Role role) const {
    return role == Role::Self ? controls.self_runtime_controls() : wall.controls().runtime;
}
namespace {
void Common(n::ContactSourceInput& out, std::uint64_t id, std::uint64_t topology,
            const n::startup::Snapshot& starter, n::lifecycle::SourceView selection,
            const n::initial_source::PreparedSource& prepared,
            const vehicle_self_contact::native::initial_controls::Controls& controls) {
    const auto removed = prepared.removals();
    output::Require(prepared.prepared() && removed.main_count == starter.main_count &&
                        removed.primary_count == starter.primary_count && removed.primary_extent,
                    "Prepared native removal/extent output differs from source topology");
    out.source_id = id;
    out.topology_generation = topology;
    out.selection = selection;
    out.selection.removed_main_by_secondary = removed.by_secondary;
    out.primary_main_count = starter.primary_count;
    out.primary_curvature = removed.primary_extent;
    out.margin = removed.engine_margin;
    out.gap_load = controls.gap_load;
    out.drad = controls.drad;
    out.force_packet_size = controls.native_packet_size;
    out.native_workers = controls.starter_workers;
    out.contact_thickness_update = controls.thickness_update;
}
}
n::MixedMovingMainSource VehicleContactStartup::Data::Self() const {
    n::MixedMovingMainSource out;
    Common(out, controls.self_interface_id(), TopologyGeneration(Role::Self), self.snapshot(),
        fields[0]->runtime_view(), prepared[0], controls.controls());
    out.starter = self.snapshot();
    const auto& c = controls.controls();
    out.activation.edge_mode = c.edge_mode;
    out.activation.foreign_rows = 0;
    out.activation.partitions = c.partitions;
    out.activation.neighbor_removal = c.neighbor_removal;
    out.activation.local_processor = 1;
    out.activation.free_roster = n::normal_activation::FreeRosterPolicy::FreshComplete;
    return out;
}
n::FixedMainSource VehicleContactStartup::Data::Wall() const {
    output::Require(controls.wall_controls() != nullptr, "Declared wall controls are unavailable");
    n::FixedMainSource out;
    Common(out, controls.wall_interface_id(), TopologyGeneration(Role::MeshWall), wall.starter(),
        fields[1]->runtime_view(), prepared[1], *controls.wall_controls());
    out.primary_parent_ids = &wall.wall().ids().shell;
    return out;
}
} // namespace crash::cases::vehicle_native_contact
