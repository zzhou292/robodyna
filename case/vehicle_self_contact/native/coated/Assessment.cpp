#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::coated {
namespace {
std::string SourceBinding(const Result& result) {
    const auto& p = result.provenance;
    return output::Sha256("v5-physical-coating-source-v1:" + p.canonical_manifest.sha256 + p.scope_report.sha256 +
        p.source_member.sha256 + p.auxiliary_sha256 + p.combine_sha256);
}
void Capacity(const Inputs& input) {
    output::Require(input.nodes.capacity() <= 2*input.nodes.size() && input.shells.capacity() <= 2*input.shells.size() &&
        input.solids.capacity() <= 2*input.solids.size(), "Coating input allocation exceeded its reserved vector capacity");
}
}
Result AssessCoatedSource(const PhysicalModel& model, const selection::OriginalSelection& selection,
        const std::string& member, Config config, Limits limits) {
    Result result;
    result.config = config;
    result.forecast = Preflight(model, selection, config, limits);
    output::Require(result.forecast.admitted, "Complete V5 coated source forecast exceeds capacity");
    const auto& canonical = selection.canonical().data();
    const auto& declared = selection.data();
    result.provenance = {canonical.inputs.canonical_manifest, canonical.inputs.scope_report, canonical.inputs.source_member,
        declared.auxiliary_sha256, declared.combine_sha256, canonical.inputs.units};
    result.contact_parts = declared.parts;
    auto input = detail::PrepareInputs(model, selection, member, config, limits);
    Capacity(input);
    result.physical_nodes = input.nodes.size(); result.physical_solids = input.solids.size();
    result.original_solids = model.source_domain().source().solid_source().data().original_solids;
    for (const auto& solid : input.solids) {
        output::Require(solid.family < result.solid_families.size(), "Unknown retained solid family");
        ++result.solid_families[solid.family];
    }
    result.coordinate_roundtrip_changes = input.coordinate_roundtrip_changes;
    const auto classified = Classify(input);
    result.physical = classified.physical; result.contact = classified.contact;
    result.candidate_visits = classified.candidate_visits; result.physical_roles_complete = classified.complete;
    result.selected_roles_complete = classified.contact_complete;
    result.first_unready = detail::ShellLocation(input, classified.first_unready_shell);
    result.role_status = classified.first_status;
    result.first_selected_unready = detail::ShellLocation(input, classified.first_unready_contact_shell);
    result.selected_role_status = classified.first_contact_status;
    if (!classified.contact_complete) {
        result.input_digest = detail::InputDigest(input, classified, nullptr, config, SourceBinding(result), limits.metadata_bytes);
        return result;
    }
    const auto order = SurfaceOrder(input, classified);
    output::Require(order.primary_to_physical.size() == declared.counts.retained_shells,
                    "Native surface order lost a selected contact shell");
    result.input_digest = detail::InputDigest(input, classified, &order, config, SourceBinding(result), limits.metadata_bytes);
    std::vector<std::uint64_t> ids;
    std::vector<double> coordinates;
    std::vector<s::PrimaryFace> primary;
    ids.reserve(input.nodes.size()); coordinates.reserve(3*input.nodes.size());
    primary.reserve(order.primary_to_physical.size());
    for (const auto& node : input.nodes) {
        ids.push_back(node.source_id);
        const auto x = node.native_position;
        coordinates.insert(coordinates.end(), {x.x, x.y, x.z});
    }
    for (const auto physical : order.primary_to_physical) {
        auto face = input.shells[physical].primary;
        face.side_role = SideRole(classified.roles[physical].state);
        primary.push_back(face);
    }
    s::Input view;
    view.profile = s::Profile::ResolvedShellSides;
    view.topology = s::TopologyPolicy::NativeResolvedShellSides;
    view.node_source_ids = ids.data(); view.node_count = ids.size();
    // Direct scalar backing avoids a full intermediate vector<Vec3> copy.
    const auto packed_bytes = ids.capacity()*sizeof(std::uint64_t) + coordinates.capacity()*sizeof(double);
    output::Require(packed_bytes <= 2*input.nodes.size()*(sizeof(std::uint64_t)+3*sizeof(double)) &&
        primary.capacity() <= 2*order.primary_to_physical.size(), "TL input packing exceeded its reserved capacity");
    view.positions = {coordinates.data(), std::uint32_t(ids.size()), 3, 1};
    view.primary = primary.data(); view.primary_count = primary.size();
    view.coordinates = s::Coordinates::Native; view.units = input.units;
    view.source_generation = 1; // Local immutable assessment scope, never a physical epoch.
    tl::util::HostArena output;
    output::Require(output.Initialize(result.forecast.topology.output_bytes), "Coated topology output allocation failed");
    s::Snapshot snapshot;
    {
        tl::util::HostArena scratch;
        output::Require(scratch.Initialize(result.forecast.topology.scratch_bytes), "Coated topology scratch allocation failed");
        result.topology_attempted = true;
        result.topology_report = s::BuildStarter(view, limits.topology, output, scratch, &snapshot);
    }
    if (result.topology_report.status != s::Status::Ok) {
        const auto ordinal = result.topology_report.primary;
        if (ordinal != SIZE_MAX) {
            output::Require(ordinal < order.primary_to_physical.size(), "Invalid TL primary diagnostic");
            result.failure = detail::ShellLocation(input, order.primary_to_physical[ordinal]);
        }
        if (result.topology_report.node != SIZE_MAX) {
            output::Require(result.topology_report.node < ids.size(), "Invalid TL node diagnostic");
            result.failure.source_node_id = ids[result.topology_report.node];
        }
        return result;
    }
    result.topology_complete = true;
    result.output_mains = snapshot.main_count; result.output_references = snapshot.starter.reference_count;
    result.output_incidences = snapshot.normal_incidence_count;
    const auto& warning = result.topology_report.neighbor_warnings;
    if (warning.count) {
        output::Require(warning.first_main < snapshot.main_count && warning.first_edge < 4, "Invalid TL warning location");
        const auto ordinal = snapshot.expanded_to_primary[warning.first_main];
        result.first_warning = detail::ShellLocation(input, order.primary_to_physical[ordinal]);
        result.first_warning.expanded_main = warning.first_main; result.first_warning.edge = warning.first_edge;
        result.first_warning.source_node_id = ids[snapshot.mains[warning.first_main].nodes[warning.first_edge]];
    }
    result.output_digest = detail::OutputDigest(snapshot, result.input_digest.sha256, limits.metadata_bytes);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
