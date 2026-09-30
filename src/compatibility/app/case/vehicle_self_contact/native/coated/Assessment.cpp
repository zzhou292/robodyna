#include "Internal.h"
#include "TopologyInput.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::coated {
namespace {
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
        result.input_digest = detail::InputDigest(input, classified, nullptr, config, detail::SourceBinding(result.provenance), limits.metadata_bytes);
        return result;
    }
    const auto order = SurfaceOrder(input, classified);
    output::Require(order.primary_to_physical.size() == declared.counts.retained_shells,
                    "Native surface order lost a selected contact shell");
    result.input_digest = detail::InputDigest(input, classified, &order, config, detail::SourceBinding(result.provenance), limits.metadata_bytes);
    detail::TopologyInput packed(input, classified, order);
    const auto view = packed.View();
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
            output::Require(result.topology_report.node < view.node_count, "Invalid TL node diagnostic");
            result.failure.source_node_id = view.node_source_ids[result.topology_report.node];
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
        result.first_warning.source_node_id = view.node_source_ids[snapshot.mains[warning.first_main].nodes[warning.first_edge]];
    }
    result.output_digest = detail::OutputDigest(snapshot, result.input_digest.sha256, limits.metadata_bytes);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
