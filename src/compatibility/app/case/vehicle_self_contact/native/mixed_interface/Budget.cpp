#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::mixed_interface::detail {
namespace {
void Add(std::size_t& total, std::size_t count, std::size_t width = 1) {
    if (!width || count > (SIZE_MAX-total)/width)
        Reject(Status::ResourceLimit, "Mixed interface source forecast overflows");
    total += count*width;
}
}
Forecast Budget(const Initial& initial, Limits limits) {
    const Limits hard;
    if (!limits.host_bytes || limits.host_bytes > hard.host_bytes ||
        !limits.metadata_bytes || limits.metadata_bytes > hard.metadata_bytes)
        Reject(Status::ResourceLimit, "Invalid mixed interface source limits");
    const auto& c = initial.census();
    const auto faces = initial.faces().size();
    const auto nodes = initial.geometry().nodes.size();
    const auto solids = initial.geometry().solids.size();
    const auto shells = initial.geometry().shells.size();
    if (!initial.certificate().membership_complete || !initial.certificate().consumed_order_complete ||
        initial.provenance().stage != initial_surfaces::Stage::InitialClauseBeforeI25Classification ||
        c.faces != faces || c.nodes != nodes || c.physical_shells != shells || c.physical_solids != solids ||
        c.extraction.shell_faces > faces || initial.emitted_solid_flags().size() != solids)
        Reject(Status::InvalidInput, "Mixed interface requires complete immutable initial surface authority");
    Forecast out;
    out.initial_source_reservation = initial.forecast().peak_bytes;
    // Each vector's actual capacity is checked against twice its admitted size.
    Add(out.packed_inputs, 2*solids, sizeof(source::Solid)+sizeof(std::uint32_t));
    Add(out.packed_inputs, 2*shells, sizeof(source::Shell)+sizeof(std::uint32_t));
    Add(out.packed_inputs, 2*initial.selection().data().selected_part_ids.size(), sizeof(std::uint64_t));
    Add(out.packed_inputs, 2*(solids+shells), sizeof(Lookup));
    Add(out.packed_inputs, 2*faces, sizeof(source::Face)+sizeof(std::uint32_t));
    Add(out.packed_inputs, 2*nodes, sizeof(std::uint64_t)+3*sizeof(double));
    // Public Classify implementation's bounded incidence, offsets and role
    // storage; all remain conservatively charged through certification/digest.
    Add(out.source_role_workspace, 16*solids, 2*sizeof(std::uint32_t));
    Add(out.source_role_workspace, 2*(nodes+1), sizeof(std::uint32_t));
    Add(out.source_role_workspace, 2*shells, sizeof(coated::Role));
    Add(out.source_role_workspace, 2*faces, sizeof(RoleObservation)+sizeof(std::uint32_t));
    Add(out.source_role_workspace, 64u << 10);
    // Borrowed descriptor validation requires real packing. Reserve the leaf's
    // enforced public ceilings now; exact preflight is checked before allocation.
    out.interface_output = f::Limits{}.output_bytes;
    out.shared_scratch = f::Limits{}.scratch_bytes;
    // This API is explicitly count-only, unlike the interface preflight.
    s::Input upper;
    upper.profile = s::Profile::MixedSurface;
    upper.topology = s::TopologyPolicy::NativeMixedSurface;
    upper.node_count = nodes;
    upper.primary_count = faces;
    upper.shell_primary_count = c.extraction.shell_faces;
    upper.raw_origin_count = faces;
    const auto side = s::PreflightMixedSides(upper);
    if (side.status != s::Status::Ok)
        Reject(Status::ResourceLimit, "Mixed sides upper count forecast was rejected");
    out.sides_output = side.output_bytes;
    out.shared_scratch = std::max(out.shared_scratch, side.scratch_bytes);
    Add(out.digest_and_report, 4u << 20);
    Add(out.digest_and_report, 16*limits.metadata_bytes);
    Add(out.digest_and_report, sizeof(AdmissionCensus)+16384);
    for (auto bytes : {out.initial_source_reservation, out.packed_inputs, out.source_role_workspace,
            out.interface_output, out.shared_scratch, out.sides_output, out.digest_and_report})
        Add(out.peak_bytes, bytes);
    if (out.peak_bytes > limits.host_bytes)
        Reject(Status::ResourceLimit, "Complete mixed interface source exceeds inclusive host cap");
    return out;
}
}
