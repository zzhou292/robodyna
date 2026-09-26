#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::initial_surfaces::detail {
namespace {
void Add(std::size_t& value, std::size_t count, std::size_t width = 1) {
    if (!width || count > (SIZE_MAX-value)/width) Reject(Status::ResourceLimit, "Initial surface forecast overflows");
    value += count*width;
}
}
Forecast Budget(const Context& context, const Selection& selection, Limits limits) {
    const Limits hard;
    if (!limits.host_bytes || limits.host_bytes > hard.host_bytes || !limits.nodes || limits.nodes > hard.nodes ||
        !limits.shells || limits.shells > hard.shells || !limits.solids || limits.solids > hard.solids ||
        !limits.parts || limits.parts > hard.parts || !limits.metadata_bytes || limits.metadata_bytes > hard.metadata_bytes)
        Reject(Status::ResourceLimit, "Invalid initial surface source limits");
    const auto& physical = context.pre_correction().physical();
    coated::Limits geometry_limits;
    geometry_limits.host_bytes = limits.host_bytes;
    geometry_limits.nodes = limits.nodes;
    geometry_limits.shells = limits.shells;
    geometry_limits.solids = limits.solids;
    geometry_limits.metadata_bytes = limits.metadata_bytes;
    const auto upstream = coated::Preflight(physical, selection, {}, geometry_limits);
    if (!upstream.admitted) Reject(Status::ResourceLimit, "Authenticated geometry reservation exceeds source cap");
    const auto shells = physical.shell_source().references().rows().size();
    const auto solids = physical.source_domain().source().solid_source().data().rows.size();
    const auto parts = selection.data().selected_part_ids.size();
    if (parts > limits.parts) Reject(Status::ResourceLimit, "Initial source part count exceeds capacity");
    Forecast f;
    // The context already retains the exact shared physical model. Add only
    // the existing geometry producer's PUBLIC input/decode/member partitions;
    // no coating classification, topology or unrelated result is constructed.
    f.context_reservation = context.forecast().peak_bytes;
    for (auto bytes : {upstream.selection_reservation, upstream.member_reservation,
            upstream.input_bytes, upstream.decode_peak})
        Add(f.upstream_geometry_reservation, bytes);
    auto common = f.context_reservation;
    Add(common, f.upstream_geometry_reservation);
    Add(f.packed_inputs, 2*solids, sizeof(values::Solid)+sizeof(std::uint32_t));
    Add(f.packed_inputs, 2*shells, sizeof(values::Shell)+sizeof(std::uint32_t));
    Add(f.packed_inputs, 2*parts, sizeof(std::uint64_t));
    Add(f.certificate_workspace, 2*shells, sizeof(ShellKey));
    Add(f.certificate_workspace, 64u << 10); // Bounded sort call-stack allowance.
    // The numerical preflight also validates borrowed ranges, which do not
    // exist until source packing. Reserve its enforced PUBLIC arena ceilings;
    // after packing, compare genuine exact preflights and allocate only those.
    // These are upper bounds, not invented pointers or copied private layouts.
    f.extraction_output = values::Limits{}.output_bytes;
    f.extraction_scratch = values::Limits{}.scratch_bytes;
    f.maximum_faces = 6*solids + shells;
    Add(f.retained_faces, 2*f.maximum_faces, sizeof(Face));
    Add(f.retained_faces, 2*solids, sizeof(std::uint8_t));
    // At most faces/2 groups. Reserve private staging and retained capacity,
    // conservatively as two full-face extents; no origin is dropped.
    Add(f.origin_group_bytes, 2*f.maximum_faces, sizeof(OriginGroup));
    Add(f.digest_workspace, 4u << 20);
    Add(f.digest_workspace, 16*limits.metadata_bytes);
    f.peak_bytes = common;
    // PrepareInputs' returned arrays remain live through the entire new stage;
    // the upstream bound also includes its decode/transient construction peak.
    for (auto bytes : {f.packed_inputs, f.certificate_workspace, f.extraction_output,
            f.extraction_scratch, f.retained_faces, f.origin_group_bytes, f.digest_workspace, std::size_t{16384}})
        Add(f.peak_bytes, bytes);
    if (f.peak_bytes > limits.host_bytes) Reject(Status::ResourceLimit, "Complete initial surface source exceeds host cap");
    return f;
}
void CheckCapacity(const coated::Inputs& geometry, const Packing& p, std::size_t faces) {
    output::Require(geometry.nodes.capacity() <= 2*geometry.nodes.size() &&
        geometry.shells.capacity() <= 2*geometry.shells.size() && geometry.solids.capacity() <= 2*geometry.solids.size(),
        "Authenticated geometry capacities exceed reservation");
    output::Require(p.solids.capacity() <= 2*geometry.solids.size() &&
        p.selected_solids.capacity() <= 2*geometry.solids.size() &&
        p.quads.capacity()+p.triangles.capacity() <= 2*geometry.shells.size() &&
        p.quad_to_physical.capacity()+p.triangle_to_physical.capacity() <= 2*geometry.shells.size() &&
        p.selected_parts.capacity() <= 2*p.selected_parts.size() && faces <= 6*geometry.solids.size()+geometry.shells.size(),
        "Initial surface staging exceeds admitted capacity");
}
}
