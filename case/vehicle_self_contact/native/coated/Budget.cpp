#include "Internal.h"
#include "modelio/source_assembly/NativeCoordinates.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::coated {
namespace {
void Add(std::size_t& bytes, std::size_t count, std::size_t width = 1) {
    output::Require(width && count <= (SIZE_MAX-bytes)/width, "Coated source forecast overflows");
    bytes += count*width;
}
}
Forecast Preflight(const PhysicalModel& model, const selection::OriginalSelection& selection,
        Config config, Limits limits) {
    const auto& source = detail::CheckSource(model, selection, config, limits);
    const auto d = model.source_domain().domain().node_count();
    const auto h = model.shell_source().references().rows().size();
    const auto c = selection.data().counts.retained_shells;
    const auto solid_count = model.source_domain().source().solid_source().data().rows.size();
    Forecast f;
    f.node_size = sizeof(Node); f.shell_size = sizeof(Shell); f.solid_size = sizeof(Solid); f.role_size = sizeof(Role);
    // Model forecast already conservatively includes all of its shared source
    // handles and retired startup work. Do not add canonical backing again.
    f.retained_model_reservation = model.forecast().total_bytes;
    f.selection_reservation = std::max(selection.data().startup_budget_bytes, selection.data().owned_payload_bytes);
    f.input_bytes = sizeof(Inputs);
    Add(f.input_bytes, 2*d, sizeof(Node)); Add(f.input_bytes, 2*h, sizeof(Shell)); Add(f.input_bytes, 2*solid_count, sizeof(Solid));
    // Bounded slack is charged before allocations; actual capacities are checked.
    Add(f.member_reservation, 2, source.inputs.source_member.bytes);
    f.decode_peak = 0;
    for (const auto* name : {"node_ids", "node_positions", "node_source_lines", "node_blank_masks", "node_codes",
         "shells_records", "shells_node_indices", "shells_source_lines"})
        Add(f.decode_peak, 2, source::FindArray(source, name).descriptor.bytes);
    Add(f.decode_peak, modelio::source_nodes::NativeCoordinateBytes(source, d));
    Add(f.decode_peak, 2*(source.canonical_nodes+d), sizeof(std::uint32_t));
    Add(f.decode_peak, source.canonical_shells);
    f.role_bytes = sizeof(Classification); Add(f.role_bytes, 2*h, sizeof(Role));
    Add(f.membership_scratch, 16*solid_count, 2*sizeof(std::uint32_t));
    Add(f.membership_scratch, 2*(d+1), sizeof(std::uint32_t));
    // Six unsigned key words + source ordinal, two retained inverse/permutation
    // arrays; std::sort uses bounded call stack rather than a second record copy.
    Add(f.ordering_scratch, 2*c, 7*sizeof(std::uint32_t));
    Add(f.ordering_scratch, 2*(h+c), sizeof(std::uint32_t));
    Add(f.ordering_scratch, 64u<<10);
    f.result_bytes = sizeof(Result);
    Add(f.result_bytes, 2*selection.data().parts.size(), sizeof(selection::PartDisposition));
    Add(f.result_bytes, 64, sizeof(FieldDigest)+256);
    // Copied provenance is charged independently of a caller-shrunken document
    // cap; field-vector allowance is not a substitute for these actual strings.
    for (const auto* text : {&source.inputs.canonical_manifest.file, &source.inputs.canonical_manifest.sha256,
         &source.inputs.scope_report.file, &source.inputs.scope_report.sha256, &source.inputs.source_member.file,
         &source.inputs.source_member.sha256, &selection.data().auxiliary_sha256, &selection.data().combine_sha256,
         &source.inputs.units.length, &source.inputs.units.mass, &source.inputs.units.time})
        Add(f.result_bytes, 2, text->capacity()+1);
    Add(f.result_bytes, 8, limits.metadata_bytes);
    s::Input topology;
    topology.profile = s::Profile::ResolvedShellSides;
    topology.topology = s::TopologyPolicy::NativeResolvedShellSides;
    topology.node_count = d; topology.primary_count = c;
    f.topology = s::Preflight(topology, limits.topology);
    std::size_t common = f.retained_model_reservation;
    Add(common, f.selection_reservation); Add(common, f.member_reservation); Add(common, f.input_bytes); Add(common, f.result_bytes); Add(common, f.role_bytes);
    auto decode = common; Add(decode, f.decode_peak);
    auto classify = common; Add(classify, f.membership_scratch);
    auto order = common; Add(order, f.ordering_scratch);
    auto build = order;
    // Exact source lists are separate from immutable typed Inputs during TL call.
    Add(build, 2*d, sizeof(std::uint64_t)+3*sizeof(double));
    Add(build, 2*c, sizeof(s::PrimaryFace));
    Add(build, f.topology.output_bytes); Add(build, f.topology.scratch_bytes);
    // Input packet vectors remain live while the output is hashed. Only the
    // topology scratch arena has ended its lifetime at this phase boundary.
    auto digest = build - f.topology.scratch_bytes;
    Add(digest, 4u<<20);
    f.peak_bytes = std::max({decode, classify, order, build, digest});
    f.admitted = f.topology.status == s::Status::Ok && f.peak_bytes <= limits.host_bytes;
    return f;
}
} // namespace crash::cases::vehicle_self_contact::native::coated
