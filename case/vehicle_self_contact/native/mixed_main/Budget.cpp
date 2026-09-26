#include "Internal.h"
#include <algorithm>
#include <type_traits>
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
namespace {
void Add(std::size_t& total, std::size_t count, std::size_t width=1) {
    if (!width || count > (SIZE_MAX-total)/width) Reject(Status::ResourceLimit, "Post-GAPM forecast overflow");
    total += count*width;
}
}
Forecast Budget(const Mixed& mixed, const GapOperands& gaps, Limits limits) {
    Check(mixed, gaps, limits);
    const auto& initial = mixed.initial();
    const auto& input = initial.geometry();
    const auto node_count = input.nodes.size(), h = input.shells.size(), p = mixed.sides().primary_count, g = mixed.sides().main_count;
    Forecast f;
    f.mixed_retained = mixed.retained_host_upper_bound(limits.host_bytes);
    f.gap_additional_retained = gaps.additional_retained_upper_bound(limits.host_bytes);
    // Both inputs share the authenticated exact corrected graph. Keep each
    // previous construction peak separate from its now-retired local scratch.
    // Either input can be prepared while the other is retained. Initial's
    // owning retained method explicitly includes this exact context bound once.
    // Remove only that named shared partition, after Check proves identity.
    const auto shared_context = initial.context().forecast().peak_bytes;
    Require(f.mixed_retained >= shared_context, "Mixed retained context partition differs");
    f.mixed_input_construction_peak = mixed.forecast().peak_bytes;
    Add(f.mixed_input_construction_peak, f.gap_additional_retained);
    f.gap_input_construction_peak = gaps.forecast().peak_bytes;
    Add(f.gap_input_construction_peak, f.mixed_retained-shared_context);
    f.prior_construction_peak = std::max(f.mixed_input_construction_peak, f.gap_input_construction_peak);
    Add(f.input_maps, 2*node_count, sizeof(std::uint64_t)+3*sizeof(double)+2*sizeof(std::uint32_t));
    Add(f.query_workspace, 2*node_count, sizeof(unsigned char));
    Add(f.query_workspace, 2*limits.parts, sizeof(old::PartValue)+256);
    Add(f.query_workspace, 2*h, sizeof(old::ShellValue)+sizeof(old::FaceKey));
    Add(f.query_workspace, 4*h, sizeof(std::size_t));
    Add(f.query_workspace, old::SupportQueryBytes(node_count,h));
    Add(f.query_workspace, old::SolidSupportQueryBytes(node_count,input.solids.size()));
    Add(f.query_workspace, 16, initial.selection().canonical().data().canonical_bytes.size());
    Add(f.query_workspace, 2, modelio::self_contact::Limits{}.combine_member_bytes);
    Add(f.output_values, 2*p, sizeof(s::PrimaryCornerPermutation)+sizeof(s::PreShellSolidSupport)+sizeof(PhysicalOwner)+sizeof(Geometry));
    Add(f.output_values, 2*g, sizeof(s::PostGapmMainSupport)+sizeof(double)+sizeof(n::source_gaps::MainGapFields));
    Add(f.output_values, 4*node_count, sizeof(double));
    Add(f.temporary_mains, 2*g, sizeof(s::Main));
    const n::source_gaps::Limits hard;
    const std::size_t requested[]{limits.gaps.nodes,limits.gaps.shells,limits.gaps.lines,limits.gaps.springs,
        limits.gaps.mains,limits.gaps.secondaries,limits.gaps.main_nodes,limits.gaps.scratch_bytes,limits.gaps.output_bytes};
    const std::size_t maximum[]{hard.nodes,hard.shells,hard.lines,hard.springs,hard.mains,hard.secondaries,
        hard.main_nodes,hard.scratch_bytes,hard.output_bytes};
    for (unsigned i=0; i<9; ++i) if (!requested[i] || requested[i]>maximum[i])
        Reject(Status::ResourceLimit, "Invalid bounded native gap value limits");
    if (node_count>limits.gaps.nodes || node_count>limits.gaps.secondaries || node_count>limits.gaps.main_nodes || g>limits.gaps.mains ||
        h>limits.gaps.shells || gaps.beams().size()>limits.gaps.lines || gaps.springs().size()>limits.gaps.springs)
        Reject(Status::ResourceLimit, "Complete gap input domain exceeds capacity");
    f.gap_scratch = limits.gaps.scratch_bytes;
    Add(f.metadata, 16, limits.metadata_bytes); Add(f.metadata, 16384);
    for (const auto bytes : {f.mixed_retained,f.gap_additional_retained,f.input_maps,f.query_workspace,
            f.output_values,f.temporary_mains,f.gap_scratch,f.metadata}) Add(f.current_phase,bytes);
    f.peak_bytes = std::max(f.prior_construction_peak,f.current_phase);
    for (const auto bytes : {f.mixed_retained,f.gap_additional_retained,f.input_maps,f.output_values,f.metadata})
        Add(f.retained_bytes,bytes);
    if (f.peak_bytes>limits.host_bytes) Reject(Status::ResourceLimit, "Complete post-GAPM source exceeds host envelope");
    return f;
}
std::size_t RetainedValues(const Values& v, std::size_t cap) {
    tl::util::BoundedArenaLayout bytes(cap); tl::util::ArenaRegion unused;
    const auto add=[&](const auto& list) {
        using T=typename std::decay_t<decltype(list)>::value_type;
        Require(bytes.Append<T>(list.capacity(),unused), "Retained post-GAPM vector payload exceeds cap");
    };
    add(v.node_ids);add(v.positions);add(v.coefficients);add(v.corners);add(v.before_shell);add(v.final_support);
    add(v.owners);add(v.geometry);add(v.secondary_nodes);add(v.main_nodes);add(v.secondary_gaps);add(v.main_node_gaps);add(v.main_gaps);
    return bytes.bytes();
}
}
