#include "Internal.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
std::vector<s::Main> OrientedMains(const s::MixedSidesSnapshot& side, const std::vector<s::PrimaryCornerPermutation>& corners) {
    Require(side.mains && side.primary_count && side.primary_count<=side.main_count && side.main_count<=1048576 &&
        corners.size()==side.primary_count, "Post-GAPM orientation extent differs");
    for(const auto& row:corners)for(const auto slot:row.source_corner)
        Require(slot<4, "Post-GAPM orientation slot exceeds primary");
    std::vector<s::Main> mains(side.mains, side.mains+side.main_count);
    Require(mains.capacity() <= 2*side.main_count, "Post-GAPM temporary main capacity exceeds forecast");
    for (std::size_t p=0; p<side.primary_count; ++p) for (unsigned k=0; k<4; ++k)
        mains[p].nodes[k] = side.mains[p].nodes[corners[p].source_corner[k]];
    return mains;
}
n::source_gaps::Profile SourceGapProfile() noexcept {
    // Existing closed direct converter/default profile. Native IGAP0 is the
    // free-edge gap switch; it is distinct from the IEDGE interaction mode.
    return {1, 0, 1, 1, 0, 0, 1., n::native_constant::ep20*n::native_constant::ep10,
        n::native_constant::ep20*n::native_constant::ep10};
}
void Gaps(const Mixed& source, const GapOperands& operands, Values& values, Limits limits) {
    const auto& side=source.sides();
    const auto mains=OrientedMains(side,values.corners);
    n::source_gaps::Input input;
    values.gap_profile = SourceGapProfile();
    input.profile = values.gap_profile;
    input.node_count = source.initial().geometry().nodes.size();
    input.shells = operands.shells().data(); input.shell_count = operands.shells().size();
    input.beams = operands.beams().data(); input.beam_count = operands.beams().size();
    input.springs = operands.springs().data(); input.spring_count = operands.springs().size();
    input.mains = mains.data(); input.main_count = mains.size(); input.primary_count = side.primary_count;
    input.secondary_nodes = values.secondary_nodes.data(); input.secondary_count = values.secondary_nodes.size();
    input.main_nodes = values.main_nodes.data(); input.main_node_count = values.main_nodes.size();
    n::source_gaps::Forecast capacity;
    auto report = n::source_gaps::Preflight(input, limits.gaps, capacity);
    if (report.status != n::source_gaps::Status::Ok)
        Reject(report.status == n::source_gaps::Status::ResourceLimit ? Status::ResourceLimit : Status::InvalidInput,
            "Native gap source preflight rejected complete operands");
    tl::util::HostArena scratch;
    Require(scratch.Initialize(capacity.scratch_bytes), "Native gap source scratch allocation failed");
    values.secondary_gaps.resize(input.secondary_count); values.main_node_gaps.resize(input.main_node_count);
    values.main_gaps.resize(input.main_count);
    values.gap_report = n::source_gaps::Build(input, limits.gaps, scratch.data(), scratch.bytes(),
        {values.secondary_gaps.data(), values.secondary_gaps.size(), values.main_node_gaps.data(),
            values.main_node_gaps.size(), values.main_gaps.data(), values.main_gaps.size()});
    Require(values.gap_report.status == n::source_gaps::Status::Ok && values.gap_report.completed,
        "Native gap source rejected complete post-GAPM rosters or arithmetic");
}
}
