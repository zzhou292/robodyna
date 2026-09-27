#include "Internal.h"
#include "../SourcePolicies.h"
#include "modelio/self_contact/OriginalSelection.h"
#include "modelio/vehicle_source/OriginalAuthority.h"
#include "modelio/solid_source/SourcePolicy.h"
#include "lib_src/collision/self_contact_filters/Environment.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail {
Counts CheckModel(const PhysicalModel& model, const JointModel& joints, Limits limits) {
    const Limits hard;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes &&
        limits.nodes && limits.nodes <= hard.nodes && limits.shells && limits.shells <= hard.shells &&
        limits.solids && limits.solids <= hard.solids && limits.springs && limits.springs <= hard.springs &&
        limits.beams && limits.beams <= hard.beams && limits.metadata_bytes &&
        limits.metadata_bytes <= hard.metadata_bytes, "Invalid complete contact seed source limits");
    Require(tlfea::contact::self_contact_filters::CompatibleHostArithmetic(),
        "Contact seed source requires RN, gradual underflow and masked floating traps");
    const auto& physical = model.source_domain();
    const auto& domain = physical.domain();
    const auto& source = physical.source();
    const auto& solid_source = source.solid_source();
    const auto& canonical = solid_source.canonical().data();
    modelio::vehicle::CheckOriginalYarisAuthority(canonical);
    Require(source_policy::Geometry(physical.policy(),solid_source.data().policy) &&
        source_policy::Joints(physical.policy(),joints.source().policy()) &&
        source_policy::Controls(physical.policy(),model.solids()),
        "Contact seed requires matching complete declared source policies and V6 packet controls");
    Require(model.SharesStorage(joints.physical()) && domain.prepared() &&
        joints.model().prepared() && joints.model().domain() && joints.model().domain()->SharesStorage(domain) &&
        model.solids().prepared() && model.solids().domain() && model.solids().domain()->SharesStorage(domain) &&
        model.welds().domain().SharesStorage(domain) && model.beams().prepared(),
        "Contact seed source models do not share the same complete physical authority");
    Require(&canonical == &model.shell_source().references().source().canonical().data() &&
        model.solids().source_instance_id() == domain.source_instance_id() &&
        model.beams().source_instance_id() == domain.source_instance_id() &&
        model.welds().model().source_instance_id() == domain.source_instance_id() &&
        model.beams().global_node_count() == domain.node_count() &&
        model.welds().model().global_node_count() == domain.node_count(),
        "Contact seed canonical/model domain identity differs");
    const auto* beam = model.structural_beams();
    Require(beam && source.structural_beam_source() && beam->prepared() && beam->domain() &&
        beam->domain()->SharesStorage(domain), "Complete V5 structural beam source is missing");
    Counts counts;
    counts.nodes = domain.node_count();
    counts.shells = model.shell_source().references().rows().size();
    counts.solids = solid_source.data().rows.size();
    counts.solid_families = {model.solids().solid18().size(), model.solids().solid24().size(),
        model.solids().solid6z().size(), model.solids().solid18_law44().size(), model.solids().solid18_law90().size()};
    counts.beams = beam->parents().size();
    counts.type13 = model.beams().connection_count();
    counts.type25 = model.welds().model().connection_count();
    counts.type45 = joints.model().joints().size();
    Require(counts.nodes <= limits.nodes && counts.shells <= limits.shells && counts.solids <= limits.solids &&
        counts.beams <= limits.beams && counts.type13 + counts.type25 + counts.type45 <= limits.springs,
        "Complete contact seed source count exceeds capacity");
    const auto expected = modelio::solid_source::detail::ExpectedCensus(solid_source.data().policy);
    Require(counts.solids == expected.parents && counts.solid_families == std::array<std::size_t, 5>{
        expected.solid18, expected.solid24, expected.solid6z, expected.solid18_law44, expected.solid18_law90} &&
        counts.type13 == source.type13_source().data().beams.size() &&
        counts.type45 == joints.source_rows().size() && counts.type45 == joints.source().data().required &&
        counts.shells == model.shell_source().references().counts().succeeded,
        "Contact seed source family coverage differs");
    counts.volume_occurrences = 8 * counts.solids;
    counts.stiffness_occurrences = 2 * (counts.beams + counts.type13 + counts.type25 + counts.type45);
    const auto& shell_counts = model.shell_source().references().counts();
    counts.shell_occurrences = 4 * (shell_counts.qeph_succeeded + shell_counts.qbat_succeeded) +
        3 * shell_counts.t3_succeeded;
    return counts;
}

Forecast Budget(const PhysicalModel& model, const JointModel& joints,
    const ids::ImportMembers& members, Limits limits) {
    Forecast result;
    result.counts = CheckModel(model, joints, limits);
    const auto& count = result.counts;
    const auto& canonical = model.shell_source().references().source().canonical();
    const auto context = ids::ImportContext::Preflight(canonical, members);
    const auto add = [&](std::size_t& field, std::size_t number, std::size_t width) {
        Require(width && result.peak_bytes <= limits.host_bytes &&
            number <= (limits.host_bytes - result.peak_bytes) / width,
            "Complete contact seed source byte cap exceeded");
        const auto bytes = number * width;
        field += bytes;
        result.peak_bytes += bytes;
    };
    add(result.physical_reservation, model.forecast().total_bytes, 1);
    add(result.joint_additional, joints.additional_owned_payload_bytes(), 1);
    add(result.source_context, context.total_bytes, 1);
    // Reuse the resolver's enforced inclusive mapping/workspace cap. Its
    // Resolution remains retained through numeric preparation and hashing;
    // do not duplicate private Row/index/hash layout formulas here.
    add(result.resolution_workspace, ids::Limits{}.resolve_bytes, 1);
    const auto rows = count.solids + count.beams + count.type13 + count.type25 + count.type45 + count.shells;
    add(result.roster_bytes, rows, sizeof(Contributor));
    add(result.context_scratch, rows, sizeof(std::size_t));
    add(result.value_inputs, count.volume_occurrences, sizeof(n::NativeVolumeOccurrence));
    add(result.value_inputs, count.stiffness_occurrences, sizeof(n::NativeStiffnessOccurrence));
    add(result.value_inputs, count.shells, sizeof(shells::PhysicalShell));
    add(result.seed_output, count.nodes, sizeof(n::NativeNodalSeed));
    add(result.nodal_output, count.nodes, sizeof(PreCorrectionFields));
    add(result.numeric_scratch, count.nodes, sizeof(n::NativeNodalSeed));
    // Final shell output and its private staging coexist. Source values stay
    // alive through Build; its selected-shell byte scratch is separate.
    add(result.numeric_scratch, 2 * count.nodes, sizeof(shells::NodeFields));
    add(result.numeric_scratch, count.shells, sizeof(unsigned char));
    add(result.numeric_scratch, 256, 1); // Arena region alignment/control slack.
    std::size_t combine = 0, auxiliary = 0;
    for (const auto& member : members.members) {
        if (member.filename == "combine.key") combine = member.bytes.size();
        if (member.filename == "set-yaris-coarse-v1l.key") auxiliary = member.bytes.size();
    }
    add(result.context_scratch,
        modelio::self_contact::OriginalSelection::Forecast(canonical, auxiliary, combine), 1);
    add(result.context_scratch, canonical.data().canonical_bytes.size(), 7);
    add(result.context_scratch, 2, output::full_shell::source::FindArray(canonical.data(), "solids_records").bytes.size());
    add(result.context_scratch, limits.metadata_bytes, 8);
    add(result.fixed_bytes, 65536, 1);
    // Shared direct declarations/evidence stay alive after the old parser phase.
    add(result.fixed_bytes, modelio::solid_control::DirectLimits{}.retained_bytes, 1);
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::detail
