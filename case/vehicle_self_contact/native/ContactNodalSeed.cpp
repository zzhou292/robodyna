#include "nodal_seed/Internal.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_self_contact::native::nodal_seed {
namespace d = detail;
struct PreCorrectionNodalSource::Data {
    Data(const PhysicalModel& p, const JointModel& j, const d::ids::ImportContext& source, Forecast f)
        : physical(p), joints(j), import(source), forecast(f) {}
    PhysicalModel physical;
    JointModel joints;
    d::ids::ImportContext import;
    d::ids::Resolution spring;
    Forecast forecast;
    Provenance provenance;
    InteriorDisposition interior;
    std::vector<Contributor> contributors;
    std::vector<n::NativeNodalSeed> seed;
    std::vector<PreCorrectionFields> fields;
};

Forecast PreCorrectionNodalSource::Preflight(const PhysicalModel& physical,
    const JointModel& joints, const modelio::native_spring_ids::ImportMembers& members, Limits limits) {
    return d::Budget(physical, joints, members, limits);
}

PreCorrectionNodalSource PreCorrectionNodalSource::Prepare(const PhysicalModel& physical,
    const JointModel& joints, const modelio::native_spring_ids::ImportMembers& members, Limits limits) {
    const auto forecast = Preflight(physical, joints, members, limits);
    const auto& canonical = physical.shell_source().references().source().canonical();
    const auto context = d::ids::ImportContext::Prepare(canonical, members);
    d::Require(context.data().diagnostic.status == d::ids::Readiness::Ready,
        "Contact seed complete source import context is unavailable");
    auto next = std::make_shared<Data>(physical, joints, context, forecast);
    next->spring = d::ids::Resolve(physical.source_domain().source().type13_source(),
        physical.welds(), joints.source(), context);
    d::Require(next->spring.diagnostic.status == d::ids::Readiness::Ready,
        "Contact seed complete native SPRING mapping is unavailable");
    auto& counts = next->forecast.counts;
    d::Require(next->spring.counts.type13 == counts.type13 &&
        next->spring.counts.default_welds == counts.type25 &&
        next->spring.counts.regular_joints == counts.type45 &&
        next->spring.physical_order.size() == counts.type13 + counts.type25 + counts.type45,
        "Contact seed resolved SPRING coverage differs from physical contributors");
    counts.namespace_only_springs = next->spring.counts.discrete_namespace_only;
    next->interior = d::ReadInterior(physical, members, limits);
    const auto& source_units = canonical.data().inputs.units;
    next->provenance.units = {source_units.length_to_m, source_units.mass_to_kg, source_units.time_to_s};
    next->provenance.canonical_sha256 = canonical.data().inputs.canonical_manifest.sha256;
    next->provenance.import_source_digest = context.data().source_digest;
    next->provenance.spring_mapping_digest = next->spring.mapping_digest;

    d::Packed packed;
    const auto parent_count = counts.solids + counts.beams + counts.type13 + counts.type25 + counts.type45 + counts.shells;
    packed.contributors.reserve(parent_count);
    packed.volumes.reserve(counts.volume_occurrences);
    packed.stiffness.reserve(counts.stiffness_occurrences);
    packed.shells.reserve(counts.shells);
    d::PackSolids(physical, next->provenance.units, packed);
    d::PackDirect(physical, joints, next->spring, next->provenance.units, packed);
    d::PackShells(physical, next->provenance.units, packed);
    d::Require(packed.contributors.size() == parent_count && packed.contributors.capacity() == parent_count &&
        packed.volumes.size() == counts.volume_occurrences && packed.volumes.capacity() == counts.volume_occurrences &&
        packed.stiffness.size() == counts.stiffness_occurrences && packed.stiffness.capacity() == counts.stiffness_occurrences &&
        packed.shells.size() == counts.shells && packed.shells.capacity() == counts.shells,
        "Contact seed packed contributor coverage or reservation differs");

    n::source_nodal::Input seed_input;
    seed_input.node_count = counts.nodes;
    seed_input.volumes = packed.volumes.empty() ? nullptr : packed.volumes.data();
    seed_input.volume_count = packed.volumes.size();
    seed_input.stiffness = packed.stiffness.empty() ? nullptr : packed.stiffness.data();
    seed_input.stiffness_count = packed.stiffness.size();
    n::source_nodal::Forecast seed_forecast;
    d::Require(n::source_nodal::Preflight(seed_input, {}, seed_forecast).status == n::source_nodal::Status::Ok,
        "Native contact seed forecast rejected the complete source");
    tl::util::HostArena seed_scratch;
    d::Require(seed_scratch.Initialize(seed_forecast.scratch_bytes), "Native contact seed scratch allocation failed");
    next->seed.resize(counts.nodes);
    d::Require(n::source_nodal::Accumulate(seed_input, {}, seed_scratch.data(), seed_scratch.bytes(),
        {next->seed.data(), next->seed.size()}).status == n::source_nodal::Status::Ok,
        "Native ordered contact seed accumulation failed");

    d::shells::Input shell_input;
    auto& profile = shell_input.profile;
    profile.population = d::shells::Population::PhysicalShellsWithNodalSeed;
    profile.property_type = 1;
    profile.input_thickness_mode = 0;
    profile.level = 1;
    profile.gap_mode = 1;
    profile.free_edge_gap = 0;
    profile.contact_thickness_update = 0;
    profile.stiffness_scale = 1.;
    // No main/secondary output is requested. Gap-only scratch is unavailable
    // from this public product; no unresolved source gap default is asserted.
    profile.gap_scale = 0.;
    profile.maximum_secondary_gap = 0.;
    profile.maximum_main_gap = 0.;
    shell_input.node_count = counts.nodes;
    shell_input.shells = packed.shells.data();
    shell_input.shell_count = packed.shells.size();
    const n::NativeNodalSeedView seed{next->seed.data(), next->seed.size()};
    d::shells::Forecast shell_forecast;
    d::Require(d::shells::Preflight(shell_input, seed, {}, shell_forecast).status == d::shells::Status::Ok,
        "Native physical-shell coefficient forecast rejected source operands");
    d::Require(seed_forecast.scratch_bytes + shell_forecast.scratch_bytes + shell_forecast.output_bytes <=
        forecast.numeric_scratch, "Native contact coefficient scratch exceeded aggregate reservation");
    tl::util::HostArena shell_scratch;
    d::Require(shell_scratch.Initialize(shell_forecast.scratch_bytes), "Native shell coefficient scratch allocation failed");
    std::vector<d::shells::NodeFields> nodes(counts.nodes);
    d::shells::Output shell_output;
    shell_output.nodes = nodes.data();
    shell_output.node_count = nodes.size();
    d::Require(d::shells::Build(shell_input, seed, {}, shell_scratch.data(), shell_scratch.bytes(), shell_output).status ==
        d::shells::Status::Ok, "Native full-shell ASSTIFI preparation failed");
    next->fields.reserve(counts.nodes);
    std::size_t actual_shell_occurrences = 0;
    for (const auto& node : nodes) {
        d::Require(node.shell_incidence_count >= 0, "Native shell incidence count is invalid");
        actual_shell_occurrences += static_cast<std::size_t>(node.shell_incidence_count);
        next->fields.push_back({node.young_thickness_sum, node.stiffness, node.shell_incidence_count});
    }
    d::Require(actual_shell_occurrences == counts.shell_occurrences,
        "Native physical shell coefficient incidence coverage differs");
    next->provenance.contributor_digest = d::ContributorDigest(packed, next->provenance, limits.metadata_bytes);
    next->contributors = std::move(packed.contributors);
    return PreCorrectionNodalSource(std::move(next));
}

const PhysicalModel& PreCorrectionNodalSource::physical() const noexcept { return data_->physical; }
const JointModel& PreCorrectionNodalSource::joints() const noexcept { return data_->joints; }
const Forecast& PreCorrectionNodalSource::forecast() const noexcept { return data_->forecast; }
const Counts& PreCorrectionNodalSource::counts() const noexcept { return data_->forecast.counts; }
const Provenance& PreCorrectionNodalSource::provenance() const noexcept { return data_->provenance; }
const InteriorDisposition& PreCorrectionNodalSource::interior() const noexcept { return data_->interior; }
const std::vector<Contributor>& PreCorrectionNodalSource::contributors() const noexcept { return data_->contributors; }
n::NativeNodalSeedView PreCorrectionNodalSource::seed() const noexcept { return {data_->seed.data(), data_->seed.size()}; }
tl::util::ConstView<PreCorrectionFields> PreCorrectionNodalSource::fields() const noexcept {
    return {data_->fields.data(), data_->fields.size()};
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed
