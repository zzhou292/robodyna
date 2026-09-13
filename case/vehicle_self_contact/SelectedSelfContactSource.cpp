#include "SelectedSelfContactSource.h"

#include "lib_src/collision/Q4ContactBounds.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::vehicle_self_contact {
namespace {
namespace fe = tl::fea;
using modelio::self_contact::PartDisposition;

struct Plan {
    Config config;
    SourceInventory inventory;
    contact::SelfContactSurfaceBinding surface;
    contact::FixedContactFacetBinding facets;
    contact::SelfContactActiveUsePreflight active;
    SourceForecast forecast;
};

bool Same(const modelio::self_contact::Counts& a,
          const modelio::self_contact::Counts& b) noexcept {
    return a.selected_parts == b.selected_parts &&
        a.shell_parts == b.shell_parts &&
        a.retained_shell_parts == b.retained_shell_parts &&
        a.excluded_shell_parts == b.excluded_shell_parts &&
        a.non_shell_parts == b.non_shell_parts &&
        a.shells == b.shells &&
        a.retained_shells == b.retained_shells &&
        a.excluded_shells == b.excluded_shells &&
        a.solids == b.solids && a.beams == b.beams;
}

bool Centered(const fe::ShellPhysicalBinding& physical,
              const fe::ShellPlasticityParentInput& parent) {
    const auto& shells = *physical.shells();
    const auto index = parent.family_index;
    if (parent.family == fe::ShellBindingFamily::Qeph)
        return shells.qeph_reference(index).input.placement ==
            fe::ShellReferencePlacement::Centered;
    if (parent.family == fe::ShellBindingFamily::T3)
        return shells.t3_reference(index).input.placement ==
            fe::ShellReferencePlacement::Centered;
    if (parent.family == fe::ShellBindingFamily::Qbat) {
        const auto& value = shells.qbat_reference(index);
        return value.quadrilateral().input.placement ==
                fe::ShellReferencePlacement::Centered &&
            value.input().options.offset_ratio == 0;
    }
    output::Require(false,
        "Selected self-contact parent has an unsupported shell family");
    return false;
}

std::size_t InventoryBytes(const SourceInventory& value) {
    tl::util::BoundedArenaLayout bytes(std::size_t{24} << 30);
    tl::util::ArenaRegion unused;
    output::Require(
        bytes.Append<modelio::self_contact::SourceId>(
            value.retained_shell_part_ids.capacity(), unused) &&
        bytes.Append<modelio::self_contact::SourceId>(
            value.selected_part_ids.capacity(), unused) &&
        bytes.Append<modelio::self_contact::SourceId>(
            value.offset_part_ids.capacity(), unused) &&
        bytes.Append<modelio::self_contact::SourceId>(
            value.omitted_tire_part_ids.capacity(), unused) &&
        bytes.Append<modelio::self_contact::SourceId>(
            value.unsupported_non_shell_part_ids.capacity(), unused) &&
        bytes.Append<modelio::self_contact::SourceId>(
            value.unsupported_solid_part_ids.capacity(), unused) &&
        bytes.Append<modelio::self_contact::SourceId>(
            value.unsupported_beam_part_ids.capacity(), unused) &&
        bytes.Append<contact::SelfContactParentSelection>(
            value.selected_parents.capacity(), unused) &&
        bytes.Append<contact::SelfContactParentSelection>(
            value.offset_parents.capacity(), unused),
        "Selected self-contact copied inventory bytes overflow");
    return bytes.bytes();
}

SourceInventory BuildInventory(const fe::ShellPhysicalBinding& physical,
    const modelio::self_contact::Data& original,
    std::size_t* temporary_bytes) {
    using output::Require;
    Require(physical.prepared() && physical.catalog() && physical.shells() &&
            physical.domain() && physical.mapping() &&
            original.profile == modelio::self_contact::
                OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1,
        "Prepared physical shells and original automatic source are required");
    Require(original.parts.size() == original.selected_part_ids.size(),
        "Original self-contact part rows and selected IDs differ");

    modelio::self_contact::Counts checked;
    std::vector<modelio::self_contact::SourceId> all_ids;
    std::vector<modelio::self_contact::SourceId> retained_ids;
    std::vector<std::size_t> retained_shell_counts;
    all_ids.reserve(original.parts.size());
    retained_ids.reserve(original.counts.retained_shell_parts);
    retained_shell_counts.reserve(original.counts.retained_shell_parts);
    SourceInventory result;
    result.retained_shell_part_ids.reserve(
        original.counts.retained_shell_parts);
    result.selected_part_ids.reserve(original.counts.retained_shell_parts);
    result.offset_part_ids.reserve(original.counts.retained_shell_parts);
    result.omitted_tire_part_ids.reserve(
        original.counts.excluded_shell_parts);
    result.unsupported_non_shell_part_ids.reserve(
        original.counts.non_shell_parts);
    for (std::size_t i = 0; i < original.parts.size(); ++i) {
        const auto& part = original.parts[i];
        Require(part.part_id && part.part_id == original.selected_part_ids[i],
            "Original self-contact selected part identity differs");
        all_ids.push_back(part.part_id);
        ++checked.selected_parts;
        checked.shell_parts += part.shells != 0;
        checked.retained_shell_parts += part.retained_shell_part;
        checked.excluded_shell_parts += part.excluded_shell_part;
        checked.non_shell_parts += !part.shell_section;
        checked.shells += part.shells;
        checked.retained_shells +=
            part.retained_shell_part ? part.shells : 0;
        checked.excluded_shells +=
            part.excluded_shell_part ? part.shells : 0;
        checked.solids += part.solids;
        checked.beams += part.beams;
        Require(!(part.retained_shell_part && part.excluded_shell_part) &&
                (!part.shells || part.retained_shell_part ||
                    part.excluded_shell_part) &&
                (!part.shells || part.shell_section),
            "Original self-contact part disposition is inconsistent");
        if (part.retained_shell_part) {
            retained_ids.push_back(part.part_id);
            retained_shell_counts.push_back(part.shells);
            result.retained_shell_part_ids.push_back(part.part_id);
        }
        if (part.excluded_shell_part)
            result.omitted_tire_part_ids.push_back(part.part_id);
        if (!part.shell_section)
            result.unsupported_non_shell_part_ids.push_back(part.part_id);
        if (part.solids)
            result.unsupported_solid_part_ids.push_back(part.part_id);
        if (part.beams)
            result.unsupported_beam_part_ids.push_back(part.part_id);
    }
    Require(Same(checked, original.counts),
        "Original self-contact declared counts differ from its part rows");
    std::sort(all_ids.begin(), all_ids.end());
    Require(std::adjacent_find(all_ids.begin(), all_ids.end()) ==
            all_ids.end(),
        "Duplicate original self-contact PID");

    std::vector<std::pair<modelio::self_contact::SourceId, std::size_t>>
        retained;
    retained.reserve(retained_ids.size());
    for (std::size_t i = 0; i < retained_ids.size(); ++i)
        retained.emplace_back(retained_ids[i], retained_shell_counts[i]);
    std::sort(retained.begin(), retained.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    Require(std::adjacent_find(retained.begin(), retained.end(),
            [](const auto& a, const auto& b) {
                return a.first == b.first;
            }) == retained.end(),
        "Duplicate retained self-contact PID");
    std::vector<std::size_t> represented(retained.size());
    std::vector<std::size_t> centered_represented(retained.size());
    std::vector<std::size_t> offset_represented(retained.size());

    result.selected_parents.reserve(original.counts.retained_shells);
    result.offset_parents.reserve(original.counts.retained_shells);
    for (std::size_t row = 0;
         row < physical.catalog()->parent_count(); ++row) {
        const auto& parent = *physical.catalog()->parent(row);
        const auto found = std::lower_bound(retained.begin(), retained.end(),
            parent.source_part_id,
            [](const auto& entry, auto id) { return entry.first < id; });
        if (found == retained.end() ||
            found->first != parent.source_part_id) continue;
        const auto retained_index =
            static_cast<std::size_t>(found - retained.begin());
        ++represented[retained_index];
        const contact::SelfContactParentSelection selected{
            row, parent.family, parent.family_index,
            parent.source_parent_id, parent.source_part_id};
        if (Centered(physical, parent)) {
            ++centered_represented[retained_index];
            result.selected_parents.push_back(selected);
            result.counts.qeph_parents +=
                parent.family == fe::ShellBindingFamily::Qeph;
            result.counts.t3_parents +=
                parent.family == fe::ShellBindingFamily::T3;
            result.counts.qbat_parents +=
                parent.family == fe::ShellBindingFamily::Qbat;
        } else {
            ++offset_represented[retained_index];
            result.offset_parents.push_back(selected);
        }
    }
    for (std::size_t i = 0; i < retained.size(); ++i)
        Require(represented[i] == retained[i].second,
            "Original selected PID and physical shell parent count differ");
    for (const auto id : retained_ids) {
        const auto found = std::lower_bound(retained.begin(),
            retained.end(), id,
            [](const auto& entry, auto value) {
                return entry.first < value;
            });
        Require(found != retained.end() && found->first == id,
            "Retained self-contact PID index differs");
        const auto index =
            static_cast<std::size_t>(found - retained.begin());
        if (centered_represented[index])
            result.selected_part_ids.push_back(id);
        if (offset_represented[index])
            result.offset_part_ids.push_back(id);
    }

    std::vector<std::uint64_t> parent_ids;
    parent_ids.reserve(result.selected_parents.size() +
        result.offset_parents.size());
    for (const auto& parent : result.selected_parents)
        parent_ids.push_back(parent.source_parent_id);
    for (const auto& parent : result.offset_parents)
        parent_ids.push_back(parent.source_parent_id);
    std::sort(parent_ids.begin(), parent_ids.end());
    Require(std::adjacent_find(parent_ids.begin(), parent_ids.end()) ==
            parent_ids.end(),
        "Duplicate selected self-contact source parent");

    auto& counts = result.counts;
    counts.original_selected_parts = checked.selected_parts;
    counts.original_shell_parts = checked.shell_parts;
    counts.original_retained_shell_parts =
        checked.retained_shell_parts;
    counts.selected_shell_parts = result.selected_part_ids.size();
    counts.selected_offset_parts = result.offset_part_ids.size();
    counts.omitted_tire_parts = checked.excluded_shell_parts;
    counts.unsupported_non_shell_parts = checked.non_shell_parts;
    counts.original_shells = checked.shells;
    counts.selected_shell_parents = result.selected_parents.size();
    counts.selected_offset_parents = result.offset_parents.size();
    counts.omitted_tire_shells = checked.excluded_shells;
    counts.unsupported_solids = checked.solids;
    counts.unsupported_beams = checked.beams;
    counts.q4_parents = counts.qeph_parents + counts.qbat_parents;
    Require(counts.selected_shell_parents +
            counts.selected_offset_parents == checked.retained_shells &&
            counts.q4_parents + counts.t3_parents ==
                counts.selected_shell_parents &&
            !result.selected_parents.empty(),
        "Centered selected self-contact shell intersection is incomplete");

    if (temporary_bytes) {
        tl::util::BoundedArenaLayout scratch(std::size_t{24} << 30);
        tl::util::ArenaRegion unused;
        Require(scratch.Append<modelio::self_contact::SourceId>(
                    all_ids.capacity(), unused) &&
                scratch.Append<modelio::self_contact::SourceId>(
                    retained_ids.capacity(), unused) &&
                scratch.Append<std::size_t>(
                    retained_shell_counts.capacity(), unused) &&
                scratch.Append<std::pair<
                    modelio::self_contact::SourceId, std::size_t>>(
                    retained.capacity(), unused) &&
                scratch.Append<std::size_t>(
                    represented.capacity(), unused) &&
                scratch.Append<std::size_t>(
                    centered_represented.capacity(), unused) &&
                scratch.Append<std::size_t>(
                    offset_represented.capacity(), unused) &&
                scratch.Append<std::uint64_t>(
                    parent_ids.capacity(), unused),
            "Selected self-contact validation scratch overflows");
        *temporary_bytes = scratch.bytes();
    }
    return result;
}

std::size_t Difference(std::size_t total,
                       std::initializer_list<std::size_t> values,
                       const char* message) {
    for (const auto value : values) {
        output::Require(value <= total, message);
        total -= value;
    }
    return total;
}

std::size_t Add(std::initializer_list<std::size_t> values,
                std::size_t cap, const char* message) {
    tl::util::BoundedArenaLayout sum(cap);
    tl::util::ArenaRegion unused;
    for (const auto value : values)
        output::Require(sum.Append<std::byte>(value, unused), message);
    return sum.bytes();
}

bool Same(const contact::SelfContactActiveUseForecast& a,
          const contact::SelfContactActiveUseForecast& b) noexcept {
    return a.parents == b.parents && a.facets == b.facets &&
        a.vertices == b.vertices && a.edges == b.edges &&
        a.vertex_uses == b.vertex_uses &&
        a.edge_uses == b.edge_uses &&
        a.node_roles == b.node_roles && a.cin_rows == b.cin_rows &&
        a.cin_witnesses == b.cin_witnesses &&
        a.arena_bytes == b.arena_bytes &&
        a.retained_facet_bytes == b.retained_facet_bytes &&
        a.retained_rigid_bytes == b.retained_rigid_bytes &&
        a.retained_cin_bytes == b.retained_cin_bytes &&
        a.owned_payload_bytes == b.owned_payload_bytes &&
        a.startup_payload_bytes == b.startup_payload_bytes;
}

SourceForecast ComposeForecast(const fe::ShellPhysicalBinding& physical,
    const SourceInventory& inventory,
    const contact::SelfContactSurfaceForecast& surface,
    const contact::FixedContactFacetForecast& facets,
    const contact::SelfContactActiveUseForecast& active,
    std::size_t selection_scratch, Limits limits,
    std::size_t fixed_bytes) {
    SourceForecast result;
    result.surface = surface;
    result.facets = facets;
    result.active_uses = active;
    result.shared_physical_bytes = physical.owned_payload_bytes();
    output::Require(surface.retained_source_bytes +
            sizeof(fe::ShellPhysicalBinding) ==
            result.shared_physical_bytes,
        "S0 retained physical forecast differs from the actual binding");
    result.shared_rigid_bytes = active.retained_rigid_bytes;
    result.shared_cin_bytes = active.retained_cin_bytes;
    result.copied_inventory_bytes = InventoryBytes(inventory);

    const auto surface_new = Difference(surface.owned_payload_bytes,
        {surface.retained_source_bytes,
         sizeof(contact::SelfContactSurfaceBinding)},
        "Invalid incremental S0 forecast");
    const auto facet_new = Difference(facets.owned_payload_bytes,
        {facets.retained_source_bytes,
         sizeof(contact::FixedContactFacetBinding)},
        "Invalid incremental fixed-facet forecast");
    const auto active_new = Difference(active.owned_payload_bytes,
        {active.retained_facet_bytes, active.retained_rigid_bytes,
         active.retained_cin_bytes,
         sizeof(contact::SelfContactActiveUseBinding)},
        "Invalid incremental active-use forecast");
    result.new_binding_bytes = Add(
        {surface_new, facet_new, active_new}, limits.host_bytes,
        "Selected self-contact immutable bindings exceed host cap");
    result.retained_bytes = Add(
        {result.shared_physical_bytes, result.shared_rigid_bytes,
         result.shared_cin_bytes, result.copied_inventory_bytes,
         result.new_binding_bytes, fixed_bytes, std::size_t{256}},
        limits.host_bytes,
        "Selected self-contact retained source exceeds host cap");
    const auto surface_scratch = Difference(surface.startup_payload_bytes,
        {surface.owned_payload_bytes}, "Invalid S0 startup forecast");
    const auto facet_scratch = Difference(facets.startup_payload_bytes,
        {facets.owned_payload_bytes}, "Invalid facet startup forecast");
    const auto active_scratch = Difference(active.startup_payload_bytes,
        {active.owned_payload_bytes}, "Invalid active-use startup forecast");
    result.peak_temporary_bytes = std::max(
        {selection_scratch, surface_scratch, facet_scratch, active_scratch});
    result.peak_host_bytes = Add(
        {result.retained_bytes, result.peak_temporary_bytes},
        limits.host_bytes,
        "Selected self-contact startup exceeds host cap");
    return result;
}

Plan BuildPlan(const fe::ShellPhysicalBinding& physical,
    const modelio::self_contact::Data& original,
    contact::SelfContactActiveUseSource support,
    Config config, Limits limits, std::size_t fixed_bytes) {
    using output::Require;
    Require(config.facet_level <= 2 && limits.host_bytes &&
            limits.host_bytes <= (std::size_t{32} << 30),
        "Selected self-contact requires explicit level 0/1/2 and bounded host limits");
    Plan plan;
    plan.config = config;
    std::size_t selection_scratch = 0;
    plan.inventory = BuildInventory(
        physical, original, &selection_scratch);
    const contact::SelfContactSurfaceInput input{
        plan.inventory.selected_parents.data(),
        plan.inventory.selected_parents.size(),
        contact::SelfContactSurfaceProfile::
            FrictionlessReferenceThicknessShellSubsetV1};
    const auto surface = contact::SelfContactSurfaceBinding::Preflight(
        physical, input, limits.surface);
    Require(surface.report.status ==
            contact::SelfContactSurfaceStatus::Ok,
        surface.report.message);
    Require(plan.surface.Initialize(physical, input, limits.surface).status ==
            contact::SelfContactSurfaceStatus::Ok,
        "Selected self-contact S0 initialization failed after preflight");

    contact::FixedContactFacetConfig facet_config;
    facet_config.level = config.facet_level;
    const auto facets = contact::FixedContactFacetBinding::Preflight(
        plan.surface, facet_config, limits.facets);
    Require(facets.report.status == contact::FixedContactFacetStatus::Ok,
        facets.report.message);
    Require(plan.facets.Initialize(
                plan.surface, facet_config, limits.facets).status ==
            contact::FixedContactFacetStatus::Ok,
        "Selected fixed contact facets failed after preflight");
    plan.active = contact::SelfContactActiveUseBinding::Preflight(
        plan.facets, support, limits.active_uses);
    Require(plan.active.report.status ==
            contact::SelfContactActiveUseStatus::Ok,
        plan.active.report.message);
    plan.forecast = ComposeForecast(physical, plan.inventory,
        surface.forecast, facets.forecast, plan.active.forecast,
        selection_scratch, limits, fixed_bytes);
    return plan;
}

void AddSupport(const contact::SelfContactSupportClassification& source,
                SupportRoleCounts& target) {
    using Status = contact::SelfContactSupportStatus;
    ++target.total;
    switch (source.status) {
        case Status::AdmittedOrdinary:
            ++target.ordinary;
            break;
        case Status::AdmittedCinMaster:
            ++target.cin_master;
            break;
        case Status::UnsupportedCinSecondary:
            ++target.cin_secondary;
            break;
        case Status::CompleteRigidGroup:
            ++target.rigid;
            ++target.complete_rigid;
            break;
        case Status::AdmittedPartialOrMixedRigid:
            ++target.rigid;
            ++target.partial_or_mixed_rigid;
            break;
    }
}

SupportRoleCounts Sum(const SupportRoleCounts& a,
                      const SupportRoleCounts& b) {
    SupportRoleCounts value;
    value.ordinary = a.ordinary + b.ordinary;
    value.rigid = a.rigid + b.rigid;
    value.cin_master = a.cin_master + b.cin_master;
    value.cin_secondary = a.cin_secondary + b.cin_secondary;
    value.complete_rigid = a.complete_rigid + b.complete_rigid;
    value.partial_or_mixed_rigid =
        a.partial_or_mixed_rigid + b.partial_or_mixed_rigid;
    value.total = a.total + b.total;
    return value;
}

void Accumulate(contact::Q4CertifiedIntegral term,
                contact::Q4CertifiedIntegral& total) {
    contact::Q4IntegralInterval truth;
    contact::Q4CertifiedIntegral next;
    const auto value = total.value + term.value;
    output::Require(std::isfinite(value) &&
            contact::q4_bounds::Add(
                {total.lower, total.upper},
                {term.lower, term.upper}, &truth) &&
            contact::q4_bounds::Certify(value, truth, &next),
        "Selected self-contact reference-area census is unrepresentable");
    total = next;
}

Census BuildCensus(const SourceInventory& inventory,
                   const contact::SelfContactActiveUseBinding& active) {
    Census result;
    result.source = inventory.counts;
    const auto forecast = active.forecast();
    auto& topology = result.topology;
    topology.parents = forecast.parents;
    topology.q4_parents = inventory.counts.q4_parents;
    topology.t3_parents = inventory.counts.t3_parents;
    topology.facets = forecast.facets;
    topology.canonical_vertices = forecast.vertices;
    topology.canonical_edges = forecast.edges;
    topology.parent_local_vertex_uses = forecast.vertex_uses;
    topology.parent_local_edge_uses = forecast.edge_uses;

    for (const auto& parent : active.parents()) {
        Accumulate(parent.reference_area_m2,
            result.reference_area.total_parent_area_m2);
        if (parent.area_model ==
            contact::SelfContactReferenceAreaModel::
                Q4CenterAreaContactModel) {
            output::Require(parent.arity == 4,
                "Q4 contact-area model has a non-Q4 parent");
            Accumulate(parent.reference_area_m2,
                result.reference_area.q4_parent_area_m2);
        } else {
            output::Require(parent.arity == 3 &&
                    parent.area_model ==
                        contact::SelfContactReferenceAreaModel::
                            T3CertifiedNativeArea,
                "T3 contact-area model has a non-T3 parent");
            Accumulate(parent.reference_area_m2,
                result.reference_area.t3_parent_area_m2);
        }
    }
    for (const auto& use : active.vertex_uses()) {
        AddSupport(use.support, result.support.vertex_uses);
        Accumulate(use.directed_vf_area_m2,
            result.reference_area.directed_vertex_area_m2);
    }
    for (const auto& use : active.edge_uses())
        for (const auto& endpoint : use.endpoint_support)
            AddSupport(endpoint, result.support.edge_endpoints);
    result.support.all_weighted_supports = Sum(
        result.support.vertex_uses, result.support.edge_endpoints);
    const auto cin = active.cin();
    result.support.cin_rows = forecast.cin_rows;
    result.support.cin_witnesses = forecast.cin_witnesses;
    result.support.complete_static_cin_roster =
        cin.range_count == forecast.cin_rows &&
        cin.witness_count == forecast.cin_witnesses &&
        ((!forecast.cin_rows && !cin.model) ||
         (forecast.cin_rows && cin.model));
    result.support.runtime_activity_and_release_pending =
        forecast.cin_rows != 0;
    output::Require(topology.parents == active.parents().size() &&
            topology.facets == active.facet_uses().size() &&
            topology.canonical_vertices == active.vertices().size() &&
            topology.canonical_edges == active.edges().size() &&
            topology.parent_local_vertex_uses ==
                active.vertex_uses().size() &&
            topology.parent_local_edge_uses ==
                active.edge_uses().size() &&
            result.support.vertex_uses.total ==
                topology.parent_local_vertex_uses &&
            result.support.edge_endpoints.total ==
                2 * topology.parent_local_edge_uses &&
            result.support.complete_static_cin_roster,
        "Selected self-contact active-use census is incomplete");
    return result;
}

}  // namespace

struct SelectedSelfContactSource::Data {
    Data(Config selected_config, SourceForecast selected_forecast,
         SourceInventory selected_inventory, Census selected_census,
         const contact::SelfContactSurfaceBinding& selected_surface,
         const contact::FixedContactFacetBinding& selected_facets,
         const contact::SelfContactActiveUseBinding& selected_active)
        : config(selected_config), forecast(selected_forecast),
          inventory(std::move(selected_inventory)),
          census(selected_census), surface(selected_surface),
          facets(selected_facets), active(selected_active) {}
    Config config;
    SourceForecast forecast;
    SourceInventory inventory;
    Census census;
    contact::SelfContactSurfaceBinding surface;
    contact::FixedContactFacetBinding facets;
    contact::SelfContactActiveUseBinding active;
};

SourceForecast SelectedSelfContactSource::Preflight(
    const fe::ShellPhysicalBinding& physical,
    const modelio::self_contact::Data& original,
    contact::SelfContactActiveUseSource support,
    Config config, Limits limits) {
    return BuildPlan(physical, original, support, config, limits,
        sizeof(Data) + sizeof(SelectedSelfContactSource)).forecast;
}

SelectedSelfContactSource SelectedSelfContactSource::Prepare(
    const fe::ShellPhysicalBinding& physical,
    const modelio::self_contact::Data& original,
    contact::SelfContactActiveUseSource support,
    Config config, Limits limits) {
    auto plan = BuildPlan(physical, original, support, config, limits,
        sizeof(Data) + sizeof(SelectedSelfContactSource));
    contact::SelfContactActiveUseBinding active;
    const auto report = active.Initialize(
        plan.facets, support, limits.active_uses);
    output::Require(report.status ==
            contact::SelfContactActiveUseStatus::Ok,
        report.message);
    output::Require(Same(active.forecast(), plan.active.forecast),
        "Prepared active-use inventory differs from exact preflight");
    const auto census = BuildCensus(plan.inventory, active);
    auto next = std::make_shared<Data>(
        plan.config, plan.forecast, std::move(plan.inventory), census,
        plan.surface, plan.facets, active);
    output::Require(next->surface.MatchesPhysical(physical) &&
            next->facets.surface() &&
            next->facets.surface()->SharesStorage(next->surface) &&
            next->active.facets() &&
            next->active.facets()->SharesStorage(next->facets),
        "Prepared selected self-contact source identity differs");
    return SelectedSelfContactSource(std::move(next));
}

bool SelectedSelfContactSource::SharesStorage(
    const SelectedSelfContactSource& other) const noexcept {
    return data_ && data_ == other.data_;
}

bool SelectedSelfContactSource::MatchesPhysical(
    const fe::ShellPhysicalBinding& physical) const noexcept {
    if (!data_ || !data_->surface.physical()) return false;
    const auto& retained = *data_->surface.physical();
    return retained.catalog() == physical.catalog() &&
        retained.failure() == physical.failure() &&
        retained.mapping() == physical.mapping() &&
        retained.domain() == physical.domain() &&
        retained.execution() == physical.execution();
}

const Config& SelectedSelfContactSource::config() const noexcept {
    return data_->config;
}

const SourceForecast& SelectedSelfContactSource::forecast() const noexcept {
    return data_->forecast;
}

const SourceInventory& SelectedSelfContactSource::inventory() const noexcept {
    return data_->inventory;
}

const Census& SelectedSelfContactSource::census() const noexcept {
    return data_->census;
}

const contact::SelfContactSurfaceBinding&
SelectedSelfContactSource::surface() const noexcept {
    return data_->surface;
}

const contact::FixedContactFacetBinding&
SelectedSelfContactSource::facets() const noexcept {
    return data_->facets;
}

const contact::SelfContactActiveUseBinding&
SelectedSelfContactSource::active_uses() const noexcept {
    return data_->active;
}

}  // namespace crash::cases::vehicle_self_contact
