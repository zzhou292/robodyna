#pragma once

#include "modelio/self_contact/OriginalSelection.h"
#include "lib_src/collision/SelfContactActiveUseBinding.h"
#include <array>
#include <memory>
#include <vector>

namespace crash::cases::vehicle_self_contact {
namespace contact = tlfea::contact;

struct Config {
    unsigned facet_level = 0;
};

struct Limits {
    std::size_t host_bytes = std::size_t{24} << 30;
    contact::SelfContactSurfaceLimits surface =
        contact::SelfContactSurfaceLimits::Vehicle();
    contact::FixedContactFacetLimits facets =
        contact::FixedContactFacetLimits::Vehicle();
    contact::SelfContactActiveUseLimits active_uses =
        contact::SelfContactActiveUseLimits::Vehicle();
};

struct SourceCounts {
    std::size_t original_selected_parts = 0;
    std::size_t original_shell_parts = 0;
    std::size_t original_retained_shell_parts = 0;
    std::size_t selected_shell_parts = 0;
    std::size_t selected_offset_parts = 0;
    std::size_t omitted_tire_parts = 0;
    std::size_t unsupported_non_shell_parts = 0;
    std::size_t original_shells = 0;
    std::size_t selected_shell_parents = 0;
    std::size_t selected_offset_parents = 0;
    std::size_t omitted_tire_shells = 0;
    std::size_t unsupported_solids = 0;
    std::size_t unsupported_beams = 0;
    std::size_t qeph_parents = 0;
    std::size_t t3_parents = 0;
    std::size_t qbat_parents = 0;
    std::size_t q4_parents = 0;
};

struct SourceInventory {
    // Exact source order from OriginalSelection for part IDs; physical catalog
    // order for parent rows. Only centered parents enter selected_parents.
    std::vector<modelio::self_contact::SourceId> retained_shell_part_ids;
    std::vector<modelio::self_contact::SourceId> selected_part_ids;
    std::vector<modelio::self_contact::SourceId> offset_part_ids;
    std::vector<modelio::self_contact::SourceId> omitted_tire_part_ids;
    std::vector<modelio::self_contact::SourceId> unsupported_non_shell_part_ids;
    std::vector<modelio::self_contact::SourceId> unsupported_solid_part_ids;
    std::vector<modelio::self_contact::SourceId> unsupported_beam_part_ids;
    std::vector<contact::SelfContactParentSelection> selected_parents;
    std::vector<contact::SelfContactParentSelection> offset_parents;
    SourceCounts counts;
};

struct TopologyCensus {
    std::size_t parents = 0, q4_parents = 0, t3_parents = 0;
    std::size_t facets = 0;
    std::size_t canonical_vertices = 0, canonical_edges = 0;
    std::size_t parent_local_vertex_uses = 0;
    std::size_t parent_local_edge_uses = 0;
};

struct SupportRoleCounts {
    std::size_t ordinary = 0;
    std::size_t rigid = 0;
    std::size_t cin_master = 0;
    std::size_t cin_secondary = 0;
    std::size_t complete_rigid = 0;
    std::size_t partial_or_mixed_rigid = 0;
    std::size_t total = 0;
};

struct SupportCensus {
    // Source support classifications, not contacts or force coverage.
    SupportRoleCounts vf_parent_local_vertex_use_support_occurrences;
    // Stored parent-local EE endpoints; nonlocal pairing/force area is unadmitted.
    SupportRoleCounts ee_stored_endpoint_support_occurrences;
    SupportRoleCounts combined_stored_support_occurrences;
    std::size_t cin_rows = 0, cin_witnesses = 0;
    // Static complete witness support is retained. It is not a runtime tied
    // exclusion: current master activity and release remain pending.
    bool complete_static_cin_roster = false;
    bool runtime_activity_and_release_pending = false;
    bool parent_activity_pending = true;
    std::size_t parent_activity_pending_parents = 0;
    bool same_parent_regularity_pending = true;
    std::size_t same_parent_regularity_pending_edge_uses = 0;
    bool nonlocal_ee_force_area_pending = true;
    std::size_t nonlocal_ee_force_area_pending_edge_uses = 0;
    std::size_t runtime_tied_exclusions = 0;
};

struct ReferenceAreaCensus {
    contact::Q4CertifiedIntegral q4_parent_area_m2;
    contact::Q4CertifiedIntegral t3_parent_area_m2;
    contact::Q4CertifiedIntegral total_parent_area_m2;
    contact::Q4CertifiedIntegral directed_vertex_area_m2;
    contact::Q4CertifiedIntegral twice_directed_vertex_area_m2;
    contact::Q4IntegralInterval directed_partition_difference_m2;
    bool directed_partition_certified = false;
};

struct RuntimeCoefficientCensus {
    // Original FS/FD/DC/SOFT/IGNORE remain source provenance only.
    std::size_t applied_source_friction_fields = 0;
    std::size_t applied_source_damping_fields = 0;
    std::size_t applied_source_soft_fields = 0;
};

struct Census {
    SourceCounts source;
    TopologyCensus topology;
    SupportCensus support;
    ReferenceAreaCensus reference_area;
    RuntimeCoefficientCensus runtime_coefficients;
};

struct SourceForecast {
    contact::SelfContactSurfaceForecast surface;
    contact::FixedContactFacetForecast facets;
    contact::SelfContactActiveUseForecast active_uses;
    // Reservations/upper bounds, not measured resident memory. In particular,
    // shared CIN/source values may include prior-construction retired scratch.
    std::size_t declared_inventory_reservation_bytes = 0;
    std::size_t declared_validation_scratch_reservation_bytes = 0;
    std::size_t shared_physical_reservation_bytes = 0;
    std::size_t shared_rigid_reservation_bytes = 0;
    std::size_t shared_cin_reservation_bytes = 0;
    std::size_t copied_inventory_capacity_bytes = 0;
    std::size_t new_binding_reservation_bytes = 0;
    std::size_t retained_reservation_bytes = 0;
    std::size_t peak_temporary_reservation_bytes = 0;
    std::size_t peak_host_reservation_bytes = 0;
};

// Immutable selected-shell source. This lower seam accepts already-authenticated
// source values so small host fixtures can qualify selection/composition. The
// app-owned VehicleSelfContactSetup below is the runtime-admission authority.
// No force coefficient, owner, accepted clock, activity history or DOF exists.
class SelectedSelfContactSource {
  public:
    static SourceForecast Preflight(const tl::fea::ShellPhysicalBinding&,
        const modelio::self_contact::Data&,
        contact::SelfContactActiveUseSource = {}, Config = {}, Limits = {});
    static SelectedSelfContactSource Prepare(
        const tl::fea::ShellPhysicalBinding&,
        const modelio::self_contact::Data&,
        contact::SelfContactActiveUseSource = {}, Config = {}, Limits = {});
    SelectedSelfContactSource(const SelectedSelfContactSource&) noexcept = default;
    SelectedSelfContactSource(SelectedSelfContactSource&& other) noexcept
        : data_(other.data_) {}
    SelectedSelfContactSource& operator=(
        const SelectedSelfContactSource&) = delete;
    bool prepared() const noexcept { return bool(data_); }
    bool SharesStorage(const SelectedSelfContactSource&) const noexcept;
    bool MatchesPhysical(const tl::fea::ShellPhysicalBinding&) const noexcept;
    const Config& config() const noexcept;
    const SourceForecast& forecast() const noexcept;
    const SourceInventory& inventory() const noexcept;
    const Census& census() const noexcept;
    const contact::SelfContactSurfaceBinding& surface() const noexcept;
    const contact::FixedContactFacetBinding& facets() const noexcept;
    const contact::SelfContactActiveUseBinding& active_uses() const noexcept;

  private:
    struct Data;
    explicit SelectedSelfContactSource(std::shared_ptr<const Data> value)
        : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};

}  // namespace crash::cases::vehicle_self_contact
