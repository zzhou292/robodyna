#pragma once
#include "TiedShellPacking.h"
#include <array>

namespace crash::modelio::tied_shell {
inline constexpr const char* SearchGeometryPolicy =
    "openradioss_a62b27e_type2_original_weld_zero_secondary_equivalent_shells_v1";
enum class SearchShellFamily : std::uint32_t { Q4, T3 };
struct SearchGeometryLimits {
    std::size_t host_bytes = 512 * 1024 * 1024;
    std::size_t member_bytes = 64 * 1024 * 1024;
    std::size_t metadata_bytes = 8 * 1024 * 1024;
    std::uint32_t masters = 524288, nodes = 1048576, shells = 524288;
    std::uint32_t parts = 1024, matches_per_master = 32;
};
struct SearchProperty {
    SourceId part_id = 0, section_id = 0, material_id = 0;
    std::array<std::size_t, 3> sources{}; // PART, SECTION, MAT evidence rows.
    double geometry_thickness = 0, rank_modulus = 0;
    double part_override = 0, element_override = 0;
    bool rank_modulus_available = false;
};
struct SearchShellMatch {
    std::uint32_t canonical_shell_row = 0, property = 0;
    SearchShellFamily family = SearchShellFamily::Q4;
    bool equivalent_winner = false;
};
struct SearchMasterGeometry {
    std::uint32_t declaration_row = 0, first_match = 0, match_count = 0;
    SearchShellFamily family = SearchShellFamily::Q4;
    std::array<std::uint32_t, 4> working_nodes{};
    double bounds_thickness = 0, projection_thickness = 0;
};
struct SearchGeometryData {
    double working_length_to_m = 0;
    std::vector<std::uint32_t> canonical_nodes;
    std::vector<std::array<double, 3>> working_positions;
    std::vector<std::uint32_t> secondary_working_nodes; // Qualified NSV order.
    std::vector<SearchMasterGeometry> masters; // Qualified IRECT order.
    std::vector<SearchShellMatch> matches; // Canonical evidence order, not native incidence order.
    std::vector<SearchProperty> properties;
    std::vector<SourceEvidence> sources;
    std::size_t startup_budget_bytes = 0, owned_payload_bytes = 0;
    std::size_t multiple_match_masters = 0, equivalent_winner_occurrences = 0;
    std::size_t source_roundtrip_changed_components = 0;
    // Per-secondary thickness and this interface maximum are both zero under
    // the checked complete-original-shell incidence policy.
    double maximum_secondary_shell_thickness = 0;
};
class TiedShellSearchGeometry {
  public:
    static std::size_t Forecast(const TiedShellPacking&, SearchGeometryLimits = {});
    static TiedShellSearchGeometry Prepare(const TiedShellPacking&,
        const std::string& original_member, SearchGeometryLimits = {});
    TiedShellSearchGeometry(const TiedShellSearchGeometry&) noexcept = default;
    TiedShellSearchGeometry(TiedShellSearchGeometry&& other) noexcept : storage_(other.storage_) {}
    TiedShellSearchGeometry& operator=(const TiedShellSearchGeometry&) = delete;
    TiedShellSearchGeometry& operator=(TiedShellSearchGeometry&&) = delete;
    const TiedShellPacking& packing() const noexcept;
    const SearchGeometryData& data() const noexcept;
    bool PhysicalOwnNode(std::size_t secondary, std::size_t master) const;
  private:
    struct Storage;
    explicit TiedShellSearchGeometry(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::tied_shell
