#pragma once
#include "CorrectedNodalSource.h"
#include "CoatedSource.h"
#include <optional>
namespace crash::cases::vehicle_self_contact::native::main_coefficients {
namespace n = tlfea::contact::radioss_type25;
namespace c = nodal_correction;
enum class Status { CoefficientValuesReady, InvalidInput, UnsupportedSource, ResourceLimit,
    NonfiniteResult, NeedsPrimaryOrientationPhase, NeedsNativeShellOrder, MissingMaterialContext };
enum class OwnerProof { UnresolvedEqualValues, UniqueBest, NativeCornerOrder, NativeMaterialGroupOrder };
enum class Coordinates { ClosedDirectKeywordV5Unprojected };
struct Report {
    Status status = Status::InvalidInput;
    std::string reason;
    std::uint64_t contact_element = 0, support_element = 0, conflicting_element = 0;
    std::size_t primary = SIZE_MAX;
};
struct PrimaryBinding {
    std::uint64_t contact_element = 0, contact_part = 0;
    std::uint64_t support_element = 0, support_part = 0; // Zero when owner remains unresolved.
    std::uint64_t solid_element = 0, solid_part = 0;
    std::size_t contact_physical = SIZE_MAX, support_physical = SIZE_MAX;
    std::size_t winner_begin = 0, winner_count = 0;
    std::uint32_t partner = 0; // One-based snapshot partner identity.
    coated::RoleState role = coated::RoleState::Unresolved;
    OwnerProof owner = OwnerProof::UnresolvedEqualValues;
};
struct CandidateOwner { std::uint64_t element = 0, part = 0; std::size_t physical = SIZE_MAX; };
struct Certificate {
    std::size_t primaries = 0, coated = 0, negative_support_volumes = 0;
    std::size_t unique_owners = 0, corner_owners = 0, material_group_owners = 0, unresolved_owners = 0;
    std::size_t physical_duplicate_groups = 0, physical_duplicate_rows = 0;
    bool orientation_identity = false, owners_complete = false, grouping_controls_certified = false;
};
struct Provenance {
    Coordinates coordinates = Coordinates::ClosedDirectKeywordV5Unprojected;
    n::UnitScale units;
    std::string input_digest, topology_digest, coefficient_digest;
    std::string property_digest, material_digest, import_digest;
};
struct Limits {
    std::size_t host_bytes = std::size_t{8}<<30, parts = 4096, metadata_bytes = 1u<<20;
    coated::Limits coating;
};
struct Forecast {
    coated::Forecast coating;
    std::size_t shared_corrected_reservation = 0, part_operands = 0, face_keys = 0;
    std::size_t output_bindings = 0, output_coefficients = 0, source_metadata = 0;
    std::size_t peak_bytes = 0;
};
struct Preparation;
// Native startup main K for the complete selected shell surface. This does not
// admit solid contact faces, gaps, erosion/removal or a runtime physical owner.
// Declared contact identity and mechanical support ownership remain distinct.
class SelectedShellMainSource {
 public:
    SelectedShellMainSource(const SelectedShellMainSource&) noexcept  =  default;
    SelectedShellMainSource(SelectedShellMainSource && other) noexcept : data_(other.data_) {}
    SelectedShellMainSource& operator = (const SelectedShellMainSource&)  =  delete;
    static Forecast Preflight(const c::CorrectedNodalSource&, const modelio::self_contact::OriginalSelection&, Limits = {});
    static Preparation Prepare(const c::CorrectedNodalSource&, const modelio::self_contact::OriginalSelection&,
        const std::string& original_member, const std::string& combine_member, Limits = {});
    const c::CorrectedNodalSource& corrected() const noexcept;
    const modelio::self_contact::OriginalSelection& selection() const noexcept;
    const coated::s::Snapshot& topology() const noexcept;
    const coated::s::Report& topology_report() const noexcept;
    tl::util::ConstView<double> coefficients() const noexcept;
    tl::util::ConstView<PrimaryBinding> bindings() const noexcept;
    tl::util::ConstView<CandidateOwner> possible_owners() const noexcept;
    const Certificate& certificate() const noexcept;
    const Provenance& provenance() const noexcept;
    const Forecast& forecast() const noexcept;
 private:
    struct Data;
    explicit SelectedShellMainSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
struct Preparation { Report report; std::optional<SelectedShellMainSource> source; };
} // namespace crash::cases::vehicle_self_contact::native::main_coefficients
