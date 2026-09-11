#pragma once
#include "modelio/vehicle_source/VehicleSourcePlan.h"

#include "lib_src/elements/qeph/QephData.h"
#include "lib_src/elements/t3/T3Data.h"
#include "lib_src/elements/qbat/QbatTypes.h"

namespace crash::modelio::vehicle { class VehicleSectionResolution; }

namespace crash::cases::vehicle_startup {
using modelio::vehicle::VehicleSourcePlan;
using modelio::vehicle::VehicleSectionResolution;
enum class ReferenceFamily { None, Qeph, T3, Qbat };
enum class ReferenceStatus {
    UnresolvedDeclaration, Success, InvalidInput, UnsupportedGeometry, NonfiniteResult, InvalidReference
};
const char* Name(ReferenceStatus) noexcept;
struct ReferenceRow {
    std::uint64_t element_id=0, part_id=0, material_id=0, section_id=0;
    std::uint32_t source_line=0, canonical_parent=0, part_index=0;
    ReferenceFamily family=ReferenceFamily::None;
    ReferenceStatus status=ReferenceStatus::UnresolvedDeclaration;
    std::size_t reference_index=SIZE_MAX;
};
struct ReferenceCounts {
    std::size_t parents=0, unresolved=0, attempted=0, succeeded=0, rejected=0;
    std::size_t qeph_attempted=0, t3_attempted=0, qeph_succeeded=0, t3_succeeded=0;
    std::size_t qbat_attempted=0, qbat_succeeded=0;
    std::array<std::size_t,6> status{}; // Indexed by ReferenceStatus.
};
enum class ReferenceProfile { Legacy, ResolvedSections };
struct ReferenceLimits {
    std::size_t parents=524288, nodes=524288, host_bytes=512*1024*1024;
    ReferenceProfile profile=ReferenceProfile::Legacy;
    static ReferenceLimits ResolvedSections() noexcept {
        ReferenceLimits limits;
        limits.host_bytes=768*1024*1024;
        limits.profile=ReferenceProfile::ResolvedSections;
        return limits;
    }
};
struct ReferenceForecast {
    // Source startup is an overinclusive bound for retained immutable source:
    // it includes already-freed parser scratch. Shared backing is charged once.
    std::size_t source_bound_bytes=0, reference_capacity_bytes=0, row_bytes=0;
    std::size_t decode_bytes=0, decode_temporary_bytes=0, fixed_bytes=0, total_bytes=0;
    std::size_t qeph_capacity=0, t3_capacity=0, qbat_capacity=0;
};
ReferenceForecast ForecastReferences(const VehicleSourcePlan&,ReferenceLimits={});
ReferenceForecast ForecastReferences(const VehicleSectionResolution&,
                                    ReferenceLimits=ReferenceLimits::ResolvedSections());
// Complete source assessment, never a partially admitted shell binding/owner.
// Native rejection is a retained row, not a factory error; malformed packing or
// resource admission throws before publication. No nodal/global M/J is summed.
class VehicleShellReferences {
  public:
    static VehicleShellReferences Prepare(const VehicleSourcePlan&,ReferenceLimits={});
    static VehicleShellReferences Prepare(const VehicleSectionResolution&,
                                         ReferenceLimits=ReferenceLimits::ResolvedSections());
    VehicleShellReferences(const VehicleShellReferences&) noexcept=default;
    VehicleShellReferences(VehicleShellReferences&& other) noexcept:data_(other.data_) {}
    VehicleShellReferences& operator=(const VehicleShellReferences&)=delete;
    VehicleShellReferences& operator=(VehicleShellReferences&&)=delete;
    const VehicleSourcePlan& source() const noexcept;
    // Null for the unchanged historical source-plan-only preparation.
    const VehicleSectionResolution* resolution() const noexcept;
    const std::vector<ReferenceRow>& rows() const noexcept;
    const ReferenceCounts& counts() const noexcept;
    const ReferenceForecast& forecast() const noexcept;
    const ReferenceRow* first_error() const noexcept;
    // Null for wrong family, unresolved/rejected parent, or invalid row index.
    const tl::fea::qeph::ReferenceData* qeph(std::size_t row) const noexcept;
    const tl::fea::t3::ReferenceData* t3(std::size_t row) const noexcept;
    const tl::fea::qbat::Reference* qbat(std::size_t row) const noexcept;
  private:
    friend ReferenceForecast ForecastReferences(const VehicleSourcePlan&,ReferenceLimits);
    friend ReferenceForecast ForecastReferences(const VehicleSectionResolution&,ReferenceLimits);
    struct Data;
    explicit VehicleShellReferences(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_startup
