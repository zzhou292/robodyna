#pragma once

#include "SelectedSelfContactSource.h"
#include "case/vehicle_runtime/SourceIdentity.h"

namespace crash::cases::vehicle_self_contact {

struct SetupLimits {
    std::size_t host_bytes = std::size_t{32} << 30;
    Limits selected;
};

struct SetupForecast {
    SourceForecast selected;
    // Reservations/upper bounds, not measured resident memory. Shared source
    // values can include prior-construction/retired CIN allocations.
    std::size_t shared_vehicle_source_reservation_bytes = 0;
    std::size_t retained_original_selection_reservation_bytes = 0;
    std::size_t selected_incremental_reservation_bytes = 0;
    std::size_t retained_setup_reservation_bytes = 0;
    std::size_t peak_temporary_reservation_bytes = 0;
    std::size_t peak_host_reservation_bytes = 0;
};

class SetupIdentity {
  public:
    SetupIdentity() = default;
    explicit operator bool() const noexcept { return bool(backing_); }
    bool Matches(const SetupIdentity& other) const noexcept {
        return backing_ == other.backing_;
    }

  private:
    explicit SetupIdentity(std::shared_ptr<const void> value)
        : backing_(std::move(value)) {}
    std::shared_ptr<const void> backing_;
    friend class VehicleSelfContactSetup;
};

// App-owned immutable original-selection intersection and fixed physical
// feature source. Original FS/FD/DC/SOFT fields remain available only through
// original(); this setup creates no runtime coefficient, force, owner or clock.
class VehicleSelfContactSetup {
  public:
    static SetupForecast Preflight(const vehicle_runtime::Execution&,
        const vehicle_runtime::Attachments&,
        const modelio::self_contact::OriginalSelection&,
        Config = {}, SetupLimits = {});
    static VehicleSelfContactSetup Prepare(
        const vehicle_runtime::Execution&,
        const vehicle_runtime::Attachments&,
        const modelio::self_contact::OriginalSelection&,
        Config = {}, SetupLimits = {});
    VehicleSelfContactSetup(
        const VehicleSelfContactSetup&) noexcept = default;
    VehicleSelfContactSetup(VehicleSelfContactSetup&& other) noexcept
        : data_(other.data_) {}
    VehicleSelfContactSetup& operator=(
        const VehicleSelfContactSetup&) = delete;
    SetupIdentity identity() const noexcept { return SetupIdentity(data_); }
    bool SharesStorage(const VehicleSelfContactSetup&) const noexcept;
    bool MatchesSource(const vehicle_runtime::Execution&,
        const vehicle_runtime::Attachments&,
        const modelio::self_contact::OriginalSelection&) const noexcept;
    const vehicle_runtime::Execution& execution() const noexcept;
    const vehicle_runtime::Attachments& attachments() const noexcept;
    const modelio::self_contact::OriginalSelection& original() const noexcept;
    const tl::fea::ShellPhysicalBinding& physical() const noexcept;
    const SelectedSelfContactSource& selected() const noexcept;
    const Config& config() const noexcept;
    const SetupForecast& forecast() const noexcept;
    const SourceInventory& inventory() const noexcept;
    const SourceCounts& counts() const noexcept;
    const Census& census() const noexcept;
    const contact::SelfContactSurfaceBinding& surface() const noexcept;
    const contact::FixedContactFacetBinding& facets() const noexcept;
    const contact::SelfContactActiveUseBinding& active_uses() const noexcept;

  private:
    struct Data;
    explicit VehicleSelfContactSetup(std::shared_ptr<const Data> value)
        : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};

}  // namespace crash::cases::vehicle_self_contact
