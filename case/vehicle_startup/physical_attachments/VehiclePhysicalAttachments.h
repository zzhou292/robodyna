#pragma once
#include "../physical_model/VehiclePhysicalModel.h"
#include "../TiedCinWitnessRoster.h"

namespace crash::cases::vehicle_startup::physical_attachments {
struct Limits {
    std::size_t host_bytes = std::size_t{8} << 30;
    TiedCinAttachmentLimits attachments;
    TiedCinWitnessLimits witnesses;
};
struct Forecast {
    std::size_t physical_source = 0, attachment_reservation = 0;
    std::size_t witness_reservation = 0, fixed_bytes = 0, total_bytes = 0;
};
// Maps the observed original CIN disposition and actual shell support onto the
// complete retained mechanical domain. This immutable composition grants no
// current geometry, runtime activity, force, DOF, or joint admission.
class VehiclePhysicalAttachments {
  public:
    static Forecast Preflight(const physical_model::VehiclePhysicalModel&,
                              const TiedSearchPostKinChk&, Limits = {});
    static VehiclePhysicalAttachments Prepare(const physical_model::VehiclePhysicalModel&,
                                              const TiedSearchPostKinChk&, Limits = {});
    const physical_model::VehiclePhysicalModel& physical() const noexcept;
    const TiedCinWitnessRoster& witnesses() const noexcept;
    const TiedCinAttachments& attachments() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Storage;
    explicit VehiclePhysicalAttachments(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::cases::vehicle_startup::physical_attachments
