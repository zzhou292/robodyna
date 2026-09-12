#pragma once
#include "Types.h"
#include "../physical_attachments/VehiclePhysicalAttachments.h"
#include "../joints/VehicleJointModel.h"

namespace crash::cases::vehicle_startup::connectivity {
using physical_attachments::VehiclePhysicalAttachments;
// Static weak incidence and potential-transfer partitions. Source geometry,
// physical coefficients and complete original CIN identities remain owned by
// the input handle. These labels prove neither current activity nor DOF rank.
class VehicleConnectivity {
  public:
    static Forecast Preflight(const VehiclePhysicalAttachments&, Limits = {});
    static VehicleConnectivity Prepare(const VehiclePhysicalAttachments&, Limits = {});
    static Forecast PreflightWithJoints(const VehiclePhysicalAttachments&,
                                       const joints::VehicleJointModel&, Limits = {});
    static VehicleConnectivity PrepareWithJoints(const VehiclePhysicalAttachments&,
                                               const joints::VehicleJointModel&, Limits = {});
    const VehiclePhysicalAttachments& source() const noexcept;
    const joints::VehicleJointModel* joint_model() const noexcept;
    const Data& data() const noexcept;
    const Forecast& forecast() const noexcept;
    Obligation required_joints() const noexcept {
        return joint_model() ? Obligation::PreparedSourceOperators : Obligation::Pending;
    }
    Obligation omitted_source_and_auxiliary_policy() const noexcept { return Obligation::Pending; }
    Obligation current_cin_activity_and_release() const noexcept { return Obligation::Pending; }
  private:
    struct Storage;
    static Forecast PreflightImpl(const VehiclePhysicalAttachments&, const joints::VehicleJointModel*, Limits);
    static VehicleConnectivity PrepareImpl(const VehiclePhysicalAttachments&, const joints::VehicleJointModel*, Limits);
    explicit VehicleConnectivity(std::shared_ptr<const Storage> value) : storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
// Bounded complete text; callers publish through existing ArtifactIO create-only
// convention. No artifact is written until complete construction succeeds.
std::string ReportJson(const VehicleConnectivity&);
} // namespace crash::cases::vehicle_startup::connectivity
