#pragma once
#include "EnvelopeExecutionSource.h"
#include "case/vehicle_startup/TiedCinAttachments.h"
#include "case/vehicle_startup/TiedCinWitnessRoster.h"
#include "case/vehicle_runtime/Packing.h"
#include "modelio/type45/VehicleType45Source.h"
#include "lib_src/elements/type45/Model.h"

namespace crash::cases::vehicle_wall::native {
struct EnvelopeOwnerLimits {
    std::size_t host_bytes = std::size_t{20}*1000*1000*1000;
    vehicle_startup::TiedCinAttachmentLimits attachments;
    vehicle_startup::TiedCinWitnessLimits witnesses;
    tl::fea::type45::ModelLimits joints;
};
struct EnvelopeOwnerForecast {
    std::size_t execution_source = 0, cin = 0, witness_reservation = 0;
    std::size_t joint_source = 0, joint_model = 0, joint_packing = 0;
    std::size_t roles = 0, owner_packing = 0, fixed = 0, peak_bytes = 0;
    std::size_t previous_execution_peak = 0, current_phase = 0;
};
// Complete immutable pre-owner source graph. It creates neither a CUDA owner
// nor a participant/clock/contact receipt. Every later batch must use startup()
// and the actual physical/domain/CIN/rigid/joint objects exposed here.
class EnvelopeOwnerSource {
  public:
    static EnvelopeOwnerForecast Preflight(const EnvelopeExecutionSource&,
        const vehicle_startup::TiedSearchPostKinChk&,const modelio::type45::VehicleType45Source&,
        EnvelopeOwnerLimits = {});
    static EnvelopeOwnerSource Prepare(const EnvelopeExecutionSource&,
        const vehicle_startup::TiedSearchPostKinChk&,const modelio::type45::VehicleType45Source&,
        EnvelopeOwnerLimits = {});
    const EnvelopeExecutionSource& execution_source() const noexcept;
    const tl::fea::ShellPhysicalBinding& physical() const noexcept;
    const vehicle_startup::TiedCinAttachments& attachments() const noexcept;
    const vehicle_startup::TiedCinWitnessRoster& witnesses() const noexcept;
    const tl::fea::type45::Model& joints() const noexcept;
    const modelio::type45::VehicleType45Source& joint_source() const noexcept;
    const std::vector<std::uint32_t>& joint_source_rows() const noexcept;
    const vehicle_runtime::SourceRoles& roles() const noexcept;
    const tl::fea::ShellBatchStartup& startup() const noexcept;
    const EnvelopeOwnerForecast& forecast() const noexcept;
    // Retained graph only: constructor scratch and unused module reservations
    // remain in forecast().peak_bytes and are not charged as live runtime data.
    std::size_t retained_host_upper_bound(std::size_t cap) const;
    tl::fea::NodalCinWitnessSource witness_source() const noexcept;
    vehicle_runtime::detail::OwnerPacking PackOwner(std::size_t cap) const;
  private:
    struct Data;
    explicit EnvelopeOwnerSource(std::shared_ptr<const Data> value) : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
}
