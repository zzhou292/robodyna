#pragma once

#include "../VehicleSelfContactStartup.h"
#include "case/vehicle_dynamics/SelfContactContribution.h"

namespace crash::cases::vehicle_self_contact::detail {

class SelfContactStages final
    : public vehicle_dynamics::detail::SelfContactContribution {
  public:
    SelfContactStages(
        VehicleSelfContactStartup&& value, RuntimeForecast budget)
        : contact(std::move(value)), budget_(budget) {}

    VehicleSelfContactStartup contact;
    RuntimeForecast budget_;
    tlfea::contact::SelfContactAcceptedAssemblyReceipt accepted_;
    tlfea::contact::SelfContactTransactionReceipt receipt_;

    void Assemble(
        tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
        const tl::fea::NodalAssemblyView&,
        vehicle_dynamics::SelfContactObservation&) override;
    void SealCandidate(
        tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
        const tl::fea::ShellPhysicalDiagnostics&,
        const tl::fea::NodalPreparedView&,
        vehicle_dynamics::SelfContactObservation&) override;
    tl::fea::ShellPhysicalScratchReceiptRoster
        scratch_receipts() const noexcept override;
    void Discard() noexcept override;
    tlfea::contact::SelfContactTransactionDiagnostics diagnostics() const noexcept override;
    const VehicleSelfContactSetup& setup() const noexcept override {
        return contact.setup();
    }
    const RuntimeForecast& forecast() const noexcept override {
        return budget_;
    }
    tlfea::contact::SelfContactTransactionAllocationInfo
        allocations() const noexcept override {
        return contact.allocations();
    }
};

}  // namespace crash::cases::vehicle_self_contact::detail
