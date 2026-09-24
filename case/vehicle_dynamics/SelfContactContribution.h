#pragma once

#include "SelfContactObservation.h"
#include "lib_src/elements/ShellBatchPublication.h"

namespace crash::cases::vehicle_self_contact {
class VehicleSelfContactSetup;
struct RuntimeForecast;
}

namespace crash::cases::vehicle_dynamics::detail {

// One private optional self-contact seam. The concrete stage retains the
// immutable app setup, transaction, and all receipt authority.
class SelfContactContribution {
  public:
    virtual ~SelfContactContribution() = default;
    virtual void Assemble(
        tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
        const tl::fea::NodalAssemblyView&, SelfContactObservation&) = 0;
    virtual void SealCandidate(
        tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
        const tl::fea::ShellPhysicalDiagnostics&,
        const tl::fea::NodalPreparedView&, SelfContactObservation&) = 0;
    virtual tl::fea::ShellPhysicalScratchReceiptRoster
        scratch_receipts() const noexcept = 0;
    virtual void Discard() noexcept = 0;
    // Value-only observation; never a publication receipt or acceptance input.
    virtual tlfea::contact::SelfContactTransactionDiagnostics diagnostics() const noexcept {return {};}
    virtual const vehicle_self_contact::VehicleSelfContactSetup&
        setup() const noexcept = 0;
    virtual const vehicle_self_contact::RuntimeForecast&
        forecast() const noexcept = 0;
    virtual tlfea::contact::SelfContactTransactionAllocationInfo
        allocations() const noexcept = 0;
};

}  // namespace crash::cases::vehicle_dynamics::detail
