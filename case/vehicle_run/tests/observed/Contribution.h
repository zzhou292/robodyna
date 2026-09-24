#pragma once

#include "case/vehicle_dynamics/SelfContactContribution.h"
#include "lib_src/collision/self_contact_transaction/CandidateFailureCapture.h"

#include <memory>

namespace crash::cases::vehicle_run::observed {

using Contribution = vehicle_dynamics::detail::SelfContactContribution;
using Observer = tlfea::contact::self_contact_transaction::CandidateFailureObserver;
inline constexpr std::size_t ContributionHostReserve = 1024;

// Qualification-only dispatch; no alternate mechanics implementation or owner.
using SealOperation = void (*)(
    Contribution&, tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
    const tl::fea::ShellPhysicalDiagnostics&, const tl::fea::NodalPreparedView&,
    vehicle_dynamics::SelfContactObservation&, const Observer&);

class ObservedContribution final : public Contribution {
  public:
    // Allocate before moving original. Invalid inputs/allocation failure leave
    // its concrete stage and all physical authority in the caller's ownership.
    static std::unique_ptr<Contribution> Wrap(
        std::unique_ptr<Contribution>& original, Observer, SealOperation);
    void Assemble(tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
                  const tl::fea::NodalAssemblyView&,
                  vehicle_dynamics::SelfContactObservation&) override;
    void SealCandidate(tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
                       const tl::fea::ShellPhysicalDiagnostics&,
                       const tl::fea::NodalPreparedView&,
                       vehicle_dynamics::SelfContactObservation&) override;
    tl::fea::ShellPhysicalScratchReceiptRoster scratch_receipts() const noexcept override;
    void Discard() noexcept override;
    tlfea::contact::SelfContactTransactionDiagnostics diagnostics() const noexcept override;
    const vehicle_self_contact::VehicleSelfContactSetup& setup() const noexcept override;
    const vehicle_self_contact::RuntimeForecast& forecast() const noexcept override;
    tlfea::contact::SelfContactTransactionAllocationInfo allocations() const noexcept override;
  private:
    ObservedContribution(Observer observer, SealOperation seal)
        : observer_(observer), seal_(seal) {}
    std::unique_ptr<Contribution> original_;
    // Descriptor storage is distinct from the external callback context.
    Observer observer_;
    SealOperation seal_;
};

} // namespace crash::cases::vehicle_run::observed
