#include "Contribution.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateFailureFixture.h"

#include "output/ArtifactIO.h"
#include <cstdint>
#include <limits>

namespace crash::cases::vehicle_run::observed {
namespace {
constexpr auto MaximumObserverContext =
    vehicle_startup::shell_execution::self_contact_test::FailureCaptureHostCap;
static_assert(sizeof(ObservedContribution) <= ContributionHostReserve);
}

std::unique_ptr<Contribution> ObservedContribution::Wrap(
    std::unique_ptr<Contribution>& original, Observer observer, SealOperation seal) {
    const auto address = reinterpret_cast<std::uintptr_t>(observer.context);
    output::Require(original && !dynamic_cast<ObservedContribution*>(original.get()) &&
        seal && observer.capture && observer.context && observer.context_bytes &&
        observer.context_bytes <= MaximumObserverContext &&
        observer.context_bytes <= std::numeric_limits<std::uintptr_t>::max() - address,
        "Observed contribution requires one concrete stage and bounded callback context");
    auto result = std::unique_ptr<ObservedContribution>(
        new ObservedContribution(observer, seal));
    const auto descriptor = reinterpret_cast<std::uintptr_t>(&result->observer_);
    output::Require(descriptor + sizeof(Observer) <= address ||
                        address + observer.context_bytes <= descriptor,
                    "Observer descriptor must not overlap its callback output context");
    result->original_ = std::move(original);
    return result;
}

void ObservedContribution::Assemble(
    tl::fea::FENodalState& owner, const tl::fea::NodalTrialToken& token,
    const tl::fea::NodalAssemblyView& assembly,
    vehicle_dynamics::SelfContactObservation& observation) {
    original_->Assemble(owner, token, assembly, observation);
}
void ObservedContribution::SealCandidate(
    tl::fea::FENodalState& owner, const tl::fea::NodalTrialToken& token,
    const tl::fea::ShellPhysicalDiagnostics& common,
    const tl::fea::NodalPreparedView& prepared,
    vehicle_dynamics::SelfContactObservation& observation) {
    seal_(*original_, owner, token, common, prepared, observation, observer_);
}
tl::fea::ShellPhysicalScratchReceiptRoster
ObservedContribution::scratch_receipts() const noexcept { return original_->scratch_receipts(); }
void ObservedContribution::Discard() noexcept { original_->Discard(); }
const vehicle_self_contact::VehicleSelfContactSetup&
ObservedContribution::setup() const noexcept { return original_->setup(); }
const vehicle_self_contact::RuntimeForecast&
ObservedContribution::forecast() const noexcept { return original_->forecast(); }
tlfea::contact::SelfContactTransactionAllocationInfo
ObservedContribution::allocations() const noexcept { return original_->allocations(); }

} // namespace crash::cases::vehicle_run::observed
