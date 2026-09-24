#include "Contribution.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "case/vehicle_self_contact/VehicleSelfContactStartup.h"

#include <gtest/gtest.h>
#include <array>
#include <exception>
#include <limits>
#include <string>

namespace crash::cases::vehicle_run::observed {
namespace {
namespace contact = tlfea::contact;
using Observation = vehicle_dynamics::SelfContactObservation;
using Error = vehicle_self_contact::SelfContactStageError;

struct Calls {
    std::string order;
    unsigned destroyed = 0, bridge = 0, callbacks = 0;
    unsigned scratch = 0, forecast = 0, allocations = 0, diagnostic_reads = 0;
    std::array<const void*, 5> arguments{};
    std::exception_ptr rejection;
};
class Fake final : public Contribution {
  public:
    explicit Fake(Calls& calls) : calls(calls) { budget.device_bytes = 79; }
    ~Fake() override { ++calls.destroyed; }
    void Assemble(tl::fea::FENodalState& owner, const tl::fea::NodalTrialToken& token,
                  const tl::fea::NodalAssemblyView& assembly, Observation& observation) override {
        calls.order += 'A';
        calls.arguments = {&owner, &token, &assembly, &observation, nullptr};
        observation.accepted_facet_pairs = 23;
    }
    void SealCandidate(tl::fea::FENodalState& owner, const tl::fea::NodalTrialToken& token,
                       const tl::fea::ShellPhysicalDiagnostics& common,
                       const tl::fea::NodalPreparedView& prepared, Observation& observation) override {
        calls.order += 'S';
        calls.arguments = {&owner, &token, &common, &prepared, &observation};
        if (calls.rejection) std::rethrow_exception(calls.rejection);
        observation.policy_outcomes = 37;
    }
    tl::fea::ShellPhysicalScratchReceiptRoster scratch_receipts() const noexcept override {
        ++calls.scratch;
        return {};
    }
    void Discard() noexcept override { calls.order += 'D'; }
    contact::SelfContactTransactionDiagnostics diagnostics() const noexcept override {
        ++calls.diagnostic_reads;
        contact::SelfContactTransactionDiagnostics result;
        result.accepted.enabled=result.candidate.enabled=true;
        result.candidate.owner_id=19;result.candidate.native_submitted_pairs=23;
        result.candidate.finished=true;result.candidate.succeeded=false;
        return result;
    }
    const vehicle_self_contact::VehicleSelfContactSetup& setup() const noexcept override {
        // A real setup needs authenticated source construction. These dispatch
        // tests deliberately neither manufacture one nor initialize a GPU owner.
        std::terminate();
    }
    const vehicle_self_contact::RuntimeForecast& forecast() const noexcept override {
        ++calls.forecast;
        return budget;
    }
    contact::SelfContactTransactionAllocationInfo allocations() const noexcept override {
        ++calls.allocations;
        contact::SelfContactTransactionAllocationInfo result;
        result.activity.host_bytes = 41;
        result.activity.host_allocations = 2;
        result.device.device_bytes = 79;
        result.device.device_allocations = 3;
        return result;
    }
    Calls& calls;
    vehicle_self_contact::RuntimeForecast budget;
};
void Capture(void* context, const contact::self_contact_transaction::CandidateFailureCapture&) noexcept {
    ++static_cast<Calls*>(context)->callbacks;
}
Observer Observe(Calls& calls) { return {&calls, sizeof(calls), &Capture}; }
void Bridge(Contribution& original, tl::fea::FENodalState& owner,
            const tl::fea::NodalTrialToken& token, const tl::fea::ShellPhysicalDiagnostics& common,
            const tl::fea::NodalPreparedView& prepared, Observation& observation,
            const Observer& observer) {
    auto& calls = static_cast<Fake&>(original).calls;
    ++calls.bridge;
    EXPECT_EQ(observer.context, &calls);
    EXPECT_EQ(observer.context_bytes, sizeof(calls));
    EXPECT_EQ(observer.capture, &Capture);
    original.SealCandidate(owner, token, common, prepared, observation);
}
std::unique_ptr<Contribution> Original(Calls& calls) { return std::make_unique<Fake>(calls); }

TEST(ObservedContribution, ForwardsOriginalOperationsAndRetainsContextThroughOwnedLifetime) {
    Calls calls;
    auto original = Original(calls);
    const auto* concrete = static_cast<Fake*>(original.get());
    auto descriptor = Observe(calls);
    auto observed = ObservedContribution::Wrap(original, descriptor, &Bridge);
    descriptor = {}; // The installed descriptor is copied; its external context is retained.
    EXPECT_FALSE(original);
    // Default state is uninitialized and allocates no CUDA buffers. The fake
    // merely checks reference forwarding and does not claim physical success.
    tl::fea::FENodalState owner;
    tl::fea::NodalTrialToken token;
    tl::fea::NodalAssemblyView assembly;
    tl::fea::ShellPhysicalDiagnostics common;
    tl::fea::NodalPreparedView prepared;
    Observation observation;
    observed->Assemble(owner, token, assembly, observation);
    EXPECT_EQ(calls.arguments, (std::array<const void*, 5>{
        &owner, &token, &assembly, &observation, nullptr}));
    EXPECT_EQ(observation.accepted_facet_pairs, 23u);
    observed->SealCandidate(owner, token, common, prepared, observation);
    EXPECT_EQ(calls.arguments, (std::array<const void*, 5>{
        &owner, &token, &common, &prepared, &observation}));
    EXPECT_EQ(observation.policy_outcomes, 37u);
    EXPECT_EQ(calls.bridge, 1u);
    EXPECT_EQ(calls.callbacks, 0u);
    EXPECT_EQ(&observed->forecast(), &concrete->budget);
    EXPECT_EQ(calls.forecast, 1u);
    const auto scratch = observed->scratch_receipts();
    EXPECT_EQ(scratch.mapped_wall, nullptr);
    EXPECT_EQ(scratch.self_contact, nullptr);
    EXPECT_EQ(calls.scratch, 1u);
    const auto allocations = observed->allocations();
    EXPECT_EQ(allocations.activity.host_bytes, 41u);
    EXPECT_EQ(allocations.activity.host_allocations, 2u);
    EXPECT_EQ(allocations.device.device_bytes, 79u);
    EXPECT_EQ(allocations.device.device_allocations, 3u);
    EXPECT_EQ(calls.allocations, 1u);
    const auto diagnostics=observed->diagnostics();
    EXPECT_TRUE(diagnostics.candidate.enabled);
    EXPECT_EQ(diagnostics.candidate.owner_id,19u);
    EXPECT_EQ(diagnostics.candidate.native_submitted_pairs,23u);
    EXPECT_EQ(calls.diagnostic_reads,1u);
    observed->Discard();
    EXPECT_EQ(calls.order, "ASD");
    EXPECT_EQ(observed->diagnostics().candidate.native_submitted_pairs,23u);
    EXPECT_FALSE(observed->diagnostics().candidate.succeeded);
    EXPECT_EQ(calls.destroyed, 0u);
    observed.reset();
    EXPECT_EQ(calls.destroyed, 1u);
}

TEST(ObservedContribution, RejectionPropagatesSameExceptionWithoutExtraDiscardOrObservationWrite) {
    Calls calls;
    contact::SelfContactTransactionReport report;
    report.status = contact::SelfContactTransactionStatus::UnresolvedCandidate;
    report.pair = 2186;
    report.candidate = 71;
    report.message = "original native candidate rejection";
    report.crossing_reason = contact::RepresentedIntervalReason::WorkExhausted;
    calls.rejection = std::make_exception_ptr(Error(
        report, vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal, 65536));
    auto original = Original(calls);
    tl::fea::FENodalState owner;
    tl::fea::NodalTrialToken token;
    tl::fea::ShellPhysicalDiagnostics common;
    tl::fea::NodalPreparedView prepared;
    Observation observation;
    observation.policy_outcomes = 89;
    const Error* direct = nullptr;
    try { original->SealCandidate(owner, token, common, prepared, observation); }
    catch (const Error& error) { direct = &error; }
    ASSERT_NE(direct, nullptr);
    auto observed = ObservedContribution::Wrap(original, Observe(calls), &Bridge);
    try {
        observed->SealCandidate(owner, token, common, prepared, observation);
        FAIL() << "Rejection was swallowed";
    } catch (const Error& error) {
        // The exception_ptr retains this exact object throughout both calls.
        EXPECT_EQ(&error, direct);
        EXPECT_EQ(error.report().pair, 2186u);
        EXPECT_EQ(error.report().candidate, 71u);
        EXPECT_EQ(error.report().crossing_reason, contact::RepresentedIntervalReason::WorkExhausted);
        EXPECT_EQ(error.stage(), vehicle_self_contact::SelfContactRuntimeStage::CandidateSeal);
        EXPECT_EQ(error.required_events(), 0u);
    }
    EXPECT_EQ(observation.policy_outcomes, 89u);
    EXPECT_EQ(calls.order, "SS");
    EXPECT_EQ(calls.callbacks, 0u);
    observed->Discard(); // Only the caller owns rollback.
    EXPECT_EQ(calls.order, "SSD");
}

TEST(ObservedContribution, MalformedAndRepeatedInstallationPreserveOriginalStage) {
    Calls calls;
    auto original = Original(calls);
    const auto* identity = original.get();
    const auto reject = [&](Observer observer, SealOperation seal = &Bridge) {
        EXPECT_THROW(ObservedContribution::Wrap(original, observer, seal), std::exception);
        EXPECT_EQ(original.get(), identity);
        EXPECT_EQ(calls.destroyed, 0u);
    };
    reject({});
    auto observer = Observe(calls);
    observer.capture = nullptr;
    reject(observer);
    observer = Observe(calls);
    observer.context_bytes = 0;
    reject(observer);
    observer.context_bytes = std::numeric_limits<std::size_t>::max();
    reject(observer);
    observer = Observe(calls);
    observer.context = reinterpret_cast<void*>(std::numeric_limits<std::uintptr_t>::max() - 1);
    reject(observer);
    reject(Observe(calls), nullptr);
    auto observed = ObservedContribution::Wrap(original, Observe(calls), &Bridge);
    const auto* installed = observed.get();
    EXPECT_THROW(ObservedContribution::Wrap(observed, Observe(calls), &Bridge), std::exception);
    EXPECT_EQ(observed.get(), installed);
    EXPECT_THROW(ObservedContribution::Wrap(original, Observe(calls), &Bridge), std::exception);
    EXPECT_EQ(calls.destroyed, 0u);
}
} // namespace
} // namespace crash::cases::vehicle_run::observed
