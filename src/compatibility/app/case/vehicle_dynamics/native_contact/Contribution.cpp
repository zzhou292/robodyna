#include "Contribution.h"

namespace crash::cases::vehicle_dynamics::native_contact {
namespace fe = tl::fea;

Contribution::Contribution(std::unique_ptr<native::Transaction> transaction,
                           native::TransactionSourceInfo source)
    : transaction_(std::move(transaction)), source_(source) {}
Contribution::~Contribution() = default;

std::unique_ptr<Contribution> Contribution::Adopt(std::unique_ptr<native::Transaction> transaction) {
    if (!transaction)
        throw StageError(Operation::Adopt,
                         {native::TransactionStatus::InvalidInput, "Missing native transaction"});
    const auto source = transaction->source_info();
    if (!source.available)
        throw StageError(Operation::Adopt, {native::TransactionStatus::NotInitialized,
                                          "Native source initialization has not completed"});
    return std::unique_ptr<Contribution>(new Contribution(std::move(transaction), source));
}

fe::ShellPhysicalScratchRosterEntry Contribution::roster_entry() noexcept {
    return transaction_->roster_entry();
}
fe::NativeContactRosterEntry Contribution::native_roster_entry() noexcept {
    return transaction_->native_roster_entry();
}
const fe::ShellPhysicalScratchParticipationReceipt* Contribution::native_receipt() const noexcept {
    return phase_==Phase::Sealed?&receipt_:nullptr;
}
void Contribution::Require(Operation operation, const native::TransactionReport& report) {
    if (report.status == native::TransactionStatus::Ok) return;
    // Copy borrowed diagnostics before cleanup. The guard also revokes scratch
    // if construction of the owned error string itself fails.
    struct Revoke {
        Contribution& contribution;
        ~Revoke() { contribution.Discard(); }
    } revoke{*this};
    throw StageError(operation, report, source_, transaction_->last_diagnostics());
}
[[noreturn]] void Contribution::Reject(Operation operation, const char* message) {
    Discard();
    throw StageError(operation, {native::TransactionStatus::StaleAttempt, message});
}
void Contribution::Assemble(fe::FENodalState& owner, const fe::NodalTrialToken& token,
                            const fe::NodalAssemblyView& view, Observation& output) {
    if (phase_ != Phase::Idle)
        Reject(Operation::Assemble, "A native contribution attempt is already pending");
    receipt_ = {};
    candidate_ = {};
    Require(Operation::Assemble, transaction_->AssembleAccepted(owner, token, view));
    Observation next;
    next.enabled = true;
    next.force_base = owner.accepted();
    next.attempt = view.attempt;
    next.source = source_;
    next.diagnostics = transaction_->last_diagnostics();
    candidate_ = next;
    phase_ = Phase::Assembled;
    output = next;
}
void Contribution::SealCandidate(fe::FENodalState& owner, const fe::NodalTrialToken& token,
                                 const fe::NodalPreparedView& prepared,
                                 const fe::ShellPhysicalDiagnostics& diagnostics,
                                 Observation& output) {
    PreflightSeal(prepared);
    fe::ShellPhysicalScratchParticipationReceipt receipt;
    Require(Operation::SealCandidate,
            transaction_->SealCandidate(owner, token, prepared, diagnostics, &receipt));
    AdoptGroupSeal(receipt,output);
}
void Contribution::PreflightSeal(const fe::NodalPreparedView& prepared) {
    if (phase_ != Phase::Assembled)
        Reject(Operation::SealCandidate, "Native accepted-force assembly is missing or already sealed");
    if (prepared.owner_id != candidate_.force_base.owner_id ||
        prepared.kinematics.base_epoch != candidate_.force_base.epoch ||
        prepared.attempt != candidate_.attempt)
        Reject(Operation::SealCandidate, "Native candidate belongs to another owner/epoch/attempt");
}
void Contribution::AdoptGroupSeal(const fe::ShellPhysicalScratchParticipationReceipt& receipt,
                                 Observation& output) noexcept {
    auto next = candidate_;
    next.diagnostics = transaction_->last_diagnostics();
    receipt_ = receipt;
    candidate_ = next;
    phase_ = Phase::Sealed;
    output = next;
}
fe::ShellPhysicalScratchReceiptRoster Contribution::scratch_receipts() const noexcept {
    return phase_ == Phase::Sealed ? fe::ShellPhysicalScratchReceiptRoster{nullptr, &receipt_}
                                  : fe::ShellPhysicalScratchReceiptRoster{};
}
void Contribution::Discard() noexcept {
    transaction_->DiscardTrial();
    receipt_ = {};
    candidate_ = {};
    phase_ = Phase::Idle;
}
void Contribution::Committed() noexcept {
    receipt_ = {};
    candidate_ = {};
    phase_ = Phase::Idle;
}
native::TransactionForecast Contribution::resources() const noexcept {
    return transaction_->allocations();
}
} // namespace crash::cases::vehicle_dynamics::native_contact
