#pragma once
#include "Error.h"
#include <memory>

namespace crash::cases::vehicle_dynamics::native_contact {
// App-private protocol adapter, separate from the legacy M2 contribution.
// Owns the stable-address transaction, never the owner/publisher/stream.
// Those objects outlive this adapter; calls/destruction are serialized.
// Adoption does not establish parser/source completeness. The common publisher
// must bind its genuine full scratch roster before any interval.
class Contribution {
  public:
    static std::unique_ptr<Contribution> Adopt(std::unique_ptr<native::Transaction>);
    ~Contribution();
    Contribution(const Contribution&) = delete;
    Contribution& operator=(const Contribution&) = delete;
    Contribution(Contribution&&) = delete;
    Contribution& operator=(Contribution&&) = delete;

    tl::fea::ShellPhysicalScratchRosterEntry roster_entry() noexcept;
    void Assemble(tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
                  const tl::fea::NodalAssemblyView&, Observation&);
    void SealCandidate(tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
                       const tl::fea::NodalPreparedView&,
                       const tl::fea::ShellPhysicalDiagnostics&, Observation&);
    tl::fea::ShellPhysicalScratchReceiptRoster scratch_receipts() const noexcept;
    void Discard() noexcept;
    // Infallible app notification AFTER successful common commit. It clears
    // bookkeeping only; it cannot publish state or advance an owner/clock.
    void Committed() noexcept;
    native::TransactionSourceInfo source_info() const noexcept { return source_; }
    native::TransactionForecast resources() const noexcept;
    // Payload only, not a claim about allocator overhead or allocation count.
    static std::size_t bookkeeping_payload_bytes() noexcept { return sizeof(Contribution); }
    const native::Transaction& transaction() const noexcept { return *transaction_; }

  private:
    enum class Phase { Idle, Assembled, Sealed };
    explicit Contribution(std::unique_ptr<native::Transaction>, native::TransactionSourceInfo);
    [[noreturn]] void Reject(Operation, const char*);
    void Require(Operation, const native::TransactionReport&);
    std::unique_ptr<native::Transaction> transaction_;
    native::TransactionSourceInfo source_;
    tl::fea::ShellPhysicalScratchParticipationReceipt receipt_;
    Observation candidate_;
    Phase phase_ = Phase::Idle;
};
} // namespace crash::cases::vehicle_dynamics::native_contact
