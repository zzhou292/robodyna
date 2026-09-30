#pragma once
#include "Contribution.h"
#include <array>
namespace crash::cases::vehicle_dynamics::native_contact {
struct GroupInput {
    Role role=Role::Self;
    std::unique_ptr<native::Transaction> transaction;
};
// Stable-address app orchestration for the already-qualified native TL group.
// Interface order is the declaration order, including force additions. Adopt
// accepts initialized transactions only; Bind uses the sole actual publisher.
// Owner/publisher/stream outlive this object. Serialize every operation.
class Group {
  public:
    static std::unique_ptr<Group> Adopt(std::array<GroupInput,tl::fea::MaxNativeContactInterfaces>,std::size_t count);
    ~Group();
    Group(const Group&)=delete;Group& operator=(const Group&)=delete;
    Group(Group&&)=delete;Group& operator=(Group&&)=delete;
    void Bind(tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,const tl::fea::ShellPhysicalBinding&,
        const tl::fea::ShellPhysicalParticipants&,const tl::fea::ShellPhysicalPublicationIdentity&);
    void Assemble(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&,const tl::fea::NodalAssemblyView&,GroupObservation&);
    void SealCandidate(tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,const tl::fea::NodalTrialToken&,const tl::fea::NodalPreparedView&,
        const tl::fea::ShellPhysicalDiagnostics&,GroupObservation&);
    tl::fea::ShellPhysicalScratchReceiptRoster scratch_receipts() const noexcept;
    void Discard() noexcept;
    void Committed() noexcept;
    std::size_t count() const noexcept { return count_; }
    Role role(std::size_t) const;
    const native::Transaction& transaction(std::size_t) const;
    // Exact declared device bytes; transaction allocation counts are not exposed.
    std::size_t device_bytes() const noexcept;
  private:
    Group()=default;
    enum class Phase { Unbound,Idle,Assembled,Sealed };
    std::array<std::unique_ptr<Contribution>,tl::fea::MaxNativeContactInterfaces> entries_;
    std::array<Role,tl::fea::MaxNativeContactInterfaces> roles_{};
    std::array<tl::fea::NativeContactRosterEntry,tl::fea::MaxNativeContactInterfaces> roster_;
    std::array<const tl::fea::ShellPhysicalScratchParticipationReceipt*,tl::fea::MaxNativeContactInterfaces> receipts_{};
    std::size_t count_=0;
    Phase phase_=Phase::Unbound;
    GroupObservation staged_;
};
}
