#include "Group.h"
#include <stdexcept>
namespace crash::cases::vehicle_dynamics::native_contact {
namespace fe=tl::fea;
namespace {
void Require(bool value,const char* reason) {if(!value)throw std::invalid_argument(reason);}
}
std::unique_ptr<Group> Group::Adopt(std::array<GroupInput,fe::MaxNativeContactInterfaces> input,std::size_t count) {
    Require(count&&count<=fe::MaxNativeContactInterfaces,"Invalid native contact group size");
    for(std::size_t i=0;i<input.size();++i) {
        Require(i<count?bool(input[i].transaction):!input[i].transaction,"Incomplete native group input or ignored trailing transaction");
        if(i>=count)continue;
        Require(input[i].role==Role::Self||input[i].role==Role::MeshWall,"Unknown native contact role");
        const auto info=input[i].transaction->source_info();
        Require(info.available,"Native group requires initialized actual transactions");
        for(std::size_t j=0;j<i;++j) {
            Require(input[j].role!=input[i].role,"Native group has a repeated case role");
            Require(input[j].transaction->source_info().source_id!=info.source_id,"Native group has repeated source identity");
        }
    }
    auto next=std::unique_ptr<Group>(new Group());next->count_=count;
    for(std::size_t i=0;i<count;++i) {
        next->roles_[i]=input[i].role;
        next->entries_[i]=Contribution::Adopt(std::move(input[i].transaction));
        next->roster_[i]=next->entries_[i]->native_roster_entry();
    }
    return next;
}
Group::~Group()=default;
void Group::Bind(fe::FENodalState& owner,fe::ShellBatchPublication& publication,
    const fe::ShellPhysicalBinding& physical,const fe::ShellPhysicalParticipants& participants,
    const fe::ShellPhysicalPublicationIdentity& identity) {
    Require(phase_==Phase::Unbound,"Native group is already bound");
    const auto report=publication.ConfigurePhysicalScratchParticipation(owner,physical,participants,identity,
        {{},{},{roster_.data(),count_}});
    if(report.status!=fe::ShellPublicationStatus::Success)throw std::runtime_error(report.message);
    phase_=Phase::Idle;
}
void Group::Assemble(fe::FENodalState& owner,const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,GroupObservation& output) {
    try {
        Require(phase_==Phase::Idle,"Native group attempt is not idle and bound");
        GroupObservation next;next.count=count_;next.roles=roles_;
        for(std::size_t i=0;i<count_;++i)entries_[i]->Assemble(owner,token,view,next.interfaces[i]);
        staged_=next;phase_=Phase::Assembled;output=next;
    } catch(...) {Discard();throw;}
}
void Group::SealCandidate(fe::FENodalState& owner,fe::ShellBatchPublication& publication,const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& view,const fe::ShellPhysicalDiagnostics& diagnostics,GroupObservation& output) {
    try {
        Require(phase_==Phase::Assembled,"Native group accepted assembly is missing");
        auto next=staged_;
        std::array<native::Transaction*,fe::MaxNativeContactInterfaces> members{};
        // Assembled is established only after every private child assembled
        // this same owner attempt. No child is externally mutable through Group.
        for(std::size_t i=0;i<count_;++i) {
            entries_[i]->PreflightSeal(view);
            members[i]=entries_[i]->transaction_.get();
        }
        std::array<fe::ShellPhysicalScratchParticipationReceipt,fe::MaxNativeContactInterfaces> staged;
        const auto sealed=native::Transaction::SealCandidateGroup(members.data(),count_,publication,
            owner,token,view,diagnostics,staged.data(),count_);
        if(sealed.report.status!=native::TransactionStatus::Ok) {
            const auto index=sealed.interface_index<count_?sealed.interface_index:0;
            entries_[index]->Require(Operation::SealCandidate,sealed.report);
        }
        for(std::size_t i=0;i<count_;++i) {
            entries_[i]->AdoptGroupSeal(staged[i],next.interfaces[i]);
            receipts_[i]=entries_[i]->native_receipt();
            Require(receipts_[i]!=nullptr,"Native group child did not seal its actual receipt");
        }
        staged_=next;phase_=Phase::Sealed;output=next;
    } catch(...) {Discard();throw;}
}
fe::ShellPhysicalScratchReceiptRoster Group::scratch_receipts() const noexcept {
    return phase_==Phase::Sealed?fe::ShellPhysicalScratchReceiptRoster{nullptr,nullptr,{receipts_.data(),count_}}:
        fe::ShellPhysicalScratchReceiptRoster{};
}
void Group::Discard() noexcept {
    for(std::size_t i=0;i<count_;++i)entries_[i]->Discard();
    receipts_={};staged_={};
    if(phase_!=Phase::Unbound)phase_=Phase::Idle;
}
void Group::Committed() noexcept {
    if(phase_!=Phase::Sealed)return;
    for(std::size_t i=0;i<count_;++i)entries_[i]->Committed();
    receipts_={};staged_={};phase_=Phase::Idle;
}
Role Group::role(std::size_t index) const {
    Require(index<count_,"Native group role index is outside its actual roster");return roles_[index];
}
const native::Transaction& Group::transaction(std::size_t index) const {
    Require(index<count_,"Native group transaction index is outside its actual roster");return entries_[index]->transaction();
}
std::size_t Group::device_bytes() const noexcept {
    std::size_t result=0;
    for(std::size_t i=0;i<count_;++i)result+=entries_[i]->resources().device_bytes;
    return result;

}
}
