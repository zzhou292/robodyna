#include "Internal.h"
namespace crash::cases::native_scene {
namespace dd=dynamics_detail;
NativeSceneDynamics::NativeSceneDynamics(std::unique_ptr<Storage> p):storage_(std::move(p)){}
NativeSceneDynamics::~NativeSceneDynamics()=default;
NativeSceneDynamics::NativeSceneDynamics(NativeSceneDynamics&&) noexcept=default;
NativeSceneDynamics NativeSceneDynamics::Prepare(const ContactSelection& source,DynamicsConfig config) {
    const auto forecast=Preflight(source,config);auto state=std::make_unique<Storage>(source,config,forecast);
    state->Initialize();return NativeSceneDynamics(std::move(state));
}
void NativeSceneDynamics::Storage::Discard() noexcept {
    if(trial||pending){contact.DiscardTrial();publication.DiscardTrial();owner.Discard();qeph.DiscardTrial();t3.DiscardTrial();}
    trial=false;pending=false;stage=0;token={};prepared={};assembly={};cin={};
}
void NativeSceneDynamics::Storage::BeginMaterials() {
    output::Require(!pending&&!trial&&stage==0,"Native scene already has an active attempt");
    auto& next=observations[1-selected];next={};next.base=owner.accepted();
    dd::RequireSuccess(owner.BeginTrial(&token,&assembly));trial=true;
    dd::RequireSuccess(qeph.AssembleMappedAccepted(owner,token,assembly));
    dd::RequireSuccess(t3.AssembleMappedAccepted(owner,token,assembly));
    dd::RequireSuccess(owner.BorrowCinAssembly(token,&cin));stage=1;
}
void NativeSceneDynamics::Storage::AssembleContact() {
    output::Require(trial&&!pending&&stage==1,"Native contact stage requires complete material assembly");
    dd::RequireSuccess(contact.AssembleAccepted(owner,token,assembly));stage=2;
}
void NativeSceneDynamics::Storage::FinishPrepare() {
    namespace f=tl::fea;auto& next=observations[1-selected];
    output::Require(trial&&!pending&&stage==2,"Native advancement requires its complete contact stage");
    dd::RequireSuccess(owner.SealAssembly(token));
    const f::NodalCinStructuralStep structural{f::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,
        source.physical_source().declared().data().nodal_scale,true};
    dd::RequireSuccess(f::AdvanceStaggeredCin(owner,token,{next.base.owner_id,next.base.epoch,assembly.attempt,
        config.qualification,config.fixed_dt,config.maximum_rotation_increment,true,structural}));
    dd::RequireSuccess(owner.CopyPreparedCinStructuralLimit(token,&next.structural_limit));
    dd::RequireSuccess(owner.BorrowPrepared(token,&prepared));
    f::qeph::BatchDiagnostics q;f::t3::BatchDiagnostics t;
    dd::RequireSuccess(qeph.EvaluateCandidate(owner,token,prepared,&q));
    dd::RequireSuccess(t3.EvaluateCandidate(owner,token,prepared,&t));
    dd::RequireSuccess(publication.PreparePhysical(owner,token,{&q,&t},&next.mechanics));
    f::ShellPhysicalScratchParticipationReceipt receipt;
    dd::RequireSuccess(contact.SealCandidate(owner,token,prepared,next.mechanics,&receipt));
    dd::RequireSuccess(publication.SealPhysicalScratchParticipation(owner,token,{nullptr,&receipt}));
    next.contact=contact.last_diagnostics();pending=true;stage=3;
}
const NativeStepObservation& NativeSceneDynamics::PrepareStep() {
    auto& s=*storage_;
    output::Require(!s.pending&&!s.trial&&s.stage==0,"Native scene already has an active attempt");
    try{s.BeginMaterials();s.AssembleContact();s.FinishPrepare();return s.observations[1-s.selected];}
    catch(...){s.Discard();throw;}
}
void NativeSceneDynamics::CommitStep() {
    auto& s=*storage_;output::Require(s.pending&&s.trial&&s.stage==3,"Native scene has no prepared interval");
    const auto& p=s.prepared;
    const auto result=s.publication.CommitPhysical(s.owner,s.token,s.observations[1-s.selected].mechanics,
        {p.owner_id,p.kinematics.base_epoch,p.attempt,s.config.qualification,true});
    if(result.status!=tl::fea::ShellPublicationStatus::Success){s.Discard();dd::RequireSuccess(result);}
    // Infallible app selector update only after the sole common commit succeeds.
    s.selected=1-s.selected;s.pending=false;s.trial=false;s.stage=0;s.token={};s.prepared={};s.assembly={};s.cin={};
}
void NativeSceneDynamics::DiscardStep() noexcept{storage_->Discard();}
bool NativeSceneDynamics::has_prepared_step() const noexcept{return storage_->pending;}
tl::fea::NodalStamp NativeSceneDynamics::accepted() const noexcept{return storage_->owner.accepted();}
const NativeStepObservation& NativeSceneDynamics::last_accepted_step() const {
    output::Require(accepted().epoch,"No native scene interval has been accepted");return storage_->observations[storage_->selected];
}
const DynamicsForecast& NativeSceneDynamics::forecast() const noexcept{return storage_->forecast;}
const ContactSelection& NativeSceneDynamics::source() const noexcept{return storage_->source;}
std::unique_ptr<output::physical_frames::NativeAcceptedFrames> NativeSceneDynamics::MakeCapture(
    const output::full_shell::source::PreparedSourceMapping& mapping,output::full_shell::Identity id,output::physical_frames::Limits limits) {
    auto& s=*storage_;output::Require(!s.pending&&!s.trial,"Cannot attach capture during a native physical attempt");
    return std::make_unique<output::physical_frames::NativeAcceptedFrames>(mapping,s.source.physical_source().physical(),s.owner,
        s.publication,s.qeph,s.t3,s.contact,s.Identity(),std::move(id),limits);
}
}
