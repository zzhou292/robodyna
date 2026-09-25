#include "NativeAcceptedFrames.h"
#include "NativeCaptureChecks.h"
#include "ParticipantPhase.h"
#include "NativeSourceMapping.h"
#include "RecordFields.h"
#include "output/full_shell/static_bundle/MappingArrays.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
namespace crash::output::physical_frames {
namespace fe=tl::fea;
namespace native=tlfea::contact::radioss_type25;
namespace source=records::source;
namespace {
detail::NativeCaptureScope ReadScope(fe::FENodalState& owner,fe::ShellBatchPublication& publication,
    native::Transaction& contact) {
    detail::NativeCaptureScope out;out.stamp=owner.accepted();out.contact=contact.accepted();out.source=contact.source_info();
    const auto read=publication.CopyAcceptedPhysicalDiagnostics(out.stamp,&out.diagnostics);
    Require(read.status==fe::ShellPublicationStatus::Success,read.message);detail::NativePhase(out);return out;
}
bool SameSource(native::TransactionSourceInfo a,native::TransactionSourceInfo b) {
    return a.available&&b.available&&a.source_id==b.source_id&&a.topology_generation==b.topology_generation&&
        a.source_generation==b.source_generation&&a.nodes==b.nodes&&a.secondaries==b.secondaries&&
        a.primary_mains==b.primary_mains&&a.expanded_mains==b.expanded_mains;
}
}
struct NativeAcceptedFrames::Impl {
    Impl(const source::PreparedSourceMapping& m,const fe::ShellPhysicalBinding& p,records::Context c,Forecast f,
        fe::FENodalState& o,fe::ShellBatchPublication& pub,fe::qeph::QephBatch& q,fe::t3::T3Batch& t,native::Transaction& n,
        native::TransactionSourceInfo source,const fe::ShellPhysicalPublicationIdentity& publication_identity)
      :mapping(m),physical(p),context(std::move(c)),forecast(f),owner(o),publication(pub),qeph(q),t3(t),contact(n),info(source),publication_identity(publication_identity),frames(context) {
        positions.resize(3*info.nodes);velocities.resize(3*info.nodes);
        layered.resize(f.layered_rows);active.resize(f.layered_rows);
        for(unsigned i=0;i<2;++i){history[i].resize(info.secondaries);icont[i].resize(info.secondaries);}
    }
    source::PreparedSourceMapping mapping;fe::ShellPhysicalBinding physical;
    records::Context context;Forecast forecast;
    fe::FENodalState& owner;fe::ShellBatchPublication& publication;fe::qeph::QephBatch& qeph;fe::t3::T3Batch& t3;native::Transaction& contact;
    native::TransactionSourceInfo info;
    fe::ShellPhysicalPublicationIdentity publication_identity;
    std::vector<std::uint32_t> nodes;std::vector<ParentField> parents;
    detail::FrameBuffers frames;
    std::vector<double> positions,velocities;
    std::vector<fe::ShellBatchLayeredSection> layered;
    std::vector<std::uint8_t> active;
    std::vector<native::NativeGeometryHistory> history[2];std::vector<int> icont[2];
    detail::NativeCaptureScope scopes[2];
    detail::NativeCaptureScope Current() const {
        auto scope=ReadScope(owner,publication,contact);const auto& id=context.identity();
        Require(SameSource(info,scope.source)&&id.owner==scope.stamp.owner_id&&
            id.configuration==scope.diagnostics.qeph.configuration_id&&id.qualification==scope.diagnostics.qeph.qualification_id&&
            id.source_instance==physical.domain()->source_instance_id()&&id.topology==scope.source.topology_generation&&
            Bits(scope.stamp.fixed_dt)==Bits(context.fixed_dt()),"Native capture belongs to another actual source/configuration");
        const auto authentic=publication.ValidatePhysicalSources(owner,physical,{&qeph,&t3},publication_identity);
        Require(authentic.status==fe::ShellPublicationStatus::Success,authentic.message);
        return scope;
    }
};
NativeAcceptedFrames::NativeAcceptedFrames(const source::PreparedSourceMapping& mapping,const fe::ShellPhysicalBinding& physical,
    fe::FENodalState& owner,fe::ShellBatchPublication& publication,fe::qeph::QephBatch& q,fe::t3::T3Batch& t,
    native::Transaction& contact,const fe::ShellPhysicalPublicationIdentity& publication_identity,records::Identity id,Limits limits) {
    const auto authentic=publication.ValidatePhysicalSources(owner,physical,{&q,&t},publication_identity);
    Require(authentic.status==fe::ShellPublicationStatus::Success,authentic.message);
    const auto scope=ReadScope(owner,publication,contact);const auto& info=scope.source;
    Require(physical.prepared()&&physical.execution()&&physical.domain()->node_count()==info.nodes&&mapping.nodes()==info.nodes&&
        physical.shells()->qeph_count()&&physical.shells()->t3_count()&&!physical.shells()->qbat_count()&&
        mapping.parents().size()==physical.shells()->qeph_count()+physical.shells()->t3_count(),
        "Native accepted capture source/family binding differs");
    Require((!id.owner||id.owner==scope.stamp.owner_id)&&(!id.source_instance||id.source_instance==physical.domain()->source_instance_id())&&
        (!id.configuration||id.configuration==scope.diagnostics.qeph.configuration_id)&&
        (!id.qualification||id.qualification==scope.diagnostics.qeph.qualification_id)&&
        (!id.topology||id.topology==info.topology_generation),"Caller capture identity conflicts with actual native source");
    id.owner=scope.stamp.owner_id;id.source_instance=physical.domain()->source_instance_id();id.topology=info.topology_generation;
    id.configuration=scope.diagnostics.qeph.configuration_id;id.qualification=scope.diagnostics.qeph.qualification_id;
    auto context=mapping.MakeFrameContext(std::move(id),scope.stamp.fixed_dt,limits.records);
    tl::util::BoundedArenaLayout extra(limits.host_bytes);tl::util::ArenaRegion region;
    Require(extra.Append<std::byte>(sizeof(Impl),region)&&extra.Append<std::byte>(mapping.payload_bytes(),region)&&
        extra.Append<std::byte>(physical.owned_payload_bytes(),region)&&
        extra.Append<std::byte>(detail::NativeMappingBytes(mapping,limits.host_bytes),region)&&
        extra.Append<native::NativeGeometryHistory>(2*info.secondaries,region)&&extra.Append<int>(2*info.secondaries,region),
        "Native capture source/history storage exceeds cap");
    const auto forecast=detail::PlanBuffers(context,info.nodes,physical.shells()->qeph_count(),physical.shells()->t3_count(),0,extra.bytes(),limits);
    // Complete active count/byte admission precedes dynamic mapping and readback buffers.
    auto bound=detail::BindNativeSource(mapping,physical,limits.host_bytes);
    auto next=std::make_unique<Impl>(mapping,physical,std::move(context),forecast,owner,publication,q,t,contact,info,publication_identity);
    next->nodes=std::move(bound.nodes);next->parents=std::move(bound.parents);next->Current();impl_=std::move(next);
}
NativeAcceptedFrames::~NativeAcceptedFrames()=default;
void NativeAcceptedFrames::Capture() {
    auto& s=*impl_;const auto before=s.Current();fe::NodalStamp stamp;
    const auto read=s.owner.CopyAccepted({s.positions.data(),s.velocities.data(),s.info.nodes},&stamp);
    Require(read.status==fe::NodalStatus::Ok,read.message);
    Require(fe::trial_identity::SameStamp(stamp,before.stamp),"Native nodal accepted readback changed endpoint");
    auto& frame=s.frames.Staging();detail::StagePositions(s.nodes,s.positions.data(),s.info.nodes,frame);
    fe::qeph::BatchDiagnostics q;
    auto qr=s.qeph.CopyAcceptedLayeredSectionHistory(stamp,s.layered.data(),s.physical.shells()->qeph_count(),&q);
    Require(qr.status==fe::qeph::BatchStatus::Success,qr.message);
    qr=s.qeph.CopyAcceptedParentActivity(stamp,s.active.data(),s.physical.shells()->qeph_count(),&q);
    Require(qr.status==fe::qeph::BatchStatus::Success,qr.message);
    detail::CheckAcceptedParticipant(q,stamp,detail::NativePhase(before),s.context.identity().configuration,s.context.identity().qualification);
    detail::StageLayered(s.context,s.parents,QephFamily,s.layered.data(),s.active.data(),s.physical.shells()->qeph_count(),frame,s.frames.flags);
    fe::t3::BatchDiagnostics t;
    auto tr=s.t3.CopyAcceptedLayeredSectionHistory(stamp,s.layered.data(),s.physical.shells()->t3_count(),&t);
    Require(tr.status==fe::t3::BatchStatus::Success,tr.message);
    tr=s.t3.CopyAcceptedParentActivity(stamp,s.active.data(),s.physical.shells()->t3_count(),&t);
    Require(tr.status==fe::t3::BatchStatus::Success,tr.message);
    detail::CheckAcceptedParticipant(t,stamp,detail::NativePhase(before),s.context.identity().configuration,s.context.identity().qualification);
    detail::StageLayered(s.context,s.parents,T3Family,s.layered.data(),s.active.data(),s.physical.shells()->t3_count(),frame,s.frames.flags);
    const auto next=1-s.frames.selected;fe::NativeContactPublicationSnapshot contact;
    const auto copied=s.contact.CopyAccepted({s.history[next].data(),s.icont[next].data(),s.info.secondaries},&contact);
    Require(copied.status==native::TransactionStatus::Ok,copied.message);
    auto copied_scope=before;copied_scope.contact=contact;detail::CheckSameNativeScope(before,copied_scope);
    const auto after=s.Current();detail::CheckSameNativeScope(before,after);s.scopes[next]=after;
    s.frames.Finish(s.context,detail::NativePhase(after));
}
physical_run::Profile NativeAcceptedFrames::ObservationProfile() noexcept {physical_run::Profile p;p.native_contact=true;return p;}
physical_run::AcceptedInterval NativeAcceptedFrames::Interval() const {
    const auto scope=impl_->Current();const auto phase=detail::NativePhase(scope);
    Require(phase.epoch,"Initial native frame has no completed physical interval");
    physical_run::Values values;values.owner=scope.stamp.owner_id;values.stamp=phase;
    values.native_contact=detail::NativeContactObservation(scope);
    const auto profile=ObservationProfile();physical_run::CheckValues(impl_->context,profile,values);
    return physical_run::AcceptedInterval(std::move(values),impl_->context.identity(),profile,{});
}
const records::Context& NativeAcceptedFrames::context() const noexcept{return impl_->context;}
const Forecast& NativeAcceptedFrames::forecast() const noexcept{return impl_->forecast;}
const records::FrameRecord* NativeAcceptedFrames::frame() const noexcept{return impl_->frames.available?&impl_->frames.frames[impl_->frames.selected]:nullptr;}
const records::activity::ActivityRecord* NativeAcceptedFrames::activity() const noexcept{return impl_->frames.available?&*impl_->frames.activity[impl_->frames.selected]:nullptr;}
const native::NativeGeometryHistory* NativeAcceptedFrames::native_history() const noexcept{return impl_->frames.available?impl_->history[impl_->frames.selected].data():nullptr;}
const int* NativeAcceptedFrames::initial_contact_flags() const noexcept{return impl_->frames.available?impl_->icont[impl_->frames.selected].data():nullptr;}
std::size_t NativeAcceptedFrames::secondary_count() const noexcept{return impl_->info.secondaries;}
}
