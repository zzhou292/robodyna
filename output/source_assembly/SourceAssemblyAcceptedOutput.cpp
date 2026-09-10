#include "SourceAssemblyAcceptedOutput.h"
#include "SourceAssemblyOwnerScope.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "lib_src/elements/t3/T3History.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_utils/BoundedArena.h"
#include <array>
#include <new>

namespace crash::output::assembly {
namespace fe=tl::fea;
namespace {
struct Sections {
    Sections(std::size_t q,std::size_t t):qeph(q),t3(t),qt(q),tt(t),scalars(q+t) {}
    std::vector<fe::ShellBatchSectionState> qeph,t3;
    std::vector<double> qt,tt;
    std::vector<ReplayParentScalar> scalars;
    fe::ShellBatchDiagnostics diagnostics;
    SectionView Q() const noexcept{return {qeph.data(),qt.data(),qeph.size()};}
    SectionView T() const noexcept{return {t3.data(),tt.data(),t3.size()};}
};
template<class D> bool Accepted(const D& d,const fe::NodalStamp& stamp) noexcept {
    return d.valid&&d.owner_id==stamp.owner_id&&d.epoch==stamp.epoch&&d.time==stamp.time&&
        d.velocity_time==stamp.velocity_time&&d.phase==decltype(d.phase)::Accepted;
}
template<class D> bool SameIdentity(const D& a,const D& b) noexcept {
    return a.owner_id==b.owner_id&&a.configuration_id==b.configuration_id&&a.qualification_id==b.qualification_id&&
        a.epoch==b.epoch&&a.time==b.time&&a.attempt==b.attempt&&a.phase==b.phase&&a.usage==b.usage;
}
}
struct SourceAssemblyAcceptedOutput::Impl {
    Impl(const cases::source_assembly::SourceAssemblyBindings& b,SourceAssemblySurface s,std::size_t bytes)
        :bindings(b),surface(std::move(s)),capture{Sections(b.shells().qeph_count(),b.shells().t3_count()),
        Sections(b.shells().qeph_count(),b.shells().t3_count())},qforce(b.shells().qeph_count()),
        tforce(b.shells().t3_count()),bytes(bytes) {}
    cases::source_assembly::SourceAssemblyBindings bindings;
    SourceAssemblySurface surface;
    visual::NodalMeshOutput nodal;
    std::array<Sections,2> capture;
    std::vector<fe::qeph::ForceTrial> qforce;
    std::vector<fe::t3::ForceTrial> tforce;
    std::size_t bytes;
    unsigned published=0;
    bool available=false;
};
SourceAssemblyAcceptedOutput::SourceAssemblyAcceptedOutput()=default;
SourceAssemblyAcceptedOutput::~SourceAssemblyAcceptedOutput()=default;
visual::Report SourceAssemblyAcceptedOutput::Initialize(const fe::FENodalState& owner,
        const cases::source_assembly::SourceAssemblyBindings& bindings,visual::Identity identity,
        std::uint64_t asset,AcceptedOutputLimits limits) {
    using visual::Status;
    if(impl_)return {Status::InvalidBinding,"Assembly output is already initialized"};
    const auto stamp=owner.accepted();const auto& b=bindings.shells();
    if(!stamp.owner_id||stamp.owner_id!=identity.owner||!stamp.has_rotations||
        stamp.node_count!=b.node_count()||!b.prepared()||!bindings.materials().Matches(b))
        return {Status::InvalidBinding,"Assembly output does not identify its source owner/binding"};
    const auto* groups=bindings.rigid_groups();
    const fe::NodalRigidGroupInfo expected_groups{bindings.source_instance_id(),
        groups?groups->group_count():0,groups?groups->member_count():0};
    if(!MatchesAssemblyRigidScope(stamp.rigid_groups,expected_groups))
        return {Status::InvalidBinding,"Assembly output rigid-group source scope does not match its owner"};
    const auto q=b.qeph_count(),t=b.t3_count();std::size_t nodal_bytes=0;
    if(!q||!t||q>1024||t>1024-q||!limits.max_section_host_bytes||limits.max_section_host_bytes>8*1024*1024||
        !visual::NodalCaptureBytes(stamp.node_count,true,limits.nodal,nodal_bytes))
        return {Status::ResourceLimit,"Assembly output node/parent/host-byte capacity exceeded"};
    tl::util::BoundedArenaLayout budget(limits.max_section_host_bytes);tl::util::ArenaRegion ignored;
    if(!budget.Append<Impl>(1,ignored)||!budget.Append<fe::ShellBatchSectionState>(2*(q+t),ignored)||
        !budget.Append<double>(2*(q+t),ignored)||!budget.Append<ReplayParentScalar>(2*(q+t),ignored)||
        !budget.Append<fe::qeph::ForceTrial>(q,ignored)||!budget.Append<fe::t3::ForceTrial>(t,ignored))
        return {Status::ResourceLimit,"Assembly section capture exceeds startup host-byte capacity"};
    try {
        auto surface=SourceAssemblySurface::Prepare(bindings.source(),identity,asset,bindings.source_instance_id(),limits.surface);
        auto next=std::make_unique<Impl>(bindings,std::move(surface),budget.bytes());
        const auto report=next->nodal.Initialize(owner,next->surface.binding(),visual::NodalOutputTiming::StaggeredHalfKick,limits.nodal);
        if(report.status!=Status::Ok)return report;
        impl_=std::move(next);return {Status::Ok,"Complete assembly accepted output initialized"};
    } catch(const std::bad_alloc&) {return {Status::ResourceLimit,"Assembly output allocation failed"};}
      catch(const std::exception&) {return {Status::InvalidBinding,"Invalid bounded assembly source surface"};}
}
visual::Report SourceAssemblyAcceptedOutput::Publish(fe::FENodalState& owner,fe::qeph::QephBatch& q,
        fe::t3::T3Batch& t,const fe::ShellBatchPublication& publication) {
    using visual::Status;
    if(!impl_)return {Status::NotInitialized,"Assembly output is not initialized"};
    auto& s=*impl_;const auto stamp=owner.accepted();
    if(stamp.owner_id!=s.surface.binding().identity.owner)return {Status::WrongOwner,"Assembly output owner changed"};
    if(const auto* shown=s.nodal.stamp();shown&&(stamp.epoch<=shown->epoch||stamp.time<=shown->time))
        return {Status::StaleFrame,"No newer accepted assembly frame"};
    auto& staged=s.capture[1-s.published];
    const auto common=publication.CopyAcceptedDiagnostics(stamp,&staged.diagnostics);
    if(common.status!=fe::ShellPublicationStatus::Success)return {Status::InvalidFrame,common.message};
    if(!staged.diagnostics.valid||!Accepted(staged.diagnostics.qeph,stamp)||!Accepted(staged.diagnostics.t3,stamp))
        return {Status::InvalidFrame,"Assembly diagnostics are not the complete accepted endpoint"};
    fe::qeph::BatchDiagnostics qd;fe::t3::BatchDiagnostics td;
    const auto qr=q.CopyAcceptedResults(stamp,s.qforce.data(),s.qforce.size(),&qd);
    if(qr.status!=fe::qeph::BatchStatus::Success)return {Status::InvalidFrame,qr.message};
    const auto tr=t.CopyAcceptedResults(stamp,s.tforce.data(),s.tforce.size(),&td);
    if(tr.status!=fe::t3::BatchStatus::Success)return {Status::InvalidFrame,tr.message};
    if(!SameIdentity(qd,staged.diagnostics.qeph)||!SameIdentity(td,staged.diagnostics.t3))
        return {Status::InvalidFrame,"Native result participants do not match accepted publication"};
    const auto qs=q.CopyAcceptedSectionHistory(stamp,staged.qeph.data(),staged.qeph.size(),&qd);
    if(qs.status!=fe::qeph::BatchStatus::Success)return {Status::InvalidFrame,qs.message};
    const auto ts=t.CopyAcceptedSectionHistory(stamp,staged.t3.data(),staged.t3.size(),&td);
    if(ts.status!=fe::t3::BatchStatus::Success)return {Status::InvalidFrame,ts.message};
    if(!SameIdentity(qd,staged.diagnostics.qeph)||!SameIdentity(td,staged.diagnostics.t3))
        return {Status::InvalidFrame,"Section participants do not match accepted publication"};
    for(std::size_t i=0;i<s.qforce.size();++i) {
        const auto& h=s.qforce[i].proposed_history;
        if(!h.matches_reference(s.bindings.shells().qeph_reference(i))||h.stamp().sample_index!=stamp.epoch||h.stamp().time!=stamp.time)
            return {Status::InvalidFrame,"QEPH output lost a source reference or accepted history identity"};
        staged.qt[i]=h.data().thickness;
    }
    for(std::size_t i=0;i<s.tforce.size();++i) {
        const auto& h=s.tforce[i].proposed_history;
        if(!h.matches_reference(s.bindings.shells().t3_reference(i))||h.stamp().sample_index!=stamp.epoch||h.stamp().time!=stamp.time)
            return {Status::InvalidFrame,"T3 output lost a source reference or accepted history identity"};
        staged.tt[i]=h.data().thickness;
    }
    if(!CopyParentScalars(s.surface,staged.Q(),staged.T(),staged.scalars)||
        !fe::trial_identity::SameStamp(stamp,owner.accepted()))
        return {Status::InvalidFrame,"Invalid assembly fields or changing accepted owner"};
    const auto report=s.nodal.Publish(owner); // Sole accepted-only geometry/capture protocol.
    if(report.status!=Status::Ok)return report;
    s.published=1-s.published;s.available=true;return report;
}
const SourceAssemblySurface* SourceAssemblyAcceptedOutput::mapping() const noexcept{return impl_?&impl_->surface:nullptr;}
const visual::NodalMeshOutput* SourceAssemblyAcceptedOutput::nodal() const noexcept{return impl_?&impl_->nodal:nullptr;}
SectionView SourceAssemblyAcceptedOutput::qeph() const noexcept{return impl_&&impl_->available?impl_->capture[impl_->published].Q():SectionView{};}
SectionView SourceAssemblyAcceptedOutput::t3() const noexcept{return impl_&&impl_->available?impl_->capture[impl_->published].T():SectionView{};}
const std::vector<ReplayParentScalar>* SourceAssemblyAcceptedOutput::parent_scalars() const noexcept {
    return impl_&&impl_->available?&impl_->capture[impl_->published].scalars:nullptr;
}
const fe::ShellBatchDiagnostics* SourceAssemblyAcceptedOutput::diagnostics() const noexcept {
    return impl_&&impl_->available?&impl_->capture[impl_->published].diagnostics:nullptr;
}
std::size_t SourceAssemblyAcceptedOutput::section_host_bytes() const noexcept{return impl_?impl_->bytes:0;}
} // namespace crash::output::assembly
