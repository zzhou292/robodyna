#include "State.h"
#include "output/source_assembly/SourceAssemblyAcceptedOutput.h"
#include "lib_utils/BoundedArena.h"
#include <new>

namespace crash::cases::source_assembly_dynamics {
SourceAssemblyWallCase::SourceAssemblyWallCase()=default;
SourceAssemblyWallCase::~SourceAssemblyWallCase()=default;
Report SourceAssemblyWallCase::Initialize(const source_assembly::SourceAssemblyBindings& b,
        const source_assembly::SourceAssemblyWallSetup& setup,const Config& config) {
    if(impl_)return Failure(Status::AlreadyInitialized,"Assembly wall case is immutable after startup");
    if(!ValidConfig(config)||!setup.initialized()||!setup.bindings()||!setup.settings()||!setup.certificate())
        return Failure(Status::InvalidInput,"Missing prepared setup or explicit assembly run configuration");
    const auto& s=b.shells();const auto* groups=b.rigid_groups();const auto& other=*setup.bindings();
    if(!s.prepared()||!b.materials().Matches(s)||!groups||!groups->prepared()||
       !groups->group_count()||groups->group_count()>64||!groups->member_count()||
       groups->global_node_count()!=s.node_count()||groups->source_instance_id()!=b.source_instance_id()||
       b.source_instance_id()!=other.source_instance_id()||s.inventory()!=other.shells().inventory()||
       !b.materials().SameScope(other.materials())||b.source().data().identity.sha256!=other.source().data().identity.sha256||
       b.source().data().identity.bytes!=other.source().data().identity.bytes)
        return Failure(Status::SourceMismatch,"Wall, material and group preparation must retain the same complete source");
    const auto n=s.node_count(),nq=s.qeph_count(),nt=s.t3_count();const auto& cap=config.storage;
    if(!n||n>cap.max_nodes||!nq||nq>cap.max_parents||!nt||nt>cap.max_parents-nq||
       n>cap.publication.max_nodes||n>cap.contact.counts.nodes||n>cap.contact.counts.global_nodes||
       nq+nt>cap.contact.counts.parents)
        return Failure(Status::ResourceLimit,"Actual assembly exceeds explicit active participant capacities");
    tl::util::BoundedArenaLayout budget(cap.max_host_bytes);tl::util::ArenaRegion ignored;
    if(!budget.Append<Impl>(1,ignored)||!budget.Append<double>(2*19*n+2*n+12*n,ignored)||
       !budget.Append<std::uint8_t>(n,ignored)||
       !budget.Append<fe::NodalRigidGroupSnapshot>(2*groups->group_count(),ignored)||
       !budget.Append<q::ForceTrial>(2*nq,ignored)||!budget.Append<t::ForceTrial>(2*nt,ignored)||
       !budget.Append<fe::ShellBatchSectionState>(2*(nq+nt),ignored)||
       !budget.Append<contact::NodalWallParentResult>(2*(nq+nt),ignored)||
       !budget.Append<contact::NodalWallPointResult>(2*n,ignored)||
       !budget.Append<std::uint64_t>(2*n+setup.placed_wall()->view().triangle_count,ignored))
        return Failure(Status::ResourceLimit,"Assembly host observation payload exceeds its startup byte budget");
    try {
        auto next=std::make_unique<Impl>(b,setup,config,budget.bytes());
        const auto report=next->Initialize();if(!report)return report;
        impl_=std::move(next);return Success();
    } catch(const std::bad_alloc&) { return Failure(Status::ResourceLimit,"Assembly startup host allocation failed"); }
}
Report SourceAssemblyWallCase::Step() {
    if(!impl_)return Failure(Status::NotInitialized,"Assembly wall case is not initialized");
    if(impl_->poisoned)return Failure(Status::DeviceFailure,"Assembly CUDA participant is poisoned");
    auto r=impl_->Prepare();if(!r)return impl_->Stop(r);
    r=impl_->Evaluate();if(!r)return impl_->Stop(r);
    r=impl_->Check();if(!r)return impl_->Stop(r);
    r=impl_->Commit();if(!r)return impl_->Stop(r);
    return r;
}
Report SourceAssemblyWallCase::CaptureAccepted(output::assembly::SourceAssemblyAcceptedOutput& output) {
    if(!impl_)return Failure(Status::NotInitialized,"Assembly wall case is not initialized");
    if(impl_->poisoned)return Failure(Status::DeviceFailure,"Assembly CUDA participant is poisoned");
    if(!fe::trial_identity::SameStamp(impl_->owner.accepted(),impl_->accepted().diagnostics.stamp))
        return Failure(Status::ComponentFailure,"Accepted assembly stamp changed outside its case");
    const auto r=output.Publish(impl_->owner,impl_->qeph,impl_->t3,impl_->publication);
    return r.status==visual::Status::Ok?Success():Failure(Status::ComponentFailure,r.message);
}
bool SourceAssemblyWallCase::initialized() const noexcept { return impl_!=nullptr; }
const fe::FENodalState* SourceAssemblyWallCase::owner() const noexcept { return impl_?&impl_->owner:nullptr; }
const source_assembly::SourceAssemblyBindings* SourceAssemblyWallCase::bindings() const noexcept { return impl_?&impl_->bindings:nullptr; }
const source_assembly::SourceAssemblyWallSetup* SourceAssemblyWallCase::setup() const noexcept { return impl_?&impl_->setup:nullptr; }
const Config* SourceAssemblyWallCase::config() const noexcept { return impl_?&impl_->config:nullptr; }
const Diagnostics* SourceAssemblyWallCase::diagnostics() const noexcept { return impl_?&impl_->accepted().diagnostics:nullptr; }
ContactView SourceAssemblyWallCase::accepted_contact() const noexcept {
    return impl_&&impl_->accepted().diagnostics.has_interval?impl_->accepted().wall.view():ContactView{};
}
fe::NodalAllocationInfo SourceAssemblyWallCase::Impl::Allocations() const noexcept {
    fe::NodalAllocationInfo sum;
    for(const auto a:{owner.allocations(),qeph.allocations(),t3.allocations(),publication.allocations(),wall.allocations()}) {
        sum.device_bytes+=a.device_bytes;sum.device_allocations+=a.device_allocations;
    }
    return sum;
}
fe::NodalAllocationInfo SourceAssemblyWallCase::allocations() const noexcept { return impl_?impl_->Allocations():fe::NodalAllocationInfo{}; }
std::size_t SourceAssemblyWallCase::host_payload_bytes() const noexcept { return impl_?impl_->host_bytes:0; }
} // namespace crash::cases::source_assembly_dynamics
