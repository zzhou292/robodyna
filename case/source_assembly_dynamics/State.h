#pragma once
#include "Case.h"
#include "ForceStageWorkspace.h"
#include "NativeRotation.h"
#include "ConnectorWorkspace.h"
#include "lib_src/elements/ShellBatchPlasticity.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <array>
#include <vector>

namespace crash::cases::source_assembly_dynamics {
namespace fe=tl::fea;
namespace q=fe::qeph;
namespace t=fe::t3;
namespace contact=tlfea::contact;
namespace observation=source_assembly_observation;
inline Report Success() noexcept { return {Status::Ok,"OK"}; }
inline Report Failure(Status s,const char* m,std::uint64_t e=0,std::size_t n=SIZE_MAX,
                      double value=0,double limit=0) noexcept { return {s,m,e,n,value,limit}; }
inline Report Convert(const fe::NodalReport& r) noexcept {
    if(r.status==fe::NodalStatus::Ok)return Success();
    return Failure(r.status==fe::NodalStatus::DeviceFailure?Status::DeviceFailure:
        r.status==fe::NodalStatus::ResourceLimit?Status::ResourceLimit:Status::ComponentFailure,r.message,0,r.node);
}
template<class R> Status ParticipantFailureStatus(const R& r) noexcept {
    using S=decltype(r.status);
    if(r.status==S::DeviceFailure||(r.status==S::NodalFailure&&r.nodal_status==fe::NodalStatus::DeviceFailure))
        return Status::DeviceFailure;
    if(r.status==S::ResourceLimit||(r.status==S::NodalFailure&&r.nodal_status==fe::NodalStatus::ResourceLimit))
        return Status::ResourceLimit;
    return Status::ComponentFailure;
}
template<class R> Report BatchReport(const R& r,std::uint64_t source_parent=0) noexcept {
    if(r.status==decltype(r.status)::Success)return Success();
    return Failure(ParticipantFailureStatus(r),r.message,source_parent,r.node);
}
inline Report Convert(const contact::NodalWallDeviceReport& r) noexcept {
    if(r.status==contact::NodalWallDeviceStatus::Ok)return Success();
    return Failure(r.status==contact::NodalWallDeviceStatus::DeviceFailure?Status::DeviceFailure:
        r.status==contact::NodalWallDeviceStatus::ResourceLimit?Status::ResourceLimit:Status::ComponentFailure,r.message,0,r.node);
}
inline Report Convert(const fe::ShellPublicationReport& r) noexcept {
    if(r.status==fe::ShellPublicationStatus::Success)return Success();
    return Failure(ParticipantFailureStatus(r),r.message);
}
inline Report Convert(const observation::Report& r) noexcept {
    return r?Success():Failure(Status::ObservationFailure,r.message,0,r.node,r.residual,r.roundoff_budget);
}
struct Fields {
    Fields(std::size_t n,std::size_t g):x(3*n),v(3*n),w(3*n),orientation(4*n),reaction(3*n),couple(3*n),groups(g) {}
    std::vector<double> x,v,w,orientation,reaction,couple;
    std::vector<fe::NodalRigidGroupSnapshot> groups;
    fe::NodalSnapshotBuffer buffer() noexcept {
        return {x.data(),v.data(),x.size()/3,orientation.data(),w.data(),reaction.data(),couple.data()};
    }
    fe::HostNodalKinematicsView view() const noexcept { return {x.data(),v.data(),w.data(),x.size()/3,orientation.data()}; }
};
struct ParentFields {
    ParentFields(std::size_t nq,std::size_t nt):qeph(nq),t3(nt),qsection(nq),tsection(nt) {}
    std::vector<q::ForceTrial> qeph;
    std::vector<t::ForceTrial> t3;
    std::vector<fe::ShellBatchSectionState> qsection,tsection;
};
struct ContactFields {
    ContactFields(std::size_t p,std::size_t n):parents(p),nodes(n),wall_face(n) {}
    contact::NodalWallDiagnostics diagnostics;
    std::vector<contact::NodalWallParentResult> parents;
    std::vector<contact::NodalWallPointResult> nodes;
    std::vector<std::uint64_t> wall_face;
    contact::NodalWallDeviceResultView buffer() noexcept {
        return {&diagnostics,parents.data(),nodes.data(),wall_face.data(),parents.size(),nodes.size()};
    }
    ContactView view() const noexcept { return {&diagnostics,parents.data(),nodes.data(),wall_face.data(),parents.size(),nodes.size()}; }
};
struct Sample {
    Sample(std::size_t n,std::size_t g,std::size_t nq,std::size_t nt):fields(n,g),parents(nq,nt),wall(nq+nt,n) {}
    Fields fields;
    ParentFields parents;
    ContactFields wall;
    Diagnostics diagnostics;
    // Startup retains the actual accepted group readback identity. Later this
    // associates verified prepared groups with their sole successful commit.
    fe::NodalStamp group_stamp;
    bool has_force_stage=false;
    observation::ForceStageSummary force_stage;
    std::array<long double,3> qwork_magnitude{},twork_magnitude{};
};
struct SourceAssemblyWallCase::Impl {
    Impl(const source_assembly::SourceAssemblyBindings& b,const source_assembly::SourceAssemblyWallSetup& w,
         const Config& c,std::size_t host_bytes,StepTimingOptions timing_options)
        :bindings(b),setup(w),config(c),sample{Sample(nodes(),groups(),quads(),triangles()),Sample(nodes(),groups(),quads(),triangles())},
         inverse_mass(nodes()),inverse_inertia(nodes()),free(nodes()),load_soa(6*nodes()),
         applied_force(3*nodes()),applied_couple(3*nodes()),wall_faces(w.placed_wall()->view().triangle_count),
         force_capture(c.observe_force_stage,nodes(),groups()),host_bytes(host_bytes),timer(timing_options) {
        if(c.observe_qeph_spin_node)spin=std::make_unique<std::array<observation::QephSpinObservation,2>>();
        if(b.connectors())connector=std::make_unique<ConnectorWorkspace>(b.connectors()->connection_count());
    }
    const source_assembly::SourceAssemblyBindings bindings;
    const source_assembly::SourceAssemblyWallSetup setup;
    const Config config;
    // Publication must be destroyed before the two participants it borrows.
    fe::FENodalState owner;
    q::QephBatch qeph;
    t::T3Batch t3;
    std::unique_ptr<ConnectorWorkspace> connector;
    fe::ShellBatchPublication publication;
    contact::NodalWallContactDevice wall;
    std::array<Sample,2> sample;
    std::vector<double> inverse_mass,inverse_inertia;
    std::vector<std::uint8_t> free;
    std::vector<double> load_soa,applied_force,applied_couple;
    std::vector<std::uint64_t> wall_faces;
    ForceStageWorkspace force_capture;
    std::unique_ptr<std::array<observation::QephSpinObservation,2>> spin;
    std::unique_ptr<const NativeRotationReferences> rotation_reference;
    std::size_t host_bytes=0;
    StepTimer timer;
    unsigned accepted_slot=0;
    fe::NodalTrialToken token;
    fe::NodalPreparedView prepared,group_prepared;
    contact::NodalWallDiagnostics base_contact;
    double contact_step_rate_upper=0;
    bool poisoned=false;
    std::size_t nodes() const noexcept { return bindings.shells().node_count(); }
    std::size_t groups() const noexcept { return bindings.rigid_groups()->group_count(); }
    std::size_t quads() const noexcept { return bindings.shells().qeph_count(); }
    std::size_t triangles() const noexcept { return bindings.shells().t3_count(); }
    Sample& accepted() noexcept { return sample[accepted_slot]; }
    const Sample& accepted() const noexcept { return sample[accepted_slot]; }
    Sample& candidate() noexcept { return sample[1-accepted_slot]; }
    Report QReport(const q::BatchReport& r) const noexcept { return BatchReport(r,
        r.element<quads()?bindings.shells().qeph_source_id(r.element):0); }
    Report TReport(const t::BatchReport& r) const noexcept { return BatchReport(r,
        r.element<triangles()?bindings.shells().t3_source_id(r.element):0); }
    Report Initialize();
    Report InitializeConnector();
    Report ReadInitialConnector();
    Report AssembleConnector(const fe::NodalAssemblyView&);
    Report EvaluateConnector();
    Report CheckConnector();
    Report PreparePublication(fe::ShellBatchDiagnostics&);
    Report Prepare();
    Report Evaluate();
    Report Check();
    Report CheckShells();
    Report CheckRotations();
    Report CheckContact();
    Report CheckMotion();
    Report CaptureForceStage();
    Report CheckForceStage();
    Report CheckQephSpin();
    Report Commit();
    void Discard() noexcept {
        owner.Discard();publication.DiscardTrial();qeph.DiscardTrial();t3.DiscardTrial();wall.DiscardTrial();
        if(connector)connector->batch.DiscardTrial();
    }
    Report Stop(Report r) noexcept {
        timer.Measure<StepStage::DiscardTrial>([&] {Discard();return Success();});
        if(r.status==Status::DeviceFailure)poisoned=true;return r;
    }
    fe::NodalAllocationInfo Allocations() const noexcept;
};
} // namespace crash::cases::source_assembly_dynamics
