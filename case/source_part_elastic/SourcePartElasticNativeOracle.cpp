#include "SourcePartElasticTestSupport.h"
#include <cstring>

namespace crash::cases::source_part_elastic::test {
namespace qo=qeph_force_port_test;
namespace to=t3_force_port_test;
void NativeSequence::Initialize(const fe::ShellBatchBinding& b) {
    for(std::size_t e=0;e<source::Q4Count;++e) {
        ASSERT_EQ(qn::Initialize(qeph_startup_test::NativeInput(b.qeph_reference(e).input),qr[e]),qn::Status::kSuccess);
        ASSERT_EQ(qn::InitializeHistory(qr[e],{},qhistory[e]),qn::Status::kSuccess);
        qeph_startup_test::Agreement(b.qeph_reference(e),qr[e].data());
    }
    for(std::size_t e=0;e<source::T3Count;++e) {
        ASSERT_EQ(tn::Initialize(to::Native(b.t3_reference(e).input),tr[e]),tn::Status::kSuccess);
        ASSERT_EQ(tn::InitializeHistory(tr[e],{},thistory[e]),tn::Status::kSuccess);
        to::StartupAgreement(b.t3_reference(e),tr[e]);
    }
}
void NativeSequence::Check(SourcePartElasticCase& c,const Snapshot& base,const Snapshot& endpoint) {
    auto& p=SourcePartElasticTestAccess::Internal(c);
    const auto& b=c.binding();
    // Independent global scatter of the previously accepted native caches
    // checks the actual owner kick/drift, in addition to the element oracle.
    // The immutable declared host pulse is compared with its CUDA contribution.
    std::array<long double,3*NodeCount> force{},couple{};
    for(std::size_t j=0;j<force.size();++j)
        force[j]=static_cast<long double>(p.pulse_force[j])*PulseScale(base.stamp.time,c.config().pulse_duration);
    auto scatter=[&](const auto& nodes,const auto& result) {
        for(unsigned local=0;local<nodes.size();++local) for(unsigned axis=0;axis<3;++axis) {
            const auto j=3*nodes[local]+axis;
            const auto f=result.internal_force[local],m=result.internal_couple[local];
            force[j]-=axis==0?f.x:axis==1?f.y:f.z;
            couple[j]-=axis==0?m.x:axis==1?m.y:m.z;
        }
    };
    for(std::size_t e=0;e<source::Q4Count;++e) scatter(b.qeph_nodes(e),qtrial[e]);
    for(std::size_t e=0;e<source::T3Count;++e) scatter(b.t3_nodes(e),ttrial[e]);
    const long double kick=base.stamp.epoch?c.config().dt:c.config().dt/2;
    for(std::size_t n=0;n<NodeCount;++n) for(unsigned a=0;a<3;++a) {
        const auto j=3*n+a; const auto& mass=b.nodes()[n].native;
        const long double v=base.velocity[j]+kick*force[j]/mass.mass;
        const long double w=base.omega[j]+kick*couple[j]/mass.isotropic_inertia;
        const long double x=base.position[j]+c.config().dt*v;
        EXPECT_LE(std::abs(endpoint.velocity[j]-v),2e-13L*(1+std::abs(v)));
        EXPECT_LE(std::abs(endpoint.omega[j]-w),2e-13L*(1+std::abs(w)));
        EXPECT_LE(std::abs(endpoint.position[j]-x),2e-13L*(1+std::abs(x)));
    }
    for(std::size_t e=0;e<source::Q4Count;++e) {
        SCOPED_TRACE(b.qeph_source_id(e));
        q::PrescribedInterval in; in.base_time=base.stamp.time; in.dt=c.config().dt; in.sample_index=endpoint.stamp.epoch;
        for(unsigned local=0;local<4;++local) {
            const auto n=b.qeph_nodes(e)[local],j=3*n;
            in.position_endpoint[local]={endpoint.position[j],endpoint.position[j+1],endpoint.position[j+2]};
            in.velocity_midpoint[local]={endpoint.velocity[j],endpoint.velocity[j+1],endpoint.velocity[j+2]};
            in.omega_midpoint[local]={endpoint.omega[j],endpoint.omega[j+1],endpoint.omega[j+2]};
        }
        ASSERT_EQ(qn::EvaluateForce(qr[e],qhistory[e],qeph_kinematics_test::NativeInterval(in),qtrial[e]),qn::Status::kSuccess);
        qo::ForceAgreement(p.qresult[e],qtrial[e],b.qeph_reference(e).input,in);
        qo::Balance(p.qresult[e],in);
    }
    for(std::size_t e=0;e<source::T3Count;++e) {
        SCOPED_TRACE(b.t3_source_id(e));
        t::PrescribedInterval in; in.base_time=base.stamp.time; in.dt=c.config().dt; in.sample_index=endpoint.stamp.epoch;
        for(unsigned local=0;local<3;++local) {
            const auto n=b.t3_nodes(e)[local],j=3*n;
            in.position[local]={endpoint.position[j],endpoint.position[j+1],endpoint.position[j+2]};
            in.velocity[local]={endpoint.velocity[j],endpoint.velocity[j+1],endpoint.velocity[j+2]};
            in.angular_velocity[local]={endpoint.omega[j],endpoint.omega[j+1],endpoint.omega[j+2]};
        }
        ASSERT_EQ(tn::EvaluateForce(tr[e],thistory[e],to::Native(in),ttrial[e]),tn::Status::kSuccess);
        to::Agreement(b.t3_reference(e),in,p.tresult[e],ttrial[e]);
        to::oracle::Check(tr[e],thistory[e].data(),to::Native(in),to::Native(tr[e],p.tresult[e]));
    }
}
void NativeSequence::Accept() {
    for(std::size_t e=0;e<source::Q4Count;++e) qhistory[e]=qtrial[e].proposed_history;
    for(std::size_t e=0;e<source::T3Count;++e) thistory[e]=ttrial[e].proposed_history;
}
void ReadResults(SourcePartElasticCase& c,Results& out) {
    auto& p=SourcePartElasticTestAccess::Internal(c);
    q::BatchDiagnostics qd; t::BatchDiagnostics td;
    ASSERT_EQ(p.qeph.CopyAcceptedResults(c.owner().accepted(),out.qeph.data(),out.qeph.size(),&qd).status,q::BatchStatus::Success);
    ASSERT_EQ(p.t3.CopyAcceptedResults(c.owner().accepted(),out.t3.data(),out.t3.size(),&td).status,t::BatchStatus::Success);
}
void SameResults(const Results& a,const Results& b) {
    // These are two readbacks of initialized device slabs. Padding remains the
    // initialized slab bytes; additionally check T3's typed public fields.
    EXPECT_EQ(std::memcmp(a.qeph.data(),b.qeph.data(),sizeof(a.qeph)),0);
    for(std::size_t e=0;e<source::T3Count;++e) to::Exact(a.t3[e],b.t3[e]);
}
void SameSnapshot(const Snapshot& a,const Snapshot& b,bool same_owner) {
    EXPECT_EQ(a.position,b.position); EXPECT_EQ(a.velocity,b.velocity);
    EXPECT_EQ(a.orientation,b.orientation); EXPECT_EQ(a.omega,b.omega);
    EXPECT_EQ(a.synchronized_velocity,b.synchronized_velocity); EXPECT_EQ(a.synchronized_omega,b.synchronized_omega);
    if(same_owner) EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id);
    EXPECT_EQ(a.stamp.epoch,b.stamp.epoch); EXPECT_EQ(a.stamp.time,b.stamp.time);
    EXPECT_EQ(a.stamp.fixed_dt,b.stamp.fixed_dt); EXPECT_EQ(a.stamp.velocity_time,b.stamp.velocity_time);
    EXPECT_EQ(a.stamp.velocity_phase,b.stamp.velocity_phase); EXPECT_EQ(a.stamp.temporal_scheme,b.stamp.temporal_scheme);
    EXPECT_EQ(a.stamp.reaction_base_epoch,b.stamp.reaction_base_epoch);
    EXPECT_EQ(a.stamp.reaction_time,b.stamp.reaction_time); EXPECT_EQ(a.stamp.reaction_kick_dt,b.stamp.reaction_kick_dt);
    EXPECT_EQ(a.diagnostics.external_kick_work,b.diagnostics.external_kick_work);
    EXPECT_EQ(a.diagnostics.external_drift_work,b.diagnostics.external_drift_work);
    EXPECT_EQ(a.diagnostics.absolute_external_drift_work,b.diagnostics.absolute_external_drift_work);
    EXPECT_EQ(a.diagnostics.kinetic_work_residual,b.diagnostics.kinetic_work_residual);
    EXPECT_EQ(a.diagnostics.energy_residual,b.diagnostics.energy_residual);
    EXPECT_EQ(a.diagnostics.maximum_chord_change,b.diagnostics.maximum_chord_change);
}
} // namespace crash::cases::source_part_elastic::test
