#include "SourcePartWallCheckFixture.h"
#include "SourcePartWallContact.h"
#include "SourcePartWallResultCheck.h"
#include "lib_src/solvers/ExplicitNodalStep.h"
#include <gtest/gtest.h>
#include <cmath>

namespace crash::cases::source_part_wall {
namespace {
using namespace check;
namespace fe=tl::fea;
bool InitializeOwner(const Fixture& f,fe::FENodalState& owner,double dt=Step) {
    fe::NodalStateConfig config;config.node_count=source::NodeCount;config.fixed_dt=dt;
    config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto report=owner.Initialize(config,{f.position.data(),f.velocity.data(),f.omega.data(),source::NodeCount,f.orientation.data()},
        f.inverse.data(),{f.free.data(),f.free.data(),f.inverse_j.data()});
    EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;return report.status==fe::NodalStatus::Ok;
}
bool Bind(const Fixture& f,fe::FENodalState& owner,SourcePartWallContact& contact) {
    const auto report=contact.Initialize(f.source,f.binding,owner.accepted(),f.inverse.data(),f.kinetic,f.wall,f.wall_bytes,f.settings);
    EXPECT_TRUE(report)<<report.message;return bool(report);
}
bool Prepare(fe::FENodalState& owner,SourcePartWallContact& contact,fe::NodalTrialToken& token,
    fe::NodalAssemblyView& assembly,fe::NodalPreparedView& candidate) {
    auto report=owner.BeginTrial(&token,&assembly);EXPECT_EQ(report.status,fe::NodalStatus::Ok);
    if(report.status!=fe::NodalStatus::Ok)return false;
    contact::NodalWallDiagnostics base;const auto force=contact.AssembleAccepted(assembly,&base);
    EXPECT_TRUE(force)<<force.message;if(!force)return false;
    EXPECT_EQ(base.phase,contact::NodalWallDevicePhase::AcceptedBase);EXPECT_EQ(base.resultant.value,0);
    report=owner.SealAssembly(token);EXPECT_EQ(report.status,fe::NodalStatus::Ok);if(report.status!=fe::NodalStatus::Ok)return false;
    // Single separated contact-only unit interval; no shell or full impact
    // admission is asserted. The zero force and gap exceed this one-step drift.
    report=fe::AdvanceStaggeredHistory(owner,token,{assembly.owner_id,assembly.accepted.base_epoch,assembly.attempt,Step,.01,Qualification});
    EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message;if(report.status!=fe::NodalStatus::Ok)return false;
    report=owner.BorrowPrepared(token,&candidate);EXPECT_EQ(report.status,fe::NodalStatus::Ok);return report.status==fe::NodalStatus::Ok;
}
void CheckAccepted(const Fixture& f,fe::FENodalState& owner) {
    std::array<double,3*source::NodeCount> x{},v{},w{};std::array<double,4*source::NodeCount> q{};fe::NodalStamp stamp;
    ASSERT_EQ(owner.CopyAccepted({x.data(),v.data(),source::NodeCount,q.data(),w.data()},&stamp).status,fe::NodalStatus::Ok);
    EXPECT_EQ(x,f.position);EXPECT_EQ(v,f.velocity);EXPECT_EQ(w,f.omega);EXPECT_EQ(q,f.orientation);
    EXPECT_EQ(stamp.epoch,0u);EXPECT_EQ(stamp.time,0);EXPECT_EQ(stamp.velocity_phase,fe::NodalVelocityPhase::Collocated);
}
TEST(SourcePartWallContactCuda, NativeOwnerIdentityAndCompleteCandidateAreCopiedWithoutCommit) {
    const auto f=std::make_unique<Fixture>();fe::FENodalState owner;ASSERT_TRUE(InitializeOwner(*f,owner));
    SourcePartWallContact contact;ASSERT_TRUE(Bind(*f,owner,contact));
    const auto allocations=contact.allocations();EXPECT_EQ(allocations.device_allocations,1u);EXPECT_LE(allocations.device_bytes,512u*1024);
    EXPECT_GT(contact.stiffness_rate_bound(),0);EXPECT_LE(contact.step_rate_upper(),f->settings.maximum_step_rate);
    EXPECT_GE(static_cast<long double>(contact.step_rate_upper()),Step*std::sqrt(static_cast<long double>(contact.stiffness_rate_bound())));
    auto result=std::make_unique<contact::NodalWallDeviceResults>();
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView candidate;
    ASSERT_TRUE(Prepare(owner,contact,token,assembly,candidate));ASSERT_TRUE(contact.EvaluateCandidate(candidate,result.get()));
    const auto& d=result->diagnostics;EXPECT_TRUE(d.valid);EXPECT_EQ(d.owner_id,owner.accepted().owner_id);
    EXPECT_EQ(d.configuration_id,Configuration);EXPECT_EQ(d.qualification_id,Qualification);EXPECT_EQ(d.wall_binding_id,WallBinding);
    EXPECT_EQ(d.base_epoch,0u);EXPECT_EQ(d.attempt,assembly.attempt);EXPECT_EQ(d.phase,contact::NodalWallDevicePhase::PreparedCandidate);
    EXPECT_EQ(d.node_count,117u);EXPECT_EQ(d.parent_count,94u);EXPECT_EQ(d.resultant.value,0);EXPECT_EQ(d.potential.value,0);
    EXPECT_EQ(d.time,Step);EXPECT_EQ(d.velocity_time,Step/2);EXPECT_EQ(d.kick_dt,Step/2);
    const auto& geometry=*contact.setup()->source_geometry();
    for(unsigned p=0;p<source::ParentCount;++p) {
        const auto& expected=geometry.weights()->parent(p);const auto& actual=result->parents[p];
        EXPECT_TRUE(actual.valid);EXPECT_EQ(actual.parent_element_id,expected.parent_element_id);
        EXPECT_EQ(actual.family,expected.family);EXPECT_EQ(actual.arity,expected.arity);
    }
    for(unsigned n=0;n<source::NodeCount;++n) {EXPECT_EQ(result->nodes[n].node,n);EXPECT_EQ(result->nodes[n].force_world.x,0);}
    const auto saved=Bytes(*result);owner.Discard();contact.DiscardTrial();
    EXPECT_EQ(Bytes(*result),saved);ASSERT_NO_FATAL_FAILURE(CheckAccepted(*f,owner));
    EXPECT_EQ(contact.allocations().device_bytes,allocations.device_bytes);EXPECT_EQ(contact.allocations().device_allocations,1u);
}
TEST(SourcePartWallContactCuda, WrongCandidateAndLateFiniteMeshFailurePreserveHeldValueAndCleanRetry) {
    const auto f=std::make_unique<Fixture>();fe::FENodalState owner;ASSERT_TRUE(InitializeOwner(*f,owner));
    SourcePartWallContact contact;ASSERT_TRUE(Bind(*f,owner,contact));auto held=std::make_unique<contact::NodalWallDeviceResults>();
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView candidate;
    ASSERT_TRUE(Prepare(owner,contact,token,assembly,candidate));ASSERT_TRUE(contact.EvaluateCandidate(candidate,held.get()));
    owner.Discard();contact.DiscardTrial();const auto saved=Bytes(*held);
    const auto clean=std::make_unique<contact::NodalWallDeviceResults>(*held);
    for(unsigned fault=0;fault<2;++fault) {
        SCOPED_TRACE(fault);ASSERT_TRUE(Prepare(owner,contact,token,assembly,candidate));
        if(fault==0)++candidate.owner_id;
        else {
            // Deliberate completed-candidate fault injection, never accepted
            // geometry or a simulated drift. Exercise the last original node.
            const double outside=2;
            ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(candidate.kinematics.position_xyz)+3*116+1,&outside,
                                      sizeof(double),cudaMemcpyHostToDevice,candidate.stream),cudaSuccess);
            ASSERT_EQ(cudaStreamSynchronize(candidate.stream),cudaSuccess);
        }
        const auto rejected=contact.EvaluateCandidate(candidate,held.get());EXPECT_FALSE(rejected);
        if(fault==1)EXPECT_EQ(rejected.node,116u);
        EXPECT_EQ(Bytes(*held),saved);owner.Discard();contact.DiscardTrial();ASSERT_NO_FATAL_FAILURE(CheckAccepted(*f,owner));
    }
    ASSERT_TRUE(Prepare(owner,contact,token,assembly,candidate));ASSERT_TRUE(contact.EvaluateCandidate(candidate,held.get()));
    SameScientificResult(*clean,*held);
    owner.Discard();contact.DiscardTrial();EXPECT_EQ(contact.allocations().device_allocations,1u);
}
TEST(SourcePartWallContactCuda, ActualDeviceNativeMassMismatchAndFailedStepGuardDoNotPublish) {
    const auto f=std::make_unique<Fixture>();fe::FENodalState owner;ASSERT_TRUE(InitializeOwner(*f,owner));
    SourcePartWallContact contact;ASSERT_TRUE(Bind(*f,owner,contact));fe::NodalTrialToken token;fe::NodalAssemblyView assembly;
    ASSERT_EQ(owner.BeginTrial(&token,&assembly).status,fe::NodalStatus::Ok);
    struct Buffer {double* data=nullptr;~Buffer(){if(data)cudaFree(data);}} device;
    auto inverse=f->inverse;inverse.back()=std::nextafter(inverse.back(),HUGE_VAL);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.data),sizeof(inverse)),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(device.data,inverse.data(),sizeof(inverse),cudaMemcpyHostToDevice,assembly.stream),cudaSuccess);
    auto wrong=assembly;wrong.mass.inverse_mass=device.data;
    contact::NodalWallDiagnostics diagnostic;diagnostic.owner_id=999;const auto saved=Bytes(diagnostic);
    const auto rejected=contact.AssembleAccepted(wrong,&diagnostic);EXPECT_FALSE(rejected);EXPECT_EQ(rejected.node,116u);
    EXPECT_EQ(Bytes(diagnostic),saved);owner.Discard();contact.DiscardTrial();ASSERT_NO_FATAL_FAILURE(CheckAccepted(*f,owner));
    fe::FENodalState slow;ASSERT_TRUE(InitializeOwner(*f,slow,.001));SourcePartWallContact refused;
    const auto rate=refused.Initialize(f->source,f->binding,slow.accepted(),f->inverse.data(),f->kinetic,f->wall,f->wall_bytes,f->settings);
    EXPECT_EQ(rate.status,SourcePartWallStatus::StepLimit);EXPECT_FALSE(refused.initialized());EXPECT_EQ(refused.setup(),nullptr);
    EXPECT_EQ(refused.allocations().device_bytes,0u);EXPECT_EQ(refused.allocations().device_allocations,0u);
}
} // namespace
} // namespace crash::cases::source_part_wall
int main(int argc,char** argv) {
    if(argc<3)return 2;
    crash::cases::source_part_wall::check::SourcePath=argv[1];crash::cases::source_part_wall::check::WallPath=argv[2];
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
