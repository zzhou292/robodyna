#include "NormalImpactCase.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <string>

namespace {
using namespace crash::case_data;
std::string asset;
class NormalImpact : public ::testing::Test {
  protected:
    CanonicalWall wall;
    void SetUp() override {
        int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
        auto r=wall.LoadFile(asset); ASSERT_EQ(r.status,WallStatus::Ok)<<r.message;
    }
};

// Closed-form solution of the velocity-first scalar recurrence, independently
// derived from its 2x2 characteristic polynomial. The spatial quadrature and
// physical-node masses must reproduce this uniform mode, including release.
struct Oracle {
    double h, omega=100, theta;
    unsigned release;
    explicit Oracle(double dt):h(dt),theta(2*std::asin(h*omega/2)),release(std::ceil(std::acos(-1.0)/theta)) {}
    double contact_gap(unsigned n) const { return -h*std::sin(n*theta)/std::sin(theta); }
    double contact_velocity(unsigned n) const { return -std::cos((double(n)-.5)*theta)/std::cos(theta/2); }
    double gap(unsigned n) const { return n<=release ? contact_gap(n) : contact_gap(release)+(n-release)*h*contact_velocity(release); }
    double velocity(unsigned n) const { return contact_velocity(std::min(n,release)); }
};

TEST_F(NormalImpact, ActualWallImpactMatchesDiscreteSolutionAndContinuousRefinement) {
    const double pi=std::acos(-1.0);
    unsigned level=0;
    for (double h : {.001,.0005,.00025}) {
        SCOPED_TRACE(h); NormalImpactConfig config; config.dt=h;
        NormalImpactCase run; auto r=run.Initialize(wall,config); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
        Oracle oracle(h); const double s=h*100;
        const auto state_alloc=run.state_allocations(); const auto contact_alloc=run.contact_allocations();
        EXPECT_NEAR(run.metrics()->mass,1,2e-14);
        const unsigned steps=std::lround(.07/h);
        unsigned first_release=0; std::set<std::uint64_t> parent_quads;
        for (unsigned n=1;n<=steps;++n) {
            auto before=*run.metrics(); r=run.Step(); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message<<" step "<<n;
            const auto m=*run.metrics(); const auto* d=run.last_interval(); ASSERT_NE(d,nullptr);
            EXPECT_EQ(m.stamp.epoch,n); EXPECT_EQ(d->base_epoch,n-1); EXPECT_EQ(d->owner_id,m.stamp.owner_id);
            EXPECT_NEAR(m.stamp.time,n*h,2e-14);
            EXPECT_NEAR(m.mean_gap,oracle.gap(n),2e-12);
            EXPECT_NEAR(m.mean_normal_velocity,oracle.velocity(n),2e-11);
            EXPECT_NEAR(d->wall_reaction.x,1e4*std::max(0.0,-before.mean_gap),2e-9);
            EXPECT_DOUBLE_EQ(d->wall_reaction.x,-d->force_on_surface.x);
            EXPECT_NEAR(m.wall_impulse,m.mass*(m.mean_normal_velocity+1),2e-11);
            EXPECT_NEAR(m.contact_work,m.kinetic_energy-.5,3e-12);
            EXPECT_EQ(d->covered_count,2u);
            for (unsigned p=0;p<d->sample_count;++p) parent_quads.insert(d->samples[p].wall_source_quad_id);
            if (!first_release && n>1 && m.mean_gap>=0) first_release=n;
            if (n<=oracle.release) {
                const double g=m.mean_gap,v=m.mean_normal_velocity;
                EXPECT_NEAR(v*v+1e4*g*g-h*1e4*g*v,1,3e-11);
            }
            EXPECT_LE(m.kinetic_energy+m.elastic_energy,.5/(1-s/2)+2e-11);
        }
        const auto m=*run.metrics(); EXPECT_EQ(first_release,oracle.release);
        EXPECT_GE(parent_quads.size(),2u); EXPECT_TRUE(parent_quads.count(1046));
        EXPECT_LE(m.peak_penetration,1/(100*std::sqrt(1-s*s/4))+2e-12);
        EXPECT_GE(m.mean_normal_velocity,1-2e-11);
        EXPECT_LE(m.mean_normal_velocity,1/std::sqrt(1-s*s/4)+2e-11);
        EXPECT_LE(m.kinetic_energy,.5/(1-s*s/4)+2e-11);
        EXPECT_DOUBLE_EQ(m.elastic_energy,0);
        const double budgets[]={1e-4,2.5e-5,6.25e-6};
        EXPECT_NEAR(m.mean_gap,.07-pi/100,budgets[level++]);
        EXPECT_EQ(run.state_allocations().device_bytes,state_alloc.device_bytes);
        EXPECT_EQ(run.state_allocations().device_allocations,state_alloc.device_allocations);
        EXPECT_EQ(run.contact_allocations().device_bytes,contact_alloc.device_bytes);
        EXPECT_EQ(run.contact_allocations().device_allocations,contact_alloc.device_allocations);
    }
}

TEST_F(NormalImpact, WallSubdivisionAndPatchRefinementPreserveIntegratedResponse) {
    ImpactMetrics reference; bool first=true;
    for (auto variant : {0u,1u,2u,3u}) {
        NormalImpactConfig config; config.dt=.001;
        config.patch_divisions=variant<2 ? 1 : (variant==2 ? 2 : 4); config.refine_wall=variant==1;
        NormalImpactCase run; auto r=run.Initialize(wall,config); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
        for (unsigned n=0;n<70;++n) { r=run.Step(); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message; }
        auto m=*run.metrics();
        if(first) { reference=m; first=false; }
        EXPECT_NEAR(m.mean_gap,reference.mean_gap,2e-12);
        EXPECT_NEAR(m.mean_normal_velocity,reference.mean_normal_velocity,2e-11);
        EXPECT_NEAR(m.wall_impulse,reference.wall_impulse,3e-11);
        EXPECT_NEAR(m.kinetic_energy,reference.kinetic_energy,3e-11);
        EXPECT_NEAR(m.contact_work,reference.contact_work,3e-11);
        EXPECT_NEAR(m.mass,reference.mass,3e-14);
        EXPECT_EQ(run.last_interval()->covered_count,2*config.patch_divisions*config.patch_divisions);
    }
}

TEST_F(NormalImpact, ACompleteMissDoesNotBecomeAnInfinitePlaneConstraint) {
    NormalImpactConfig config; config.dt=.001; config.center_y=2; config.max_penetration=.0001;
    NormalImpactCase run; auto r=run.Initialize(wall,config); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
    for (unsigned n=0;n<70;++n) { r=run.Step(); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message; }
    auto m=*run.metrics(); EXPECT_NEAR(m.mean_gap,-.07,2e-13); EXPECT_NEAR(m.mean_normal_velocity,-1,2e-14);
    EXPECT_EQ(run.last_interval()->covered_count,0u); EXPECT_DOUBLE_EQ(m.wall_impulse,0);
    EXPECT_DOUBLE_EQ(m.contact_work,0); EXPECT_DOUBLE_EQ(m.elastic_energy,0);
    EXPECT_DOUBLE_EQ(m.peak_penetration,0);
}

TEST_F(NormalImpact, PreparedPenetrationRejectionPreservesStateLedgerAndChronoFrame) {
    NormalImpactConfig config; config.dt=.001; config.max_penetration=.0015;
    NormalImpactCase run; auto r=run.Initialize(wall,config); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
    r=run.Step(); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
    ASSERT_EQ(run.Publish().status,crash::visual::Status::Ok);
    const auto before=*run.metrics();
    ASSERT_NE(run.last_interval(),nullptr); const auto interval=*run.last_interval();
    for (unsigned retry=0;retry<2;++retry) {
        EXPECT_EQ(run.Step().status,ImpactStatus::ContactFailure);
        EXPECT_EQ(run.metrics()->stamp.epoch,before.stamp.epoch); EXPECT_DOUBLE_EQ(run.metrics()->stamp.time,before.stamp.time);
        EXPECT_DOUBLE_EQ(run.metrics()->mean_gap,before.mean_gap);
        EXPECT_DOUBLE_EQ(run.metrics()->wall_impulse,before.wall_impulse); EXPECT_DOUBLE_EQ(run.metrics()->contact_work,before.contact_work);
        ASSERT_NE(run.last_interval(),nullptr);
        EXPECT_EQ(run.last_interval()->base_epoch,interval.base_epoch); EXPECT_EQ(run.last_interval()->attempt,interval.attempt);
        EXPECT_DOUBLE_EQ(run.last_interval()->wall_reaction.x,interval.wall_reaction.x);
        EXPECT_EQ(run.Publish().status,crash::visual::Status::StaleFrame);
        EXPECT_EQ(run.output()->surface().frame()->epoch,1u);
    }
}

TEST_F(NormalImpact, CombinedStabilityGateRejectsAnOversizedStepBeforeCommit) {
    NormalImpactConfig config; config.dt=.1;
    NormalImpactCase run; auto r=run.Initialize(wall,config); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
    EXPECT_EQ(run.Step().status,ImpactStatus::StateFailure);
    EXPECT_EQ(run.metrics()->stamp.epoch,0u); EXPECT_EQ(run.last_interval(),nullptr);
    EXPECT_EQ(run.output()->surface().frame()->epoch,0u);
}

TEST_F(NormalImpact, ChronoCadenceReceivesOnlyCommittedPositionsAndSourceTopology) {
    NormalImpactCase run; auto r=run.Initialize(wall); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
    const auto owner=run.metrics()->stamp.owner_id;
    EXPECT_EQ(run.Initialize(wall).status,ImpactStatus::AlreadyInitialized);
    EXPECT_EQ(run.metrics()->stamp.owner_id,owner);
    for (unsigned n=1;n<=20;++n) {
        r=run.Step(); ASSERT_EQ(r.status,ImpactStatus::Ok)<<r.message;
        EXPECT_EQ(run.output()->surface().frame()->epoch,((n-1)/5)*5);
        if (n%5==0) {
            ASSERT_EQ(run.Publish().status,crash::visual::Status::Ok);
            const auto& surface=run.output()->surface();
            ASSERT_EQ(surface.frame()->epoch,n); EXPECT_EQ(surface.frame()->identity.owner,owner);
            EXPECT_DOUBLE_EQ(surface.frame()->time,run.metrics()->stamp.time);
            for (const auto& p:surface.mesh()->GetCoordsVertices())
                EXPECT_NEAR(p.x(),.05-run.metrics()->mean_gap,2e-13);
            EXPECT_EQ(surface.binding()->triangles.size(),2u);
            EXPECT_EQ(surface.binding()->vertices[3].source.node,4u);
            EXPECT_EQ(run.Publish().status,crash::visual::Status::StaleFrame);
        }
    }
}

TEST_F(NormalImpact, InvalidCaseAdmissionLeavesNoPartialCase) {
    NormalImpactCase run; CanonicalWall empty;
    EXPECT_EQ(run.Step().status,ImpactStatus::NotInitialized);
    EXPECT_EQ(run.Initialize(empty).status,ImpactStatus::InvalidInput);
    NormalImpactConfig config; config.dt=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(run.Initialize(wall,config).status,ImpactStatus::InvalidInput);
    config={}; config.patch_divisions=8;
    EXPECT_EQ(run.Initialize(wall,config).status,ImpactStatus::InvalidInput);
    config={}; config.speed=1e200;  // Finite state input, overflowing initial energy.
    EXPECT_EQ(run.Initialize(wall,config).status,ImpactStatus::InvalidInput);
    EXPECT_EQ(run.metrics(),nullptr); EXPECT_EQ(run.output(),nullptr);
    EXPECT_EQ(run.state_allocations().device_bytes,0u);
}
}  // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2) { std::cerr<<"Required canonical wall manifest argument\n"; return 2; }
    asset=argv[1]; return RUN_ALL_TESTS();
}
