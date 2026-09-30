#include "T3BatchFixture.h"
#include <limits>

namespace t3_uniform_startup_test {
using namespace t3_batch_test;
constexpr double Speed=8;
struct MovingRig:Rig {
  std::array<double,3*N> velocity{},omega{};
  explicit MovingRig(unsigned cells=2):Rig(cells) {
    for(unsigned n=0;n<this->n;++n) velocity[3*n]=Speed;
  }
  bool Owner() {
    fe::NodalStateConfig c; c.node_count=n; c.fixed_dt=H;
    c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto report=owner.Initialize(c,{x.data(),velocity.data(),omega.data(),n,orientation.data()},inverse.data(),
        {fixed.data(),rotation_fixed.data(),inverse_j.data()});
    EXPECT_EQ(report.status,fe::NodalStatus::Ok); return report.status==fe::NodalStatus::Ok;
  }
  t::T3BatchConfig Config() const {
    auto c=Rig::Config(t::BatchUsage::CoupledForces);
    c.startup={t::BatchStartupKind::ReferenceUniformTranslation,{Speed,0,0}}; return c;
  }
  bool InitializeMoving() {
    if(!Owner()) return false;
    const auto report=batch.Initialize(Config(),element.data());
    EXPECT_EQ(report.status,t::BatchStatus::Success); return report.status==t::BatchStatus::Success;
  }
  bool BindMoving() {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    EXPECT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    const auto report=batch.AssembleAccepted(owner,view);
    EXPECT_EQ(report.status,t::BatchStatus::Success);
    owner.Discard(); batch.DiscardTrial(); return report.status==t::BatchStatus::Success;
  }
};
TEST_F(T3BatchCuda, UniformTranslationMeasuresK0AndTwoIntervalsMatchNative) {
  for(unsigned cells:{1u,2u}) {
    SCOPED_TRACE(cells);
    auto r=std::make_unique<MovingRig>(cells); ASSERT_TRUE(r->InitializeMoving()); ASSERT_TRUE(r->BindMoving());
    auto cache=std::make_unique<Results>(),next=std::make_unique<Results>(); t::BatchDiagnostics initial,d;
    ASSERT_TRUE(Accepted(*r,*cache,initial));
    long double expected=0;
    for(unsigned n=0;n<r->n;++n) expected+=.5L*r->mass[n]*Speed*Speed;
    EXPECT_TRUE(initial.kinetic_available); EXPECT_GT(initial.kinetic_translation,0);
    EXPECT_LE(std::abs(initial.kinetic_translation-expected),256*std::numeric_limits<double>::epsilon()*expected+1e-18L);
    EXPECT_EQ(initial.kinetic_rotation,0); EXPECT_FALSE(initial.has_completed_interval);
    EXPECT_EQ(initial.epoch,0u); EXPECT_EQ(initial.time,0); EXPECT_EQ(initial.kick_dt,0);
    for(unsigned e=0;e<cells;++e) {
      EXPECT_EQ((*cache)[e].proposed_history.stamp().sample_index,0u);
      for(const auto f:(*cache)[e].internal_force) { EXPECT_EQ(f.x,0); EXPECT_EQ(f.y,0); EXPECT_EQ(f.z,0); }
    }
    const auto bytes=r->batch.allocations();
    for(unsigned step=0;step<2;++step) {
      fe::NodalTrialToken token; fe::NodalPreparedView p;
      ASSERT_TRUE(Prepare(*r,{},token,p,true)); ASSERT_TRUE(Candidate(*r,p,d,*next));
      const auto endpoint=Endpoint(*r,p);
      for(unsigned e=0;e<cells;++e) {
        const auto interval=Interval(*r,e,p,endpoint);
        ASSERT_NO_FATAL_FAILURE(oracle::Check(r->element[e].reference,(*cache)[e].proposed_history,interval,(*next)[e],true));
      }
      for(unsigned n=0;n<r->n;++n) {
        EXPECT_NEAR(endpoint.v[3*n],Speed,2e-13*(1+Speed));
        EXPECT_NEAR(endpoint.x[3*n],r->x[3*n]+(step+1)*H*Speed,2e-13);
      }
      EXPECT_EQ(p.kick_dt,step?H:H/2);
      ASSERT_TRUE(Commit(*r,token,d)); ASSERT_TRUE(Accepted(*r,*cache,d));
    }
    EXPECT_EQ(r->batch.allocations().device_bytes,bytes.device_bytes);
    EXPECT_EQ(r->batch.allocations().device_allocations,1u);
  }
  RecordProperty("native_t3_intervals",6);
}
TEST_F(T3BatchCuda, UniformMetadataOverflowAndLateMotionFaultsDoNotPublishKinetic) {
  auto valid=std::make_unique<MovingRig>(); ASSERT_TRUE(valid->Owner());
  for(unsigned fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault);
    auto c=valid->Config(); t::T3Batch batch;
    if(fault==0) c.startup.kind=static_cast<t::BatchStartupKind>(99);
    if(fault==1) c.startup.uniform_velocity.y=std::numeric_limits<double>::quiet_NaN();
    if(fault==2) c.startup.kind=t::BatchStartupKind::ReferenceRest;
    if(fault==3) c.usage=t::BatchUsage::PrescribedFields;
    if(fault==4) c.startup.uniform_velocity.x=std::numeric_limits<double>::max();
    EXPECT_EQ(batch.Initialize(c,valid->element.data()).status,fault==4?t::BatchStatus::NonfiniteResult:t::BatchStatus::InvalidInput);
    EXPECT_EQ(batch.allocations().device_allocations,0u);
  }
  for(unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);
    auto r=std::make_unique<MovingRig>(); const auto n=r->n-1,j=3*n;
    if(fault==0) r->velocity[j]=std::nextafter(Speed,9.);
    if(fault==1) r->omega[j]=.125;
    if(fault==2) { r->orientation[4*n]=0; r->orientation[4*n+1]=1; }
    if(fault==3) r->inverse[n]*=2;
    if(fault==4) r->inverse_j[n]*=2;
    if(fault==5) r->x[j]+=.125;
    if(fault==6) r->velocity[j+1]=-0.;
    ASSERT_TRUE(r->InitializeMoving());
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(r->owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    const auto report=fault==7?r->batch.AssembleAccepted(view):r->batch.AssembleAccepted(r->owner,view);
    EXPECT_EQ(report.status,fault==3||fault==4?t::BatchStatus::InvalidMass:t::BatchStatus::InvalidInput);
    if(fault!=7) EXPECT_EQ(report.node,n);
    EXPECT_EQ(r->owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
    r->owner.Discard(); r->batch.DiscardTrial();
    auto result=std::make_unique<Results>(); t::BatchDiagnostics d;
    d.kinetic_translation=123; const auto saved=Bytes(d);
    EXPECT_EQ(r->batch.CopyAcceptedResults(r->owner.accepted(),result->data(),result->size(),&d).status,t::BatchStatus::NotBound);
    EXPECT_EQ(Bytes(d),saved);
    if(fault==7) ASSERT_TRUE(r->BindMoving());
  }
}
} // namespace t3_uniform_startup_test
