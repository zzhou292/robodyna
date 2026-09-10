#include "../coupled/QephCoupledLedger.h"
#include "../../t3/mixed_binding/ShellBatchBindingFixture.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace qeph_uniform_startup_test {
namespace c=qeph_coupled_test;
namespace q=tl::fea::qeph;
namespace fe=tl::fea;
using namespace qeph_batch_test;
constexpr std::uint64_t StartupQualification=0x5155505354415231ULL;
constexpr double Speed=8,Gap=.000375/4;
struct UniformRig:Rig {
  std::array<double,3*N> velocity{},omega{};
  explicit UniformRig(unsigned cells):Rig(cells) {
    h=c::H0; mass.fill(0); inertia.fill(0); physical.fill(0); added.fill(0);
    for(unsigned e=0;e<count;++e) {
      auto input=element[e].reference.input;
      input.young_modulus=c::Young; input.poisson_ratio=c::Poisson;
      input.density=c::Density; input.thickness=c::Thickness;
      for(unsigned l=0;l<4;++l) {
        const auto old=input.position[l]; const auto n=element[e].nodes[l];
        input.position[l]={-Gap,c::Side*(old.x-.5*count),c::Side*old.y};
        const auto p=input.position[l]; x[3*n]=p.x; x[3*n+1]=p.y; x[3*n+2]=p.z;
      }
      const auto status=q::InitializeReference(input,element[e].reference);
      EXPECT_EQ(status,q::Status::kSuccess); valid&=status==q::Status::kSuccess;
      for(unsigned l=0;l<4;++l) {
        const auto n=element[e].nodes[l]; const auto& r=element[e].reference;
        mass[n]+=r.nodal_mass[l]; inertia[n]+=r.isotropic_inertia[l];
        physical[n]+=r.physical_inertia[l]; added[n]+=r.added_inertia[l];
      }
    }
    for(unsigned n=0;n<this->n;++n) { velocity[3*n]=Speed; inverse[n]=1/mass[n]; inverse_j[n]=1/inertia[n]; }
  }
  bool Owner() {
    fe::NodalStateConfig config; config.node_count=n; config.fixed_dt=h;
    config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto r=owner.Initialize(config,{x.data(),velocity.data(),omega.data(),n,orientation.data()},inverse.data(),
                                 {fixed.data(),fixed.data(),inverse_j.data()});
    EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return valid&&r.status==fe::NodalStatus::Ok;
  }
  q::QephBatchConfig Config() const {
    auto config=Rig::Config(q::BatchUsage::CoupledForces); config.qualification_id=StartupQualification;
    config.startup={q::BatchStartupKind::ReferenceUniformTranslation,{Speed,0,0}}; return config;
  }
  bool InitializeUniform() {
    if(!Owner()) return false;
    const auto r=batch.Initialize(Config(),element.data()); EXPECT_EQ(r.status,q::BatchStatus::Success)<<r.message;
    return r.status==q::BatchStatus::Success;
  }
  bool Bind() {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    const auto begun=owner.BeginTrial(&token,&view);
    EXPECT_EQ(begun.status,fe::NodalStatus::Ok); if(begun.status!=fe::NodalStatus::Ok) return false;
    const auto report=batch.AssembleAccepted(owner,view);
    EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
    owner.Discard(); batch.DiscardTrial(); return report.status==q::BatchStatus::Success;
  }
};
void Kinetic(const UniformRig& r,const q::BatchDiagnostics& d) {
  long double expected=0,terms=0;
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    const long double v=r.velocity[3*n+a],term=.5L*r.mass[n]*v*v; expected+=term; terms+=std::abs(term);
  }
  const long double scale=.5L*c::Density*c::Thickness*c::Side*c::Side*Speed*Speed*r.count;
  EXPECT_LE(std::abs(static_cast<long double>(d.kinetic_translation)-expected),
      256*std::numeric_limits<double>::epsilon()*terms+1e-12L*scale);
  EXPECT_TRUE(d.kinetic_available); EXPECT_GT(d.kinetic_translation,0);
  EXPECT_EQ(d.kinetic_rotation,0); EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
  EXPECT_EQ(d.epoch,0u); EXPECT_EQ(d.time,0); EXPECT_EQ(d.velocity_time,0); EXPECT_EQ(d.kick_dt,0);
  EXPECT_FALSE(d.has_completed_interval); EXPECT_FALSE(d.accepted_force_assembled); EXPECT_EQ(d.phase,q::BatchPhase::Accepted);
}
void Unbound(UniformRig& r) {
  c::PortResults output{}; q::BatchDiagnostics d; d.owner_id=999;
  const auto bytes=Bytes(output); const auto diag=Bytes(d);
  EXPECT_EQ(r.batch.CopyAcceptedResults(r.owner.accepted(),output.data(),output.size(),&d).status,q::BatchStatus::NotBound);
  EXPECT_EQ(Bytes(output),bytes); EXPECT_EQ(Bytes(d),diag);
}

TEST_F(QephBatchCuda, UniformStartupMeasuresNativeKineticAndKeepsZeroHistoryCache) {
  for(unsigned cells:{1u,2u}) {
    SCOPED_TRACE(cells); UniformRig r(cells); ASSERT_TRUE(r.InitializeUniform()); Unbound(r);
    const auto owner_bytes=r.owner.allocations(),batch_bytes=r.batch.allocations();
    ASSERT_TRUE(r.Bind()); c::PortResults cache{}; q::BatchDiagnostics d;
    ASSERT_TRUE(ReadAccepted(r,cache,d)); Kinetic(r,d);
    for(unsigned e=0;e<r.count;++e) {
      const auto& result=cache[e]; ASSERT_TRUE(result.proposed_history.prepared());
      EXPECT_EQ(result.proposed_history.stamp().time,0); EXPECT_EQ(result.proposed_history.stamp().sample_index,0u);
      const auto& history=result.proposed_history.data();
      EXPECT_EQ(history.thickness,c::Thickness);
      for(double x:history.stress) EXPECT_EQ(x,0);
      for(double x:history.material_stress) EXPECT_EQ(x,0);
      for(double x:history.bending_stress) EXPECT_EQ(x,0);
      for(double x:history.stabilization) EXPECT_EQ(x,0);
      for(double x:history.strain_curvature) EXPECT_EQ(x,0);
      for(double x:history.internal_work) EXPECT_EQ(x,0);
      EXPECT_EQ(history.hourglass_viscous_work,0);
      for(unsigned l=0;l<4;++l) {
        EXPECT_EQ(result.internal_force[l].x,0); EXPECT_EQ(result.internal_force[l].y,0); EXPECT_EQ(result.internal_force[l].z,0);
        EXPECT_EQ(result.internal_couple[l].x,0); EXPECT_EQ(result.internal_couple[l].y,0); EXPECT_EQ(result.internal_couple[l].z,0);
      }
    }
    const auto cache_bytes=Bytes(cache); const auto diag_bytes=Bytes(d);
    ASSERT_TRUE(ReadAccepted(r,cache,d)); EXPECT_EQ(Bytes(cache),cache_bytes); EXPECT_EQ(Bytes(d),diag_bytes);
    EXPECT_EQ(r.owner.allocations().device_bytes,owner_bytes.device_bytes);
    EXPECT_EQ(r.batch.allocations().device_bytes,batch_bytes.device_bytes);
    EXPECT_EQ(r.owner.allocations().device_allocations,6u); EXPECT_EQ(r.batch.allocations().device_allocations,1u);
  }
}

TEST_F(QephBatchCuda, UniformStartupFirstHalfKickAndZeroForceFlightMatchNative) {
  for(unsigned cells:{1u,2u}) for(unsigned refinement:{1u,2u}) for(bool accelerated:{false,true}) {
    SCOPED_TRACE(cells);
    SCOPED_TRACE(refinement);
    SCOPED_TRACE(accelerated);
    UniformRig r(cells); r.h=c::H0/refinement; ASSERT_TRUE(r.InitializeUniform()); ASSERT_TRUE(r.Bind());
    c::NativeSequence native; ASSERT_TRUE(native.Initialize(r));
    std::copy_n(r.velocity.begin(),3*r.n,native.state.v.begin());
    Loads load; if(accelerated) for(unsigned n=0;n<r.n;++n) load.force[3*n]=r.mass[n]; // Known 1 m/s^2.
    c::LedgerEvidence evidence;
    for(unsigned step=0;step<2;++step) {
      Snapshot base; ASSERT_TRUE(Read(r.owner,base)); c::OwnerAgreement(r,base,native.state);
      c::PortResults cache{},next{}; q::BatchDiagnostics old,d;
      ASSERT_TRUE(ReadAccepted(r,cache,old)); c::NativeProposal expected;
      ASSERT_TRUE(native.Propose(r,load,expected)); c::Prepared p;
      ASSERT_TRUE(c::PrepareCoupled(r,load,p,StartupQualification));
      ASSERT_TRUE(Candidate(r,p.view,d,next)); c::OwnerAgreement(r,p.state,expected.state);
      for(unsigned e=0;e<r.count;++e) {
        qeph_force_port_test::ForceAgreement(next[e],expected.cache[e],r.element[e].reference.input,
            c::Interval(r,e,p.state,native.time,native.epoch));
        for(unsigned i=0;i<8;++i)
          EXPECT_LE(std::abs(next[e].kinematics.regular_rate[i]),2e-12*Speed/(i<5?c::Side:c::Side*c::Side));
        for(unsigned i=0;i<6;++i)
          EXPECT_LE(std::abs(next[e].kinematics.hourglass_rate[i]),2e-12*Speed/(i==2||i==3?c::Side:1.));
      }
      c::CheckLedgers(r,base,p,load,cache,d,evidence); c::CheckSourceWork(r,cache,next,d,evidence);
      EXPECT_EQ(p.view.kick_dt,step?r.h:r.h/2); EXPECT_EQ(p.view.velocity_time,step*r.h+r.h/2);
      for(unsigned n=0;n<r.n;++n) {
        EXPECT_NEAR(p.state.v[3*n],Speed+(accelerated?(step+.5)*r.h:0),c::VelocityBudget());
        EXPECT_EQ(p.state.v[3*n+1],0); EXPECT_EQ(p.state.v[3*n+2],0);
        const double time=(step+1)*r.h;
        EXPECT_NEAR(p.state.x[3*n],-Gap+time*Speed+(accelerated?.5*time*time:0),2e-12*c::Side);
      }
      ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,p.token,d)); native.Accept(r,expected);
    }
  }
}

TEST_F(QephBatchCuda, UniformStartupMetadataOverflowAndJoinedScopeRejectBeforeAllocation) {
  UniformRig r(2); ASSERT_TRUE(r.Owner());
  for(unsigned fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault); auto config=r.Config(); q::QephBatch batch;
    if(fault==0) config.startup.kind=static_cast<q::BatchStartupKind>(99);
    if(fault==1) config.startup.uniform_velocity.y=std::numeric_limits<double>::quiet_NaN();
    if(fault==2) config.startup.kind=q::BatchStartupKind::ReferenceRest;
    if(fault==3) config.usage=q::BatchUsage::PrescribedFields;
    if(fault==4) config.startup.uniform_velocity.x=std::numeric_limits<double>::max();
    EXPECT_EQ(batch.Initialize(config,r.element.data()).status,fault==4?q::BatchStatus::NonfiniteResult:q::BatchStatus::InvalidInput);
    EXPECT_EQ(batch.allocations().device_allocations,0u);
  }
  // Each finite term fits; the repeated native mass contributions overflow
  // the aggregate after earlier positive terms. Failure-only unit-scale patch.
  Rig overflow(2); ASSERT_TRUE(overflow.InitializeOwner()); auto huge=overflow.Config(q::BatchUsage::CoupledForces);
  huge.startup={q::BatchStartupKind::ReferenceUniformTranslation,{4e153,0,0}};
  q::QephBatch failed; const auto report=failed.Initialize(huge,overflow.element.data());
  EXPECT_EQ(report.status,q::BatchStatus::NonfiniteResult); EXPECT_GT(report.node,0u);
  EXPECT_EQ(failed.allocations().device_allocations,0u);
  fe::ShellBatchBinding binding; const auto input=shell_binding_test::Edge();
  ASSERT_EQ(binding.Initialize(input).status,fe::ShellBindingStatus::Success);
  tl_test::nodal_temporal::Initial initial; initial.n=binding.node_count();
  for(unsigned n=0;n<initial.n;++n) {
    const auto& node=binding.nodes()[n]; initial.x[3*n]=node.position.x; initial.x[3*n+1]=node.position.y; initial.x[3*n+2]=node.position.z;
    initial.inverse[n]=1/node.native.mass; initial.inverse_inertia[n]=1/node.native.isotropic_inertia;
  }
  fe::FENodalState owner; ASSERT_EQ(initial.Initialize(owner).status,fe::NodalStatus::Ok);
  auto config=r.Config(); config.owner=owner.accepted(); config.element_count=1;
  for(auto usage:{q::BatchUsage::PrescribedFields,q::BatchUsage::CoupledForces}) {
    config.usage=usage; q::QephBatch joined;
    EXPECT_EQ(joined.InitializeJoined(config,binding).status,q::BatchStatus::InvalidInput);
    EXPECT_EQ(joined.allocations().device_allocations,0u);
  }
  config.startup={}; config.usage=q::BatchUsage::PrescribedFields; q::QephBatch legacy;
  ASSERT_EQ(legacy.InitializeJoined(config,binding).status,q::BatchStatus::Success);
}

TEST_F(QephBatchCuda, UniformStartupLateSourceFaultsKeepInitialDiagnosticsUnbound) {
  for(unsigned fault=0;fault<9;++fault) {
    SCOPED_TRACE(fault); UniformRig r(2); const auto i=3*(r.n-1);
    if(fault==0) r.velocity[i]=std::nextafter(Speed,9.);
    if(fault==1) for(unsigned n=0;n<r.n;++n) r.velocity[3*n]=9;
    if(fault==2) r.omega[i+1]=.125;
    if(fault==3) { r.orientation[4*(r.n-1)]=0; r.orientation[4*(r.n-1)+1]=1; }
    if(fault==4) r.inverse[r.n-1]*=2;
    if(fault==5) r.inverse_j[r.n-1]*=2;
    if(fault==6) r.x[i]+=.125;
    if(fault==7) r.velocity[i+1]=-0.; // Declared common vector binds represented zero sign too.
    ASSERT_TRUE(r.InitializeUniform()); Snapshot before; ASSERT_TRUE(Read(r.owner,before));
    fe::NodalTrialToken token; fe::NodalAssemblyView a; ASSERT_EQ(r.owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    if(fault==8) {
      Loads invalid; invalid.force[i]=std::numeric_limits<double>::infinity();
      AddLoads<<<1,1,0,a.stream>>>(a,invalid,1.);
      ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
    }
    const auto report=r.batch.AssembleAccepted(r.owner,a);
    EXPECT_EQ(report.status,fault==8?q::BatchStatus::AssemblyFailure:
              (fault==4||fault==5?q::BatchStatus::InvalidMass:q::BatchStatus::InvalidInput));
    if(fault==8) EXPECT_EQ(report.element,1u);
    else if(fault!=1) EXPECT_EQ(report.node,r.n-1);
    EXPECT_EQ(r.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
    r.owner.Discard(); r.batch.DiscardTrial(); Unbound(r);
    Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(before,after);
  }
}

TEST_F(QephBatchCuda, UniformStartupFirstTrialRejectionKeepsKineticAndAuthenticSourceForRetry) {
  UniformRig r(2); ASSERT_TRUE(r.InitializeUniform()); ASSERT_TRUE(r.Bind());
  c::PortResults cache{},next{}; q::BatchDiagnostics initial,d;
  ASSERT_TRUE(ReadAccepted(r,cache,initial)); Kinetic(r,initial);
  const auto bytes=Bytes(cache); const auto diag=Bytes(initial); Snapshot accepted; ASSERT_TRUE(Read(r.owner,accepted));
  {
    UniformRig wrong_mass(2); wrong_mass.inverse[wrong_mass.n-1]*=2; ASSERT_TRUE(wrong_mass.Owner());
    fe::NodalTrialToken token,other_token; fe::NodalAssemblyView actual,other;
    ASSERT_EQ(r.owner.BeginTrial(&token,&actual).status,fe::NodalStatus::Ok);
    ASSERT_EQ(wrong_mass.owner.BeginTrial(&other_token,&other).status,fe::NodalStatus::Ok);
    auto bad=actual; bad.mass.inverse_mass=other.mass.inverse_mass;
    const auto failed=r.batch.AssembleAccepted(bad); EXPECT_EQ(failed.status,q::BatchStatus::InvalidMass);
    EXPECT_EQ(failed.node,r.n-1); r.owner.Discard(); r.batch.DiscardTrial(); wrong_mass.owner.Discard();
    ASSERT_TRUE(ReadAccepted(r,cache,initial)); EXPECT_EQ(Bytes(cache),bytes); EXPECT_EQ(Bytes(initial),diag);
  }
  c::Prepared rejected; ASSERT_TRUE(c::PrepareCoupled(r,Loads{},rejected,StartupQualification));
  ASSERT_TRUE(Candidate(r,rejected.view,d,next)); auto receipt=Receipt(d); ++receipt.qualification_id;
  EXPECT_EQ(q::CommitQephTrial(r.owner,rejected.token,r.batch,d,receipt).status,q::BatchStatus::StaleTrial);
  ASSERT_TRUE(ReadAccepted(r,cache,initial)); EXPECT_EQ(Bytes(cache),bytes); EXPECT_EQ(Bytes(initial),diag);
  Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(accepted,after);
  c::Prepared retry; ASSERT_TRUE(c::PrepareCoupled(r,Loads{},retry,StartupQualification));
  EXPECT_EQ(retry.view.kick_dt,r.h/2); EXPECT_EQ(retry.view.base_time,0); EXPECT_EQ(retry.view.velocity_time,r.h/2);
  EXPECT_EQ(retry.state.x,rejected.state.x); EXPECT_EQ(retry.state.v,rejected.state.v);
  ASSERT_TRUE(Candidate(r,retry.view,d,next)); ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,retry.token,d));
  EXPECT_EQ(r.owner.accepted().epoch,1u);
  // A forged declared-speed buffer must not expose K0 while the actual owner
  // is at rest. Raw first binding rejects too, even with genuine source bits.
  UniformRig bad(1),foreign(1); bad.velocity.fill(0);
  ASSERT_TRUE(bad.InitializeUniform()); ASSERT_TRUE(foreign.Owner());
  Snapshot before; ASSERT_TRUE(Read(bad.owner,before));
  fe::NodalTrialToken token,other_token; fe::NodalAssemblyView actual,other;
  ASSERT_EQ(foreign.owner.BeginTrial(&other_token,&other).status,fe::NodalStatus::Ok);
  for(unsigned path=0;path<3;++path) {
    SCOPED_TRACE(path);
    ASSERT_EQ(bad.owner.BeginTrial(&token,&actual).status,fe::NodalStatus::Ok);
    auto forged=actual; if(path<2) forged.accepted.velocity_xyz=other.accepted.velocity_xyz;
    const auto failed=path==0?bad.batch.AssembleAccepted(forged):bad.batch.AssembleAccepted(bad.owner,forged);
    EXPECT_EQ(failed.status,path==1?q::BatchStatus::StaleTrial:q::BatchStatus::InvalidInput);
    if(path==1) EXPECT_EQ(failed.nodal_status,fe::NodalStatus::StaleTrial);
    EXPECT_EQ(bad.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
    bad.owner.Discard(); bad.batch.DiscardTrial(); Unbound(bad);
  }
  foreign.owner.Discard();
  Snapshot bad_after; ASSERT_TRUE(Read(bad.owner,bad_after)); SameState(before,bad_after);
  // A genuine uniform owner can retry first binding after a raw-view rejection;
  // no partial K0/source/attempt publication makes the clean overload stale.
  UniformRig poisoned(1); ASSERT_TRUE(poisoned.InitializeUniform());
  ASSERT_EQ(poisoned.owner.BeginTrial(&token,&actual).status,fe::NodalStatus::Ok);
  EXPECT_EQ(poisoned.batch.AssembleAccepted(actual).status,q::BatchStatus::InvalidInput);
  poisoned.owner.Discard(); poisoned.batch.DiscardTrial(); Unbound(poisoned);
  ASSERT_TRUE(poisoned.Bind());
  ASSERT_TRUE(ReadAccepted(poisoned,cache,initial)); const auto prior=Bytes(cache); const auto prior_d=Bytes(initial);
  Kinetic(poisoned,initial);
  Noop<<<1,0>>>(); ASSERT_NE(cudaPeekAtLastError(),cudaSuccess);
  EXPECT_EQ(poisoned.batch.CopyAcceptedResults(poisoned.owner.accepted(),cache.data(),cache.size(),&initial).status,
            q::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(cache),prior); EXPECT_EQ(Bytes(initial),prior_d);
}
} // namespace qeph_uniform_startup_test
