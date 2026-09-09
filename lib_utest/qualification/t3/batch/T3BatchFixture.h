#pragma once
#include "../T3ForcePortFixture.h"
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"
#include <array>

namespace t3_batch_test {
namespace fe=tl::fea;
namespace t=tl::fea::t3;
namespace native=tl::qualification::t3;
namespace oracle=t3_force_port_test;
using oracle::Bytes;
using tl_test::nodal_temporal::Snapshot;
using tl_test::nodal_temporal::Read;
using tl_test::nodal_temporal::SameStamp;
using tl_test::nodal_temporal::SameState;
using tl_test::nodal_temporal::Loads;
using tl_test::nodal_temporal::AddLoads;
using tl_test::nodal_temporal::Noop;
using T3BatchCuda=tl_test::nodal_temporal::NodalTemporalCuda;
constexpr unsigned N=t::MaxBatchNodes;
constexpr std::uint64_t Qualification=0x5433423250524531ULL; // T3 prescribed publication only.
constexpr double H=1./1024;
constexpr long double EnergyScale=1e-6L; // J, frozen nonzero aggregate comparison scale.
constexpr double Targets[4]{1,0,-1,0}; // Exact schedule: load, hold, reverse, hold.
using Results=std::array<t::ForceTrial,t::MaxBatchElements>;
struct Rig {
  unsigned count,n;
  std::array<t::T3BatchElement,2> element{};
  std::array<double,3*N> x{},zero{};
  std::array<double,4*N> orientation{};
  std::array<double,N> mass{},inertia{},physical{},added{},inverse{},inverse_j{};
  std::array<std::uint8_t,N> fixed{},rotation_fixed{};
  fe::FENodalState owner; t::T3Batch batch;
  explicit Rig(unsigned cells=2):count(cells),n(cells==1?3:4) {
    const t::Vec3 position[4]{{0,0,0},{1,0,0},{.25,.75,0},{1.5,1,0}};
    const unsigned connectivity[2][3]{{0,1,2},{1,3,2}};
    for(unsigned i=0;i<n;++i) {
      orientation[4*i]=1; x[3*i]=position[i].x; x[3*i+1]=position[i].y;
    }
    for(unsigned e=0;e<count;++e) {
      auto& cell=element[e]; t::ReferenceInput in;
      in.young_modulus=2e6; in.density=1024; in.thickness=1./32;
      for(unsigned i=0;i<3;++i) {
        const auto node=connectivity[e][i]; cell.nodes[i]=node;
        in.position[i]=position[node]; in.node_ids[i]=(std::uint64_t{1}<<54)+100+node;
      }
      cell.reference=oracle::Reference(in);
      for(unsigned i=0;i<3;++i) {
        const auto node=cell.nodes[i]; mass[node]+=cell.reference.nodal_mass[i];
        inertia[node]+=cell.reference.isotropic_inertia[i]; physical[node]+=cell.reference.physical_inertia[i];
        added[node]+=cell.reference.added_inertia[i];
      }
    }
    for(unsigned i=0;i<n;++i) { inverse[i]=1/mass[i]; inverse_j[i]=1/inertia[i]; }
  }
  t::T3BatchConfig Config(t::BatchUsage usage=t::BatchUsage::PrescribedFields) const {
    t::T3BatchConfig c; c.owner=owner.accepted(); c.element_count=count;
    c.configuration_id=0x5433424d4f444c31ULL; c.qualification_id=Qualification; c.usage=usage; return c;
  }
  bool InitializeOwner() {
    fe::NodalStateConfig c; c.node_count=n; c.fixed_dt=H; c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto r=owner.Initialize(c,{x.data(),zero.data(),zero.data(),n,orientation.data()},inverse.data(),
        {fixed.data(),rotation_fixed.data(),inverse_j.data()});
    EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return r.status==fe::NodalStatus::Ok;
  }
  bool Initialize(t::BatchUsage usage=t::BatchUsage::PrescribedFields) {
    if(!InitializeOwner()) return false;
    const auto r=batch.Initialize(Config(usage),element.data());
    EXPECT_EQ(r.status,t::BatchStatus::Success)<<r.message; return r.status==t::BatchStatus::Success;
  }
  bool Bind() {
    fe::NodalTrialToken token; fe::NodalAssemblyView v;
    const auto begin=owner.BeginTrial(&token,&v); EXPECT_EQ(begin.status,fe::NodalStatus::Ok);
    if(begin.status!=fe::NodalStatus::Ok) return false;
    const auto r=batch.AssembleAccepted(v); EXPECT_EQ(r.status,t::BatchStatus::Success)<<r.message;
    owner.Discard(); batch.DiscardTrial(); return r.status==t::BatchStatus::Success;
  }
};
// State-independent per-interval load schedule determined entirely before any
// force/history evaluation; zero target gives a true prescribed-rate hold up
// to kick roundoff, not merely zero acceleration with a carried velocity.
inline Loads Schedule(const Rig& r,unsigned interval) {
  Loads f; const double previous=interval?Targets[interval-1]:0;
  const double kick=interval?H:.5*H,change=(Targets[interval]-previous)/kick;
  for(unsigned n=0;n<r.n;++n) {
    const double x=r.x[3*n],y=r.x[3*n+1];
    const double v[3]{.001*(x+.25*y),.0005*(y-.5*x),.00075*(x-y)};
    const double w[3]{.002*y,.003*x,.001*(x+y)};
    for(unsigned a=0;a<3;++a) { f.force[3*n+a]=r.mass[n]*change*v[a]; f.couple[3*n+a]=r.inertia[n]*change*w[a]; }
  }
  return f;
}
inline bool Prepare(Rig& r,const Loads& load,fe::NodalTrialToken& token,fe::NodalPreparedView& p,bool consume=false) {
  fe::NodalAssemblyView v; auto report=r.owner.BeginTrial(&token,&v);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok) return false;
  if(consume) {
    const auto b=r.batch.AssembleAccepted(v); EXPECT_EQ(b.status,t::BatchStatus::Success);
    if(b.status!=t::BatchStatus::Success) return false;
  }
  AddLoads<<<1,1,0,v.stream>>>(v,load,1.);
  report=r.owner.SealAssembly(token); EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok) return false;
  report=fe::AdvanceStaggeredHistory(r.owner,token,{v.owner_id,v.accepted.base_epoch,v.attempt,H,1.,Qualification});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok)<<report.message; if(report.status!=fe::NodalStatus::Ok) return false;
  report=r.owner.BorrowPrepared(token,&p); EXPECT_EQ(report.status,fe::NodalStatus::Ok); return report.status==fe::NodalStatus::Ok;
}
inline bool Candidate(Rig& r,const fe::NodalPreparedView& p,t::BatchDiagnostics& d,Results& out) {
  auto report=r.batch.EvaluateCandidate(p,&d); EXPECT_EQ(report.status,t::BatchStatus::Success)<<report.message;
  if(report.status!=t::BatchStatus::Success) return false;
  report=r.batch.CopyPreparedResults(d,out.data(),out.size()); EXPECT_EQ(report.status,t::BatchStatus::Success);
  return report.status==t::BatchStatus::Success;
}
inline fe::NodalValidationReceipt Receipt(const t::BatchDiagnostics& d) {
  return {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true};
}
inline bool Commit(Rig& r,const fe::NodalTrialToken& token,const t::BatchDiagnostics& d) {
  const auto report=t::CommitT3Trial(r.owner,token,r.batch,d,Receipt(d));
  EXPECT_EQ(report.status,t::BatchStatus::Success)<<report.message; return report.status==t::BatchStatus::Success;
}
inline bool Accepted(Rig& r,Results& out,t::BatchDiagnostics& d) {
  const auto report=r.batch.CopyAcceptedResults(r.owner.accepted(),out.data(),out.size(),&d);
  EXPECT_EQ(report.status,t::BatchStatus::Success)<<report.message; return report.status==t::BatchStatus::Success;
}
inline Snapshot Endpoint(const Rig& r,const fe::NodalPreparedView& p) {
  Snapshot s;
  EXPECT_EQ(cudaMemcpyAsync(s.x.data(),p.kinematics.position_xyz,3*r.n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(s.v.data(),p.kinematics.velocity_xyz,3*r.n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaMemcpyAsync(s.omega.data(),p.kinematics.angular_velocity_xyz,3*r.n*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess); return s;
}
inline t::PrescribedInterval Interval(const Rig& r,unsigned e,const fe::NodalPreparedView& p,const Snapshot& s) {
  t::PrescribedInterval in; in.base_time=p.base_time; in.dt=H; in.sample_index=p.kinematics.base_epoch+1;
  for(unsigned i=0;i<3;++i) {
    const auto n=r.element[e].nodes[i]; in.position[i]={s.x[3*n],s.x[3*n+1],s.x[3*n+2]};
    in.velocity[i]={s.v[3*n],s.v[3*n+1],s.v[3*n+2]}; in.angular_velocity[i]={s.omega[3*n],s.omega[3*n+1],s.omega[3*n+2]};
  }
  return in;
}
} // namespace t3_batch_test
