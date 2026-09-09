#pragma once
#include "../QephForceFixture.h"
#include "lib_src/elements/qeph/QephBatch.h"
#include "lib_utest/qualification/nodal/NodalTemporalFixture.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <array>
#include <cstring>

namespace qeph_batch_test {
namespace fe=tl::fea;
namespace q=tl::fea::qeph;
namespace native=tl::qualification::qeph;
using qeph_force_port_test::Bytes;
constexpr std::size_t N=q::MaxBatchNodes;
constexpr std::uint64_t Qualification=0x4251325052455331ULL; // Prescribed fields/publication ONLY.
constexpr double LedgerTolerance=2e-12; // Frozen SI roundoff, no dynamics acceptance.
using tl_test::nodal_temporal::Snapshot;
using tl_test::nodal_temporal::Read;
using tl_test::nodal_temporal::SameStamp;
using tl_test::nodal_temporal::SameState;
using tl_test::nodal_temporal::Loads;
using tl_test::nodal_temporal::AddLoads;
using tl_test::nodal_temporal::Noop;
using QephBatchCuda=tl_test::nodal_temporal::NodalTemporalCuda;
struct Rig {
  unsigned count,n;
  double h=1./1024;
  std::array<q::QephBatchElement,4> element{};
  std::array<double,3*N> x{},zero{};
  std::array<double,4*N> orientation{};
  std::array<double,N> mass{},inertia{},physical{},added{},inverse{},inverse_j{};
  std::array<std::uint8_t,N> fixed{};
  fe::FENodalState owner; q::QephBatch batch;
  bool valid=true;
  explicit Rig(unsigned cells=2,bool warped=false,bool disjoint=false):count(cells),n(disjoint?4*cells:2*(cells+1)) {
    for(unsigned i=0;i<n;++i) orientation[4*i]=1;
    for(unsigned e=0;e<count;++e) {
      auto& item=element[e]; q::ReferenceInput input;
      input.young_modulus=2e6; input.density=1024; input.thickness=1./32;
      const unsigned ids[4]={disjoint?4*e:2*e,disjoint?4*e+1:2*e+2,
                             disjoint?4*e+2:2*e+3,disjoint?4*e+3:2*e+1};
      const double left=disjoint?3.*e:e,right=left+1;
      const q::Vec3 positions[4]={{left,-.5,0},{right,-.5,0},{right,.5,0},{left,.5,0}};
      for(unsigned i=0;i<4;++i) {
        item.nodes[i]=ids[i]; input.node_ids[i]=100+ids[i]; input.position[i]=positions[i];
        if(warped) input.position[i].z=((e+i)%2?-.015625:.015625);
        x[3*ids[i]]=input.position[i].x; x[3*ids[i]+1]=input.position[i].y; x[3*ids[i]+2]=input.position[i].z;
      }
      const auto status=q::InitializeReference(input,item.reference); valid&=status==q::Status::kSuccess;
      EXPECT_EQ(status,q::Status::kSuccess);
      for(unsigned i=0;i<4;++i) { const auto node=ids[i];
        mass[node]+=item.reference.nodal_mass[i]; inertia[node]+=item.reference.isotropic_inertia[i];
        physical[node]+=item.reference.physical_inertia[i]; added[node]+=item.reference.added_inertia[i];
      }
    }
    for(unsigned i=0;i<n;++i) { inverse[i]=1/mass[i]; inverse_j[i]=1/inertia[i]; }
  }
  q::QephBatchConfig Config(q::BatchUsage usage=q::BatchUsage::PrescribedFields) const {
    q::QephBatchConfig c; c.owner=owner.accepted(); c.element_count=count;
    c.configuration_id=0x42514d4f44454c31ULL; c.qualification_id=Qualification; c.usage=usage; return c;
  }
  bool InitializeOwner() {
    fe::NodalStateConfig c; c.node_count=n; c.fixed_dt=h; c.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    const auto r=owner.Initialize(c,{x.data(),zero.data(),zero.data(),n,orientation.data()},inverse.data(),
      fe::NodalDofConfig{fixed.data(),fixed.data(),inverse_j.data()});
    EXPECT_EQ(r.status,fe::NodalStatus::Ok)<<r.message; return valid&&r.status==fe::NodalStatus::Ok;
  }
  bool Initialize(q::BatchUsage usage=q::BatchUsage::PrescribedFields) {
    if(!InitializeOwner()) return false;
    const auto r=batch.Initialize(Config(usage),element.data()); EXPECT_EQ(r.status,q::BatchStatus::Success)<<r.message;
    return r.status==q::BatchStatus::Success;
  }
  bool Bind() {
    fe::NodalTrialToken token; fe::NodalAssemblyView v;
    auto r=owner.BeginTrial(&token,&v); EXPECT_EQ(r.status,fe::NodalStatus::Ok); if(r.status!=fe::NodalStatus::Ok) return false;
    const auto report=batch.AssembleAccepted(v); EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
    owner.Discard(); batch.DiscardTrial(); return report.status==q::BatchStatus::Success;
  }
};
inline Loads Pattern(const Rig& rig,double sign=1,bool rotations=true) {
  Loads f;
  for(unsigned n=0;n<rig.n;++n) for(unsigned a=0;a<3;++a) {
    f.force[3*n+a]=sign*rig.mass[n]*(.125*(static_cast<int>((n+2*a)%5)-2));
    if(rotations) f.couple[3*n+a]=sign*rig.inertia[n]*(.0625*(static_cast<int>((n+a)%5)-2));
  }
  return f;
}
inline bool Prepare(Rig& r,const Loads& load,fe::NodalTrialToken& token,fe::NodalPreparedView& p,bool consume=false) {
  fe::NodalAssemblyView v; auto report=r.owner.BeginTrial(&token,&v);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok) return false;
  if(consume) { const auto b=r.batch.AssembleAccepted(v); EXPECT_EQ(b.status,q::BatchStatus::Success); if(b.status!=q::BatchStatus::Success) return false; }
  AddLoads<<<1,1,0,v.stream>>>(v,load,1.);
  report=r.owner.SealAssembly(token); EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok) return false;
  // This qualification admits only stated prescribed operands/transaction
  // checks, not a free coupled response or any whole-recurrence timestep claim.
  report=fe::AdvanceStaggeredHistory(r.owner,token,{v.owner_id,v.accepted.base_epoch,v.attempt,r.h,1.,Qualification});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok) return false;
  report=r.owner.BorrowPrepared(token,&p); EXPECT_EQ(report.status,fe::NodalStatus::Ok); return report.status==fe::NodalStatus::Ok;
}
inline fe::NodalValidationReceipt Receipt(const q::BatchDiagnostics& d) {
  return {d.owner_id,d.base_epoch,d.attempt,d.qualification_id,true};
}
inline bool Candidate(Rig& r,const fe::NodalPreparedView& p,q::BatchDiagnostics& d,std::array<q::ForceTrial,4>& out) {
  auto report=r.batch.EvaluateCandidate(p,&d); EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
  if(report.status!=q::BatchStatus::Success) return false;
  report=r.batch.CopyPreparedResults(d,out.data(),out.size()); EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
  return report.status==q::BatchStatus::Success;
}
inline bool Commit(Rig& r,const fe::NodalTrialToken& t,const q::BatchDiagnostics& d) {
  const auto report=q::CommitQephTrial(r.owner,t,r.batch,d,Receipt(d)); EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message;
  return report.status==q::BatchStatus::Success;
}
inline bool ReadAccepted(Rig& r,std::array<q::ForceTrial,4>& out,q::BatchDiagnostics& d) {
  const auto report=r.batch.CopyAcceptedResults(r.owner.accepted(),out.data(),out.size(),&d);
  EXPECT_EQ(report.status,q::BatchStatus::Success)<<report.message; return report.status==q::BatchStatus::Success;
}
inline q::PrescribedInterval ReadInterval(const Rig& r,unsigned e,const fe::NodalPreparedView& view) {
  std::array<double,3*N> x{},v{},w{};
  EXPECT_EQ(cudaMemcpy(x.data(),view.kinematics.position_xyz,3*r.n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(cudaMemcpy(v.data(),view.kinematics.velocity_xyz,3*r.n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  EXPECT_EQ(cudaMemcpy(w.data(),view.kinematics.angular_velocity_xyz,3*r.n*sizeof(double),cudaMemcpyDeviceToHost),cudaSuccess);
  q::PrescribedInterval p; p.base_time=view.base_time; p.dt=r.h; p.sample_index=view.kinematics.base_epoch+1;
  for(unsigned i=0;i<4;++i) { const auto n=r.element[e].nodes[i];
    p.position_endpoint[i]={x[3*n],x[3*n+1],x[3*n+2]}; p.velocity_midpoint[i]={v[3*n],v[3*n+1],v[3*n+2]};
    p.omega_midpoint[i]={w[3*n],w[3*n+1],w[3*n+2]};
  }
  return p;
}
} // namespace qeph_batch_test
