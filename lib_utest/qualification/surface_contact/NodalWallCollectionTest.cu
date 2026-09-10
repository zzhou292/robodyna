#include "NodalWallCollectionFixture.h"
#include <cmath>
#include <memory>

namespace {
using namespace nodal_wall_collection_test;
__global__ void Set(double* values,unsigned index,double value) { if(!threadIdx.x) values[index]=value; }
__global__ void Seed(fe::NodalAssemblyView v) {
  const unsigned n=threadIdx.x; if(n>=v.accepted.node_count) return;
  double* fields[]{v.forces.force_x,v.forces.force_y,v.forces.force_z,
                  v.forces.couple_x,v.forces.couple_y,v.forces.couple_z};
  for(unsigned c=0;c<6;++c) fields[c][n]=(c+1)*(n+1)/1024.;
}
bool Advance(fe::FENodalState& owner,const fe::NodalTrialToken& token,const fe::NodalAssemblyView& a,
             fe::NodalPreparedView& p) {
  auto r=owner.SealAssembly(token); EXPECT_EQ(r.status,fe::NodalStatus::Ok); if(r.status!=fe::NodalStatus::Ok) return false;
  r=fe::AdvanceStaggeredHistory(owner,token,{a.owner_id,a.accepted.base_epoch,a.attempt,Step,.1,Qualification});
  EXPECT_EQ(r.status,fe::NodalStatus::Ok); if(r.status!=fe::NodalStatus::Ok) return false;
  r=owner.BorrowPrepared(token,&p); EXPECT_EQ(r.status,fe::NodalStatus::Ok); return r.status==fe::NodalStatus::Ok;
}
void CheckScatter(const Forces& before,const Forces& after,const sc::NodalWallDeviceResults& result) {
  for(unsigned n=0;n<Nodes;++n) {
    EXPECT_EQ(after[n],before[n]+result.nodes[n].force_world.x);
    for(unsigned c=1;c<6;++c) EXPECT_EQ(after[c*Nodes+n],before[c*Nodes+n]);
  }
}
void Retry(Fixture& f,fe::FENodalState& owner,sc::NodalWallContactDevice& contact,sc::NodalWallConfig law) {
  fe::NodalTrialToken token; fe::NodalAssemblyView a;
  ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  sc::NodalWallDiagnostics d; ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
  sc::NodalWallDeviceResults out; ASSERT_EQ(contact.CopyResults(d,&out).status,Code::Ok);
  Same(out,f.Host(f.x,f.v,0,a.attempt,law));
  owner.Discard(); contact.DiscardTrial();
}

TEST(NodalWallCollection, CompleteCapacityNativeWeightsAndLateModelFaultsAreStaged) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare());
  EXPECT_EQ(f->q4_count,82u); EXPECT_EQ(f->t3_count,46u);
  EXPECT_EQ(f->weights.node_count(),128u); EXPECT_EQ(f->weights.parent_count(),128u);
  EXPECT_EQ(f->weights.node(127).node,127u);
  EXPECT_EQ(sizeof(detail::Model),105976u); EXPECT_EQ(sizeof(sc::NodalWallDeviceResults),79248u);
  EXPECT_EQ(sizeof(detail::Storage),471864u); EXPECT_LE(sizeof(detail::Storage),sc::MaxNodalWallDeviceBytes);
  EXPECT_EQ(detail::Workers,128u); EXPECT_EQ(sc::MaxNodalWallDeviceBytes,512u*1024);
  detail::Model model;
  auto prepare=[&](sc::NodalWallDeviceConfig c) { return detail::PrepareModel(c,f->wall.view(),f->weights,
      f->View(f->x),f->inverse.data(),f->fixed.data(),f->motion,&model); };
  ASSERT_EQ(prepare(f->Config()).status,Code::Ok); const auto saved=Bytes(model);
  auto c=f->Config(); c.max_device_bytes=sizeof(detail::Storage)-1;
  EXPECT_EQ(prepare(c).status,Code::ResourceLimit); Unchanged(model,saved);
  f->fixed[127]=6; EXPECT_EQ(prepare(f->Config()).status,Code::InvalidMass); Unchanged(model,saved); f->fixed[127]=0;
  const double inverse=f->inverse[127]; f->inverse[127]=0;
  EXPECT_EQ(prepare(f->Config()).status,Code::InvalidMass); Unchanged(model,saved); f->inverse[127]=inverse;
  f->x[3*127+1]=1.25; EXPECT_EQ(prepare(f->Config()).status,Code::GeometryFailure); Unchanged(model,saved);
  f->x[3*127+1]=f->reference[3*127+1]; ASSERT_EQ(prepare(f->Config()).status,Code::Ok);
  const auto weights=Bytes(f->weights);
  EXPECT_EQ(f->weights.Initialize(129,f->input.data(),128).status,sc::NodalWallStatus::Capacity); Unchanged(f->weights,weights);
  EXPECT_EQ(f->weights.Initialize(128,f->input.data(),129).status,sc::NodalWallStatus::Capacity); Unchanged(f->weights,weights);
  auto reverse=std::make_unique<Fixture>(); ASSERT_TRUE(reverse->Prepare(128,true));
  EXPECT_EQ(f->mass,reverse->mass); EXPECT_EQ(f->inertia,reverse->inertia);
  for(unsigned p=0;p<Parents;++p) {
    EXPECT_EQ(f->weights.parent(p).parent_element_id,reverse->weights.parent(p).parent_element_id);
    Same(f->weights.parent(p).share,reverse->weights.parent(p).share);
  }
  RecordProperty("device_bytes",std::to_string(sizeof(detail::Storage)));
}

TEST(NodalWallCollectionCuda, All94And128ParentsMatchHostAndIndependentLoadsAtHighNodes) {
  for(unsigned count:{94u,128u}) {
    SCOPED_TRACE(count); auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare(count,true));
    if(count==94) { EXPECT_EQ(f->q4_count,88u); EXPECT_EQ(f->t3_count,6u); }
    fe::FENodalState owner; ASSERT_TRUE(f->Owner(owner)); sc::NodalWallContactDevice contact;
    ASSERT_TRUE(f->Bind(owner,contact,f->Config()));
    const auto oa=owner.allocations(),ca=contact.allocations();
    EXPECT_EQ(oa.device_allocations,6u); EXPECT_EQ(ca.device_allocations,1u);
    EXPECT_EQ(ca.device_bytes,471864u);
    fe::NodalTrialToken token; fe::NodalAssemblyView a;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    Seed<<<1,128,0,a.stream>>>(a); ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    const auto before=ReadForces(a);
    sc::NodalWallDiagnostics d; ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
    sc::NodalWallDeviceResults result; ASSERT_EQ(contact.CopyResults(d,&result).status,Code::Ok);
    Same(result,f->Host(f->x,f->v,0,a.attempt,f->Config().law));
    CheckLoads(*f,result,f->x,f->v,16); CheckScatter(before,ReadForces(a),result);
    EXPECT_EQ(result.nodes[127].node,127u); EXPECT_GT(result.nodes[127].force.lower,0);
    for(unsigned p=0;p<count;++p) EXPECT_TRUE(result.parents[p].valid);
    for(unsigned p=count;p<Parents;++p) { EXPECT_FALSE(result.parents[p].valid); EXPECT_EQ(result.parents[p].resultant.value,0); }
    for(unsigned p=0;p<count;++p) if(result.parents[p].arity==3) EXPECT_EQ(result.parents[p].force[3].upper,0);
    owner.Discard(); contact.DiscardTrial();
    ASSERT_NO_FATAL_FAILURE(Retry(*f,owner,contact,f->Config().law));
    EXPECT_EQ(owner.allocations().device_bytes,oa.device_bytes); EXPECT_EQ(owner.allocations().device_allocations,oa.device_allocations);
    EXPECT_EQ(contact.allocations().device_bytes,ca.device_bytes); EXPECT_EQ(contact.allocations().device_allocations,ca.device_allocations);
  }
}

TEST(NodalWallCollectionCuda, TwoContactOnlyKicksRetainBaseWorkAndWholeCandidateWithoutAllocations) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare());
  for(unsigned n=0;n<Nodes;++n) { f->x[3*n]=1./32; f->v[3*n]=0; }
  fe::FENodalState owner; ASSERT_TRUE(f->Owner(owner)); sc::NodalWallContactDevice contact;
  ASSERT_TRUE(f->Bind(owner,contact,f->Config()));
  EXPECT_LT(Step*std::sqrt(contact.stiffness_rate_bound()),.01);
  const auto oa=owner.allocations(),ca=contact.allocations(); auto accepted=Read(owner);
  for(unsigned step=0;step<2;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView a; fe::NodalPreparedView p;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics base,end; ASSERT_EQ(contact.AssembleAccepted(a,&base).status,Code::Ok);
    const auto force=ReadForces(a); ASSERT_TRUE(Advance(owner,token,a,p)); const auto candidate=ReadPrepared(p);
    ASSERT_EQ(contact.EvaluateCandidate(p,&end).status,Code::Ok); EXPECT_EQ(ReadForces(a),force); Same(Read(owner),accepted);
    sc::NodalWallDeviceResults result; ASSERT_EQ(contact.CopyResults(end,&result).status,Code::Ok);
    Same(result,f->Host(candidate.x,candidate.v,step,a.attempt,f->Config().law));
    const long double kick=step?Step:Step/2.L; long double work=0,drift=0,kinetic=0,terms=0,impulse=0;
    for(unsigned n=0;n<Nodes;++n) {
      const long double mass=1.L/f->inverse[n],va=accepted.v[3*n],vb=candidate.v[3*n];
      const long double fn=-16.L*f->area[n]*accepted.x[3*n];
      Near(vb,va+kick*fn/mass); Near(candidate.x[3*n],accepted.x[3*n]+Step*(va+kick*fn/mass));
      const long double dw=kick*fn*(va+vb)/2,dx=candidate.x[3*n]-static_cast<long double>(accepted.x[3*n]);
      work+=dw; drift+=fn*dx; kinetic+=mass*(vb*vb-va*va)/2; impulse-=kick*fn;
      terms+=std::abs(dw)+std::abs(fn*dx)+mass*(vb*vb+va*va)/2;
    }
    const long double budget=256*std::numeric_limits<double>::epsilon()*terms+1e-12L/64;
    EXPECT_LE(std::abs(end.kick_work-work),budget); EXPECT_LE(std::abs(end.kick_work-kinetic),budget);
    EXPECT_LE(std::abs(end.drift_work-drift),budget);
    EXPECT_LE(std::abs(end.wall_kick_impulse-impulse),256*std::numeric_limits<double>::epsilon()*std::abs(impulse)+1e-12L/1024);
    EXPECT_EQ(end.kick_dt,static_cast<double>(kick)); EXPECT_NE(end.resultant.value,base.resultant.value);
    EXPECT_GE(end.conservative_defect+end.work_uncertainty,0);
    EXPECT_LE(end.conservative_defect-end.work_uncertainty,end.quadratic_work_upper);
    ASSERT_FALSE(::testing::Test::HasFailure());
    ASSERT_EQ(fe::CompleteNodalValidation(owner,token,{p.owner_id,p.kinematics.base_epoch,p.attempt,Qualification,true}).status,fe::NodalStatus::Ok);
    ASSERT_EQ(owner.Commit(token).status,fe::NodalStatus::Ok); accepted=Read(owner); contact.DiscardTrial();
  }
  EXPECT_EQ(owner.allocations().device_bytes,oa.device_bytes); EXPECT_EQ(owner.allocations().device_allocations,oa.device_allocations);
  EXPECT_EQ(contact.allocations().device_bytes,ca.device_bytes); EXPECT_EQ(contact.allocations().device_allocations,ca.device_allocations);
}

TEST(NodalWallCollectionCuda, LateHighNodeMassGeometryFixedAndParentFailuresPreserveOutputsAndRetry) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare(94));
  fe::FENodalState owner; ASSERT_TRUE(f->Owner(owner)); sc::NodalWallContactDevice contact;
  ASSERT_TRUE(f->Bind(owner,contact,f->Config())); const auto accepted=Read(owner);
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault); fe::NodalTrialToken token; fe::NodalAssemblyView a; fe::NodalPreparedView p;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics d; d.owner_id=999; const auto sentinel=Bytes(d);
    if(fault==0) {
      struct Buffer { double* p=nullptr; ~Buffer(){if(p) cudaFree(p);} } mass;
      ASSERT_EQ(cudaMalloc(&mass.p,Nodes*sizeof(double)),cudaSuccess);
      auto bad=f->inverse; bad[127]=0;
      ASSERT_EQ(cudaMemcpyAsync(mass.p,bad.data(),Nodes*sizeof(double),cudaMemcpyHostToDevice,a.stream),cudaSuccess);
      auto wrong=a; wrong.mass.inverse_mass=mass.p; const auto before=ReadForces(a);
      const auto r=contact.AssembleAccepted(wrong,&d); EXPECT_EQ(r.status,Code::InvalidMass); EXPECT_EQ(r.node,127u);
      EXPECT_EQ(ReadForces(a),before); Unchanged(d,sentinel);
    } else {
      ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
      sc::NodalWallDeviceResults held; ASSERT_EQ(contact.CopyResults(d,&held).status,Code::Ok); const auto bytes=Bytes(held);
      ASSERT_TRUE(Advance(owner,token,a,p)); const auto before=ReadForces(a); const auto diagnostic=Bytes(d);
      const unsigned coordinate=3*127+(fault==2?1:0);
      const double value=fault==1?.75:(fault==2?1.25:std::numeric_limits<double>::quiet_NaN());
      Set<<<1,1,0,a.stream>>>(const_cast<double*>(p.kinematics.position_xyz),coordinate,value);
      ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
      const auto r=contact.EvaluateCandidate(p,&d);
      EXPECT_EQ(r.status,fault==1?Code::PointFailure:Code::GeometryFailure); EXPECT_EQ(r.node,127u);
      Unchanged(d,diagnostic); EXPECT_NE(contact.CopyResults(d,&held).status,Code::Ok); Unchanged(held,bytes);
      EXPECT_EQ(ReadForces(a),before);
    }
    owner.Discard(); contact.DiscardTrial(); Same(Read(owner),accepted);
    ASSERT_NO_FATAL_FAILURE(Retry(*f,owner,contact,f->Config().law));
  }
  auto sparse=std::make_unique<Fixture>(); ASSERT_TRUE(sparse->Prepare(94));
  for(unsigned n=0;n<Nodes;++n) { sparse->x[3*n]=0; sparse->v[3*n]=0; }
  sparse->x[3*127]=1./32;
  fe::FENodalState sparse_owner; ASSERT_TRUE(sparse->Owner(sparse_owner)); sc::NodalWallContactDevice tiny;
  auto strict=sparse->Config(); strict.law.parent_energy_error=1e-30; ASSERT_TRUE(sparse->Bind(sparse_owner,tiny,strict));
  fe::NodalTrialToken token; fe::NodalAssemblyView a;
  ASSERT_EQ(sparse_owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  sc::NodalWallDiagnostics d; d.owner_id=999; const auto diagnostic=Bytes(d);
  const auto before=ReadForces(a);
  const auto r=tiny.AssembleAccepted(a,&d); EXPECT_EQ(r.status,Code::Accuracy); EXPECT_GT(r.parent,64u);
  EXPECT_EQ(ReadForces(a),before); Unchanged(d,diagnostic); sparse_owner.Discard(); tiny.DiscardTrial();
  sparse->x[3*127]=-1./32; sparse->fixed[127]=7; sparse->inverse[127]=0;
  fe::FENodalState fixed_owner; ASSERT_TRUE(sparse->Owner(fixed_owner)); sc::NodalWallContactDevice fixed;
  ASSERT_TRUE(sparse->Bind(fixed_owner,fixed,sparse->Config()));
  ASSERT_EQ(fixed_owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  ASSERT_EQ(fixed.AssembleAccepted(a,&d).status,Code::Ok); sc::NodalWallDeviceResults held;
  ASSERT_EQ(fixed.CopyResults(d,&held).status,Code::Ok); EXPECT_TRUE(held.nodes[127].fixed); EXPECT_EQ(held.nodes[127].force.upper,0);
  fe::NodalPreparedView p; ASSERT_TRUE(Advance(fixed_owner,token,a,p)); const auto bytes=Bytes(held);
  const auto db=Bytes(d);
  Set<<<1,1,0,a.stream>>>(const_cast<double*>(p.kinematics.position_xyz),3*127,-1./64);
  ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
  EXPECT_EQ(fixed.EvaluateCandidate(p,&d).status,Code::GeometryFailure); Unchanged(d,db);
  EXPECT_NE(fixed.CopyResults(d,&held).status,Code::Ok); Unchanged(held,bytes);
  fixed_owner.Discard(); fixed.DiscardTrial();
  ASSERT_NO_FATAL_FAILURE(Retry(*sparse,fixed_owner,fixed,sparse->Config().law));
}

TEST(NodalWallCollectionCuda, Late128thDestinationOverflowLeavesAllSixArraysAndExactCleanRetry) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare(94));
  fe::FENodalState owner; ASSERT_TRUE(f->Owner(owner)); sc::NodalWallContactDevice contact;
  auto config=f->Config(); config.law.stiffness_per_area=1e308;
  config.law.parent_force_error=1e296; config.law.parent_energy_error=1e296;
  ASSERT_TRUE(f->Bind(owner,contact,config)); const auto accepted=Read(owner);
  fe::NodalTrialToken token; fe::NodalAssemblyView a;
  ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  Seed<<<1,128,0,a.stream>>>(a);
  Set<<<1,1,0,a.stream>>>(a.forces.force_x,127,-std::numeric_limits<double>::max());
  ASSERT_EQ(cudaGetLastError(),cudaSuccess); const auto before=ReadForces(a);
  sc::NodalWallDiagnostics d; d.owner_id=999; const auto db=Bytes(d);
  const auto r=contact.AssembleAccepted(a,&d); EXPECT_EQ(r.status,Code::AssemblyFailure); EXPECT_EQ(r.node,127u);
  EXPECT_EQ(ReadForces(a),before); Unchanged(d,db); Same(Read(owner),accepted);
  owner.Discard(); contact.DiscardTrial();
  ASSERT_NO_FATAL_FAILURE(Retry(*f,owner,contact,config.law)); Same(Read(owner),accepted);
  EXPECT_EQ(contact.allocations().device_allocations,1u); EXPECT_EQ(contact.allocations().device_bytes,471864u);
}
} // namespace
