#include "NodalWallCapacityFixture.h"
#include "NodalWallCapacityCudaProbe.h"
#include <cuda_runtime.h>

namespace {
using namespace nodal_wall_capacity_test;
using nodal_wall_owner_test::Bytes;
using nodal_wall_owner_test::Near;
using nodal_wall_owner_test::Same;
__global__ void Set(double* p,unsigned n,double value) { p[n]=value; }
std::vector<double> Forces(const fe::NodalAssemblyView& a) {
  std::vector<double> result(6*Nodes);
  const double* fields[]{a.forces.force_x,a.forces.force_y,a.forces.force_z,
      a.forces.couple_x,a.forces.couple_y,a.forces.couple_z};
  for(unsigned i=0;i<6;++i) EXPECT_EQ(cudaMemcpyAsync(result.data()+i*Nodes,fields[i],Nodes*sizeof(double),
      cudaMemcpyDeviceToHost,a.stream),cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess); return result;
}
void CheckLoads(const Fixture& f,const Results& result,const std::vector<double>& x) {
  long double force=0,energy=0;
  for(unsigned n=0;n<Nodes;++n) {
    const long double depth=std::max(0.L,static_cast<long double>(x[3*n]));
    const long double expected=16*f.area[n]*depth,potential=expected*depth/2;
    const auto& node=result.nodes[n]; EXPECT_EQ(node.node,n); EXPECT_TRUE(node.valid);
    EXPECT_LE(node.force.lower,expected); EXPECT_GE(node.force.upper,expected);
    EXPECT_LE(node.potential.lower,potential); EXPECT_GE(node.potential.upper,potential);
    EXPECT_GT(result.faces[n],FirstId); force+=expected; energy+=potential;
  }
  for(unsigned p=0;p<Parents;++p) {
    const auto& actual=result.parents[p]; const auto& source=f.weights.parent(p);
    EXPECT_TRUE(actual.valid); EXPECT_EQ(actual.parent_element_id,source.parent_element_id);
    EXPECT_EQ(actual.feature_id,source.feature_id); EXPECT_EQ(actual.arity,p<Quads?4u:3u);
    long double expected=0;
    for(unsigned l=0;l<actual.arity;++l) {
      const long double area=p<Quads?SquareArea/4:SquareArea/6;
      const long double term=16*area*std::max(0.L,static_cast<long double>(x[3*source.nodes[l]]));
      EXPECT_LE(actual.force[l].lower,term); EXPECT_GE(actual.force[l].upper,term); expected+=term;
    }
    if(p>=Quads) EXPECT_EQ(actual.force[3].value,0);
    Near(actual.resultant.value,expected);
  }
  Near(result.diagnostics.resultant.value,force); Near(result.diagnostics.potential.value,energy);
  EXPECT_EQ(result.diagnostics.parent_count,Parents); EXPECT_EQ(result.diagnostics.node_count,Nodes);
}
void ReadPositions(fe::FENodalState& owner,std::vector<double>& x,std::vector<double>& v) {
  fe::NodalStamp stamp; EXPECT_EQ(owner.CopyAccepted({x.data(),v.data(),Nodes},&stamp).status,fe::NodalStatus::Ok);
}
TEST(NodalWallCapacityCuda, Complete915NativeParentsAnd1030NodesAdvanceWithStableArenaAndFullLoads) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare()); fe::FENodalState owner;
  ASSERT_TRUE(f->Owner(owner)); sc::NodalWallContactDevice contact; ASSERT_TRUE(f->Bind(owner,contact));
  const auto contact_allocations=contact.allocations(),owner_allocations=owner.allocations();
  EXPECT_EQ(contact_allocations.device_allocations,1u); EXPECT_GT(contact_allocations.device_bytes,512u*1024);
  auto x=f->x,v=f->v; Results result;
  nodal_wall_capacity_probe::CountAllocations(true);
  for(unsigned step=0;step<3;++step) {
    fe::NodalTrialToken token; fe::NodalAssemblyView a;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok); sc::NodalWallDiagnostics base;
    ASSERT_EQ(contact.AssembleAccepted(a,&base).status,Code::Ok);
    ASSERT_EQ(contact.CopyResults(base,result.View()).status,Code::Ok); CheckLoads(*f,result,x);
    const auto force=Forces(a);
    for(unsigned n=0;n<Nodes;++n) {
      EXPECT_EQ(force[n],result.nodes[n].force_world.x);
      for(unsigned c=1;c<6;++c) EXPECT_EQ(force[c*Nodes+n],0);
    }
    ASSERT_EQ(owner.SealAssembly(token).status,fe::NodalStatus::Ok);
    ASSERT_EQ(fe::AdvanceStaggeredHistory(owner,token,{a.owner_id,a.accepted.base_epoch,a.attempt,
        Step,.1,Qualification}).status,fe::NodalStatus::Ok);
    fe::NodalPreparedView p; ASSERT_EQ(owner.BorrowPrepared(token,&p).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics end; ASSERT_EQ(contact.EvaluateCandidate(p,&end).status,Code::Ok);
    ASSERT_EQ(cudaMemcpyAsync(x.data(),p.kinematics.position_xyz,3*Nodes*sizeof(double),cudaMemcpyDeviceToHost,p.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
    ASSERT_EQ(contact.CopyResults(end,result.View()).status,Code::Ok); CheckLoads(*f,result,x);
    ASSERT_EQ(fe::CompleteNodalValidation(owner,token,{p.owner_id,p.kinematics.base_epoch,p.attempt,Qualification,true}).status,fe::NodalStatus::Ok);
    ASSERT_EQ(owner.Commit(token).status,fe::NodalStatus::Ok); contact.DiscardTrial(); ReadPositions(owner,x,v);
    EXPECT_EQ(contact.allocations().device_bytes,contact_allocations.device_bytes);
    EXPECT_EQ(contact.allocations().device_allocations,contact_allocations.device_allocations);
    EXPECT_EQ(owner.allocations().device_bytes,owner_allocations.device_bytes);
    EXPECT_EQ(owner.allocations().device_allocations,owner_allocations.device_allocations);
  }
  nodal_wall_capacity_probe::CountAllocations(false);
  EXPECT_EQ(nodal_wall_capacity_probe::AllocationCalls(),0u);
  EXPECT_EQ(owner.accepted().epoch,3u); RecordProperty("active_contact_device_bytes",std::to_string(contact_allocations.device_bytes));
}
TEST(NodalWallCapacityCuda, LastGlobalScatterOverflowPreservesEveryPhysicalForceAndRetriesExactly) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare()); fe::FENodalState owner; ASSERT_TRUE(f->Owner(owner));
  sc::NodalWallContactDevice contact; auto config=f->Config(owner.accepted());
  config.law.stiffness_per_area=1e308; config.law.parent_force_error=1e305; config.law.parent_energy_error=1e302;
  ASSERT_EQ(contact.Initialize(config,f->wall.view(),f->weights,f->Positions(),f->inverse.data(),f->fixed.data(),f->motion).status,Code::Ok);
  std::vector<double> clean; Results output;
  for(unsigned pass=0;pass<3;++pass) {
    fe::NodalTrialToken token; fe::NodalAssemblyView a; ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics d; d.owner_id=998; const auto prior=Bytes(d);
    if(pass==1) {
      Set<<<1,1,0,a.stream>>>(a.forces.force_x,Nodes-1,-std::numeric_limits<double>::max());
      ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess); const auto held=Forces(a);
      const auto r=contact.AssembleAccepted(a,&d); EXPECT_EQ(r.status,Code::AssemblyFailure); EXPECT_EQ(r.node,Nodes-1);
      EXPECT_EQ(Forces(a),held); EXPECT_EQ(Bytes(d),prior); EXPECT_NE(owner.SealAssembly(token).status,fe::NodalStatus::Ok);
    } else {
      ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
      ASSERT_EQ(contact.CopyResults(d,output.View()).status,Code::Ok);
      if(pass==0) clean=Forces(a); else EXPECT_EQ(Forces(a),clean);
    }
    owner.Discard(); contact.DiscardTrial(); auto x=f->x,v=f->v; ReadPositions(owner,x,v);
    EXPECT_EQ(x,f->x); EXPECT_EQ(v,f->v); EXPECT_EQ(owner.accepted().epoch,0u);
  }
}
TEST(NodalWallCapacityCuda, ActiveReadbackRejectsLegacyCapacityAliasingAndStaleIdentityWithoutPublication) {
  auto f=std::make_unique<Fixture>(); ASSERT_TRUE(f->Prepare()); fe::FENodalState owner; ASSERT_TRUE(f->Owner(owner));
  sc::NodalWallContactDevice contact; ASSERT_TRUE(f->Bind(owner,contact));
  fe::NodalTrialToken token; fe::NodalAssemblyView a; ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  sc::NodalWallDiagnostics d; ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
  Results result; ASSERT_EQ(contact.CopyResults(d,result.View()).status,Code::Ok);
  const auto diag=Bytes(result.diagnostics); const auto parents=result.parents; const auto nodes=result.nodes; const auto faces=result.faces;
  const auto held=[&] { EXPECT_EQ(Bytes(result.diagnostics),diag);
    EXPECT_EQ(std::memcmp(result.parents.data(),parents.data(),parents.size()*sizeof(parents[0])),0);
    EXPECT_EQ(std::memcmp(result.nodes.data(),nodes.data(),nodes.size()*sizeof(nodes[0])),0); EXPECT_EQ(result.faces,faces); };
  EXPECT_EQ(contact.CopyResults(d,reinterpret_cast<sc::NodalWallDeviceResults*>(std::uintptr_t{8})).status,Code::ResourceLimit);
  auto bad=result.View(); --bad.parent_capacity;
  EXPECT_EQ(contact.CopyResults(d,bad).status,Code::InvalidInput); held();
  bad=result.View(); bad.wall_face=reinterpret_cast<std::uint64_t*>(bad.nodes);
  EXPECT_EQ(contact.CopyResults(d,bad).status,Code::InvalidInput); held();
  auto stale=d; ++stale.attempt; EXPECT_EQ(contact.CopyResults(stale,result.View()).status,Code::StaleAttempt); held();
  ASSERT_EQ(contact.CopyResults(d,result.View()).status,Code::Ok); held();
  // Two successful transfers populate private diagnostics/parent staging; the
  // third is rejected. No output range may expose this incomplete readback.
  nodal_wall_capacity_probe::FailDeviceReadAfter(2);
  EXPECT_EQ(contact.CopyResults(d,result.View()).status,Code::DeviceFailure); held();
  EXPECT_EQ(nodal_wall_capacity_probe::DeviceReads(),3u);
  nodal_wall_capacity_probe::FailDeviceReadAfter(-1);
  EXPECT_EQ(contact.CopyResults(d,result.View()).status,Code::DeviceFailure); held();
  owner.Discard(); contact.DiscardTrial();
}
} // namespace
