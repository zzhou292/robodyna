#include "NodalWallNativeFixture.h"
#include "NodalWallStepFixture.h"
#include <algorithm>

namespace {
using namespace nodal_wall_native_test;
constexpr long double EnergyScale=1.L/64,ImpulseScale=1.L/1024;

TEST(NodalWallNativeCuda, NativeScaleneMassesSharesAndMixedUnionMatchIndependentLoads) {
  for(auto layout:{Layout::SingleT3,Layout::PairT3,Layout::Mixed}) for(bool source:{false,true}) {
    SCOPED_TRACE(static_cast<unsigned>(layout));
    SCOPED_TRACE(source);
    NativeFixture f(layout); const double depth=source?.00025:1./32,stiffness=source?4e5:16;
    for(unsigned i=f.first();i<f.n;++i) {
      f.x[3*i]=(static_cast<int>(i)-2)*depth/4; f.v[3*i]=(static_cast<int>(i)-1)/8.;
    }
    if(layout==Layout::Mixed) f.x[0]=depth; // Decisive shared global-0 / unused T3 slot-3 sentinel.
    ASSERT_TRUE(f.PrepareNative()); CheckMass(f);
    const auto mass=f.mass; const auto area=f.node_area;
    EXPECT_FALSE(f.PrepareNative()); EXPECT_EQ(f.mass,mass); EXPECT_EQ(f.node_area,area);
    if(layout!=Layout::Mixed) EXPECT_NE(f.mass[1],f.mass[2]); // Native angle weights, not equal thirds.
    fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
    sc::NodalWallContactDevice contact; auto config=f.Config(); config.law.stiffness_per_area=stiffness;
    if(source) config.law.maximum_penetration=.0005;
    ASSERT_TRUE(f.Bind(owner,contact,config));
    EXPECT_EQ(config.law.parent_force_error,ForceBudget); EXPECT_EQ(config.law.parent_energy_error,EnergyBudget);
    EXPECT_EQ(owner.allocations().device_allocations,6u);
    EXPECT_EQ(contact.allocations().device_allocations,1u);
    EXPECT_EQ(contact.allocations().device_bytes,sizeof(detail::Storage));
    EXPECT_EQ(sizeof(detail::Storage),471864u);
    long double rate=0;
    for(unsigned i=f.first();i<f.n;++i) rate=std::max(rate,stiffness*f.node_area[i]*f.inverse[i]);
    EXPECT_GE(static_cast<long double>(contact.stiffness_rate_bound()),rate);
    Near(contact.stiffness_rate_bound(),rate);
    if(!source) EXPECT_LT(Step*std::sqrt(contact.stiffness_rate_bound()),.01);
    fe::NodalTrialToken token; fe::NodalAssemblyView a;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    SetEntry<<<1,1,0,a.stream>>>(a.forces.force_x,0,3);
    SetEntry<<<1,1,0,a.stream>>>(a.forces.couple_z,f.n-1,2);
    ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
    sc::NodalWallDiagnostics d; ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
    sc::NodalWallDeviceResults result; ASSERT_EQ(contact.CopyResults(d,&result).status,Code::Ok);
    ASSERT_EQ(d.node_count,f.n-f.first());
    const auto host=f.Host(f.x,f.v,0,a.attempt,config.law); Same(result,host);
    CheckIndependent(f,result,f.x,f.v,stiffness); CheckFaces(f,result,f.x);
    const auto forces=Forces(a);
    for(unsigned i=0;i<f.n;++i) {
      const long double fi=-stiffness*f.node_area[i]*std::max(0.L,static_cast<long double>(f.x[3*i]));
      Near(forces[i],fi+(i==0?3:0));
      for(unsigned c=1;c<6;++c) EXPECT_EQ(forces[c*Capacity+i],c==5 && i==f.n-1?2.:0.);
    }
    if(f.first()) {
      EXPECT_EQ(forces[0],3); EXPECT_EQ(f.weights.parent(0).nodes[3],0u);
      for(unsigned j=0;j<d.node_count;++j) EXPECT_NE(result.nodes[j].node,0u);
    }
    // Both tuples stop before Seal/Advance: the source tuple supplies no
    // recurrence/timestep admission merely by evaluating a contact load.
    owner.Discard(); contact.DiscardTrial();
  }
}

TEST(NodalWallNativeCuda, NativeMassHalfAndFullKicksUseOnlyAcceptedContactAndSeparateWork) {
  for(auto layout:{Layout::PairT3,Layout::Mixed}) {
    SCOPED_TRACE(static_cast<unsigned>(layout)); NativeFixture f(layout); ASSERT_TRUE(f.PrepareNative());
    fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner)); sc::NodalWallContactDevice contact;
    ASSERT_TRUE(f.Bind(owner,contact,f.Config())); auto accepted=Read(owner);
    const auto owner_allocations=owner.allocations(); const auto contact_allocations=contact.allocations();
    for(unsigned step=0;step<8;++step) {
      SCOPED_TRACE(step); fe::NodalTrialToken token; fe::NodalAssemblyView a; fe::NodalPreparedView p;
      ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
      sc::NodalWallDiagnostics base,end;
      ASSERT_EQ(contact.AssembleAccepted(a,&base).status,Code::Ok); const auto force_before=Forces(a);
      ASSERT_TRUE(Prepare(owner,token,a,p)); const auto endpoint=PreparedSnapshot(p);
      ASSERT_EQ(contact.EvaluateCandidate(p,&end).status,Code::Ok);
      EXPECT_EQ(Forces(a),force_before); Same(Read(owner),accepted);
      sc::NodalWallDeviceResults output; ASSERT_EQ(contact.CopyResults(end,&output).status,Code::Ok);
      Same(output,f.Host(endpoint.x,endpoint.v,step,a.attempt,f.Config().law));
      const long double kick=step?Step:Step/2.L;
      long double work=0,drift=0,kinetic=0,terms=0,impulse=0,delta_p=0,my=0,mz=0,mt=0;
      for(unsigned i=f.first();i<f.n;++i) {
        const long double va=accepted.v[3*i],vb=endpoint.v[3*i],mass=1.L/f.inverse[i];
        const long double force=-16.L*f.node_area[i]*std::max(0.L,static_cast<long double>(accepted.x[3*i]));
        const long double next_v=va+kick*force/mass;
        Near(endpoint.v[3*i],next_v); Near(endpoint.x[3*i],static_cast<long double>(accepted.x[3*i])+Step*next_v);
        const long double dx=static_cast<long double>(endpoint.x[3*i])-accepted.x[3*i];
        const long double dw=kick*force*(va+vb)/2;
        work+=dw; drift+=force*dx; kinetic+=mass*(vb*vb-va*va)/2;
        terms+=std::abs(dw)+std::abs(force*dx)+mass*(vb*vb+va*va)/2;
        impulse-=kick*force; delta_p+=mass*(vb-va);
        my-=kick*force*accepted.x[3*i+2]; mz+=kick*force*accepted.x[3*i+1];
        mt+=std::abs(kick*force*accepted.x[3*i+2])+std::abs(kick*force*accepted.x[3*i+1]);
      }
      Ledger(end.kick_work,work,terms,EnergyScale); Ledger(end.kick_work,kinetic,terms,EnergyScale);
      Ledger(end.drift_work,drift,terms,EnergyScale);
      Ledger(end.wall_kick_impulse,impulse,std::abs(impulse),ImpulseScale);
      Ledger(end.wall_kick_impulse,-delta_p,std::abs(impulse)+std::abs(delta_p),ImpulseScale);
      Ledger(end.wall_kick_moment.y,my,mt,ImpulseScale); Ledger(end.wall_kick_moment.z,mz,mt,ImpulseScale);
      EXPECT_EQ(end.kick_dt,static_cast<double>(kick)); EXPECT_EQ(end.time,accepted.stamp.time+Step);
      EXPECT_EQ(end.velocity_time,accepted.stamp.time+Step/2);
      EXPECT_GE(end.conservative_defect+end.work_uncertainty,0);
      EXPECT_LE(end.conservative_defect-end.work_uncertainty,end.quadratic_work_upper);
      EXPECT_NE(end.resultant.value,base.resultant.value);
      if(!step) EXPECT_GT(std::abs(end.drift_work-end.kick_work),1e-10);
      if(f.first()) { EXPECT_EQ(endpoint.x[0],accepted.x[0]); EXPECT_EQ(endpoint.v[0],0); }
      ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(owner,token,p));
      accepted=Read(owner); contact.DiscardTrial();
    }
    EXPECT_EQ(owner.allocations().device_allocations,owner_allocations.device_allocations);
    EXPECT_EQ(owner.allocations().device_bytes,owner_allocations.device_bytes);
    EXPECT_EQ(contact.allocations().device_allocations,contact_allocations.device_allocations);
    EXPECT_EQ(contact.allocations().device_bytes,contact_allocations.device_bytes);
  }
}

TEST(NodalWallNativeCuda, T3UniqueNodeActivationAndReleaseNeverFeedCandidateLoadIntoItsKick) {
  for(bool entering:{true,false}) {
    SCOPED_TRACE(entering); NativeFixture f(Layout::Mixed); f.Depth(-1./65536);
    f.x[12]=entering?-1./65536:1./65536; f.v[12]=entering?.125:-.125;
    ASSERT_TRUE(f.PrepareNative()); fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
    sc::NodalWallContactDevice contact; ASSERT_TRUE(f.Bind(owner,contact,f.Config()));
    const auto accepted=Read(owner); fe::NodalTrialToken token; fe::NodalAssemblyView a; fe::NodalPreparedView p;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics b,c; ASSERT_EQ(contact.AssembleAccepted(a,&b).status,Code::Ok);
    const auto force_before=Forces(a); ASSERT_TRUE(Prepare(owner,token,a,p)); const auto endpoint=PreparedSnapshot(p);
    ASSERT_EQ(contact.EvaluateCandidate(p,&c).status,Code::Ok); EXPECT_EQ(Forces(a),force_before);
    const long double base_force=-16.L*f.node_area[4]*std::max(0.L,static_cast<long double>(f.x[12]));
    Near(endpoint.v[12],static_cast<long double>(f.v[12])+Step/2.L*base_force*f.inverse[4]);
    if(entering) {
      EXPECT_EQ(b.potential.upper,0); EXPECT_GT(c.potential.lower,0);
      EXPECT_EQ(c.kick_work,0); EXPECT_EQ(c.wall_kick_impulse,0); EXPECT_EQ(endpoint.v[12],f.v[12]);
    } else { EXPECT_GT(b.potential.lower,0); EXPECT_EQ(c.potential.upper,0); EXPECT_GT(c.wall_kick_impulse,0); }
    sc::NodalWallDeviceResults r; ASSERT_EQ(contact.CopyResults(c,&r).status,Code::Ok);
    EXPECT_EQ(r.parents[0].potential.upper,0); Same(r,f.Host(endpoint.x,endpoint.v,0,a.attempt,f.Config().law));
    EXPECT_GE(c.conservative_defect+c.work_uncertainty,0);
    EXPECT_LE(c.conservative_defect-c.work_uncertainty,c.quadratic_work_upper);
    Same(Read(owner),accepted); owner.Discard(); contact.DiscardTrial();
  }
}

TEST(NodalWallNativeCuda, CyclicAndParentPermutationsEdgeOnReferencesAndActualSeamFaces) {
  for(auto layout:{Layout::SingleT3,Layout::PairT3,Layout::Mixed}) for(unsigned variant=0;variant<4;++variant) {
    SCOPED_TRACE(static_cast<unsigned>(layout));
    SCOPED_TRACE(variant);
    NativeFixture f(layout,variant>=2,variant%3); f.wall=q4_planar_test::Square(variant);
    // An actual projected point on the diagonal/central tessellation seam.
    f.x[3*(f.n-1)+1]=f.x[3*(f.n-1)+2]=0;
    ASSERT_TRUE(f.PrepareNative(variant%2)); CheckMass(f);
    if(variant>=2) for(unsigned i=f.first();i<f.n;++i) EXPECT_EQ(f.reference[3*i+1],0);
    fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner)); sc::NodalWallContactDevice contact;
    ASSERT_TRUE(f.Bind(owner,contact,f.Config())); fe::NodalTrialToken token; fe::NodalAssemblyView a;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics d; ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
    sc::NodalWallDeviceResults r; ASSERT_EQ(contact.CopyResults(d,&r).status,Code::Ok);
    Same(r,f.Host(f.x,f.v,0,a.attempt,f.Config().law)); CheckIndependent(f,r,f.x,f.v); CheckFaces(f,r,f.x);
    EXPECT_LE(d.resultant.error,2*ForceBudget); EXPECT_LE(d.potential.error,2*EnergyBudget);
    owner.Discard(); contact.DiscardTrial();
  }
}

TEST(NodalWallNativeCuda, LateT3BudgetMassAndCandidateFaultsPreserveOutputsAndCleanRetry) {
  NativeFixture f(Layout::Mixed); f.Depth(0); f.x[12]=1./32;
  ASSERT_TRUE(f.PrepareNative()); fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
  const auto accepted=Read(owner); fe::NodalTrialToken token; fe::NodalAssemblyView a;
  sc::NodalWallContactDevice tiny; auto strict=f.Config(); strict.law.parent_energy_error=1e-30;
  ASSERT_TRUE(f.Bind(owner,tiny,strict)); ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  sc::NodalWallDiagnostics d; d.owner_id=999; const auto db=Bytes(d); const auto empty=Forces(a);
  const auto accuracy=tiny.AssembleAccepted(a,&d);
  EXPECT_EQ(accuracy.status,Code::Accuracy); EXPECT_EQ(accuracy.parent,1u);
  EXPECT_EQ(Forces(a),empty); Unchanged(d,db); Same(Read(owner),accepted); owner.Discard(); tiny.DiscardTrial();
  sc::NodalWallContactDevice contact; ASSERT_TRUE(f.Bind(owner,contact,f.Config()));
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault); ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    if(fault==0) {
      // Borrowed alternate mass does not alter the actual owner or its union.
      struct Buffer { double* p=nullptr; ~Buffer() { if(p) cudaFree(p); } } alternate;
      ASSERT_EQ(cudaMalloc(&alternate.p,f.n*sizeof(double)),cudaSuccess);
      auto inverse=f.inverse; inverse[4]=0;
      ASSERT_EQ(cudaMemcpyAsync(alternate.p,inverse.data(),f.n*sizeof(double),cudaMemcpyHostToDevice,a.stream),cudaSuccess);
      auto bad=a; bad.mass.inverse_mass=alternate.p; const auto before=Forces(a);
      const auto report=contact.AssembleAccepted(bad,&d);
      EXPECT_EQ(report.status,Code::InvalidMass); EXPECT_EQ(report.node,4u);
      EXPECT_EQ(Forces(a),before); Unchanged(d,db);
    } else {
      ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
      sc::NodalWallDeviceResults held; ASSERT_EQ(contact.CopyResults(d,&held).status,Code::Ok);
      const auto bytes=Bytes(held); fe::NodalPreparedView p; ASSERT_TRUE(Prepare(owner,token,a,p));
      const auto before=Forces(a); const auto diagnostic=Bytes(d);
      const unsigned coordinate=fault==2?13:12;
      const double value=fault==1?.75:(fault==2?1.5:std::numeric_limits<double>::quiet_NaN());
      SetEntry<<<1,1,0,a.stream>>>(const_cast<double*>(p.kinematics.position_xyz),coordinate,value);
      ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
      const auto report=contact.EvaluateCandidate(p,&d);
      EXPECT_EQ(report.status,fault==1?Code::PointFailure:Code::GeometryFailure); EXPECT_EQ(report.node,4u);
      Unchanged(d,diagnostic); EXPECT_NE(contact.CopyResults(d,&held).status,Code::Ok);
      Unchanged(held,bytes); EXPECT_EQ(Forces(a),before);
    }
    owner.Discard(); contact.DiscardTrial(); Same(Read(owner),accepted);
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    ASSERT_EQ(contact.AssembleAccepted(a,&d).status,Code::Ok);
    sc::NodalWallDeviceResults retry; ASSERT_EQ(contact.CopyResults(d,&retry).status,Code::Ok);
    Same(retry,f.Host(f.x,f.v,0,a.attempt,f.Config().law)); CheckIndependent(f,retry,f.x,f.v);
    owner.Discard(); contact.DiscardTrial();
  }
  // A selected T3 node may be fully fixed only with zero inverse mass and
  // nonpenetrating, stationary position. Its native startup mass is retained.
  NativeFixture fixed(Layout::Mixed); fixed.Depth(0); ASSERT_TRUE(fixed.PrepareNative());
  fixed.fixed[4]=7; fixed.inverse[4]=0;
  fe::FENodalState fixed_owner; ASSERT_TRUE(fixed.Owner(fixed_owner)); sc::NodalWallContactDevice fixed_contact;
  ASSERT_TRUE(fixed.Bind(fixed_owner,fixed_contact,fixed.Config()));
  ASSERT_EQ(fixed_owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
  ASSERT_EQ(fixed_contact.AssembleAccepted(a,&d).status,Code::Ok);
  sc::NodalWallDeviceResults held; ASSERT_EQ(fixed_contact.CopyResults(d,&held).status,Code::Ok);
  EXPECT_TRUE(held.nodes[4].fixed); EXPECT_EQ(held.nodes[4].force.upper,0);
  const auto bytes=Bytes(held); const auto diagnostic=Bytes(d); fe::NodalPreparedView p;
  ASSERT_TRUE(Prepare(fixed_owner,token,a,p));
  SetEntry<<<1,1,0,a.stream>>>(const_cast<double*>(p.kinematics.position_xyz),12,-1./65536);
  ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess);
  EXPECT_EQ(fixed_contact.EvaluateCandidate(p,&d).status,Code::GeometryFailure); Unchanged(d,diagnostic);
  EXPECT_NE(fixed_contact.CopyResults(d,&held).status,Code::Ok); Unchanged(held,bytes);
  fixed_owner.Discard(); fixed_contact.DiscardTrial();
  {
    // The retained overflow-only arithmetic tuple is distinct from both
    // physical fixtures. A late T3 destination fails after an earlier Q4
    // destination was staged, so all six additive arrays must remain intact.
    NativeFixture overflow(Layout::Mixed); ASSERT_TRUE(overflow.PrepareNative());
    fe::FENodalState late_owner; ASSERT_TRUE(overflow.Owner(late_owner));
    sc::NodalWallContactDevice large; auto config=overflow.Config();
    config.law.stiffness_per_area=1e308; config.law.parent_force_error=1e296;
    config.law.parent_energy_error=1e296; ASSERT_TRUE(overflow.Bind(late_owner,large,config));
    const auto state=Read(late_owner);
    ASSERT_EQ(late_owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    SetEntry<<<1,1,0,a.stream>>>(a.forces.force_x,4,-std::numeric_limits<double>::max());
    SetEntry<<<1,1,0,a.stream>>>(a.forces.couple_y,0,2);
    ASSERT_EQ(cudaStreamSynchronize(a.stream),cudaSuccess); const auto forces=Forces(a);
    const auto before=Bytes(d); const auto report=large.AssembleAccepted(a,&d);
    EXPECT_EQ(report.status,Code::AssemblyFailure); EXPECT_EQ(report.node,4u);
    EXPECT_EQ(Forces(a),forces); Unchanged(d,before); Same(Read(late_owner),state);
    late_owner.Discard(); large.DiscardTrial();
    ASSERT_EQ(late_owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    ASSERT_EQ(large.AssembleAccepted(a,&d).status,Code::Ok);
    sc::NodalWallDeviceResults clean; ASSERT_EQ(large.CopyResults(d,&clean).status,Code::Ok);
    Same(clean,overflow.Host(overflow.x,overflow.v,0,a.attempt,config.law));
    late_owner.Discard(); large.DiscardTrial();
  }
}
} // namespace
