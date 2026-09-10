#include "NodalWallStepFixture.h"
#include <cmath>

namespace {
using namespace nodal_wall_owner_test;
TEST(NodalWallOwnerCuda, ActualOwnerMatchesHostSharesFiniteFacesAndAdditiveAssembly) {
  for (unsigned variant=0;variant<4;++variant) {
    SCOPED_TRACE(variant); Fixture f; if (variant==0) f.count=1;
    f.wall=q4_planar_test::Square(variant); f.x[3]=-.01; f.x[9]=0;
    for (unsigned i=0;i<f.n;++i) f.v[3*i]=.125*(static_cast<int>(i)-2);
    ASSERT_TRUE(f.Prepare(variant==3)); fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
    sc::NodalWallContactDevice contact; ASSERT_TRUE(f.Bind(owner,contact,f.Config()));
    EXPECT_EQ(contact.allocations().device_allocations,1u);
    EXPECT_EQ(contact.allocations().device_bytes,sizeof(detail::Storage));
    EXPECT_EQ(owner.allocations().device_allocations,6u);
    fe::NodalTrialToken token; fe::NodalAssemblyView v;
    ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
    SetEntry<<<1,1,0,v.stream>>>(v.forces.force_x,2,3);
    SetEntry<<<1,1,0,v.stream>>>(v.forces.couple_z,5,2);
    ASSERT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess);
    sc::NodalWallDiagnostics d; ASSERT_EQ(contact.AssembleAccepted(v,&d).status,Code::Ok);
    sc::NodalWallDeviceResults result; ASSERT_EQ(contact.CopyResults(d,&result).status,Code::Ok);
    const auto host=f.Host(f.x,f.v,0,v.attempt,f.Config().law); Same(result,host);
    const auto assembled=Forces(v);
    for (unsigned i=0;i<f.n;++i) {
      double force=0;
      for (unsigned j=0;j<host.node_count;++j) if (host.nodes[j].node==i) force=host.nodes[j].force_world.x;
      EXPECT_EQ(assembled[i],(i==2?3.:0.)+force);
      for (unsigned c=1;c<6;++c) EXPECT_EQ(assembled[c*Capacity+i],c==5 && i==5?2.:0.);
    }
    sc::PlanarWallGeometry wall; ASSERT_EQ(wall.Initialize(f.wall.view()).status,sc::PlanarContactStatus::Ok);
    for (unsigned i=0;i<d.node_count;++i) {
      const auto n=result.nodes[i].node; unsigned face=UINT32_MAX; sc::TrianglePointGeometry point;
      ASSERT_EQ(sc::planar_detail::FindOwner({0,f.x[3*n+1],f.x[3*n+2]},wall.faces().data(),
          wall.faces().size(),wall.tolerance(),&face,&point),sc::Status::kOk);
      ASSERT_NE(face,UINT32_MAX); EXPECT_EQ(result.wall_face[i],wall.faces()[face].geometry.face_id);
    }
    owner.Discard(); contact.DiscardTrial();
  }
}

TEST(NodalWallOwnerCuda, HalfKickThenFullKickHasIndependentWorkImpulseAndEndpointPotential) {
  Fixture f; ASSERT_TRUE(f.Prepare()); fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
  sc::NodalWallContactDevice contact; ASSERT_TRUE(f.Bind(owner,contact,f.Config()));
  auto accepted=Read(owner); constexpr long double EnergyScale=.015625L;
  for (unsigned step=0;step<8;++step) {
    SCOPED_TRACE(step); fe::NodalTrialToken token; fe::NodalAssemblyView v; fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics base,candidate;
    ASSERT_EQ(contact.AssembleAccepted(v,&base).status,Code::Ok);
    sc::NodalWallDeviceResults base_result; ASSERT_EQ(contact.CopyResults(base,&base_result).status,Code::Ok);
    ASSERT_TRUE(Prepare(owner,token,v,prepared)); const auto endpoint=PreparedSnapshot(prepared);
    Same(Read(owner),accepted); ASSERT_EQ(contact.EvaluateCandidate(prepared,&candidate).status,Code::Ok);
    sc::NodalWallDeviceResults endpoint_result; ASSERT_EQ(contact.CopyResults(candidate,&endpoint_result).status,Code::Ok);
    const auto host=f.Host(endpoint.x,endpoint.v,step,v.attempt,f.Config().law); Same(endpoint_result,host);
    const long double kick=step?Step:.5L*Step;
    long double kinetic=0,work=0,drift=0,terms=0,impulse=0,delta_p=0,moment_y=0,moment_z=0,moment_terms=0;
    for (unsigned i=0;i<f.n;++i) {
      const long double a=accepted.v[3*i],b=endpoint.v[3*i],mass=1/f.inverse[i];
      const long double force=-16.L*f.weights.node(i).area.value*std::max(0.L,static_cast<long double>(accepted.x[3*i]));
      const long double expected_v=a+kick*force/mass;
      Near(endpoint.v[3*i],expected_v); Near(endpoint.x[3*i],static_cast<long double>(accepted.x[3*i])+Step*expected_v);
      const long double dw=kick*force*(a+b)*.5L,dx=static_cast<long double>(endpoint.x[3*i])-accepted.x[3*i];
      work+=dw; drift+=force*dx; kinetic+=.5L*mass*(b*b-a*a);
      terms+=std::abs(dw)+std::abs(force*dx)+.5L*mass*(b*b+a*a);
      impulse-=kick*force; delta_p+=mass*(b-a);
      moment_y-=kick*force*accepted.x[3*i+2]; moment_z+=kick*force*accepted.x[3*i+1];
      moment_terms+=std::abs(kick*force*accepted.x[3*i+2])+std::abs(kick*force*accepted.x[3*i+1]);
    }
    Ledger(candidate.kick_work,work,terms,EnergyScale); Ledger(candidate.kick_work,kinetic,terms,EnergyScale);
    Ledger(candidate.drift_work,drift,terms,EnergyScale);
    Ledger(candidate.wall_kick_impulse,impulse,std::abs(impulse),1.L/1024);
    Ledger(candidate.wall_kick_impulse,-delta_p,std::abs(impulse)+std::abs(delta_p),1.L/1024);
    Ledger(candidate.wall_kick_moment.y,moment_y,moment_terms,1.L/1024);
    Ledger(candidate.wall_kick_moment.z,moment_z,moment_terms,1.L/1024);
    EXPECT_LE(std::abs(static_cast<long double>(candidate.wall_kick_moment.y)-moment_y),candidate.wall_kick_moment_error.y);
    EXPECT_LE(std::abs(static_cast<long double>(candidate.wall_kick_moment.z)-moment_z),candidate.wall_kick_moment_error.z);
    EXPECT_EQ(candidate.kick_dt,static_cast<double>(kick));
    EXPECT_EQ(candidate.velocity_time,accepted.stamp.time+.5*Step);
    EXPECT_EQ(candidate.time,accepted.stamp.time+Step);
    EXPECT_GE(candidate.conservative_defect+candidate.work_uncertainty,0);
    EXPECT_LE(candidate.conservative_defect-candidate.work_uncertainty,candidate.quadratic_work_upper);
    if (!step) {
      EXPECT_GT(std::abs(candidate.drift_work-candidate.kick_work),1e-8);
      EXPECT_NE(endpoint_result.diagnostics.resultant.value,base_result.diagnostics.resultant.value);
    }
    ASSERT_TRUE(Commit(owner,token,prepared)); accepted=Read(owner); contact.DiscardTrial();
  }
}

TEST(NodalWallOwnerCuda, ForeignDuplicateAndLateScatterFailurePreserveOwnerAndRetry) {
  Fixture f; ASSERT_TRUE(f.Prepare()); fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
  sc::NodalWallContactDevice contact; ASSERT_TRUE(f.Bind(owner,contact,f.Config()));
  const auto accepted=Read(owner); sc::NodalWallDiagnostics sentinel; sentinel.owner_id=911; const auto before=Bytes(sentinel);
  for (unsigned variant=0;variant<6;++variant) {
    SCOPED_TRACE(variant); fe::NodalTrialToken token; fe::NodalAssemblyView v;
    ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok); auto bad=v;
    if (variant==0) ++bad.owner_id;
    if (variant==1) bad.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint;
    if (variant==2) ++bad.mass.base_epoch;
    if (variant==3) {
      SetEntry<<<1,1,0,v.stream>>>(v.forces.force_z,5,std::numeric_limits<double>::infinity());
    }
    if (variant==5) {
      // The existing CUDA default stream is valid but is not the borrowed
      // owner stream established by the preceding attempts. No extra stream
      // is created or owned by this contributor.
      ASSERT_NE(v.stream,nullptr);
      bad.stream=nullptr;
    }
    ASSERT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); const auto forces=Forces(v);
    if (variant==4) {
      sc::NodalWallDiagnostics first; ASSERT_EQ(contact.AssembleAccepted(v,&first).status,Code::Ok);
      const auto after=Forces(v); EXPECT_EQ(contact.AssembleAccepted(v,&sentinel).status,Code::StaleAttempt);
      EXPECT_EQ(Forces(v),after);
    } else { EXPECT_NE(contact.AssembleAccepted(bad,&sentinel).status,Code::Ok); EXPECT_EQ(Forces(v),forces); }
    Unchanged(sentinel,before); EXPECT_NE(owner.SealAssembly(token).status,fe::NodalStatus::Ok);
    owner.Discard(); contact.DiscardTrial(); Same(Read(owner),accepted);
  }
  fe::NodalTrialToken token; fe::NodalAssemblyView v;
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  ASSERT_EQ(contact.AssembleAccepted(v,&sentinel).status,Code::Ok);
  sc::NodalWallDeviceResults result; ASSERT_EQ(contact.CopyResults(sentinel,&result).status,Code::Ok);
  Same(result,f.Host(f.x,f.v,0,v.attempt,f.Config().law)); owner.Discard();
  // Separate finite-overflow fixture; these large diagnostic budgets are
  // explicit units of this failure test, never a source-tuple tolerance change.
  Fixture huge; ASSERT_TRUE(huge.Prepare());
  fe::FENodalState large_owner; ASSERT_TRUE(huge.Owner(large_owner)); sc::NodalWallContactDevice large;
  auto config=huge.Config(); config.law.stiffness_per_area=1e308;
  config.law.parent_force_error=1e296; config.law.parent_energy_error=1e296;
  ASSERT_TRUE(huge.Bind(large_owner,large,config));
  ASSERT_EQ(large_owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  SetEntry<<<1,1,0,v.stream>>>(v.forces.force_x,5,-std::numeric_limits<double>::max());
  ASSERT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); const auto original=Forces(v);
  EXPECT_EQ(large.AssembleAccepted(v,&sentinel).status,Code::AssemblyFailure);
  EXPECT_EQ(Forces(v),original); large_owner.Discard();
}

TEST(NodalWallOwnerCuda, CandidateCoverageDepthFixedAndReadbackIdentityFailBeforePublication) {
  Fixture f; f.fixed[0]=7; f.inverse[0]=0; f.x[0]=0; ASSERT_TRUE(f.Prepare());
  fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner)); sc::NodalWallContactDevice contact;
  ASSERT_TRUE(f.Bind(owner,contact,f.Config())); const auto accepted=Read(owner);
  sc::NodalWallDeviceResults out; out.diagnostics.owner_id=321; const auto output_bytes=Bytes(out);
  for (unsigned variant=0;variant<5;++variant) {
    SCOPED_TRACE(variant); fe::NodalTrialToken token; fe::NodalAssemblyView v; fe::NodalPreparedView prepared;
    ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok); sc::NodalWallDiagnostics base,candidate;
    ASSERT_EQ(contact.AssembleAccepted(v,&base).status,Code::Ok); ASSERT_TRUE(Prepare(owner,token,v,prepared));
    auto bad=prepared;
    if (variant==0) ++bad.attempt;
    if (variant==1) bad.kick_dt=Step;
    if (variant==2) SetEntry<<<1,1,0,v.stream>>>(const_cast<double*>(prepared.kinematics.position_xyz),16,1.1);
    if (variant==3) SetEntry<<<1,1,0,v.stream>>>(const_cast<double*>(prepared.kinematics.position_xyz),15,.6);
    if (variant==4) SetEntry<<<1,1,0,v.stream>>>(const_cast<double*>(prepared.kinematics.position_xyz),0,-.01);
    ASSERT_EQ(cudaStreamSynchronize(v.stream),cudaSuccess); const auto candidate_before=Bytes(candidate);
    EXPECT_NE(contact.EvaluateCandidate(bad,&candidate).status,Code::Ok); Unchanged(candidate,candidate_before);
    EXPECT_EQ(contact.CopyResults(base,&out).status,Code::StaleAttempt); Unchanged(out,output_bytes);
    owner.Discard(); contact.DiscardTrial(); Same(Read(owner),accepted);
  }
  fe::NodalTrialToken token; fe::NodalAssemblyView v; fe::NodalPreparedView prepared;
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok); sc::NodalWallDiagnostics base,candidate;
  ASSERT_EQ(contact.AssembleAccepted(v,&base).status,Code::Ok); ASSERT_TRUE(Prepare(owner,token,v,prepared));
  ASSERT_EQ(contact.EvaluateCandidate(prepared,&candidate).status,Code::Ok);
  auto foreign=candidate; ++foreign.configuration_id;
  EXPECT_EQ(contact.CopyResults(foreign,&out).status,Code::StaleAttempt); Unchanged(out,output_bytes);
  EXPECT_EQ(contact.CopyResults(candidate,nullptr).status,Code::InvalidInput);
  ASSERT_EQ(contact.CopyResults(candidate,&out).status,Code::Ok);
  EXPECT_TRUE(out.nodes[0].fixed); EXPECT_EQ(out.nodes[0].force.value,0); EXPECT_EQ(out.nodes[0].potential.value,0);
  owner.Discard(); contact.DiscardTrial(); Same(Read(owner),accepted);
}

TEST(NodalWallOwnerCuda, ActivationChangesAndLateMassAccuracyFailurePreserveCompleteOutputs) {
  Fixture f; f.Depth(-1./65536); f.v[0]=.125; f.v[3]=-.125; ASSERT_TRUE(f.Prepare());
  fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner)); sc::NodalWallContactDevice contact;
  ASSERT_TRUE(f.Bind(owner,contact,f.Config()));
  fe::NodalTrialToken token; fe::NodalAssemblyView v; fe::NodalPreparedView prepared;
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok); sc::NodalWallDiagnostics base,candidate;
  ASSERT_EQ(contact.AssembleAccepted(v,&base).status,Code::Ok); EXPECT_EQ(base.potential.value,0);
  ASSERT_TRUE(Prepare(owner,token,v,prepared)); ASSERT_EQ(contact.EvaluateCandidate(prepared,&candidate).status,Code::Ok);
  EXPECT_GT(candidate.potential.lower,0); EXPECT_EQ(candidate.kick_work,0); EXPECT_EQ(candidate.wall_kick_impulse,0);
  EXPECT_GE(candidate.conservative_defect+candidate.work_uncertainty,0);
  EXPECT_LE(candidate.conservative_defect-candidate.work_uncertainty,candidate.quadratic_work_upper);
  owner.Discard(); contact.DiscardTrial();
  {
    Fixture leaving; leaving.Depth(0); leaving.x[0]=1./65536; leaving.v[0]=-.125;
    ASSERT_TRUE(leaving.Prepare()); fe::FENodalState departing; ASSERT_TRUE(leaving.Owner(departing));
    sc::NodalWallContactDevice release; ASSERT_TRUE(leaving.Bind(departing,release,leaving.Config()));
    fe::NodalTrialToken t; fe::NodalAssemblyView a; fe::NodalPreparedView p;
    ASSERT_EQ(departing.BeginTrial(&t,&a).status,fe::NodalStatus::Ok);
    sc::NodalWallDiagnostics b,c; ASSERT_EQ(release.AssembleAccepted(a,&b).status,Code::Ok);
    EXPECT_GT(b.potential.lower,0); ASSERT_TRUE(Prepare(departing,t,a,p));
    ASSERT_EQ(release.EvaluateCandidate(p,&c).status,Code::Ok); EXPECT_EQ(c.potential.value,0);
    EXPECT_GT(c.wall_kick_impulse,0); EXPECT_GE(c.conservative_defect+c.work_uncertainty,0);
    EXPECT_LE(c.conservative_defect-c.work_uncertainty,c.quadratic_work_upper);
    departing.Discard(); release.DiscardTrial();
  }
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  // Borrowed malformed mass descriptors do not mutate actual owner mass.
  auto invalid=v; invalid.mass.model=sc::TranslationMassModel::kUnspecified;
  const auto forces=Forces(v); const auto base_before=Bytes(base);
  EXPECT_EQ(contact.AssembleAccepted(invalid,&base).status,Code::InvalidMass);
  EXPECT_EQ(Forces(v),forces); Unchanged(base,base_before); owner.Discard(); contact.DiscardTrial();
  double* wrong_inverse=nullptr; ASSERT_EQ(cudaMalloc(&wrong_inverse,f.n*sizeof(double)),cudaSuccess);
  auto bad_inverse=f.inverse; bad_inverse[5]=2;
  ASSERT_EQ(cudaMemcpy(wrong_inverse,bad_inverse.data(),f.n*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok); invalid=v; invalid.mass.inverse_mass=wrong_inverse;
  const auto zero=Forces(v); const auto mismatch=contact.AssembleAccepted(invalid,&base);
  EXPECT_EQ(mismatch.status,Code::InvalidMass); EXPECT_EQ(mismatch.node,5u); EXPECT_EQ(Forces(v),zero);
  owner.Discard(); contact.DiscardTrial(); EXPECT_EQ(cudaFree(wrong_inverse),cudaSuccess);
  Fixture strict; strict.Depth(0); strict.x[12]=strict.x[15]=.1;
  ASSERT_TRUE(strict.Prepare()); fe::FENodalState other; ASSERT_TRUE(strict.Owner(other));
  sc::NodalWallContactDevice tiny; auto config=strict.Config(); config.law.parent_energy_error=1e-30;
  ASSERT_TRUE(strict.Bind(other,tiny,config)); ASSERT_EQ(other.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  const auto clean=Forces(v); const auto inaccurate=tiny.AssembleAccepted(v,&base);
  EXPECT_EQ(inaccurate.status,Code::Accuracy); EXPECT_EQ(inaccurate.parent,1u);
  EXPECT_EQ(Forces(v),clean); other.Discard();
}

TEST(NodalWallOwnerCuda, PendingCudaReadbackFailurePoisonsContactAndPreservesAcceptedState) {
  Fixture f; ASSERT_TRUE(f.Prepare()); fe::FENodalState owner; ASSERT_TRUE(f.Owner(owner));
  sc::NodalWallContactDevice contact; ASSERT_TRUE(f.Bind(owner,contact,f.Config())); const auto accepted=Read(owner);
  fe::NodalTrialToken token; fe::NodalAssemblyView v;
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok); sc::NodalWallDiagnostics d;
  ASSERT_EQ(contact.AssembleAccepted(v,&d).status,Code::Ok);
  sc::NodalWallDeviceResults output; output.diagnostics.owner_id=888; const auto before=Bytes(output);
  Noop<<<1,0>>>(); ASSERT_NE(cudaPeekAtLastError(),cudaSuccess);
  EXPECT_EQ(contact.CopyResults(d,&output).status,Code::DeviceFailure); Unchanged(output,before);
  owner.Discard(); contact.DiscardTrial(); Same(Read(owner),accepted);
  EXPECT_EQ(contact.CopyResults(d,&output).status,Code::DeviceFailure); Unchanged(output,before);
  ASSERT_EQ(owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  EXPECT_EQ(contact.AssembleAccepted(v,&d).status,Code::DeviceFailure);
  EXPECT_NE(owner.SealAssembly(token).status,fe::NodalStatus::Ok); owner.Discard();
}
} // namespace
