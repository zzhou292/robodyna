#include "lib_utest/q4_planar_contact_fixture.h"
#include "lib_src/collision/Q4ContactIntegration.h"

#include <cfloat>
#include <cmath>
#include <limits>

namespace {
namespace test=q4_contact_batch_test;
namespace fixture=q4_planar_test;
namespace sc=tlfea::contact;
namespace fea=tl::fea;
using Code=sc::Q4PlanarContactStatus;

__global__ void Seed(fea::NodalAssemblyView view,double late=0) {
  for (unsigned n=0;n<6;++n) {
    view.forces.force_x[n]=.01*(n+1); view.forces.force_y[n]=.02*(n+1); view.forces.force_z[n]=-.03*(n+1);
    view.forces.couple_x[n]=.04*(n+1); view.forces.couple_y[n]=-.05*(n+1); view.forces.couple_z[n]=.06*(n+1);
  }
  if (late != 0) view.forces.force_x[5]=late;
}
__global__ void Change(double* values,unsigned at,double value) { values[at]=value; }
__global__ void InvalidConfigurationInjection() {}
struct DeviceArray {
  explicit DeviceArray(unsigned count) { status=cudaMalloc(&data,count*sizeof(double)); }
  ~DeviceArray() { if (data) cudaFree(data); }
  double* data=nullptr; cudaError_t status=cudaSuccess;
};
void Initialize(test::Rig& rig,fea::FENodalState& owner,sc::Q4PlanarContact& batch,const fixture::Wall& wall) {
  ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
  const auto report=batch.Initialize(rig.config(owner),wall.view(),rig.surface(),rig.mass_for(owner));
  ASSERT_EQ(report.status,Code::Ok) << report.message;
}
std::array<double,6> ContactForces(const std::array<sc::Q4PlanarParentResult,2>& result) {
  std::array<double,6> force{};
  for (const auto& parent:result) if (parent.covered)
    for (unsigned n=0;n<4;++n) force[parent.integration.nodal.nodes[n]]+=parent.integration.nodal.forces[n].x;
  return force;
}
void SameNumerics(const sc::Q4PlanarContactDiagnostics& a,const sc::Q4PlanarContactDiagnostics& b) {
  EXPECT_EQ(a.force_on_surface.x,b.force_on_surface.x); EXPECT_EQ(a.wall_moment.y,b.wall_moment.y);
  EXPECT_EQ(a.wall_moment.z,b.wall_moment.z); EXPECT_EQ(a.potential.value,b.potential.value);
  EXPECT_EQ(a.potential.lower,b.potential.lower); EXPECT_EQ(a.potential.upper,b.potential.upper);
  EXPECT_EQ(a.surface_power,b.surface_power); EXPECT_EQ(a.active_area.lower,b.active_area.lower);
  EXPECT_EQ(a.active_area.upper,b.active_area.upper); EXPECT_EQ(a.leaves,b.leaves); EXPECT_EQ(a.visited,b.visited);
}
}

TEST(Q4PlanarContactCUDA, SharedPartialParentsAddActualForcesAndPreserveOtherComponents) {
  test::Rig rig; const auto wall=fixture::Square(2); fea::FENodalState owner; sc::Q4PlanarContact batch;
  rig.rotation_fixed[5]=0; rig.inverse_inertia[5]=2; rig.omega[17]=.5;
  ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
  const auto state_allocation=owner.allocations(),allocation=batch.allocations();
  EXPECT_EQ(allocation.device_allocations,1); EXPECT_LT(allocation.device_bytes,430*1024u);
  const auto accepted=test::Read(owner);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  Seed<<<1,1,0,view.stream>>>(view);
  const auto before=test::Forces(view);
  sc::Q4PlanarContactDiagnostics diagnostic;
  ASSERT_EQ(batch.Assemble(view,&diagnostic).status,Code::Ok);
  std::array<sc::Q4PlanarParentResult,2> result;
  ASSERT_EQ(batch.CopyParentResults(diagnostic,result.data(),2).status,Code::Ok);
  const auto actual=test::Forces(view);
  const auto contact=ContactForces(result);
  // Independent exact saddle/half-cut integration with shared physical nodes.
  const unsigned numerator[6]={5,13,43,11,30,6};
  std::array<double,6> error{};
  for (const auto& parent:result) for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(parent.integration.nodal.couples[n].x,0);
    EXPECT_EQ(parent.integration.nodal.couples[n].y,0); EXPECT_EQ(parent.integration.nodal.couples[n].z,0);
    error[parent.integration.nodal.nodes[n]]+=parent.integration.force[n].error;
  }
  for (unsigned n=0;n<6;++n) {
    EXPECT_EQ(actual[n],before[n]+contact[n]);
    EXPECT_NEAR(contact[n],-.5*numerator[n]/288,error[n]+16*DBL_EPSILON);
  }
  for (unsigned i=6;i<36;++i) EXPECT_EQ(actual[i],before[i]);
  EXPECT_NEAR(diagnostic.force_on_surface.x,-.5*3/8,diagnostic.force_error.x+16*DBL_EPSILON);
  EXPECT_NEAR(diagnostic.wall_moment.y,.5/12,diagnostic.wall_moment_error.y+16*DBL_EPSILON);
  EXPECT_NEAR(diagnostic.wall_moment.z,-.5/16,diagnostic.wall_moment_error.z+16*DBL_EPSILON);
  EXPECT_NEAR(diagnostic.potential.value,16*fixture::Depth*fixture::Depth/9,diagnostic.potential.error);
  EXPECT_EQ(diagnostic.covered_count,2);
  EXPECT_LE(diagnostic.active_area.lower,1); EXPECT_GE(diagnostic.active_area.upper,1);
  EXPECT_EQ(diagnostic.phase,sc::Q4PlanarContactPhase::AcceptedBase);
  EXPECT_EQ(diagnostic.owner_id,owner.accepted().owner_id); EXPECT_EQ(diagnostic.attempt,view.attempt);
  ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
  owner.Discard(); test::SameState(test::Read(owner),accepted);
  EXPECT_EQ(batch.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(owner.allocations().device_bytes,state_allocation.device_bytes);
}

TEST(Q4PlanarContactCUDA, CopiedGeometryMassAndWallBindingAreImmutableAcrossMeshVariants) {
  sc::Q4PlanarContactDiagnostics first;
  for (unsigned variant=0;variant<4;++variant) {
    test::Rig rig; auto wall=fixture::Square(variant); fea::FENodalState owner; sc::Q4PlanarContact batch;
    ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
    const auto config=rig.config(owner);
    wall.vertices[0].position.y=100; rig.parents[1].feature_id=0;
    rig.x[5+6]=100; rig.inverse[5]=0; rig.fixed[5]=7;
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    sc::Q4PlanarContactDiagnostics d;
    ASSERT_EQ(batch.Assemble(view,&d).status,Code::Ok);
    EXPECT_EQ(d.wall_binding_id,config.wall_binding_id); EXPECT_EQ(d.configuration_id,config.configuration_id);
    if (!variant) first=d; else SameNumerics(d,first);
    EXPECT_GT(d.stiffness_rate_bound,0);
    owner.Discard();
  }
}

TEST(Q4PlanarContactCUDA, TinyCornerMissedByRootGaussRetainsPositiveCertifiedParentOutput) {
  test::Rig rig; constexpr double e=1./64;
  for (unsigned n=0;n<6;++n) rig.x[n]=fixture::Depth*(e-(rig.x[n+6]+1)-(rig.x[n+12]+.5));
  const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
  auto config=rig.config(owner);
  config.integration.force_error=.5*1e-6*e;
  config.integration.energy_error=16*fixture::Depth*fixture::Depth*5e-9*e*e;
  ASSERT_EQ(batch.Initialize(config,wall.view(),rig.surface(),rig.mass_for(owner)).status,Code::Ok);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  sc::Q4PlanarContactDiagnostics d;
  ASSERT_EQ(batch.Assemble(view,&d).status,Code::Ok);
  std::array<sc::Q4PlanarParentResult,2> result;
  ASSERT_EQ(batch.CopyParentResults(d,result.data(),2).status,Code::Ok);
  const auto exact=q4_contact_test::Corner(e);
  EXPECT_GT(result[0].integration.resultant.value,0); EXPECT_GT(result[0].integration.leaf_count,1);
  for (unsigned n=0;n<4;++n)
    EXPECT_NEAR(result[0].integration.force[n].value,static_cast<double>(.5L*exact.force[n]),result[0].integration.force[n].error);
  EXPECT_NEAR(d.potential.value,static_cast<double>(16*fixture::Depth*fixture::Depth*exact.potential),d.potential.error);
  EXPECT_TRUE(result[1].covered); EXPECT_TRUE(result[1].integration.valid);
  EXPECT_EQ(result[1].integration.resultant.value,0); EXPECT_EQ(result[1].integration.potential.value,0);
  owner.Discard();
}

TEST(Q4PlanarContactCUDA, ExactCoordinateAreaExpandsCertificatesAndCanRejectARawC2Budget) {
  test::Rig rig;
  for (unsigned n=0;n<6;++n) {
    rig.x[n]=fixture::Depth;
    rig.x[n+6]*=.3; rig.x[n+12]*=.7;
  }
  const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
  const auto accepted=test::Read(owner); auto config=rig.config(owner);
  sc::PlanarWallGeometry wall_geometry;
  ASSERT_EQ(wall_geometry.Initialize(wall.view()).status,sc::PlanarContactStatus::Ok);
  sc::Q4PlanarGeometry geometry;
  ASSERT_EQ(geometry.Initialize(wall_geometry,rig.surface(),rig.mass_for(owner),config.exposed_clearance).status,
            sc::PlanarContactStatus::Ok);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  sc::Q4PlanarContactDiagnostics base;
  ASSERT_EQ(batch.Assemble(view,&base).status,Code::Ok);
  std::array<sc::Q4PlanarParentResult,2> expanded;
  ASSERT_EQ(batch.CopyParentResults(base,expanded.data(),expanded.size()).status,Code::Ok);
  fixture::Scratch scratch;
  double raw_force_error=0,expanded_force_error=0;
  for (unsigned p=0;p<2;++p) {
    sc::Q4PreparedIntegration prepared;
    ASSERT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),rig.surface(),rig.mass_for(owner),p,
        config.stiffness_per_area,config.maximum_penetration,view.attempt,&prepared),sc::PlanarContactStatus::Ok);
    sc::Q4IntegrationResult raw;
    ASSERT_EQ(sc::IntegrateQ4NormalContact(prepared.input,config.integration,scratch.view(),&raw).status,
              sc::Q4IntegrationStatus::Ok);
    const auto& reference=geometry.view().parents[p]; const auto& result=expanded[p].integration;
    // Independent fully active uniform-pressure oracle. Long double resolves
    // this rectangle's area rounding; it is not a general exact predicate.
    const auto& x=reference.reference_projection;
    const long double area=(static_cast<long double>(x[0].y)-x[1].y)*
                           (static_cast<long double>(x[0].z)-x[3].z);
    ASSERT_NE(area,static_cast<long double>(reference.projected_area));
    EXPECT_LE(static_cast<long double>(reference.area_enclosure.lower),area);
    EXPECT_GE(static_cast<long double>(reference.area_enclosure.upper),area);
    const long double resultant=area*config.stiffness_per_area*fixture::Depth;
    EXPECT_LE(static_cast<long double>(result.resultant.lower),resultant);
    EXPECT_GE(static_cast<long double>(result.resultant.upper),resultant);
    const long double potential=resultant*fixture::Depth/2;
    EXPECT_LE(static_cast<long double>(result.potential.lower),potential);
    EXPECT_GE(static_cast<long double>(result.potential.upper),potential);
    EXPECT_EQ(result.resultant.value,raw.resultant.value); EXPECT_EQ(result.potential.value,raw.potential.value);
    EXPECT_LE(result.active_area.lower,raw.active_area.lower); EXPECT_GE(result.active_area.upper,raw.active_area.upper);
    raw_force_error=std::max(raw_force_error,raw.resultant.error);
    expanded_force_error=std::max(expanded_force_error,result.resultant.error);
    for (unsigned n=0;n<4;++n) {
      EXPECT_EQ(result.force[n].value,raw.force[n].value);
      EXPECT_LE(result.force[n].lower,raw.force[n].lower); EXPECT_GE(result.force[n].upper,raw.force[n].upper);
      EXPECT_LE(static_cast<long double>(result.force[n].lower),resultant/4);
      EXPECT_GE(static_cast<long double>(result.force[n].upper),resultant/4);
      raw_force_error=std::max(raw_force_error,raw.force[n].error);
      expanded_force_error=std::max(expanded_force_error,result.force[n].error);
    }
  }
  owner.Discard(); test::SameState(test::Read(owner),accepted);
  ASSERT_GT(expanded_force_error,raw_force_error);
  // This derived budget isolates certificate admission, not physics accuracy.
  // Require a real strict gap; never relax either existing integration budget.
  config.integration.force_error=raw_force_error+.5*(expanded_force_error-raw_force_error);
  ASSERT_GT(config.integration.force_error,raw_force_error);
  ASSERT_LT(config.integration.force_error,expanded_force_error);
  sc::Q4PlanarContact narrow;
  ASSERT_EQ(narrow.Initialize(config,wall.view(),rig.surface(),rig.mass_for(owner)).status,Code::Ok);
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  for (unsigned p=0;p<2;++p) {
    sc::Q4PreparedIntegration prepared;
    ASSERT_EQ(sc::PrepareQ4PlanarIntegration(geometry.view(),rig.surface(),rig.mass_for(owner),p,
        config.stiffness_per_area,config.maximum_penetration,view.attempt,&prepared),sc::PlanarContactStatus::Ok);
    sc::Q4IntegrationResult raw;
    ASSERT_EQ(sc::IntegrateQ4NormalContact(prepared.input,config.integration,scratch.view(),&raw).status,
              sc::Q4IntegrationStatus::Ok);
  }
  Seed<<<1,1,0,view.stream>>>(view); const auto force_before=test::Forces(view);
  sc::Q4PlanarContactDiagnostics output; output.attempt=913; output.potential.value=17;
  const auto before=output;
  const auto report=narrow.Assemble(view,&output);
  EXPECT_EQ(report.status,Code::IntegrationFailure); EXPECT_EQ(report.integration.status,sc::Q4IntegrationStatus::UnattainableAccuracy);
  test::Unchanged(output,before); EXPECT_EQ(test::Forces(view),force_before);
  EXPECT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::ContributorFailure);
  owner.Discard(); test::SameState(test::Read(owner),accepted);
}

TEST(Q4PlanarContactCUDA, LateSecondParentCapacityAndAdditiveOverflowNeverPublishPartialForces) {
  for (unsigned kind=0;kind<2;++kind) {
    test::Rig rig;
    for (unsigned n=0;n<6;++n) rig.x[n]=kind ? 1 : fixture::Depth;
    if (!kind) rig.x[5]=-fixture::Depth;
    const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
    ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
    auto config=rig.config(owner);
    if (!kind) { config.integration.max_leaves=1; config.integration.force_error=1e-8; config.integration.energy_error=1e-10; }
    else { config.stiffness_per_area=4e307; config.maximum_penetration=2; config.integration.force_error=1e295; config.integration.energy_error=1e295; }
    ASSERT_EQ(batch.Initialize(config,wall.view(),rig.surface(),rig.mass_for(owner)).status,Code::Ok);
    const auto accepted=test::Read(owner);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    Seed<<<1,1,0,view.stream>>>(view,kind ? -DBL_MAX : 0);
    const auto before_force=test::Forces(view);
    sc::Q4PlanarContactDiagnostics output; output.attempt=919; output.potential.value=17;
    const auto before=output;
    const auto report=batch.Assemble(view,&output);
    EXPECT_EQ(report.status,kind ? Code::AssemblyFailure : Code::IntegrationFailure) << report.message;
    if (!kind) { EXPECT_EQ(report.parent,1); EXPECT_EQ(report.integration.status,sc::Q4IntegrationStatus::LeafLimit); }
    else EXPECT_EQ(report.node,5);
    test::Unchanged(output,before); EXPECT_EQ(test::Forces(view),before_force);
    EXPECT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::ContributorFailure);
    owner.Discard(); test::SameState(test::Read(owner),accepted);
    if (kind) {
      ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
      ASSERT_EQ(batch.Assemble(view,&output).status,Code::Ok);
      ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok); owner.Discard();
    }
  }
}

TEST(Q4PlanarContactCUDA, ForeignDuplicateAndChangedActualMassRejectThenCleanRetryMatches) {
  test::Rig rig; const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
  DeviceArray inverse(6); ASSERT_EQ(inverse.status,cudaSuccess);
  ASSERT_EQ(cudaMemcpy(inverse.data,rig.inverse.data(),6*sizeof(double),cudaMemcpyHostToDevice),cudaSuccess);
  Change<<<1,1>>>(inverse.data,5,rig.inverse[5]*2); ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  sc::Q4PlanarContactDiagnostics reference;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Assemble(view,&reference).status,Code::Ok); owner.Discard();
  for (unsigned kind=0;kind<4;++kind) {
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    auto bad=view;
    if (kind == 0) bad.owner_id+=1000;
    if (kind == 1) bad.mass.inverse_mass=inverse.data;
    if (kind == 2) bad.mass.base_epoch+=1;
    sc::Q4PlanarContactDiagnostics output; output.attempt=313; const auto before=output;
    if (kind == 3) ASSERT_EQ(batch.Assemble(view,&output).status,Code::Ok);
    const auto retained=output; const auto force=test::Forces(view);
    const auto report=batch.Assemble(bad,&output);
    EXPECT_EQ(report.status,kind == 0 ? Code::WrongOwner : (kind == 1 ? Code::InvalidMass : Code::StaleAttempt));
    test::Unchanged(output,kind == 3 ? retained : before); EXPECT_EQ(test::Forces(view),force);
    EXPECT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::ContributorFailure); owner.Discard();
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    ASSERT_EQ(batch.Assemble(view,&output).status,Code::Ok); SameNumerics(output,reference); owner.Discard();
  }
}

TEST(Q4PlanarContactCUDA, RecoverableLaunchPoisonRejectsBeforeScatterAndRemainsAStickyContributorFailure) {
  // Exercise both first detection during Assemble and prior poisoning during
  // result readback. The invalid launch executes no kernel or invalid memory.
  for (unsigned poison_during_readback=0;poison_during_readback<2;++poison_during_readback) {
    SCOPED_TRACE(poison_during_readback);
    test::Rig rig; const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
    ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
    const auto accepted=test::Read(owner);
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    sc::Q4PlanarContactDiagnostics base;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    if (poison_during_readback) {
      ASSERT_EQ(batch.Assemble(view,&base).status,Code::Ok);
      std::array<sc::Q4PlanarParentResult,2> output{};
      output[0].integration.resultant.value=91; output[1].integration.attempt=919;
      const auto before=output;
      InvalidConfigurationInjection<<<1,0,0,view.stream>>>();
      const auto pending=cudaPeekAtLastError();
      ASSERT_TRUE(pending == cudaErrorInvalidValue || pending == cudaErrorInvalidConfiguration);
      EXPECT_EQ(batch.CopyParentResults(base,output.data(),output.size()).status,Code::DeviceFailure);
      test::Unchanged(output,before); EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess);
      owner.Discard(); test::SameState(test::Read(owner),accepted);
      ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    }
    Seed<<<1,1,0,view.stream>>>(view);
    const auto force_before=test::Forces(view);
    sc::Q4PlanarContactDiagnostics output; output.attempt=313; output.potential.value=17;
    const auto before=output;
    if (!poison_during_readback) {
      InvalidConfigurationInjection<<<1,0,0,view.stream>>>();
      const auto pending=cudaPeekAtLastError();
      ASSERT_TRUE(pending == cudaErrorInvalidValue || pending == cudaErrorInvalidConfiguration);
    }
    EXPECT_EQ(batch.Assemble(view,&output).status,Code::DeviceFailure);
    EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess);
    test::Unchanged(output,before); EXPECT_EQ(test::Forces(view),force_before);
    EXPECT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::ContributorFailure);
    owner.Discard(); test::SameState(test::Read(owner),accepted);
  }
}

TEST(Q4PlanarContactCUDA, CandidateWorkUsesRetainedBaseForceAndHasSeparateContinuumBudget) {
  test::Rig rig; const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
  const auto accepted=test::Read(owner);
  const auto allocation=batch.allocations();
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  Seed<<<1,1,0,view.stream>>>(view);
  sc::Q4PlanarContactDiagnostics base,candidate;
  ASSERT_EQ(batch.Assemble(view,&base).status,Code::Ok);
  std::array<sc::Q4PlanarParentResult,2> parent;
  ASSERT_EQ(batch.CopyParentResults(base,parent.data(),2).status,Code::Ok);
  const auto contact=ContactForces(parent);
  const auto force=test::Forces(view);
  ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
  const double h=owner.accepted().fixed_dt;
  ASSERT_LE(h,.1/std::sqrt(batch.stiffness_rate_bound()));
  ASSERT_EQ(fea::AdvanceNodal(owner,token,test::Admission(view,batch,h)).status,fea::NodalStatus::Ok);
  fea::NodalPreparedView prepared;
  ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.EvaluateCandidate(prepared,&candidate).status,Code::Ok);
  std::array<double,18> x{},v{};
  ASSERT_EQ(cudaMemcpyAsync(x.data(),prepared.kinematics.position_xyz,sizeof(x),cudaMemcpyDeviceToHost,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaMemcpyAsync(v.data(),prepared.kinematics.velocity_xyz,sizeof(v),cudaMemcpyDeviceToHost,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess);
  double midpoint=0,coordinate=0,other_work=0,old_kinetic=0,new_kinetic=0;
  for (unsigned n=0;n<6;++n) {
    const double mean=.5*(rig.v[3*n]+v[3*n]);
    midpoint+=h*contact[n]*mean; coordinate+=contact[n]*(x[3*n]-rig.position[3*n]);
    other_work+=h*.01*(n+1)*mean;
    if (rig.fixed[n] == 7) { EXPECT_EQ(x[3*n],rig.position[3*n]); EXPECT_EQ(mean,0); }
    else {
      old_kinetic+=.5*rig.v[3*n]*rig.v[3*n]/rig.inverse[n];
      new_kinetic+=.5*v[3*n]*v[3*n]/rig.inverse[n];
      EXPECT_NEAR(v[3*n],rig.v[3*n]+h*rig.inverse[n]*force[n],8*DBL_EPSILON);
    }
    EXPECT_EQ(x[3*n+1],rig.position[3*n+1]); EXPECT_EQ(x[3*n+2],rig.position[3*n+2]);
    EXPECT_EQ(v[3*n+1],0); EXPECT_EQ(v[3*n+2],0);
  }
  EXPECT_NEAR(candidate.kinetic_midpoint_work,midpoint,candidate.kinetic_midpoint_roundoff+16*DBL_EPSILON*std::abs(midpoint));
  EXPECT_NEAR(candidate.force_coordinate_work,coordinate,candidate.force_coordinate_roundoff+16*DBL_EPSILON*std::abs(coordinate));
  EXPECT_NEAR(new_kinetic-old_kinetic-other_work-candidate.kinetic_midpoint_work,0,
              candidate.kinetic_midpoint_roundoff+128*DBL_EPSILON*old_kinetic);
  EXPECT_EQ(candidate.base_potential,base.potential.value);
  EXPECT_EQ(candidate.potential_increment,candidate.potential.value-base.potential.value);
  EXPECT_GE(candidate.conservative_force_coordinate_defect,-candidate.continuum_work_uncertainty);
  EXPECT_LE(candidate.conservative_force_coordinate_defect,candidate.quadratic_work_upper+candidate.continuum_work_uncertainty);
  EXPECT_GT(candidate.continuum_work_uncertainty,candidate.kinetic_midpoint_roundoff);
  EXPECT_EQ(batch.CopyParentResults(base,parent.data(),2).status,Code::StaleAttempt);
  ASSERT_EQ(batch.CopyParentResults(candidate,parent.data(),2).status,Code::Ok);
  // This contact-only, Gram-bounded admission probes a prepared transaction.
  // The coupled guided plate receives its own qualification and dynamics gate.
  owner.Discard(); test::SameState(test::Read(owner),accepted);
  EXPECT_EQ(batch.allocations().device_bytes,allocation.device_bytes);
}

TEST(Q4PlanarContactCUDA, RejectedCandidateInvalidatesScratchAndAcceptedReevaluationCanRetry) {
  test::Rig rig; for (unsigned n=0;n<6;++n) rig.x[n]=.0999;
  const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
  const auto accepted=test::Read(owner); const auto allocation=batch.allocations();
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  sc::Q4PlanarContactDiagnostics base,output; output.attempt=919; const auto before=output;
  ASSERT_EQ(batch.Assemble(view,&base).status,Code::Ok);
  ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
  ASSERT_EQ(fea::AdvanceNodal(owner,token,test::Admission(view,batch,owner.accepted().fixed_dt)).status,fea::NodalStatus::Ok);
  fea::NodalPreparedView prepared;
  ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
  const auto report=batch.EvaluateCandidate(prepared,&output);
  EXPECT_EQ(report.status,Code::IntegrationFailure); EXPECT_EQ(report.integration.status,sc::Q4IntegrationStatus::PenetrationLimit);
  test::Unchanged(output,before);
  std::array<sc::Q4PlanarParentResult,2> result{}; result[1].integration.attempt=122; const auto retained=result;
  EXPECT_EQ(batch.CopyParentResults(base,result.data(),2).status,Code::StaleAttempt); test::Unchanged(result,retained);
  owner.Discard(); test::SameState(test::Read(owner),accepted);
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  ASSERT_EQ(batch.Assemble(view,&output).status,Code::Ok); SameNumerics(output,base);
  ASSERT_EQ(batch.CopyParentResults(output,result.data(),2).status,Code::Ok);
  owner.Discard(); test::SameState(test::Read(owner),accepted);
  EXPECT_EQ(batch.allocations().device_bytes,allocation.device_bytes);
}

TEST(Q4PlanarContactCUDA, PreparedViewGeometryAndIdentityFailuresPreserveCallerOutput) {
  test::Rig rig; const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_NO_FATAL_FAILURE(Initialize(rig,owner,batch,wall));
  DeviceArray positions(18),velocities(18);
  ASSERT_EQ(positions.status,cudaSuccess); ASSERT_EQ(velocities.status,cudaSuccess);
  for (unsigned kind=0;kind<5;++kind) {
    fea::NodalTrialToken token; fea::NodalAssemblyView view;
    ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
    sc::Q4PlanarContactDiagnostics base,output; output.attempt=44; const auto before=output;
    ASSERT_EQ(batch.Assemble(view,&base).status,Code::Ok);
    ASSERT_EQ(owner.SealAssembly(token).status,fea::NodalStatus::Ok);
    ASSERT_EQ(fea::AdvanceNodal(owner,token,test::Admission(view,batch,owner.accepted().fixed_dt)).status,fea::NodalStatus::Ok);
    fea::NodalPreparedView prepared;
    ASSERT_EQ(owner.BorrowPrepared(token,&prepared).status,fea::NodalStatus::Ok);
    // Deliberately malformed borrowed views test validation without modifying
    // the owner's immutable accepted state or its prepared candidate buffers.
    auto bad=prepared;
    if (kind == 0) bad.owner_id+=1000;
    if (kind == 1) ++bad.attempt;
    if (kind == 2) {
      ASSERT_EQ(cudaMemcpyAsync(positions.data,prepared.kinematics.position_xyz,18*sizeof(double),cudaMemcpyDeviceToDevice,prepared.stream),cudaSuccess);
      Change<<<1,1,0,prepared.stream>>>(positions.data,16,1.01); bad.kinematics.position_xyz=positions.data;
    }
    if (kind >= 3) {
      ASSERT_EQ(cudaMemcpyAsync(velocities.data,prepared.kinematics.velocity_xyz,18*sizeof(double),cudaMemcpyDeviceToDevice,prepared.stream),cudaSuccess);
      Change<<<1,1,0,prepared.stream>>>(velocities.data,kind == 3 ? 17 : 0,.01); bad.kinematics.velocity_xyz=velocities.data;
    }
    const auto report=batch.EvaluateCandidate(bad,&output);
    EXPECT_EQ(report.status,kind == 0 ? Code::WrongOwner : (kind == 1 ? Code::StaleAttempt : Code::GeometryFailure));
    test::Unchanged(output,before); owner.Discard();
  }
}

TEST(Q4PlanarContactCUDA, StartupBudgetsAndOutsideOnlyClassificationAreExplicit) {
  test::Rig rig; for (unsigned n=0;n<6;++n) rig.x[n+6]+=4;
  const auto wall=fixture::Square(); fea::FENodalState owner; sc::Q4PlanarContact batch;
  ASSERT_EQ(rig.Initialize(owner).status,fea::NodalStatus::Ok);
  auto config=rig.config(owner); config.max_device_bytes=1;
  EXPECT_EQ(batch.Initialize(config,wall.view(),rig.surface(),rig.mass_for(owner)).status,Code::ResourceLimit);
  EXPECT_EQ(batch.allocations().device_bytes,0);
  config.max_device_bytes=sc::MaxPlanarContactDeviceBytes;
  ASSERT_EQ(batch.Initialize(config,wall.view(),rig.surface(),rig.mass_for(owner)).status,Code::Ok);
  EXPECT_EQ(batch.stiffness_rate_bound(),0);
  fea::NodalTrialToken token; fea::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fea::NodalStatus::Ok);
  Seed<<<1,1,0,view.stream>>>(view); const auto before=test::Forces(view);
  sc::Q4PlanarContactDiagnostics d; ASSERT_EQ(batch.Assemble(view,&d).status,Code::Ok);
  EXPECT_EQ(d.covered_count,0); EXPECT_EQ(d.leaves,0); EXPECT_EQ(d.force_on_surface.x,0); EXPECT_EQ(d.potential.value,0);
  EXPECT_EQ(test::Forces(view),before);
  std::array<sc::Q4PlanarParentResult,2> result{}; const auto original=result;
  EXPECT_EQ(batch.CopyParentResults(d,result.data(),1).status,Code::ResourceLimit); test::Unchanged(result,original);
  ASSERT_EQ(batch.CopyParentResults(d,result.data(),2).status,Code::Ok);
  for (const auto& parent:result) { EXPECT_FALSE(parent.covered); EXPECT_FALSE(parent.integration.valid); }
  owner.Discard();
}
