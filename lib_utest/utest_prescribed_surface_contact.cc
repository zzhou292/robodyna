#include "lib_utest/prescribed_surface_contact_fixture.h"
#include "lib_src/collision/T3ContactIntegration.h"

#include <gtest/gtest.h>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace test=prescribed_surface_test;
using Code=sc::PrescribedSurfaceStatus;
using Family=sc::PrescribedSurfaceFamily;
using Result=sc::PrescribedSurfaceResult;
using q4_prescribed_test::Bytes;
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
void SameValues(const Result& a,const Result& b) {
  ASSERT_EQ(a.parent_count,b.parent_count); EXPECT_EQ(a.valid,b.valid);
  EXPECT_EQ(test::Forces(a),test::Forces(b));
  for (unsigned p=0;p<a.parent_count;++p) {
    EXPECT_EQ(a.parents[p].family,b.parents[p].family);
    if (a.parents[p].family==Family::Q4CenterAreaUniformNatural) {
      Same(a.parents[p].q4.integration.resultant,b.parents[p].q4.integration.resultant);
      Same(a.parents[p].q4.integration.potential,b.parents[p].q4.integration.potential);
    } else { Same(a.parents[p].t3.resultant,b.parents[p].t3.resultant); Same(a.parents[p].t3.potential,b.parents[p].t3.potential); }
  }
}

TEST(PrescribedSurfaceContact, NativeTriangleFiniteWallEdgeOnNeedsNoQ4ScratchOrLimits) {
  test::Fixture f; ASSERT_TRUE(f.Initialize());
  for (unsigned node:f.triangle.nodes) { f.endpoint[3*node+1]=0; }
  f.endpoint[3*2]=.0390625; f.endpoint[3*3]=.0234375; f.endpoint[3*4]=.046875;
  f.base=f.endpoint;
  const auto requests=f.Requests(); auto input=f.Input(&requests[1],1);
  auto config=f.config; config.q4.max_leaves=0; config.q4.force_error=0;
  Result result;
  ASSERT_EQ(sc::IntegratePrescribedSurfaceContact(f.wall,input,config,7,{},&result).status,Code::Ok);
  ASSERT_EQ(result.parent_count,1u); EXPECT_TRUE(result.valid);
  const auto& parent=result.parents[0]; EXPECT_EQ(parent.family,Family::T3NativeLinear);
  EXPECT_FALSE(parent.q4.integration.valid); EXPECT_TRUE(parent.t3.valid);
  EXPECT_EQ(parent.coverage.physical.minimum.y,0); EXPECT_EQ(parent.coverage.physical.maximum.y,0);
  EXPECT_GT(parent.coverage.upper_expansion_upper.y,0);
  const sc::LinearTriangleSurfaceView surface{f.View(f.endpoint),f.View(f.velocity),f.mass.inverse.data(),&f.triangle_endpoint,1};
  const sc::T3NormalIntegrationInput raw{&f.t3_reference,surface,f.mass.view(),0,7,0,16,.125};
  sc::T3IntegrationResult independent;
  ASSERT_EQ(sc::IntegrateT3NormalContact(raw,config.t3,&independent).status,sc::T3IntegrationStatus::Ok);
  Same(parent.t3.resultant,independent.resultant); Same(parent.t3.potential,independent.potential);
  for (unsigned n=0;n<3;++n) EXPECT_EQ(parent.t3.nodal.nodes[n],f.triangle.nodes[n]);
}

TEST(PrescribedSurfaceContact, MixedSharedNodesMatchStandaloneAndIndependentUniformMomentWork) {
  test::Fixture f; ASSERT_TRUE(f.Initialize()); Result mixed;
  ASSERT_EQ(f.Evaluate(&mixed).status,Code::Ok);
  const auto requests=f.Requests(); Result quad,triangle;
  auto only_quad=f.config; only_quad.t3={}; // An unused family's limits do not constrain this call.
  ASSERT_EQ(sc::IntegratePrescribedSurfaceContact(f.wall,f.Input(&requests[0],1),only_quad,7,f.scratch.view(),&quad).status,Code::Ok);
  ASSERT_EQ(sc::IntegratePrescribedSurfaceContact(f.wall,f.Input(&requests[1],1),f.config,7,{},&triangle).status,Code::Ok);
  Same(mixed.parents[0].q4.integration.resultant,quad.parents[0].q4.integration.resultant);
  Same(mixed.parents[1].t3.resultant,triangle.parents[0].t3.resultant);
  EXPECT_FALSE(mixed.parents[0].t3.valid); EXPECT_FALSE(mixed.parents[1].q4.integration.valid);
  const auto force=test::Forces(mixed);
  const long double expected[5]={-1.L/8,-1.L/8,-5.L/24,-5.L/24,-1.L/12};
  long double moment_y=0,moment_z=0,power=0,expected_power=0,sum=0;
  for (unsigned n=0;n<5;++n) {
    EXPECT_NEAR(force[n],expected[n],2e-15); sum+=force[n];
    moment_y+=static_cast<long double>(f.endpoint[3*n+2])*force[n];
    moment_z-=static_cast<long double>(f.endpoint[3*n+1])*force[n];
    power+=static_cast<long double>(f.velocity[3*n])*force[n];
    expected_power+=static_cast<long double>(f.velocity[3*n])*expected[n];
  }
  EXPECT_NEAR(sum,-.75L,2e-15); EXPECT_NEAR(moment_y,0,2e-15);
  EXPECT_NEAR(moment_z,-1.L/6,2e-15); EXPECT_NEAR(power,expected_power,2e-15);
  EXPECT_GT(mixed.parents[0].q4.integration.potential.value,0); EXPECT_GT(mixed.parents[1].t3.potential.value,0);
}

TEST(PrescribedSurfaceContact, DistinctPartialActivationAndFamilyOrderIgnoreWallTessellation) {
  std::array<double,5> expected{};
  for (unsigned variant=0;variant<4;++variant) {
    test::Fixture f; ASSERT_TRUE(f.Initialize(variant));
    const double gap[5]={-.03125,.03125,.03125,-.03125,.03125};
    for (unsigned n=0;n<5;++n) { f.endpoint[3*n]=gap[n]; f.endpoint[3*n+1]+=.125; f.endpoint[3*n+2]-=.125; }
    Result result; ASSERT_EQ(f.Evaluate(&result).status,Code::Ok);
    EXPECT_GT(result.parents[0].q4.integration.active_area.lower,0);
    EXPECT_LT(result.parents[0].q4.integration.active_area.upper,1);
    EXPECT_GT(result.parents[1].t3.active_area.lower,0); EXPECT_LT(result.parents[1].t3.active_area.upper,.5);
    if (variant==0) expected=test::Forces(result); else EXPECT_EQ(test::Forces(result),expected);
    auto requests=f.Requests(); std::swap(requests[0],requests[1]); Result reversed;
    ASSERT_EQ(sc::IntegratePrescribedSurfaceContact(f.wall,f.Input(requests.data()),f.config,7,f.scratch.view(),&reversed).status,Code::Ok);
    EXPECT_EQ(reversed.parents[0].family,Family::T3NativeLinear); EXPECT_EQ(test::Forces(reversed),expected);
  }
}

TEST(PrescribedSurfaceContact, EverySecondParentLimitEndpointAndCoverageRejectsBeforeAnyScratchWrite) {
  test::Fixture f; ASSERT_TRUE(f.Initialize()); Result result;
  ASSERT_EQ(f.Evaluate(&result).status,Code::Ok); const auto before=Bytes(result);
  f.scratch.leaves[0].bounds.integrals[0]={123,456}; f.scratch.heap[0]=UINT32_MAX;
  const auto leaves=f.scratch.leaves; const auto heap=f.scratch.heap;
  const auto original_endpoint=f.endpoint,original_velocity=f.velocity;
  for (unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault); auto requests=f.Requests(); auto config=f.config;
    if (fault==0) config.t3.force_error=0;
    if (fault==1) f.velocity[3*4+2]=HUGE_VAL;
    if (fault==2) f.endpoint[3*4+1]=3;
    if (fault==3) f.endpoint[3*4]=.25;
    if (fault==4) requests[1].q4.reference_parent=1; // Inactive arm selector must also be empty.
    if (fault==5) { std::swap(requests[0],requests[1]); config.q4.max_visited=0; }
    const auto report=sc::IntegratePrescribedSurfaceContact(f.wall,f.Input(requests.data()),config,7,f.scratch.view(),&result);
    EXPECT_NE(report.status,Code::Ok); EXPECT_EQ(report.parent,1u); EXPECT_EQ(Bytes(result),before);
    EXPECT_EQ(std::memcmp(f.scratch.leaves.data(),leaves.data(),leaves.size()*sizeof(sc::Q4RectangularCell)),0);
    EXPECT_EQ(f.scratch.heap,heap); f.endpoint=original_endpoint; f.velocity=original_velocity;
  }
}

TEST(PrescribedSurfaceContact, LaterNativeSampleFailureNeverPublishesEarlierQ4AndRetryMatches) {
  test::Fixture f; ASSERT_TRUE(f.Initialize()); Result clean;
  ASSERT_EQ(f.Evaluate(&clean).status,Code::Ok); auto result=clean; const auto before=Bytes(result);
  const auto base=f.base,endpoint=f.endpoint,velocity=f.velocity; const auto mass=f.mass; const auto config=f.config;
  for (unsigned n=0;n<4;++n) f.endpoint[3*n]=-1;
  f.endpoint[3*4]=1e-10; f.base=f.endpoint;
  f.mass.fixed[4]=1; f.mass.inverse[4]=0; f.velocity[12]=f.velocity[13]=f.velocity[14]=0;
  f.mass.inverse[2]=f.mass.inverse[3]=1e-308;
  f.config.maximum_penetration=4;
  const auto report=f.Evaluate(&result);
  EXPECT_EQ(report.status,Code::IntegrationFailure); EXPECT_EQ(report.parent,1u);
  EXPECT_EQ(report.q4.status,sc::Q4IntegrationStatus::Ok);
  EXPECT_EQ(report.t3.status,sc::T3IntegrationStatus::NonFiniteArithmetic); EXPECT_NE(report.t3.sample,UINT32_MAX);
  EXPECT_EQ(Bytes(result),before);
  f.base=base; f.endpoint=endpoint; f.velocity=velocity; f.mass=mass; f.config=config;
  ASSERT_EQ(f.Evaluate(&result).status,Code::Ok); SameValues(result,clean);
}

TEST(PrescribedSurfaceContact, MalformedFamiliesForeignReferenceExtentsDuplicatesAndFixedNodesStayHonest) {
  test::Fixture f; ASSERT_TRUE(f.Initialize()); Result result;
  ASSERT_EQ(f.Evaluate(&result).status,Code::Ok); const auto before=Bytes(result);
  std::array<double,18> bigger{}; std::copy(f.reference.begin(),f.reference.end(),bigger.begin());
  sc::Q4ParametricReference foreign;
  ASSERT_EQ(foreign.Initialize({bigger.data(),6,3,1},&f.quad,1).status,sc::Q4ParametricStatus::Ok);
  auto duplicate=f.triangle; duplicate.parent_element_id=f.quad.parent_element_id;
  sc::T3MaterialMeasure duplicate_reference;
  ASSERT_EQ(sc::PrepareT3MaterialMeasure(f.View(f.reference),duplicate,&duplicate_reference),sc::SurfaceMeasureStatus::Ok);
  for (unsigned fault=0;fault<8;++fault) {
    auto requests=f.Requests(); auto input=f.Input(requests.data());
    if (fault==0) requests[0].family=Family::Unspecified;
    if (fault==1) requests[0].family=static_cast<Family>(19);
    if (fault==2) requests[1].t3.reference=nullptr;
    if (fault==3) requests[0].q4.reference_parent=1;
    if (fault==4) requests[0].q4.reference=&foreign;
    if (fault==5) requests[1].t3={&duplicate_reference,&duplicate,&duplicate};
    if (fault==6) input.parent_count=3;
    if (fault==7) input.mass.model=sc::TranslationMassModel::kUnspecified;
    EXPECT_NE(sc::IntegratePrescribedSurfaceContact(f.wall,input,f.config,7,f.scratch.view(),&result).status,Code::Ok);
    EXPECT_EQ(Bytes(result),before);
  }
  f.base=f.endpoint; f.mass.fixed[4]=1; f.mass.inverse[4]=0; f.velocity[12]=f.velocity[13]=f.velocity[14]=0;
  ASSERT_EQ(f.Evaluate(&result).status,Code::Ok); EXPECT_LT(result.parents[1].t3.nodal.forces[2].x,0);
  const auto fixed_before=Bytes(result); f.velocity[13]=.001;
  EXPECT_EQ(f.Evaluate(&result).status,Code::FixedMotion); EXPECT_EQ(Bytes(result),fixed_before);
  f.velocity[13]=0; f.mass.fixed[4]=6;
  EXPECT_EQ(f.Evaluate(&result).status,Code::MassFailure); EXPECT_EQ(Bytes(result),fixed_before);
}
} // namespace
