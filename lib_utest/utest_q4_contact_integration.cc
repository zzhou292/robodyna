#include "lib_utest/q4_contact_integration_fixture.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>

namespace {
namespace sc=tlfea::contact;
namespace qb=sc::q4_bounds;
using q4_contact_test::Fixture;
using q4_contact_test::Limits;
struct Scratch {
  explicit Scratch(unsigned count=sc::MaxQ4IntegrationLeaves) : leaves(count),heap(count) {}
  std::vector<sc::Q4IntegrationCell> leaves;
  std::vector<std::uint32_t> heap;
  sc::Q4IntegrationScratch View() {
    return {leaves.data(),heap.data(),static_cast<unsigned>(leaves.size()),static_cast<unsigned>(heap.size())};
  }
};
void Same(sc::Vec3 a,sc::Vec3 b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.y,b.y); EXPECT_EQ(a.z,b.z);
}
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
  EXPECT_EQ(a.value,b.value); EXPECT_EQ(a.lower,b.lower); EXPECT_EQ(a.upper,b.upper); EXPECT_EQ(a.error,b.error);
}
void Same(const sc::Q4IntegrationResult& a,const sc::Q4IntegrationResult& b) {
  for (unsigned i=0;i<4;++i) {
    EXPECT_EQ(a.nodal.nodes[i],b.nodal.nodes[i]); Same(a.nodal.forces[i],b.nodal.forces[i]);
    Same(a.nodal.couples[i],b.nodal.couples[i]); Same(a.force[i],b.force[i]);
  }
  Same(a.resultant,b.resultant); Same(a.potential,b.potential);
  EXPECT_EQ(a.active_area.lower,b.active_area.lower); EXPECT_EQ(a.active_area.upper,b.active_area.upper);
  EXPECT_EQ(a.feature_id,b.feature_id); EXPECT_EQ(a.parent_element_id,b.parent_element_id);
  EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
  EXPECT_EQ(a.leaf_count,b.leaf_count); EXPECT_EQ(a.visited,b.visited); EXPECT_EQ(a.deepest_leaf,b.deepest_leaf);
  EXPECT_EQ(a.valid,b.valid);
}
void Encloses(sc::Q4CertifiedIntegral result,long double truth,double budget) {
  EXPECT_LE(static_cast<long double>(result.lower),truth);
  EXPECT_GE(static_cast<long double>(result.upper),truth);
  EXPECT_LE(std::abs(static_cast<long double>(result.value)-truth),static_cast<long double>(result.error));
  EXPECT_LE(result.error,budget); EXPECT_GE(result.value,0);
}
void Check(const sc::Q4IntegrationResult& result,const Fixture& fixture,
           const q4_contact_test::Oracle& oracle,const sc::Q4IntegrationLimits& limits) {
  ASSERT_TRUE(result.valid);
  long double force=0;
  for (unsigned i=0;i<4;++i) {
    Encloses(result.force[i],oracle.force[i],limits.force_error); force+=oracle.force[i];
    EXPECT_EQ(result.nodal.nodes[i],fixture.parent.nodes[i]);
    Same(result.nodal.forces[i],{-result.force[i].value,0,0}); Same(result.nodal.couples[i],{});
  }
  Encloses(result.resultant,force,limits.force_error);
  Encloses(result.potential,oracle.potential,limits.energy_error);
  EXPECT_EQ(result.feature_id,73u); EXPECT_EQ(result.parent_element_id,42u);
  EXPECT_EQ(result.base_epoch,9u); EXPECT_EQ(result.attempt,7u);
}

TEST(Q4ContactBounds, ExactCancellationSignedRoundingAndLostPositiveTerms) {
  sc::Q4IntegralInterval interval;
  ASSERT_TRUE(qb::Difference(1,1,&interval)); EXPECT_EQ(interval.lower,0); EXPECT_EQ(interval.upper,0);
  ASSERT_TRUE(qb::Difference(1,std::ldexp(1.,-54),&interval));
  const long double exact=1.L-std::ldexp(1.L,-54);
  EXPECT_LE(static_cast<long double>(interval.lower),exact); EXPECT_GE(static_cast<long double>(interval.upper),exact);
  sc::Q4IntegralInterval zero;
  ASSERT_TRUE(qb::Add({.5,.5},{-.5,-.5},&zero)); EXPECT_EQ(zero.lower,0); EXPECT_EQ(zero.upper,0);
  EXPECT_FALSE(qb::Scale({std::numeric_limits<double>::denorm_min(),std::numeric_limits<double>::denorm_min()},.25,&interval));
  EXPECT_FALSE(qb::Scale({1e308,1e308},4,&interval));
  sc::Q4CertifiedIntegral outside;
  ASSERT_TRUE(qb::Certify(.1,{.2,.3},&outside));
  EXPECT_GE(outside.error,.3-.1);  // Width alone would falsely certify this estimate.
}

TEST(Q4ContactBounds, PowerOfTwoScalingEnclosesSignedNormalBoundaryProducts) {
  const double below_twice=std::nextafter(2*DBL_MIN,0.);
  const long double exact=static_cast<long double>(DBL_MIN)-
      static_cast<long double>(std::numeric_limits<double>::denorm_min())/2;
  sc::Q4IntegralInterval positive,negative;
  ASSERT_TRUE(qb::Scale({below_twice,below_twice},.5,&positive));
  ASSERT_TRUE(qb::Scale({-below_twice,-below_twice},.5,&negative));
  EXPECT_LE(static_cast<long double>(positive.lower),exact);
  EXPECT_GE(static_cast<long double>(positive.upper),exact);
  EXPECT_LE(static_cast<long double>(negative.lower),-exact);
  EXPECT_GE(static_cast<long double>(negative.upper),-exact);
  EXPECT_LT(positive.lower,DBL_MIN); EXPECT_GT(negative.upper,-DBL_MIN);
}

TEST(Q4ContactIntegration, UniformAndBilinearActiveMomentsHaveIndependentOracles) {
  Scratch scratch; Fixture fixture; auto limits=Limits(2);
  for (unsigned variant=0;variant<2;++variant) {
    if (variant == 0) fixture.Gaps(1,1,1,1);
    else fixture.Gaps(1.875,1.5,1,1.25);  // 1 + s/4 + t/2 + st/8.
    sc::Q4IntegrationResult result;
    ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
    Check(result,fixture,variant == 0 ? q4_contact_test::Uniform(1) : q4_contact_test::Bilinear(1,.25,.5,.125),limits);
    EXPECT_EQ(result.leaf_count,1u); EXPECT_EQ(result.visited,1u);
    EXPECT_LE(result.active_area.lower,1); EXPECT_GE(result.active_area.upper,1);
  }
}

TEST(Q4ContactIntegration, PartialCutHasUnequalForcesAndTighteningCertificates) {
  Fixture fixture; fixture.Gaps(.625,-.375,-.375,.625); Scratch scratch;
  auto coarse=Limits(); coarse.force_error=1e-3; coarse.energy_error=1e-4;
  const auto fine=Limits(); sc::Q4IntegrationResult a,b;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),coarse,scratch.View(),&a).status,sc::Q4IntegrationStatus::Ok);
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),fine,scratch.View(),&b).status,sc::Q4IntegrationStatus::Ok);
  Check(a,fixture,q4_contact_test::Cut(.375),coarse); Check(b,fixture,q4_contact_test::Cut(.375),fine);
  EXPECT_GT(b.force[0].value,b.force[1].value); EXPECT_GT(b.leaf_count,1u);
  EXPECT_LE(b.resultant.error,a.resultant.error); EXPECT_LE(b.potential.error,a.potential.error);
  EXPECT_LE(b.active_area.lower,.625); EXPECT_GE(b.active_area.upper,.625);
}

TEST(Q4ContactIntegration, TinyCornerMissedByRootGaussIsResolvedAndCertified) {
  constexpr double e=1./64; Fixture fixture; fixture.Gaps(e-2,e-1,e,e-1);
  Scratch scratch; const auto limits=Limits(e); sc::Q4IntegrationResult result;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
  Check(result,fixture,q4_contact_test::Corner(e),limits);
  EXPECT_GT(result.resultant.value,0); EXPECT_GT(result.potential.value,0);
  EXPECT_GT(result.leaf_count,1u); EXPECT_LE(result.active_area.lower,e*e/2); EXPECT_GE(result.active_area.upper,e*e/2);
}

TEST(Q4ContactIntegration, DisconnectedSaddleDoesNotMistakeEnergyAgreementForForceAccuracy) {
  Fixture fixture; fixture.Gaps(1,-1,1,-1); Scratch scratch;
  const auto limits=Limits(); sc::Q4IntegrationResult result;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
  Check(result,fixture,q4_contact_test::Saddle(),limits);
  EXPECT_NEAR(result.resultant.value,1./8,limits.force_error);
  EXPECT_GT(std::abs(result.resultant.value-1./6),.04);
  EXPECT_EQ(result.leaf_count,4u); EXPECT_EQ(result.visited,5u);
}

TEST(Q4ContactIntegration, ExactTouchAndNoContactAreZeroWhileUnresolvedAreaRemainsExplicit) {
  Fixture fixture; Scratch scratch; sc::Q4IntegrationResult result;
  for (double gap:{-1.,0.}) {
    fixture.Gaps(gap,gap,gap,gap);
    ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
    EXPECT_EQ(result.resultant.value,0); EXPECT_EQ(result.resultant.upper,0); EXPECT_EQ(result.resultant.error,0);
    EXPECT_EQ(result.potential.value,0); EXPECT_EQ(result.potential.upper,0); EXPECT_EQ(result.active_area.upper,0);
  }
  const double e=1e-8; fixture.Gaps(e-2,e-1,e,e-1);
  auto loose=Limits(); loose.force_error=1e-6; loose.energy_error=1e-8;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),loose,scratch.View(),&result).status,sc::Q4IntegrationStatus::Ok);
  EXPECT_EQ(result.resultant.value,0); EXPECT_GT(result.resultant.upper,0);
  EXPECT_GT(result.active_area.upper,0); EXPECT_EQ(result.active_area.lower,0);
}

TEST(Q4ContactIntegration, ActivationDerivativeUsesLinearTruncationAndEnergyUncertainty) {
  Fixture fixture; Scratch scratch; const auto limits=Limits();
  sc::Q4IntegrationResult base,plus,minus;
  fixture.Gaps(0,0,0,0);
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&base).status,sc::Q4IntegrationStatus::Ok);
  double previous=0;
  for (double delta:{.001,.0005}) {
    fixture.Gaps(delta,delta,delta,delta);
    ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&plus).status,sc::Q4IntegrationStatus::Ok);
    fixture.Gaps(-delta,-delta,-delta,-delta);
    ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&minus).status,sc::Q4IntegrationStatus::Ok);
    const double derivative=(plus.potential.value-minus.potential.value)/(2*delta);
    const double uncertainty=(plus.potential.error+minus.potential.error)/(2*delta)+base.resultant.error;
    EXPECT_NEAR(derivative,delta/4,1e-15); EXPECT_GT(derivative,delta*delta);
    EXPECT_LE(std::abs(derivative-base.resultant.value),.5*delta+uncertainty);
    if (previous > 0) EXPECT_NEAR(derivative,previous/2,1e-15);
    previous=derivative;
  }
}

TEST(Q4ContactIntegration, EveryCapacityFailurePreservesAllResultFieldsAndCleanRetry) {
  Fixture fixture; fixture.Gaps(1,1,1,1); Scratch scratch; sc::Q4IntegrationResult before;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&before).status,sc::Q4IntegrationStatus::Ok);
  const Fixture accepted=fixture;
  constexpr double e=1./64; fixture.Gaps(e-2,e-1,e,e-1);
  const sc::Q4IntegrationStatus expected[3]={sc::Q4IntegrationStatus::LeafLimit,
      sc::Q4IntegrationStatus::VisitLimit,sc::Q4IntegrationStatus::DepthLimit};
  for (unsigned variant=0;variant<3;++variant) {
    auto limits=Limits(e); if (variant == 0) limits.max_leaves=1;
    if (variant == 1) limits.max_visited=1;
    if (variant == 2) limits.max_depth=0;
    auto output=before;
    EXPECT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),limits,scratch.View(),&output).status,expected[variant]);
    Same(output,before);
    ASSERT_EQ(sc::IntegrateQ4NormalContact(accepted.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok);
    Same(output,before);
  }
}

TEST(Q4ContactIntegration, ExtremeThinEdgeRejectsAtDeclaredCapsWithoutFalseZeroCertificate) {
  Fixture fixture; fixture.Gaps(1,1,1,1); Scratch scratch; sc::Q4IntegrationResult output;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok);
  const auto before=output;
  fixture.Gaps(1,-1048575,-1048575,1);
  const auto report=sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&output);
  EXPECT_TRUE(report.status == sc::Q4IntegrationStatus::LeafLimit || report.status == sc::Q4IntegrationStatus::DepthLimit ||
              report.status == sc::Q4IntegrationStatus::VisitLimit);
  Same(output,before);
}

TEST(Q4ContactIntegration, InvalidLateInputsAndPositiveUnderflowPreserveCompleteOutput) {
  Fixture fixture; fixture.Gaps(1,1,1,1); Scratch scratch; sc::Q4IntegrationResult before;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&before).status,sc::Q4IntegrationStatus::Ok);
  for (unsigned variant=0;variant<9;++variant) {
    auto bad=fixture; auto limits=Limits(); auto input=bad.Input();
    if (variant == 0) input.wall_x=std::numeric_limits<double>::quiet_NaN();
    if (variant == 1) input.projected_area=0;
    if (variant == 2) bad.velocity[3*bad.parent.nodes[3]]=std::numeric_limits<double>::quiet_NaN();
    if (variant == 3) bad.fixed[bad.parent.nodes[3]]=0;
    if (variant == 4) bad.parent.half_thickness=.1;
    if (variant == 5) input.max_penetration=.5;
    if (variant == 6) limits.force_error=std::numeric_limits<double>::quiet_NaN();
    if (variant == 7) bad.inverse[bad.parent.nodes[0]]=0;
    if (variant == 8) limits.max_depth=sc::MaxQ4IntegrationDepth+1;
    auto output=before;
    EXPECT_NE(sc::IntegrateQ4NormalContact(input,limits,scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok) << variant;
    Same(output,before);
  }
  fixture.Gaps(1e-200,1e-200,1e-200,1e-200); auto output=before;
  EXPECT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
  Same(output,before);
  fixture=Fixture(); fixture.Gaps(1,1,1,1);
  for (unsigned i=0;i<4;++i) { fixture.fixed[i]=7; fixture.inverse[i]=0; }
  EXPECT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::NoDynamicDofs);
  Same(output,before);
}

TEST(Q4ContactIntegration, LateGaussMassFailurePreservesSeededResultAndNormalMassRetry) {
  Fixture fixture; fixture.Gaps(1,1,1,1); Scratch scratch;
  sc::Q4IntegrationResult output;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(fixture.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok);
  const auto before=output; const Fixture accepted=fixture;
  for (unsigned i=0;i<4;++i)
    if (fixture.fixed[i] == 6) fixture.inverse[i]=16*std::numeric_limits<double>::denorm_min();
  sc::NormalJacobian center,sample;
  const auto input=fixture.Input();
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(input.mass,fixture.parent,0,0,7,&center),sc::Status::kOk);
  const double gauss=-1/std::sqrt(3.);
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(input.mass,fixture.parent,gauss,gauss,7,&sample),sc::Status::kNonFiniteResult);
  const auto report=sc::IntegrateQ4NormalContact(input,Limits(),scratch.View(),&output);
  EXPECT_EQ(report.status,sc::Q4IntegrationStatus::NonFiniteArithmetic);
  EXPECT_EQ(report.cell,0u); EXPECT_EQ(report.visited,1u);
  Same(output,before);
  ASSERT_EQ(sc::IntegrateQ4NormalContact(accepted.Input(),Limits(),scratch.View(),&output).status,sc::Q4IntegrationStatus::Ok);
  Same(output,before);
}

TEST(Q4ContactIntegration, AreaStiffnessAndCommonWorldTranslationRespectPhysicalScaling) {
  Fixture fixture; fixture.Gaps(.25,.25,.25,.25); Scratch scratch;
  // Width 1/2 and height 1/4 give the supplied projected area 1/8.
  for (unsigned i=0;i<4;++i) { fixture.position[i+4]*=.5; fixture.position[i+8]*=.25; }
  auto input=fixture.Input(); input.projected_area=.125; input.stiffness_per_area=16;
  const auto a=input.surface.positions.at(fixture.parent.nodes[0]);
  const auto b=input.surface.positions.at(fixture.parent.nodes[1]);
  const auto c=input.surface.positions.at(fixture.parent.nodes[2]);
  EXPECT_EQ((a.y-b.y)*(b.z-c.z),input.projected_area);
  sc::Q4IntegrationResult base,shifted;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(input,Limits(),scratch.View(),&base).status,sc::Q4IntegrationStatus::Ok);
  auto oracle=q4_contact_test::Uniform(.25);
  for (auto& f:oracle.force) f*=2; oracle.potential*=2;
  Check(base,fixture,oracle,Limits());
  for (unsigned i=0;i<4;++i) fixture.position[i]+=8;
  input.wall_x=8;
  ASSERT_EQ(sc::IntegrateQ4NormalContact(input,Limits(),scratch.View(),&shifted).status,sc::Q4IntegrationStatus::Ok);
  Same(base,shifted);
}
}  // namespace
