#include "lib_src/collision/T3ContactIntegration.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
namespace sc=tlfea::contact;
using Code=sc::T3IntegrationStatus;
// Declared before first execution. These unit-scale arithmetic budgets do not
// replace any physical case's stricter independently declared N/J budgets.
constexpr sc::T3IntegrationLimits UnitBudget{1e-10,1e-11};
constexpr unsigned Physical[3]={5,1,7};
struct Fixture {
  double reference_position[24]{},position[24]{},velocity[24]{},inverse[8]{};
  std::uint8_t fixed[8]{};
  sc::SurfaceTriangle parents[2];
  sc::T3MaterialMeasure reference;
  Fixture() {
    for (unsigned i=0;i<8;++i) inverse[i]=1+i;
    parents[0]={{0,2,3},11,21,0,0,sc::SurfaceInterpolation::kLinearTriangle};
    parents[1]={{5,1,7},103,203,9,0,sc::SurfaceInterpolation::kLinearTriangle};
    reference_position[3*1+1]=2; reference_position[3*7+2]=1; // Area exactly one.
    std::copy(reference_position,reference_position+24,position);
    for (unsigned n=0;n<3;++n) {
      velocity[3*Physical[n]]=.25+n;
      velocity[3*Physical[n]+1]=-.5*n; velocity[3*Physical[n]+2]=.125;
    }
  }
  sc::VectorView View(const double* pointer) const { return {pointer,8,3,1}; }
  bool Prepare() { return sc::PrepareT3MaterialMeasure(View(reference_position),parents[1],&reference)==sc::SurfaceMeasureStatus::Ok; }
  void Gaps(std::array<double,3> gap) { for(unsigned n=0;n<3;++n) position[3*Physical[n]]=gap[n]; }
  sc::T3NormalIntegrationInput Input() const {
    return {&reference,{View(position),View(velocity),inverse,parents,2},
            {inverse,fixed,8,17,sc::TranslationMassModel::kIsotropicLumped},1,31,0,1,4};
  }
};
sc::T3IntegrationReport IntegrateFixture(const Fixture& fixture,sc::T3IntegrationResult* output,
                          sc::T3IntegrationLimits budget=UnitBudget) {
  return sc::IntegrateT3NormalContact(fixture.Input(),budget,output);
}
void Encloses(sc::Q4CertifiedIntegral c,long double truth) {
  EXPECT_LE(static_cast<long double>(c.lower),truth);
  EXPECT_GE(static_cast<long double>(c.upper),truth);
  EXPECT_LE(std::abs(static_cast<long double>(c.value)-truth),static_cast<long double>(c.error));
}
void Encloses(sc::Q4IntegralInterval c,long double truth) {
  EXPECT_LE(static_cast<long double>(c.lower),truth); EXPECT_GE(static_cast<long double>(c.upper),truth);
}
void SameBytes(const sc::T3IntegrationResult& a,const sc::T3IntegrationResult& b) {
  EXPECT_EQ(std::memcmp(&a,&b,sizeof(a)),0);
}
void SameValues(const sc::T3IntegrationResult& a,const sc::T3IntegrationResult& b) {
  EXPECT_EQ(a.valid,b.valid); EXPECT_EQ(a.feature_id,b.feature_id); EXPECT_EQ(a.parent_element_id,b.parent_element_id);
  EXPECT_EQ(a.parent_face_id,b.parent_face_id); EXPECT_EQ(a.base_epoch,b.base_epoch); EXPECT_EQ(a.attempt,b.attempt);
  auto cert=[](const auto& x,const auto& y) {
    EXPECT_EQ(x.value,y.value); EXPECT_EQ(x.lower,y.lower); EXPECT_EQ(x.upper,y.upper); EXPECT_EQ(x.error,y.error);
  };
  cert(a.resultant,b.resultant); cert(a.potential,b.potential);
  for (unsigned n=0;n<3;++n) {
    cert(a.force[n],b.force[n]); EXPECT_EQ(a.nodal.nodes[n],b.nodal.nodes[n]);
    EXPECT_EQ(a.nodal.forces[n].x,b.nodal.forces[n].x);
    EXPECT_EQ(a.nodal.forces[n].y,b.nodal.forces[n].y); EXPECT_EQ(a.nodal.forces[n].z,b.nodal.forces[n].z);
  }
  EXPECT_EQ(a.active_area.lower,b.active_area.lower); EXPECT_EQ(a.active_area.upper,b.active_area.upper);
  EXPECT_EQ(a.subtriangle_count,b.subtriangle_count); EXPECT_EQ(a.sample_count,b.sample_count);
}

TEST(T3ContactIntegration, IndependentSimplexTruthsAndAllSixNativePermutations) {
  struct Truth { std::array<double,3> gap; long double force[3],energy,area; unsigned pieces; };
  const Truth truths[]={
    {{-1,-1,-1},{0,0,0},0,0,0},{{0,0,0},{0,0,0},0,0,0},
    {{1,1,1},{1.L/3,1.L/3,1.L/3},.5L,1,1},
    {{1,0,0},{1.L/6,1.L/12,1.L/12},1.L/12,1,1},
    {{1,-1,-1},{1.L/16,1.L/96,1.L/96},1.L/48,.25L,1},
    {{1,1,-1},{17.L/96,17.L/96,1.L/16},7.L/48,.75L,2}};
  for (unsigned t=0;t<6;++t) {
    SCOPED_TRACE(t);
    std::array<unsigned,3> permutation{0,1,2};
    do {
      Fixture f; f.Gaps(truths[t].gap);
      for (unsigned n=0;n<3;++n) f.parents[1].nodes[n]=Physical[permutation[n]];
      ASSERT_TRUE(f.Prepare()); sc::T3IntegrationResult result;
      ASSERT_EQ(IntegrateFixture(f,&result).status,Code::Ok);
      long double force=0;
      for (unsigned n=0;n<3;++n) {
        const auto expected=truths[t].force[permutation[n]]; force+=expected;
        Encloses(result.force[n],expected); EXPECT_EQ(result.nodal.nodes[n],Physical[permutation[n]]);
        EXPECT_EQ(result.nodal.forces[n].x,-result.force[n].value);
        EXPECT_EQ(result.nodal.forces[n].y,0); EXPECT_EQ(result.nodal.forces[n].z,0);
      }
      Encloses(result.resultant,force); Encloses(result.potential,truths[t].energy);
      Encloses(result.active_area,truths[t].area); EXPECT_EQ(result.subtriangle_count,truths[t].pieces);
      EXPECT_EQ(result.sample_count,3*truths[t].pieces); EXPECT_TRUE(result.valid);
      EXPECT_EQ(result.feature_id,103); EXPECT_EQ(result.parent_element_id,203); EXPECT_EQ(result.parent_face_id,9);
      EXPECT_EQ(result.base_epoch,17); EXPECT_EQ(result.attempt,31);
    } while (std::next_permutation(permutation.begin(),permutation.end()));
  }
}

TEST(T3ContactIntegration, SmallSliverAndExactCoordinateSubtractionHaveCertifiedTruth) {
  // Independent one-dimensional section integration: g=d at vertex0, -b on
  // the opposite edge; alpha=d/(d+b), active area=alpha² (unit parent area).
  for (unsigned scenario=0;scenario<3;++scenario) {
    SCOPED_TRACE(scenario);
    Fixture f; ASSERT_TRUE(f.Prepare());
    auto input=f.Input();
    if (scenario==0) f.Gaps({std::ldexp(1.,-20),-1,-1});
    if (scenario==1) { input.wall_x=.1; f.Gaps({1,std::ldexp(1.,-20),std::ldexp(1.,-20)}); }
    if (scenario==2) {
      input.wall_x=.125;
      f.Gaps({std::nextafter(.125,1.),std::nextafter(.125,0.),std::nextafter(.125,0.)});
    }
    const long double d=static_cast<long double>(f.position[3*5])-input.wall_x;
    const long double b=input.wall_x-static_cast<long double>(f.position[3*1]);
    const long double alpha=d/(d+b),area=alpha*alpha;
    sc::T3IntegrationResult result;
    ASSERT_EQ(sc::IntegrateT3NormalContact(input,UnitBudget,&result).status,Code::Ok);
    Encloses(result.active_area,area);
    Encloses(result.force[0],area*d*(2-alpha)/6);
    Encloses(result.force[1],area*d*alpha/12); Encloses(result.force[2],area*d*alpha/12);
    Encloses(result.resultant,area*d/3); Encloses(result.potential,area*d*d/12);
    EXPECT_GT(result.resultant.value,0); EXPECT_GT(result.potential.value,0);
  }
}

TEST(T3ContactIntegration, ImmutableReferenceMeasureSurvivesCurrentProjectionChangesAndScaling) {
  Fixture f; f.Gaps({.5,.5,-.5}); ASSERT_TRUE(f.Prepare());
  sc::T3IntegrationResult baseline,moved;
  ASSERT_EQ(IntegrateFixture(f,&baseline).status,Code::Ok);
  // Reverse and shear the wall projection, retaining exact X and physical
  // source IDs. Raw integration makes no current-shell regularity claim.
  for (unsigned n=0;n<3;++n) {
    const auto i=Physical[n]; const double y=f.position[3*i+1],z=f.position[3*i+2];
    f.position[3*i+1]=3-y+2*z; f.position[3*i+2]=-7+3*z;
  }
  ASSERT_EQ(IntegrateFixture(f,&moved).status,Code::Ok); SameValues(baseline,moved);
  // Proper cyclic world-axis permutation of the reference, followed by scale2.
  for (unsigned n=0;n<3;++n) {
    const auto i=Physical[n]; const double x=f.reference_position[3*i],y=f.reference_position[3*i+1],z=f.reference_position[3*i+2];
    f.reference_position[3*i]=2*z+4; f.reference_position[3*i+1]=2*x-3; f.reference_position[3*i+2]=2*y+8;
  }
  ASSERT_TRUE(f.Prepare()); ASSERT_EQ(IntegrateFixture(f,&moved).status,Code::Ok);
  for (unsigned n=0;n<3;++n) Encloses(moved.force[n],n<2 ? 17.L/48 : 1.L/8);
  Encloses(moved.resultant,5.L/6); Encloses(moved.potential,7.L/48);
}

TEST(T3ContactIntegration, EveryCoordinateForceMatchesIndependentEnergyDifferencesAndWork) {
  for (const auto gaps:{std::array<double,3>{.25,.5,.75},std::array<double,3>{.5,-.25,-.75}}) {
    Fixture f; f.Gaps(gaps); ASSERT_TRUE(f.Prepare()); sc::T3IntegrationResult center;
    ASSERT_EQ(IntegrateFixture(f,&center).status,Code::Ok);
    for (double h:{1e-4,5e-5}) for (unsigned n=0;n<3;++n) for (unsigned axis=0;axis<3;++axis) {
      const unsigned coordinate=3*Physical[n]+axis; const double x=f.position[coordinate];
      sc::T3IntegrationResult plus,minus;
      f.position[coordinate]=x+h; ASSERT_EQ(IntegrateFixture(f,&plus).status,Code::Ok);
      f.position[coordinate]=x-h; ASSERT_EQ(IntegrateFixture(f,&minus).status,Code::Ok);
      f.position[coordinate]=x;
      const long double derivative=(static_cast<long double>(plus.potential.value)-minus.potential.value)/(2*h);
      const long double uncertainty=(static_cast<long double>(plus.potential.error)+minus.potential.error)/(2*h);
      const long double force=axis==0 ? center.nodal.forces[n].x : 0;
      // Partial positive-region shape varies smoothly in this fixed-sign
      // fixture. Predeclared centered truncation allowance50*h², plus BOTH
      // energy radii and arithmetic allowance; no continuum tolerance tuning.
      EXPECT_LE(std::abs(derivative+force),50*h*h+uncertainty+2e-10);
    }
    // Virtual rigid rotation about world Z: dx=-y*dtheta, so the force work
    // predicts the independently rotated configuration's energy derivative.
    const double h=5e-5; double torque=0;
    for (unsigned n=0;n<3;++n) torque-=f.position[3*Physical[n]+1]*center.nodal.forces[n].x;
    std::array<double,24> original; std::copy(f.position,f.position+24,original.begin());
    sc::T3IntegrationResult spin[2];
    for (unsigned side=0;side<2;++side) {
      const double angle=side ? -h : h,c=std::cos(angle),s=std::sin(angle);
      for (unsigned n=0;n<3;++n) {
        const unsigned i=3*Physical[n]; f.position[i]=c*original[i]-s*original[i+1];
        f.position[i+1]=s*original[i]+c*original[i+1];
      }
      ASSERT_EQ(IntegrateFixture(f,&spin[side]).status,Code::Ok);
    }
    const long double derivative=(static_cast<long double>(spin[0].potential.value)-spin[1].potential.value)/(2*h);
    EXPECT_LE(std::abs(derivative+torque),50*h*h+(spin[0].potential.error+spin[1].potential.error)/(2*h)+2e-10);
  }
}

TEST(T3ContactIntegration, GenuineMassAndFixedMaskUseNativeInterpolationAndPhysicalImpulse) {
  Fixture f; f.Gaps({1,1,1}); ASSERT_TRUE(f.Prepare());
  f.inverse[5]=1; f.inverse[1]=4; f.inverse[7]=9;
  const double weights[3]={.5,.25,.25}; sc::NormalJacobian jacobian;
  ASSERT_EQ(sc::BuildLinearTriangleNormalJacobian(f.Input().mass,f.parents[1],weights,nullptr,nullptr,
      {-1,0,0},31,&jacobian),sc::Status::kOk);
  const double impulse=.125;
  double projected_velocity=0;
  for (unsigned n=0;n<3;++n) {
    const double delta_velocity=-impulse*weights[n]*f.inverse[Physical[n]];
    projected_velocity-=weights[n]*delta_velocity;
  }
  EXPECT_DOUBLE_EQ(projected_velocity/impulse,.25+4./16+9./16);
  EXPECT_NEAR(jacobian.inverse_effective_mass,projected_velocity/impulse,2e-15);
  sc::T3IntegrationResult free,fixed;
  ASSERT_EQ(IntegrateFixture(f,&free).status,Code::Ok);
  f.inverse[1]=0; f.fixed[1]=1; f.velocity[3*1]=f.velocity[3*1+1]=f.velocity[3*1+2]=0;
  ASSERT_EQ(IntegrateFixture(f,&fixed).status,Code::Ok); SameValues(free,fixed);
  // The prescribed wall has no mass entry, and unused global nodes need not
  // belong to this local parent. Free zeros never stand in for fixed nodes.
  f.inverse[0]=-1; ASSERT_EQ(IntegrateFixture(f,&fixed).status,Code::Ok);
  f.fixed[1]=0; EXPECT_EQ(IntegrateFixture(f,&fixed).status,Code::InvalidInput);
}

TEST(T3ContactIntegration, InvalidInputsAndReferenceMismatchPreserveEveryOutputByteThenRetry) {
  Fixture good; good.Gaps({.5,.25,-.5}); ASSERT_TRUE(good.Prepare()); sc::T3IntegrationResult result;
  ASSERT_EQ(IntegrateFixture(good,&result).status,Code::Ok);
  std::array<unsigned char,sizeof(result)> retained{}; std::memcpy(retained.data(),&result,sizeof(result));
  for (unsigned fault=0;fault<16;++fault) {
    SCOPED_TRACE(fault); Fixture f=good; auto input=f.Input(); auto limits=UnitBudget;
    sc::T3MaterialMeasure unprepared;
    if (fault==0) input.reference=nullptr;
    if (fault==1) input.reference=&unprepared;
    if (fault==2) input.triangle_index=2;
    if (fault==3) input.attempt=0;
    if (fault==4) input.wall_x=std::numeric_limits<double>::infinity();
    if (fault==5) input.stiffness_per_area=0;
    if (fault==6) input.max_penetration=.25;
    if (fault==7) limits.force_error=0;
    if (fault==8) input.mass.model=sc::TranslationMassModel::kGeneralizedOrRotational;
    if (fault==9) f.parents[1].feature_id++;
    if (fault==10) f.parents[1].nodes[2]=f.parents[1].nodes[1];
    if (fault==11) f.parents[1].half_thickness=.01;
    if (fault==12) f.velocity[3*7+2]=std::numeric_limits<double>::quiet_NaN();
    if (fault==13) f.fixed[7]=2;
    if (fault==14) input.surface.inverse_node_mass=nullptr;
    if (fault==15) input.mass.node_count=7;
    EXPECT_NE(sc::IntegrateT3NormalContact(input,limits,&result).status,Code::Ok);
    EXPECT_EQ(std::memcmp(retained.data(),&result,sizeof(result)),0);
  }
  EXPECT_EQ(sc::IntegrateT3NormalContact(good.Input(),UnitBudget,nullptr).status,Code::InvalidInput);
  sc::T3IntegrationResult retry; ASSERT_EQ(IntegrateFixture(good,&retry).status,Code::Ok); SameValues(result,retry);
}

TEST(T3ContactIntegration, InactiveMassChecksAccuracyAndLateArithmeticFailuresAreTransactional) {
  Fixture f; f.Gaps({1,1,-1}); ASSERT_TRUE(f.Prepare()); sc::T3IntegrationResult result;
  ASSERT_EQ(IntegrateFixture(f,&result).status,Code::Ok);
  sc::T3IntegrationResult before; std::memcpy(&before,&result,sizeof(result));
  EXPECT_EQ(IntegrateFixture(f,&result,{std::numeric_limits<double>::min(),std::numeric_limits<double>::min()}).status,
            Code::UnattainableAccuracy); SameBytes(result,before);
  Fixture inactive=f; inactive.Gaps({-1,-1,-1});
  for (unsigned n:Physical) { inactive.fixed[n]=1; inactive.inverse[n]=0; }
  EXPECT_EQ(IntegrateFixture(inactive,&result).status,Code::NoDynamicDofs); SameBytes(result,before);
  Fixture overflow=f; auto input=overflow.Input(); input.stiffness_per_area=std::numeric_limits<double>::max();
  overflow.Gaps({2,2,2});
  EXPECT_EQ(sc::IntegrateT3NormalContact(input,UnitBudget,&result).status,Code::NonFiniteArithmetic);
  SameBytes(result,before);
  Fixture underflow=f; underflow.Gaps({1e-200,1e-200,1e-200});
  EXPECT_EQ(IntegrateFixture(underflow,&result).status,Code::NonFiniteArithmetic); SameBytes(result,before);
  // Center mass is representable, but a clipped sample concentrates near one
  // fixed vertex, making the actual free-node squared weight underflow.
  Fixture late=f; late.Gaps({1e-10,-1,-1});
  late.fixed[5]=1; late.inverse[5]=0;
  late.inverse[1]=late.inverse[7]=1e-308;
  const auto report=IntegrateFixture(late,&result);
  EXPECT_EQ(report.status,Code::NonFiniteArithmetic); EXPECT_NE(report.sample,UINT32_MAX);
  SameBytes(result,before);
  sc::T3IntegrationResult retry; ASSERT_EQ(IntegrateFixture(f,&retry).status,Code::Ok); SameValues(before,retry);
}
} // namespace
