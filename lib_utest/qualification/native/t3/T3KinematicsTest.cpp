#include "T3KinematicsTestOracle.h"
#include <gtest/gtest.h>
#include <limits>

namespace {
namespace native=tl::qualification::t3;
namespace test=native::kinematic_test;
using native::Kinematics;
using native::Status;
using native::Vec3;

void Affine(native::PrescribedInterval& in) {
  for(unsigned n=0;n<3;++n) {
    const auto x=in.position[n];
    in.velocity[n]={.2+1.1*x.x-.4*x.y,-.1+.3*x.x+.7*x.y,.4-.6*x.x+.8*x.y};
    in.angular_velocity[n]={-.2+.9*x.x+.5*x.y,.6-.3*x.x+.4*x.y,2+.2*x.x};
  }
}
void CheckIndependent(const native::Reference& reference,const native::PrescribedInterval& in) {
  Kinematics value;
  ASSERT_EQ(native::EvaluatePrescribed(reference,in,value),Status::kSuccess);
  test::Check(value,in);
}

TEST(T3KinematicsCheck, CurrentGeometryUsesItsOwnAreaAndLeavesReferenceUntouched) {
  const auto r=test::Triangle(); const auto reference=test::MakeReference(r);
  const auto before=test::Bytes(reference);
  for(double scale:{1.,.02,.01}) {
    SCOPED_TRACE(scale);
    auto in=test::Interval(test::Triangle(scale));
    in.position[2].z=.17*scale;
    in.base_time=.125; in.sample_index=UINT64_C(9007199254741017);
    CheckIndependent(reference,in);
  }
  EXPECT_EQ(test::Bytes(reference),before);
  EXPECT_EQ(reference.data().startup_derivative,(std::array<double,3>{0,0,0}));
}

TEST(T3KinematicsCheck, AffineMembraneShearCurvatureAndQuarterStepHaveIndependentOracle) {
  for(double scale:{1.,.02,.01})for(double step:{.01,.0025}) {
    SCOPED_TRACE(scale);
    SCOPED_TRACE(step);
    const auto r=test::Triangle(scale); const auto reference=test::MakeReference(r);
    auto in=test::Interval(r,step); Affine(in); CheckIndependent(reference,in);
    Kinematics finite_h,half_h;
    ASSERT_EQ(native::EvaluatePrescribed(reference,in,finite_h),Status::kSuccess);
    in.dt*=.5;
    ASSERT_EQ(native::EvaluatePrescribed(reference,in,half_h),Status::kSuccess);
    // The normal corrections contain -h/2*(dvz/dx^2+dvy/dx^2),
    // and independently distinguish this complete leaf from an ordinary Bv.
    EXPECT_NEAR(finite_h.normalized_rate[0]-half_h.normalized_rate[0],
                -.25*step*(.6*.6+.3*.3),2e-12);
    EXPECT_NEAR(finite_h.normalized_rate[1]-half_h.normalized_rate[1],
                -.25*step*(.8*.8+.4*.4),2e-12);
  }
}

TEST(T3KinematicsCheck, EveryPhysicalVelocityAndAngularColumnRetainsNativeSigns) {
  const auto r=test::Triangle(.02); const auto reference=test::MakeReference(r);
  for(unsigned column=0;column<18;++column) {
    SCOPED_TRACE(column);
    auto positive=test::Interval(r),negative=positive;
    const unsigned n=column/6,axis=column%3;
    auto& vector=column%6<3?positive.velocity[n]:positive.angular_velocity[n];
    if(axis==0)vector.x=.125; else if(axis==1)vector.y=.125; else vector.z=.125;
    negative=positive;
    negative.velocity[n]=test::Narrow(test::Scale(test::Widen(positive.velocity[n]),-1));
    negative.angular_velocity[n]=test::Narrow(test::Scale(test::Widen(positive.angular_velocity[n]),-1));
    Kinematics plus,minus;
    ASSERT_EQ(native::EvaluatePrescribed(reference,positive,plus),Status::kSuccess);
    ASSERT_EQ(native::EvaluatePrescribed(reference,negative,minus),Status::kSuccess);
    test::Check(plus,positive); test::Check(minus,negative);
    auto infinitesimal=positive; infinitesimal.dt=0; // Analytic derivative only; never passed to native.
    const auto linear=test::Independent(infinitesimal);
    for(unsigned k=0;k<8;++k)
      EXPECT_NEAR(.5*(plus.raw_rate[k]-minus.raw_rate[k]),double(linear.raw[k]),2e-12*.125*.05);
  }
}

TEST(T3KinematicsCheck, TranslationAndInfinitesimalRigidRotationAreNull) {
  const auto r=test::Triangle(.02); const auto reference=test::MakeReference(r);
  auto translation=test::Interval(r);
  translation.velocity.fill({.25,-.375,.125});
  Kinematics moved;
  ASSERT_EQ(native::EvaluatePrescribed(reference,translation,moved),Status::kSuccess);
  for(double rate:moved.raw_rate)EXPECT_NEAR(rate,0,2e-12*.02*.5);
  for(unsigned axis=0;axis<3;++axis) {
    auto plus=test::Interval(r),minus=plus;
    test::Wide omega{}; omega[axis]=.5;
    for(unsigned n=0;n<3;++n) {
      plus.angular_velocity[n]=test::Narrow(omega);
      plus.velocity[n]=test::Narrow(test::Cross(omega,test::Widen(r.position[n])));
      minus.angular_velocity[n]=test::Narrow(test::Scale(omega,-1));
      minus.velocity[n]=test::Narrow(test::Scale(test::Widen(plus.velocity[n]),-1));
    }
    Kinematics a,b;
    ASSERT_EQ(native::EvaluatePrescribed(reference,plus,a),Status::kSuccess);
    ASSERT_EQ(native::EvaluatePrescribed(reference,minus,b),Status::kSuccess);
    test::Check(a,plus); test::Check(b,minus);
    for(unsigned k=0;k<8;++k)EXPECT_NEAR(.5*(a.raw_rate[k]-b.raw_rate[k]),0,2e-12*.02*.5);
  }
}

TEST(T3KinematicsCheck, WorldCovarianceCyclicNativeOrderAndExactEdgeOnRemainSupported) {
  const auto r=test::Triangle(.02); const auto reference=test::MakeReference(r);
  auto in=test::Interval(r); Affine(in);
  Kinematics baseline;
  ASSERT_EQ(native::EvaluatePrescribed(reference,in,baseline),Status::kSuccess);
  const auto q=test::Rotation(); auto transformed=in;
  for(unsigned n=0;n<3;++n) {
    transformed.position[n]=test::Rotate(q,in.position[n]);
    transformed.position[n].x+=.75; transformed.position[n].y-=1.25;
    transformed.velocity[n]=test::Rotate(q,in.velocity[n]);
    transformed.angular_velocity[n]=test::Rotate(q,in.angular_velocity[n]);
  }
  Kinematics rotated;
  ASSERT_EQ(native::EvaluatePrescribed(reference,transformed,rotated),Status::kSuccess);
  test::Check(rotated,transformed);
  for(unsigned k=0;k<8;++k)EXPECT_NEAR(rotated.raw_rate[k],baseline.raw_rate[k],2e-11*.05);
  for(unsigned shift=0;shift<3;++shift) {
    auto cyclic=transformed;
    for(unsigned n=0;n<3;++n) {
      cyclic.position[n]=transformed.position[(n+shift)%3];
      cyclic.velocity[n]=transformed.velocity[(n+shift)%3];
      cyclic.angular_velocity[n]=transformed.angular_velocity[(n+shift)%3];
    }
    // Edge-defined local interpolation changes under cyclic relabeling; check
    // its actual independent local polynomial, not forced equality of rates.
    CheckIndependent(reference,cyclic);
  }
  auto edge_on=in;
  for(unsigned n=0;n<3;++n) {
    const auto p=in.position[n],v=in.velocity[n],w=in.angular_velocity[n];
    edge_on.position[n]={p.y,p.z,p.x}; edge_on.velocity[n]={v.y,v.z,v.x};
    edge_on.angular_velocity[n]={w.y,w.z,w.x};
  }
  CheckIndependent(reference,edge_on);
}

TEST(T3KinematicsCheck, ConsistentFiniteZSpinHasItsOwnCubicRateResidual) {
  const auto r=test::Triangle(); const auto reference=test::MakeReference(r);
  double previous=0;
  for(double h:{.04,.02,.01}) {
    auto in=test::Interval(r,h);
    for(unsigned n=0;n<3;++n) {
      const auto x=r.position[n];
      in.position[n]={std::cos(h)*x.x-std::sin(h)*x.y,std::sin(h)*x.x+std::cos(h)*x.y,0};
      const Vec3 midpoint{std::cos(h/2)*x.x-std::sin(h/2)*x.y,std::sin(h/2)*x.x+std::cos(h/2)*x.y,0};
      in.velocity[n]={-midpoint.y,midpoint.x,0}; in.angular_velocity[n]={0,0,1};
    }
    Kinematics actual;
    ASSERT_EQ(native::EvaluatePrescribed(reference,in,actual),Status::kSuccess);
    test::Check(actual,in);
    const long double angle=static_cast<long double>(h)/2;
    const long double expected=std::sin(angle)-angle*std::cos(angle)*std::cos(angle);
    native::test::Near(actual.normalized_rate[0],expected,1);
    native::test::Near(actual.normalized_rate[1],expected,1);
    const double residual=std::max(std::abs(actual.normalized_rate[0]),std::abs(actual.normalized_rate[1]));
    EXPECT_GT(residual,0);
    if(previous>0) { EXPECT_GT(residual/previous,.12); EXPECT_LT(residual/previous,.13); }
    previous=residual;
    for(unsigned k=2;k<8;++k)EXPECT_NEAR(actual.normalized_rate[k],0,2e-12);
  }
}

TEST(T3KinematicsCheck, MalformedCurrentFloorAndLateArithmeticFailurePreserveBytesAndRetry) {
  const auto r=test::Triangle(); const auto reference=test::MakeReference(r);
  auto good=test::Interval(r); Affine(good);
  Kinematics value;
  ASSERT_EQ(native::EvaluatePrescribed(reference,good,value),Status::kSuccess);
  const auto before=test::Bytes(value);
  const auto reference_before=test::Bytes(reference);
  EXPECT_EQ(native::EvaluatePrescribed(native::Reference{},good,value),Status::kInvalidInput);
  for(unsigned fault=0;fault<8;++fault) {
    auto bad=good;
    if(fault==0)bad.dt=0;
    if(fault==1)bad.sample_index=0;
    if(fault==2)bad.base_time=std::numeric_limits<double>::max();
    if(fault==3)bad.position[2].z=std::numeric_limits<double>::quiet_NaN();
    if(fault==4)bad.angular_velocity[2].y=std::numeric_limits<double>::infinity();
    if(fault==5)bad.position[2]=bad.position[1];
    if(fault==6)bad.dt=std::numeric_limits<double>::denorm_min();
    if(fault==7)bad.position[2].x=2*native::kMaximumCoordinate;
    EXPECT_NE(native::EvaluatePrescribed(reference,bad,value),Status::kSuccess)<<fault;
    EXPECT_EQ(test::Bytes(value),before);
  }
  const double floor_margin=32*(1./1e15);
  for(double height:{std::nextafter(floor_margin,0.),floor_margin}) {
    auto bad=test::Interval(r); bad.position={{{0,0,0},{4e-9,0,0},{2e-9,height,0}}};
    EXPECT_EQ(native::EvaluatePrescribed(reference,bad,value),Status::kUnsupportedGeometry);
    EXPECT_EQ(test::Bytes(value),before);
  }
  auto boundary=test::Interval(r); boundary.position={{{0,0,0},{4e-9,0,0},{2e-9,std::nextafter(floor_margin,1.),0}}};
  Kinematics just_inside;
  ASSERT_EQ(native::EvaluatePrescribed(reference,boundary,just_inside),Status::kSuccess);
  // Admission at the boundary is distinct from broad high-condition accuracy.
  EXPECT_GT(just_inside.local_position[2].y,floor_margin);
  auto overflow=good; overflow.velocity[2].z=std::numeric_limits<double>::max();
  EXPECT_EQ(native::EvaluatePrescribed(reference,overflow,value),Status::kNonfiniteResult);
  EXPECT_EQ(test::Bytes(value),before); EXPECT_EQ(test::Bytes(reference),reference_before);
  Kinematics retry;
  ASSERT_EQ(native::EvaluatePrescribed(reference,good,retry),Status::kSuccess);
  EXPECT_EQ(retry.raw_rate,value.raw_rate); EXPECT_EQ(retry.normalized_rate,value.normalized_rate);
  EXPECT_EQ(retry.corrected_velocity_difference,value.corrected_velocity_difference);
}
}  // namespace
