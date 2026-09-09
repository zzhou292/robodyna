#include "T3PortFixture.h"
namespace t3_port_test {
namespace {
void Affine(port::PrescribedInterval& in) {
  for(unsigned n=0;n<3;++n) { const auto x=in.position[n];
    in.velocity[n]={.2+1.1*x.x-.4*x.y,-.1+.3*x.x+.7*x.y,.4-.6*x.x+.8*x.y};
    in.angular_velocity[n]={-.2+.9*x.x+.5*x.y,.6-.3*x.x+.4*x.y,2+.2*x.x}; }
}
}
TEST(T3KinematicsPort, AllEighteenSignedColumnsMatchNativeAndIndependentAngularInterpolation) {
  for(double scale:{1.,.02,1e-5}) {
    const auto r=Triangle(scale); const auto reference=Reference(r);
    for(unsigned column=0;column<18;++column) for(double sign:{1.,-1.}) {
      auto in=Interval(r,.0025); const unsigned n=column/6,axis=column%3;
      Set(column%6<3?in.velocity[n]:in.angular_velocity[n],axis,sign*.125*(column%6<3?scale:1));
      ASSERT_NO_FATAL_FAILURE(CheckRates(reference,in));
    }
    auto in=Interval(r); Affine(in); ASSERT_NO_FATAL_FAILURE(CheckRates(reference,in));
    port::Kinematics full,half; ASSERT_EQ(port::EvaluatePrescribed(reference,in,full),port::Status::kSuccess);
    in.dt*=.5; ASSERT_EQ(port::EvaluatePrescribed(reference,in,half),port::Status::kSuccess);
    EXPECT_NEAR(full.normalized_rate[0]-half.normalized_rate[0],-.0025*(.6*.6+.3*.3),2e-12);
    EXPECT_NEAR(full.normalized_rate[1]-half.normalized_rate[1],-.0025*(.8*.8+.4*.4),2e-12);
  }
}
TEST(T3KinematicsPort, CurrentAreaRotationsNativeOrderingAndEdgeOnKeepReferenceImmutable) {
  const auto r=Triangle(.02); const auto reference=Reference(r); const auto saved=Bytes(reference);
  auto base=Interval(r); Affine(base);
  port::Kinematics original; ASSERT_EQ(port::EvaluatePrescribed(reference,base,original),port::Status::kSuccess);
  for(unsigned variant=0;variant<3;++variant) {
    auto in=base;
    for(unsigned n=0;n<3;++n) {
      auto transform=[&](port::Vec3 v) { return variant?kt::Rotate(kt::Rotation(),v):port::Vec3{v.y,v.z,v.x}; };
      in.position[n]=transform(in.position[n]); in.velocity[n]=transform(in.velocity[n]);
      in.angular_velocity[n]=transform(in.angular_velocity[n]);
      in.position[n].x+=.75; in.position[n].y-=1.25;
    }
    if(variant==2) in.position[2].z+=.013; // Current geometry differs from immutable startup.
    in.base_time=.125; in.sample_index=(1ULL<<53)+17;
    for(unsigned order=0;order<6;++order) {
      auto permuted=in;
      for(unsigned n=0;n<3;++n) { const unsigned i=(order%3+(order>=3?3-n:n))%3;
        permuted.position[n]=in.position[i]; permuted.velocity[n]=in.velocity[i];
        permuted.angular_velocity[n]=in.angular_velocity[i]; }
      ASSERT_NO_FATAL_FAILURE(CheckRates(reference,permuted));
    }
    if(variant<2) {
      port::Kinematics rotated; ASSERT_EQ(port::EvaluatePrescribed(reference,in,rotated),port::Status::kSuccess);
      for(unsigned c=0;c<8;++c) EXPECT_NEAR(rotated.raw_rate[c],original.raw_rate[c],2e-11*.05);
    }
  }
  // Safely admitted tiny translated current triangle, without replacing startup.
  auto tiny=Interval(Triangle(1e-5));
  for(auto& x:tiny.position) { x.x+=5; x.y-=3; x.z+=.125; }
  ASSERT_NO_FATAL_FAILURE(CheckRates(reference,tiny)); EXPECT_EQ(Bytes(reference),saved);
}
TEST(T3KinematicsPort, TranslationAndFiniteSpinRetainNativeQuarterStepCharacterization) {
  const auto r=Triangle(); const auto reference=Reference(r);
  auto translated=Interval(r); for(auto& v:translated.velocity) v={.25,-.375,.125};
  port::Kinematics moved; ASSERT_EQ(port::EvaluatePrescribed(reference,translated,moved),port::Status::kSuccess);
  for(double rate:moved.raw_rate) EXPECT_NEAR(rate,0,2e-12);
  double previous=0;
  for(double h:{.04,.02,.01}) {
    auto in=Interval(r,h);
    for(unsigned n=0;n<3;++n) { const auto x=r.position[n];
      in.position[n]={std::cos(h)*x.x-std::sin(h)*x.y,std::sin(h)*x.x+std::cos(h)*x.y,0};
      const port::Vec3 midpoint{std::cos(h/2)*x.x-std::sin(h/2)*x.y,std::sin(h/2)*x.x+std::cos(h/2)*x.y,0};
      in.velocity[n]={-midpoint.y,midpoint.x,0}; in.angular_velocity[n]={0,0,1}; }
    ASSERT_NO_FATAL_FAILURE(CheckRates(reference,in));
    port::Kinematics result; ASSERT_EQ(port::EvaluatePrescribed(reference,in,result),port::Status::kSuccess);
    const long double a=static_cast<long double>(h)/2,expected=std::sin(a)-a*std::cos(a)*std::cos(a);
    native::test::Near(result.normalized_rate[0],expected,1); native::test::Near(result.normalized_rate[1],expected,1);
    const double residual=std::max(std::abs(result.normalized_rate[0]),std::abs(result.normalized_rate[1]));
    EXPECT_GT(residual,0);
    if(previous>0) { EXPECT_GT(residual/previous,.12); EXPECT_LT(residual/previous,.13); }
    previous=residual;
  }
}
TEST(T3KinematicsPort, MalformedAndScaleAwareFloorBandsPreserveEveryCallerField) {
  const auto r=Triangle(); const auto reference=Reference(r); auto good=Interval(r); Affine(good);
  port::Kinematics output; ASSERT_EQ(port::EvaluatePrescribed(reference,good,output),port::Status::kSuccess);
  const auto saved=Bytes(output); const auto saved_reference=Bytes(reference);
  EXPECT_EQ(port::EvaluatePrescribed({},good,output),port::Status::kInvalidReference); EXPECT_EQ(Bytes(output),saved);
  for(unsigned fault=0;fault<8;++fault) { auto bad=good;
    if(fault==0)bad.dt=0;
    if(fault==1)bad.sample_index=0;
    if(fault==2)bad.base_time=std::numeric_limits<double>::max();
    if(fault==3)bad.position[2].z=std::numeric_limits<double>::quiet_NaN();
    if(fault==4)bad.angular_velocity[2].y=std::numeric_limits<double>::infinity();
    if(fault==5)bad.position[2]=bad.position[1];
    if(fault==6)bad.dt=std::numeric_limits<double>::denorm_min();
    if(fault==7)bad.position[2].x=2*port::detail::MaximumCoordinate;
    EXPECT_NE(port::EvaluatePrescribed(reference,bad,output),port::Status::kSuccess); EXPECT_EQ(Bytes(output),saved);
  }
  const double cutoff=32*port::detail::Em15,band=port::detail::GuardBand*4e-9;
  for(double height:{cutoff*.5,cutoff,cutoff+.25*band}) {
    auto bad=Interval(r); bad.position[0]={0,0,0}; bad.position[1]={4e-9,0,0}; bad.position[2]={2e-9,height,0};
    EXPECT_EQ(port::EvaluatePrescribed(reference,bad,output),port::Status::kUnsupportedGeometry); EXPECT_EQ(Bytes(output),saved);
  }
  auto inside=Interval(r); inside.position[0]={0,0,0}; inside.position[1]={4e-9,0,0}; inside.position[2]={2e-9,2*cutoff,0};
  port::Kinematics admitted; ASSERT_EQ(port::EvaluatePrescribed(reference,inside,admitted),port::Status::kSuccess);
  EXPECT_GT(admitted.local_position[2].y,cutoff+band);
  EXPECT_EQ(Bytes(reference),saved_reference);
}
TEST(T3KinematicsPort, LateFiniteOverflowDoesNotPublishAndCleanRetryMatchesAllFields) {
  const auto r=Triangle(.02); const auto reference=Reference(r); auto in=Interval(r); Affine(in);
  port::Kinematics result; ASSERT_EQ(port::EvaluatePrescribed(reference,in,result),port::Status::kSuccess);
  const auto clean=result;
  const auto bytes=Bytes(result); const auto reference_bytes=Bytes(reference);
  auto bad=in; bad.velocity[2].z=std::numeric_limits<double>::max();
  EXPECT_EQ(port::EvaluatePrescribed(reference,bad,result),port::Status::kNonfiniteResult);
  EXPECT_EQ(Bytes(result),bytes); EXPECT_EQ(Bytes(reference),reference_bytes);
  auto malformed=reference; malformed.area=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(port::EvaluatePrescribed(malformed,in,result),port::Status::kInvalidReference); EXPECT_EQ(Bytes(result),bytes);
  ASSERT_EQ(port::EvaluatePrescribed(reference,in,result),port::Status::kSuccess);
  ASSERT_NO_FATAL_FAILURE(CheckRates(reference,in));
  ExactRates(result,clean);
}
} // namespace t3_port_test
