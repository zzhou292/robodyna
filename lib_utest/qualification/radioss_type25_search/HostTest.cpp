#include "Fixture.h"
#include "lib_src/collision/radioss_type25/search/Layout.h"
#include <cmath>
#include <limits>
namespace type25_search_test {
TEST(Type25SearchValues, CompleteNativeBudgetBoundariesAndTwoMotionFactors) {
  Fixture f;const auto reference=f.positions;
  for(double v:{0.,.25,1.,2.,5.,8.})for(double gap:{-.5,0.,.25}) {
    f.velocities[0]=v;const auto c=f.Current();
    auto old=c;old.positions.data=reference.data();
    auto extrema=NativeExtrema(f.source,c,old);extrema.maximum_gap_change=gap;
    for(bool forced:{false,true}) {
      s::Budget actual;ASSERT_EQ(s::EvaluateBudget(extrema,1.,1.,forced,actual),s::Status::Ok);
      Same(actual,NativeBudget(extrema,1.,1.,forced));
    }
  }
}
TEST(Type25SearchValues, ExactAndAdjacentRefreshAndVelocityThresholds) {
  s::Extrema e;e.secondary_uses=e.main_uses=1;
  e.secondary_displacement={s::Vector{},s::Vector{}};e.main_displacement=e.secondary_displacement;
  e.secondary_velocity={s::Vector{1,0,0},s::Vector{1,0,0}};
  e.main_velocity={s::Vector{},s::Vector{}};e.maximum_gap_change=0;
  for(double margin:{0.,1.,2.,5.})for(double dt:{0.,margin/s::MotionFactor,
      std::nextafter(margin/s::MotionFactor,0.),std::nextafter(margin/s::MotionFactor,INFINITY),
      2*margin/s::MotionFactor,5*margin/s::MotionFactor,6*margin/s::MotionFactor}) {
    s::Budget actual;ASSERT_EQ(s::EvaluateBudget(e,margin,dt,false,actual),s::Status::Ok);
    Same(actual,NativeBudget(e,margin,dt,false));
  }
}
TEST(Type25SearchValues, EmptyInvalidAndOverflowPreservePreviousValue) {
  s::Budget before;before.distance=123;s::Budget value=before;s::Extrema empty;
  EXPECT_EQ(s::EvaluateBudget(empty,1,1,false,value),s::Status::UnsupportedLifecycle);
  EXPECT_EQ(value.distance,123);
  EXPECT_EQ(s::EvaluateBudget(empty,-1,1,false,value),s::Status::InvalidInput);
  EXPECT_EQ(value.distance,123);
  Fixture f;auto c=f.Current();auto e=NativeExtrema(f.source,c,c);
  e.secondary_velocity.maximum.x=std::numeric_limits<double>::max();
  EXPECT_EQ(s::EvaluateBudget(e,1,1,false,value),s::Status::NonfiniteResult);
  EXPECT_EQ(value.distance,123);
}
TEST(Type25SearchValues, ForecastCapsLayoutsSourceValidationAndNoDeviceInitialization) {
  for(bool compact:{false,true}) {
    Fixture f(compact);s::Forecast result;
    ASSERT_EQ(s::Maintenance::Preflight(f.source,{},result),s::Status::Ok);
    EXPECT_EQ(result.compact_reference,compact);
    s::Limits limits;limits.max_device_bytes=result.device_bytes;limits.max_host_bytes=result.startup_host_bytes;
    s::Forecast exact;ASSERT_EQ(s::Maintenance::Preflight(f.source,limits,exact),s::Status::Ok);
    EXPECT_EQ(exact.device_bytes,result.device_bytes);
    --limits.max_device_bytes;exact.device_bytes=73;
    EXPECT_EQ(s::Maintenance::Preflight(f.source,limits,exact),s::Status::ResourceLimit);EXPECT_EQ(exact.device_bytes,73u);
    f.source.processors=2;EXPECT_EQ(s::Maintenance::Preflight(f.source,{},exact),s::Status::UnsupportedProfile);
    f.source.processors=1;f.secondary.back()=f.source.physical_nodes;
    EXPECT_EQ(s::Maintenance::Preflight(f.source,{},exact),s::Status::InvalidInput);
  }
}
TEST(Type25SearchValues, SourceSignedZeroAndMainOneDimensionalRolesUseNativeMasks) {
  Fixture f;f.stiffness[0]=-0.;f.velocities[3*4]=3;
  const auto e=NativeExtrema(f.source,f.Current(),f.Current());
  EXPECT_EQ(e.secondary_uses,2u);EXPECT_EQ(e.main_uses,3u);
  s::Budget current;ASSERT_EQ(s::EvaluateBudget(e,4,.25,false,current),s::Status::Ok);
  Same(current,NativeBudget(e,4,.25,false));
}
TEST(Type25SearchValues, NativeNegativeStiffnessIsObservedBeforeItsPostQueryClamp) {
  Fixture fixture;
  fixture.stiffness[0] = -2.;
  fixture.velocities[0] = 19.;
  const auto input = fixture.Current();
  std::vector<double> normalized;
  const auto extrema = NativeExtrema(fixture.source, input, input, &normalized);
  ASSERT_EQ(normalized.size(), fixture.stiffness.size());
  EXPECT_EQ(normalized[0], 0.);
  EXPECT_EQ(normalized[1], 2.);
  EXPECT_EQ(fixture.stiffness[0], -2.);
  EXPECT_EQ(extrema.secondary_velocity.maximum.x, 19.);
  EXPECT_EQ(extrema.secondary_uses, 3u);
}
TEST(Type25SearchValues, MalformedHostMapRangesRejectBeforeAnyDereference) {
  Fixture fixture;
  const auto original = fixture.source;
  s::Forecast output;
  output.device_bytes = 97;
  const auto address = reinterpret_cast<std::uintptr_t>(original.secondary_nodes);
  fixture.source.secondary_nodes = reinterpret_cast<const std::uint32_t*>(address + 1);
  EXPECT_EQ(s::Maintenance::Preflight(fixture.source, {}, output), s::Status::InvalidInput);
  EXPECT_EQ(output.device_bytes, 97u);
  fixture.source = original;
  fixture.source.main_nodes = reinterpret_cast<const std::uint32_t*>(
      UINTPTR_MAX - alignof(std::uint32_t) + 1);
  EXPECT_EQ(s::Maintenance::Preflight(fixture.source, {}, output), s::Status::InvalidInput);
  EXPECT_EQ(output.device_bytes, 97u);
  fixture.source = original;
  fixture.source.main_1d_nodes = nullptr;
  EXPECT_EQ(s::Maintenance::Preflight(fixture.source, {}, output), s::Status::InvalidInput);
  EXPECT_EQ(output.device_bytes, 97u);
}
} // namespace type25_search_test
