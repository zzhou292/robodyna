#include "lib_src/collision/Q4IntegralMeasure.h"
#include "lib_utest/q4_prescribed_contact_fixture.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
namespace sc=tlfea::contact;
sc::Q4IntegrationResult Integral() {
  sc::Q4IntegrationResult result; result.valid=true; result.leaf_count=1; result.visited=1;
  result.feature_id=73; result.parent_element_id=42; result.base_epoch=9; result.attempt=7;
  for (unsigned n=0;n<4;++n) {
    result.force[n]={.03125,.03125,.03125,0}; result.nodal.nodes[n]=n;
    result.nodal.forces[n]={-.03125,0,0};
  }
  result.resultant={.125,.125,.125,0}; result.potential={.00048828125,.00048828125,.00048828125,0};
  result.active_area={.125,.125}; return result;
}
void Encloses(sc::Q4CertifiedIntegral result,long double lower,long double upper) {
  EXPECT_LE(static_cast<long double>(result.lower),lower);
  EXPECT_GE(static_cast<long double>(result.upper),upper);
  EXPECT_GE(static_cast<long double>(result.error),std::abs(static_cast<long double>(result.value)-lower));
  EXPECT_GE(static_cast<long double>(result.error),std::abs(static_cast<long double>(result.value)-upper));
}

TEST(Q4IntegralMeasure, ExactNonunitMeasurePreservesNominalIntegralAndMetadata) {
  auto result=Integral();
  ASSERT_TRUE(sc::ExpandQ4IntegralMeasure(.125,{.125,.125},&result));
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(result.force[n].value,.03125); EXPECT_EQ(result.force[n].lower,.03125);
    EXPECT_EQ(result.force[n].upper,.03125); EXPECT_EQ(result.force[n].error,0);
    EXPECT_EQ(result.nodal.nodes[n],n); EXPECT_EQ(result.nodal.forces[n].x,-.03125);
  }
  EXPECT_EQ(result.potential.value,.00048828125); EXPECT_EQ(result.potential.error,0);
  EXPECT_EQ(result.active_area.lower,.125); EXPECT_EQ(result.active_area.upper,.125);
  EXPECT_EQ(result.feature_id,73u); EXPECT_EQ(result.parent_element_id,42u);
  EXPECT_EQ(result.base_epoch,9u); EXPECT_EQ(result.attempt,7u); EXPECT_TRUE(result.valid);
}

TEST(Q4IntegralMeasure, CoordinateAreaUncertaintyExpandsTruthWithoutRescalingEstimatedForces) {
  auto result=Integral();
  const sc::Q4IntegralInterval area{std::nextafter(.125,0),std::nextafter(.125,1)};
  ASSERT_TRUE(sc::ExpandQ4IntegralMeasure(.125,area,&result));
  const long double lower=static_cast<long double>(area.lower)/.125L,upper=static_cast<long double>(area.upper)/.125L;
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(result.force[n].value,.03125); EXPECT_EQ(result.nodal.forces[n].x,-.03125);
    Encloses(result.force[n],.03125L*lower,.03125L*upper);
  }
  Encloses(result.resultant,.125L*lower,.125L*upper);
  Encloses(result.potential,.00048828125L*lower,.00048828125L*upper);
  EXPECT_LE(static_cast<long double>(result.active_area.lower),static_cast<long double>(area.lower));
  EXPECT_GE(static_cast<long double>(result.active_area.upper),static_cast<long double>(area.upper));
}

TEST(Q4IntegralMeasure, MalformedAndLateExpansionFailurePreserveCompleteOutputForRetry) {
  for (unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    auto result=Integral(); double defined=.125; sc::Q4IntegralInterval area{.125,.125};
    if (fault == 0) defined=0;
    if (fault == 1) area.upper=.0625;
    if (fault == 2) area.lower=0;
    if (fault == 3) result.valid=false;
    if (fault == 4) result.active_area={-1,1};  // Fails after all six force/potential certificates are staged.
    if (fault == 5) result.potential.value=std::numeric_limits<double>::quiet_NaN();
    const auto before=q4_prescribed_test::Bytes(result);
    EXPECT_FALSE(sc::ExpandQ4IntegralMeasure(defined,area,&result));
    EXPECT_EQ(q4_prescribed_test::Bytes(result),before);
    result=Integral(); EXPECT_TRUE(sc::ExpandQ4IntegralMeasure(.125,{.125,.125},&result));
  }
  EXPECT_FALSE(sc::ExpandQ4IntegralMeasure(.125,{.125,.125},nullptr));
}
}  // namespace
