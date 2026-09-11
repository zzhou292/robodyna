// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SourceFixture.h"
#include "NativeOracle.h"
#include "Agreement.h"
#include <sstream>
#include <iomanip>
using namespace law90_reference_test;
TEST(Law90Solid18Source, AllOriginalReferencePijMassAndWorkingUnits) {
  double minimum_volume=std::numeric_limits<double>::max();
  for(unsigned row=0;row<fixture::element_count;++row) {
    const auto input=Original(row);
    SCOPED_TRACE(input.source_element_id);
    t::Reference reference;
    ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
    const auto native=ReferenceOracle(input);
    ASSERT_EQ(native.status,0);
    const auto actual=ReferenceValues(reference);
    ASSERT_TRUE(ReferenceAgreement(actual,native.values));
    for(unsigned n=0;n<8;++n) ASSERT_EQ(reference.source_slot(n),native.source_slot[n]);
    const auto working=ReferenceOracle(Original(row,true));
    ASSERT_EQ(working.status,0);
    for(unsigned n=0;n<8;++n) ASSERT_EQ(working.source_slot[n],native.source_slot[n]);
    double coordinate=0;
    for(const auto& x:input.position_m)
      coordinate=std::max({coordinate,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
    const double conditioning=coordinate/reference.geometry().characteristic_length_m;
    ASSERT_TRUE(ReferenceAgreement(actual,WorkingReferenceToSI(working.values),conditioning));
    for(const auto& p:reference.geometry().point)
      minimum_volume=std::min(minimum_volume,p.initial_volume_m3);
  }
  EXPECT_GT(minimum_volume,1e-15); // Actual element-domain floor evidence.
  RecordProperty("original_cells",fixture::element_count);
  RecordProperty("original_nodes",fixture::node_count);
  std::ostringstream volume_text;
  volume_text<<std::setprecision(17)<<minimum_volume;
  RecordProperty("minimum_point_volume_m3",volume_text.str());
}
TEST(Law90Solid18Source, AllOriginalCurrentGeometryTotalTensorAndRates) {
  for(unsigned row=0;row<fixture::element_count;++row) {
    const auto input=Original(row);
    SCOPED_TRACE(input.source_element_id);
    t::Reference reference;
    ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
    for(const auto current:{Current(input),OriginalCurrent(input)}) {
      const auto expected=CurrentOracle(input,current);
      ASSERT_EQ(expected.status,0);
      t::Kinematics actual;
      ASSERT_EQ(t::EvaluateKinematics90(reference,current,actual),s::Status::Success);
      ASSERT_TRUE(CurrentAgreement(CurrentValues(actual),expected.values));
    }
  }
  RecordProperty("original_current_packets",2*fixture::element_count);
}
TEST(Law90Solid18Source, WorkingModeCancellationRejectsPerturbations) {
  unsigned row=fixture::element_count;
  for(unsigned n=0;n<fixture::element_count;++n)
    if(fixture::solid_records_u64[n][0]==2191748)row=n;
  ASSERT_LT(row,fixture::element_count);
  const auto input=Original(row);
  t::Reference reference;
  ASSERT_EQ(t::InitializeReference90(input,reference),s::Status::Success);
  const auto actual=ReferenceValues(reference);
  const auto native_si=ReferenceOracle(input);
  const auto working=ReferenceOracle(Original(row,true));
  ASSERT_EQ(native_si.status,0);ASSERT_EQ(working.status,0);
  for(unsigned n=0;n<8;++n) ASSERT_EQ(native_si.source_slot[n],working.source_slot[n]);
  ASSERT_TRUE(ReferenceAgreement(actual,native_si.values));
  auto expected=WorkingReferenceToSI(working.values);
  double coordinate=0;
  for(const auto& x:input.position_m)
    coordinate=std::max({coordinate,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
  const double conditioning=coordinate/reference.geometry().characteristic_length_m;
  ASSERT_TRUE(ReferenceAgreement(actual,expected,conditioning));
  const auto record=[&](const char* name,long double value) {
    std::ostringstream stream;stream<<std::setprecision(21)<<value;RecordProperty(name,stream.str());
  };
  record("actual_mode51_m",actual[51]);record("native_working_mode51_m",expected[51]);
  record("delta_mode51_m",actual[51]-expected[51]);
  record("forward_bound_mode51_m",HigherModeWorkingBound(actual,expected,3,0));
  for(unsigned mode=0;mode<4;++mode)for(unsigned axis=0;axis<3;++axis) {
    const unsigned i=42+3*mode+axis;
    auto bad=expected;
    const double bound=static_cast<double>(HigherModeWorkingBound(actual,expected,mode,axis));
    bad[i]+=std::max(1e-12,64*bound);
    EXPECT_FALSE(ReferenceAgreement(actual,bad,conditioning))<<i;
    EXPECT_FALSE(ReferenceAgreement(actual,bad))<<i;
  }
  auto bad=expected;bad[147]*=1.000001;
  EXPECT_FALSE(ReferenceAgreement(actual,bad,conditioning));
}
