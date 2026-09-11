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
