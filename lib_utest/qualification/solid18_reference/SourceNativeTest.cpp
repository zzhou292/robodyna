// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "SourceFixture.h"
#include <gtest/gtest.h>
#include <limits>

namespace solid18_test {
TEST(Solid18ReferenceSource, All908OriginalCellsNativeGeometryMassAndWorkingUnits) {
  unsigned reversed = 0;
  unsigned nonuniform_points = 0;
  for (unsigned i = 0; i < SourceCount; ++i) {
    const auto input = Source(i);
    SCOPED_TRACE(input.source_element_id);
    s::Reference actual;
    const auto expected = Native(input);
    ASSERT_EQ(expected.status,0);
    ASSERT_EQ(s::InitializeReference(input,actual),s::Status::Success);
    ASSERT_TRUE(Agree(Values(actual),expected.values));
    for (unsigned n = 0; n < 8; ++n) {
      ASSERT_EQ(actual.input().source_node_id[n],input.source_node_id[n]);
      ASSERT_EQ(actual.source_slot(n),expected.source_slot[n]);
      ASSERT_GT(actual.mass().source_nodal_mass_kg[n],0);
      ASSERT_GT(actual.geometry().point[n].initial_volume_m3,0);
    }
    reversed += actual.source_slot(0) != 0;
    for (unsigned n = 1; n < 8; ++n)
      ASSERT_EQ(actual.mass().source_nodal_mass_kg[n],actual.mass().source_nodal_mass_kg[0]);
    double minimum = actual.geometry().point[0].initial_volume_m3;
    double maximum = minimum;
    for (const auto& point : actual.geometry().point) {
      minimum = std::min(minimum,point.initial_volume_m3);
      maximum = std::max(maximum,point.initial_volume_m3);
    }
    nonuniform_points += maximum-minimum > actual.geometry().integrated_volume_m3*1e-5;
    // The original coordinate bits enter native mm/t arithmetic directly.
    // Only once-converted outputs use a geometric conditioning allowance;
    // admission and original-slot permutation remain exact in both runs.
    const auto working_input = Source(i,true);
    for (unsigned n = 0; n < 8; ++n) {
      ASSERT_DOUBLE_EQ(working_input.position_m[n].x*.001,input.position_m[n].x);
      ASSERT_DOUBLE_EQ(working_input.position_m[n].y*.001,input.position_m[n].y);
      ASSERT_DOUBLE_EQ(working_input.position_m[n].z*.001,input.position_m[n].z);
    }
    const auto working = Native(working_input);
    ASSERT_EQ(working.status,0);
    ASSERT_EQ(working.source_slot,expected.source_slot);
    double coordinate_scale = 0;
    for (const auto& x : input.position_m)
      coordinate_scale = std::max({coordinate_scale,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
    const double conditioning = coordinate_scale/actual.geometry().characteristic_length_m;
    ASSERT_TRUE(AgreeWorkingUnits(Values(actual),NativeWorkingToSI(working.values),conditioning));
  }
  EXPECT_GT(nonuniform_points,0u);
  RecordProperty("source_cells",SourceCount);
  RecordProperty("native_orientation_reversals",reversed);
  RecordProperty("nonuniform_points_mass_cells",nonuniform_points);
}
TEST(Solid18ReferenceSource, WorkingUnitGeometryConditioningRejectsPhysicalChange) {
  unsigned selected = SourceCount;
  for (unsigned i = 0; i < SourceCount; ++i) {
    if (original::solids[i].id == 2200907) selected = i;
  }
  ASSERT_LT(selected,SourceCount);
  const auto input = Source(selected);
  s::Reference reference;
  ASSERT_EQ(s::InitializeReference(input,reference),s::Status::Success);
  const auto native_si = Native(input);
  const auto native_working = Native(Source(selected,true));
  ASSERT_EQ(native_si.status,0);
  ASSERT_EQ(native_working.status,0);
  ASSERT_EQ(native_si.source_slot,native_working.source_slot);
  const auto actual = Values(reference);
  auto expected = NativeWorkingToSI(native_working.values);
  ASSERT_TRUE(Agree(actual,native_si.values));
  double coordinate_scale = 0;
  for (const auto& x : input.position_m)
    coordinate_scale = std::max({coordinate_scale,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
  const double conditioning = coordinate_scale/reference.geometry().characteristic_length_m;
  ASSERT_TRUE(AgreeWorkingUnits(actual,expected,conditioning));
  // A physical change to the first point Jacobian is larger than the declared
  // coordinate-conversion allowance, even in a cancelling local component.
  expected[54] += 1e-5;
  EXPECT_FALSE(AgreeWorkingUnits(actual,expected,conditioning));
  expected = NativeWorkingToSI(native_working.values);
  expected[147] *= 1.000001;
  EXPECT_FALSE(AgreeWorkingUnits(actual,expected,conditioning));
}
}  // namespace solid18_test
