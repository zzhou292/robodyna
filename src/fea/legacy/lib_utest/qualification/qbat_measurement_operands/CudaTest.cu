// SPDX-License-Identifier: MIT
#include "DeviceFixture.cuh"

namespace qbat_measurement_test {
TEST(QbatMeasurementCuda,CompleteCallerAcrossBlocksKeepsBothSlabsAndExactDiagnostics) {
  for (std::size_t parents : {1u,127u,128u,129u,513u}) {
    DeviceFixture actual(parents),expected(parents);
    ASSERT_FALSE(HasFailure());
    for (unsigned epoch=0; epoch<4; ++epoch) {
      SCOPED_TRACE(::testing::Message()<<parents<<":"<<epoch);
      actual.Upload(epoch);
      expected.Upload(epoch);
      std::vector<unsigned char> accepted(parents*sizeof(q::BatchResult));
      std::memcpy(accepted.data(),actual.storage->slab[0].element,accepted.size());
      actual.Evaluate(false,epoch,true);
      expected.Evaluate(true,epoch,true);
      ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
      Same(actual,expected);
      EXPECT_EQ(std::memcmp(accepted.data(),actual.storage->slab[0].element,accepted.size()),0);
    }
  }
}
TEST(QbatMeasurementCuda,CompetingFailuresAndOverflowRetainOriginalPrefixThenRetry) {
  DeviceFixture actual(129),expected(129);
  ASSERT_FALSE(HasFailure());
  for (unsigned fault=0; fault<=10; ++fault) {
    SCOPED_TRACE(fault);
    for (auto* packet : {&actual,&expected}) {
      packet->source.Reset();
      Fault(packet->source,fault);
      packet->Restore();
    }
    actual.Evaluate(false);
    expected.Evaluate(true);
    Same(actual,expected);
    if (fault) EXPECT_NE(actual.storage->control.status,q::BatchStatus::Success);
    actual.Upload();
    expected.Upload();
    actual.Evaluate(false);
    expected.Evaluate(true);
    ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
    Same(actual,expected);
  }
}
TEST(QbatMeasurementCuda,RemovalMasksAndUnmappedFallbackRetainEveryDiagnosticBit) {
  DeviceFixture actual,expected;
  ASSERT_FALSE(HasFailure());
  for (unsigned mask=0; mask<16; ++mask) {
    actual.Upload();
    expected.Upload();
    for (auto* packet : {&actual,&expected}) {
      for (unsigned parent=0; parent<4; ++parent) {
        if (mask&(1u<<parent)) g::Remove(packet->storage->slab[1].element[parent]);
      }
      auto* rows=packet->storage->slab[1].element;
      rows[0].history.internal_work_j[0]=0x1p54;
      rows[1].history.internal_work_j[0]=1;
      rows[2].history.internal_work_j[0]=-0x1p54;
      rows[3].history.internal_work_j[0]=.25;
    }
    actual.Evaluate(false);
    expected.Evaluate(true);
    ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
    Same(actual,expected);
  }
  actual.Upload();
  expected.Upload();
  actual.storage->assembly.measurement=nullptr;
  actual.Evaluate(false,2,true,false);
  expected.Evaluate(true,2,true,false);
  ASSERT_EQ(actual.storage->control.status,q::BatchStatus::Success);
  Same(actual,expected);
}
} // namespace qbat_measurement_test
