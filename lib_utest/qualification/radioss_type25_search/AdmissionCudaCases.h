#pragma once
#include "CudaFixture.h"
#include <new>
namespace type25_search_test {
TEST_F(Type25SearchCuda, NegativeStiffnessPreservesPublicationAndCopiesFirstFailureRow) {
  DeviceFixture device;
  device.Initialize();
  device.Reference();
  auto token = device.Stage();
  const auto old_stiffness = device.fixture.stiffness;
  const auto old_gaps = device.fixture.gaps;
  device.fixture.stiffness[0] = -1.;
  device.fixture.gaps[1] = -1.; // A later invalid row must not replace row zero.
  device.Upload();
  s::Report result;
  result.budget.distance = 97.;
  EXPECT_EQ(device.owner.Evaluate(device.current, 1., false, result),
      s::Status::UnsupportedLifecycle);
  EXPECT_EQ(result.budget.distance, 97.);
  auto failure = device.owner.last_failure();
  EXPECT_EQ(failure.status, s::Status::UnsupportedLifecycle);
  EXPECT_TRUE(failure.query_available);
  EXPECT_TRUE(failure.row_available);
  EXPECT_EQ(failure.input_row, 0u);
  EXPECT_EQ(failure.stamp.epoch, device.current.stamp.epoch);
  EXPECT_EQ(failure.stamp.attempt, device.current.stamp.attempt);
  EXPECT_TRUE(s::detail::Same(failure.stamp.source, device.current.stamp.source));
  EXPECT_EQ(device.owner.StageReference(device.current, token),
      s::Status::UnsupportedLifecycle);
  EXPECT_EQ(device.owner.reference_generation(), 1u);
  EXPECT_EQ(token.generation(), 2u); // Failed staging did not write caller output.
  device.owner.DiscardReference();
  EXPECT_EQ(device.owner.last_failure().input_row, 0u);
  EXPECT_EQ(device.owner.PublishReference(token), s::Status::StaleReference);
  EXPECT_FALSE(device.owner.last_failure().query_available);
  device.fixture.stiffness = old_stiffness;
  device.fixture.gaps = old_gaps;
  device.Upload();
  device.Compare(device.fixture.Current());
  EXPECT_EQ(device.owner.last_failure().status, s::Status::Ok);
  EXPECT_FALSE(device.owner.last_failure().row_available);
}
TEST_F(Type25SearchCuda, ManagedReportAliasingBorrowedStiffnessIsRejectedBeforeOverwrite) {
  DeviceFixture device;
  device.fixture.secondary.resize(1);
  device.fixture.stiffness.resize(1);
  device.fixture.Bind();
  device.Upload();
  device.Initialize();
  device.Reference();
  struct ManagedReport {
    s::Report* value = nullptr;
    ManagedReport() {
      Cuda(cudaMallocManaged(reinterpret_cast<void**>(&value), sizeof(s::Report)));
      new (value) s::Report();
    }
    ~ManagedReport() {
      if (value) {
        value->~Report();
        cudaFree(value);
      }
    }
  } managed;
  managed.value->budget.distance = 2.;
  auto aliased = device.current;
  // A real live managed double subobject supplies the one consumed stiffness.
  // No fabricated object or reinterpretation of a token's private storage.
  aliased.secondary_stiffness = &managed.value->budget.distance;
  EXPECT_EQ(device.owner.Evaluate(aliased, 1., false, *managed.value), s::Status::InvalidInput);
  EXPECT_EQ(managed.value->budget.distance, 2.);
  EXPECT_EQ(device.owner.reference_generation(), 1u);
  EXPECT_EQ(device.owner.last_failure().status, s::Status::InvalidInput);
  EXPECT_TRUE(device.owner.last_failure().query_available);
  EXPECT_FALSE(device.owner.last_failure().row_available);
  // The same borrowed managed input succeeds with a disjoint output.
  s::Report separate;
  ASSERT_EQ(device.owner.Evaluate(aliased, 1., false, separate), s::Status::Ok);
  EXPECT_EQ(managed.value->budget.distance, 2.);
}
TEST_F(Type25SearchCuda, MisalignedBorrowedViewsAndAddressOverflowFailWithoutPoisoning) {
  DeviceFixture device;
  device.Initialize();
  device.Reference();
  s::Report result;
  result.budget.distance = 71.;
  auto wrong = device.current;
  const auto position = reinterpret_cast<std::uintptr_t>(wrong.positions.data);
  wrong.positions.data = reinterpret_cast<const double*>(position + 1);
  EXPECT_EQ(device.owner.Evaluate(wrong, 1., false, result), s::Status::InvalidInput);
  EXPECT_EQ(result.budget.distance, 71.);
  wrong = device.current;
  const auto stiffness = reinterpret_cast<std::uintptr_t>(wrong.secondary_stiffness);
  wrong.secondary_stiffness = reinterpret_cast<const double*>(stiffness + 1);
  s::ReferenceToken token;
  EXPECT_EQ(device.owner.StageReference(wrong, token), s::Status::InvalidInput);
  wrong = device.current;
  wrong.main_gaps = reinterpret_cast<const double*>(UINTPTR_MAX - alignof(double) + 1);
  EXPECT_EQ(device.owner.Evaluate(wrong, 1., false, result), s::Status::InvalidInput);
  EXPECT_EQ(result.budget.distance, 71.);
  EXPECT_EQ(device.owner.reference_generation(), 1u);
  device.Compare(device.fixture.Current());
}
} // namespace type25_search_test
