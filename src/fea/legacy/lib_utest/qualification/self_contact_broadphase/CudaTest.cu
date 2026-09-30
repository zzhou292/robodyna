#include "CudaFixture.h"
#include "ManySource.h"

namespace surface_broadphase_test {
TEST(SelfContactBroadphaseCuda, AllAxesStridesAndExplicitLinearEndpointsMatchExhaustive) {
  self_contact_test::Fixture fixture(true, 77, true);
  const auto selected = fixture.Selection();
  ct::SelfContactSurfaceBinding source;
  ASSERT_EQ(source.Initialize(fixture.physical, self_contact_test::Input(selected)).status, ct::SelfContactSurfaceStatus::Ok);
  Stream stream;
  ct::SelfContactBroadphase broadphase;
  ASSERT_EQ(broadphase.Initialize(source, {}, stream.value).status, S::Ok);
  auto current = Positions(source, true), endpoint = current;
  for (auto& point : endpoint) point.x = -point.x;
  const auto discrete = Exhaustive(Boxes(source, current));
  const auto swept = Exhaustive(Boxes(source, current, &endpoint));
  ASSERT_LT(discrete.size(), swept.size());
  ASSERT_EQ(swept.size(), selected.size() * (selected.size() - 1) / 2);
  Device<double> first(current.size() * 3), last(current.size() * 3);
  const Key* stable = nullptr;
  for (bool soa : {false, true}) {
    const auto a = Pack(current, soa), b = Pack(endpoint, !soa);
    first.Upload(a, stream.value); last.Upload(b, stream.value);
    for (unsigned axis = 0; axis < 3; ++axis) {
      ct::SelfContactBroadphaseInput input{View(first, soa), {}, ct::SelfContactBoundsMotion::Current, axis};
      ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
      EXPECT_EQ(Read(broadphase, stream.value), discrete);
      if (stable) EXPECT_EQ(broadphase.pairs().device_keys, stable);
      stable = broadphase.pairs().device_keys;
      input.endpoint = View(last, !soa); input.motion = ct::SelfContactBoundsMotion::LinearNodalEndpoints;
      ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
      EXPECT_EQ(Read(broadphase, stream.value), swept);
      EXPECT_EQ(broadphase.pairs().device_keys, stable);
    }
  }
}
TEST(SelfContactBroadphaseCuda, CapacityAndInvalidBoundsRevokePriorViewAndPermitRetry) {
  self_contact_test::Fixture fixture(true, 77, true);
  const auto selected = fixture.Selection();
  ct::SelfContactSurfaceBinding source;
  ASSERT_EQ(source.Initialize(fixture.physical, self_contact_test::Input(selected)).status, ct::SelfContactSurfaceStatus::Ok);
  Stream stream;
  ct::SelfContactBroadphaseLimits limits;
  const auto total = selected.size() * (selected.size() - 1) / 2;
  limits.max_pairs = total - 1;
  ct::SelfContactBroadphase broadphase;
  ASSERT_EQ(broadphase.Initialize(source, limits, stream.value).status, S::Ok);
  const auto separated = Positions(source, true);
  Device<double> positions(separated.size() * 3);
  auto values = Pack(separated, false);
  positions.Upload(values, stream.value);
  ct::SelfContactBroadphaseInput input{View(positions, false)};
  ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
  const auto accepted = Read(broadphase, stream.value);
  const auto* storage = broadphase.pairs().device_keys;
  values = Pack(Positions(source, false), false); positions.Upload(values, stream.value);
  const auto capacity = broadphase.Evaluate(input, stream.value);
  EXPECT_EQ(capacity.status, S::PairCapacity); EXPECT_EQ(capacity.required_pairs, total);
  Revoked(broadphase);
  values = Pack(separated, false); positions.Upload(values, stream.value);
  ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
  EXPECT_EQ(Read(broadphase, stream.value), accepted);
  EXPECT_EQ(broadphase.pairs().device_keys, storage);
  auto late = separated;
  std::uint32_t first_bad = UINT32_MAX;
  for (std::size_t n = 0; n < late.size(); ++n) {
    const auto id = source.physical()->domain()->nodes()[n].source_id;
    if (id >= 20 && id <= 23) late[n].x = std::numeric_limits<double>::quiet_NaN();
  }
  for (std::size_t p = 0; p < source.parents().size(); ++p) {
    const auto& parent = source.parents()[p];
    for (unsigned c = 0; c < parent.arity; ++c) {
      const auto n = parent.arity == 3 ? parent.t3.nodes[c] : parent.q4.nodes[c];
      if (!std::isfinite(late[n].x)) first_bad = std::min(first_bad, static_cast<std::uint32_t>(p));
    }
  }
  ASSERT_GT(first_bad, 0u); ASSERT_NE(first_bad, UINT32_MAX);
  values = Pack(late, false); positions.Upload(values, stream.value);
  const auto later = broadphase.Evaluate(input, stream.value);
  EXPECT_EQ(later.status, S::InvalidBounds); EXPECT_EQ(later.parent, first_bad); Revoked(broadphase);
  // Multiple malformed parents must report the lowest S0 ordinal, regardless
  // of thread completion order. NaN is checked before CUB sorting.
  std::fill(values.begin(), values.end(), std::numeric_limits<double>::quiet_NaN());
  positions.Upload(values, stream.value);
  const auto invalid = broadphase.Evaluate(input, stream.value);
  EXPECT_EQ(invalid.status, S::InvalidBounds); EXPECT_EQ(invalid.parent, 0u); Revoked(broadphase);
  // Finite coordinates whose outward thickness overflows are also rejected.
  std::fill(values.begin(), values.end(), std::numeric_limits<double>::max());
  positions.Upload(values, stream.value);
  EXPECT_EQ(broadphase.Evaluate(input, stream.value).status, S::InvalidBounds); Revoked(broadphase);
  values = Pack(separated, false); positions.Upload(values, stream.value);
  ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
  auto bad = input; bad.current.node_stride = UINT64_MAX;
  EXPECT_EQ(broadphase.Evaluate(bad, stream.value).status, S::InvalidInput); Revoked(broadphase);
  ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
  bad = input; bad.current.data = reinterpret_cast<const double*>(broadphase.pairs().device_keys);
  EXPECT_EQ(broadphase.Evaluate(bad, stream.value).status, S::InvalidInput); Revoked(broadphase);
  bad = input; bad.endpoint = input.current;
  EXPECT_EQ(broadphase.Evaluate(bad, stream.value).status, S::InvalidInput); Revoked(broadphase);
  bad = input; bad.motion = static_cast<ct::SelfContactBoundsMotion>(99);
  EXPECT_EQ(broadphase.Evaluate(bad, stream.value).status, S::InvalidInput); Revoked(broadphase);
  bad = input; bad.axis = 3;
  EXPECT_EQ(broadphase.Evaluate(bad, stream.value).status, S::InvalidInput); Revoked(broadphase);
  EXPECT_EQ(broadphase.Evaluate(input, nullptr).status, S::InvalidInput); Revoked(broadphase);
  EXPECT_EQ(broadphase.Evaluate(input, cudaStreamLegacy).status, S::InvalidInput); Revoked(broadphase);
  EXPECT_EQ(broadphase.Evaluate(input, cudaStreamPerThread).status, S::InvalidInput); Revoked(broadphase);
  ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
  EXPECT_EQ(Read(broadphase, stream.value), accepted);
}
TEST(SelfContactBroadphaseCuda, CompleteZeroViewExactCapsAndRetainedSourceLifetime) {
  Stream stream;
  ct::SelfContactBroadphase broadphase;
  std::vector<double> values;
  {
    self_contact_test::Fixture fixture;
    auto selected = fixture.Selection(); selected.resize(1);
    ct::SelfContactSurfaceBinding source;
    ASSERT_EQ(source.Initialize(fixture.physical, self_contact_test::Input(selected)).status, ct::SelfContactSurfaceStatus::Ok);
    values = Pack(Positions(source, false), false);
    ct::SelfContactBroadphaseLimits limits; limits.max_pairs = 1;
    const auto preflight = ct::SelfContactBroadphase::Preflight(source, limits);
    ASSERT_EQ(preflight.report.status, S::Ok);
    limits.max_device_bytes = preflight.forecast.device_bytes - 1;
    EXPECT_EQ(broadphase.Initialize(source, limits, stream.value).status, S::ResourceLimit);
    EXPECT_FALSE(broadphase.initialized());
    ++limits.max_device_bytes; limits.max_host_bytes = preflight.forecast.startup_host_bytes - 1;
    EXPECT_EQ(broadphase.Initialize(source, limits, stream.value).status, S::ResourceLimit);
    EXPECT_FALSE(broadphase.initialized()); ++limits.max_host_bytes;
    EXPECT_EQ(broadphase.Initialize(source, limits, nullptr).status, S::InvalidInput);
    ASSERT_EQ(broadphase.Initialize(source, limits, stream.value).status, S::Ok);
    EXPECT_TRUE(broadphase.source()->SharesStorage(source));
    EXPECT_EQ(broadphase.forecast().device_bytes, preflight.forecast.device_bytes);
    EXPECT_EQ(broadphase.Initialize(source, limits, stream.value).status, S::AlreadyInitialized);
  }
  ASSERT_TRUE(broadphase.source()->prepared());
  Device<double> device(values.size()); device.Upload(values, stream.value);
  for (unsigned axis = 0; axis < 3; ++axis) {
    ct::SelfContactBroadphaseInput input{View(device, false), {}, ct::SelfContactBoundsMotion::Current, axis};
    ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
    EXPECT_TRUE(Read(broadphase, stream.value).empty());
  }
}
TEST(SelfContactBroadphaseCuda, MoreThanOneBlockProducesCompleteCoincidentInventory) {
  const auto source = ManySource(513);
  ASSERT_TRUE(source.prepared());
  Stream stream;
  ct::SelfContactBroadphaseLimits limits; limits.max_pairs = 513 * 512 / 2;
  ct::SelfContactBroadphase broadphase;
  ASSERT_EQ(broadphase.Initialize(source, limits, stream.value).status, S::Ok);
  std::vector<double> values(source.physical()->domain()->node_count() * 3, -0.);
  Device<double> device(values.size()); device.Upload(values, stream.value);
  std::vector<Key> expected;
  for (unsigned a = 0; a < 513; ++a)
    for (unsigned b = a + 1; b < 513; ++b) expected.push_back(Canonical(a, b));
  for (unsigned axis = 0; axis < 3; ++axis) {
    ct::SelfContactBroadphaseInput input{View(device, false), {}, ct::SelfContactBoundsMotion::Current, axis};
    ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
    EXPECT_EQ(Read(broadphase, stream.value), expected);
  }
}
namespace { __global__ void InvalidLaunchProbe() {} }
TEST(SelfContactBroadphaseCuda, CudaLaunchFailureRevokesSuccessAndPoisonsStorage) {
  const auto source = ManySource(2);
  Stream stream;
  ct::SelfContactBroadphase broadphase;
  ASSERT_EQ(broadphase.Initialize(source, {}, stream.value).status, S::Ok);
  std::vector<double> values(source.physical()->domain()->node_count() * 3, 0.);
  Device<double> device(values.size()); device.Upload(values, stream.value);
  ct::SelfContactBroadphaseInput input{View(device, false)};
  ASSERT_EQ(broadphase.Evaluate(input, stream.value).status, S::Ok);
  ASSERT_EQ(broadphase.pairs().count, 1u);
  // Zero grid is a recoverable synchronous launch-argument/configuration
  // error. CUDA 13 reports InvalidValue while older runtimes can report
  // InvalidConfiguration; preserve the exact backend error through the owner.
  // No illegal memory access or destroyed stream tests failure propagation.
  InvalidLaunchProbe<<<0, 1, 0, stream.value>>>();
  const auto injected = cudaPeekAtLastError();
  ASSERT_TRUE(injected == cudaErrorInvalidValue ||
              injected == cudaErrorInvalidConfiguration)
      << cudaGetErrorString(injected);
  EXPECT_EQ(broadphase.Evaluate(input, stream.value).status, S::DeviceFailure);
  Revoked(broadphase);
  EXPECT_EQ(cudaGetLastError(), injected);
  EXPECT_EQ(broadphase.Evaluate(input, stream.value).status, S::DeviceFailure);
  Revoked(broadphase);
}
} // namespace surface_broadphase_test
