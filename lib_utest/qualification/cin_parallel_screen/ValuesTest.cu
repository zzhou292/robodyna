// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <memory>
#include <stdexcept>
namespace tl::fea::cin_screen_test {
namespace {
void Check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
struct Delete { void operator()(void* p) const noexcept { cudaFree(p); } };
struct Header {
  nodal_detail::Control control;
  cin_advance::FailureKey key = 9876;
  dt::Result result{19.25, 123, 456, true};
  std::uint32_t invalid = UINT32_MAX;
  bool success = false;
};
class Device {
 public:
  Header header;
  cin_advance::Input input;
  explicit Device(const Fixture& fixture, double factor, double duration = 0) {
    header.control.limit.dt = 19.25;
    device_header_ = static_cast<Header*>(Upload(&header, sizeof(header)));
    input.control = &device_header_->control;
    input.failure = &device_header_->key;
    input.accepted = Copy(fixture.accepted);
    input.model.node_count = fixture.nodes;
    input.model.dependent_nodes = Copy(fixture.dependent);
    input.rotation_present = Copy(fixture.present);
    std::vector<double> coefficients = fixture.mass;
    coefficients.insert(coefficients.end(), fixture.inertia.begin(), fixture.inertia.end());
    input.tail = Copy(coefficients);
    auto stiffness = fixture.translation;
    stiffness.insert(stiffness.end(), fixture.rotation.begin(), fixture.rotation.end());
    input.work = Copy(stiffness);
    std::vector<std::uint8_t> fixed(fixture.nodes);
    fixed.insert(fixed.end(), fixture.fixed.begin(), fixture.fixed.end());
    fixed.insert(fixed.end(), fixture.fixed_rotation.begin(), fixture.fixed_rotation.end());
    input.fixed = Copy(fixed);
    input.groups = fixture.View().rigid;
    input.groups.groups = Copy(fixture.groups);
    input.groups.members = Copy(fixture.members);
    input.groups.member_nodes = Copy(fixture.member);
    const std::vector<tree::Summary> scratch(tree::Blocks(fixture.nodes), {-7, 123, 456});
    input.screen = Copy(scratch);
    input.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, factor};
    input.durations = {fixture.previous, 0, duration};
  }
  template<class T> T* Copy(const std::vector<T>& source) {
    return static_cast<T*>(Upload(source.data(), source.size()*sizeof(T)));
  }
  void* Upload(const void* source, std::size_t bytes) {
    if (!bytes) return nullptr;
    void* pointer = nullptr;
    Check(cudaMalloc(&pointer, bytes));
    allocations_.emplace_back(pointer);
    Check(cudaMemcpy(pointer, source, bytes, cudaMemcpyHostToDevice));
    return pointer;
  }
  void Run(bool frozen);
  tree::Summary Summary() const {
    tree::Summary result;
    Check(cudaMemcpy(&result, input.screen, sizeof(result), cudaMemcpyDeviceToHost));
    return result;
  }
 private:
  std::vector<std::unique_ptr<void, Delete>> allocations_;
  Header* device_header_ = nullptr;
};
__global__ void FrozenValue(cin_advance::Input input, Header* output) {
  const auto n = input.model.node_count;
  const cin_screen_frozen::Sources source{input.accepted, input.tail, input.tail+n,
      input.work, input.work+n, input.fixed+n, input.fixed+2*n, input.rotation_present,
      input.model.dependent_nodes, input.groups, n, input.durations.previous_drift_dt};
  cin_screen_frozen::Result result{output->result.minimum_dt, output->result.limiting_node,
      output->result.limiting_group, output->result.valid};
  output->success = cin_screen_frozen::Screen(source, input.structural.factor, result, output->invalid);
  output->result = {result.minimum_dt, result.limiting_node, result.limiting_group, result.valid};
  if (!output->success) {
    output->control.status = NodalStatus::InvalidOutput;
    output->control.node = output->invalid;
    return;
  }
  output->control.limit.dt = result.minimum_dt;
  if (input.durations.drift_dt > result.minimum_dt) {
    output->control.status = NodalStatus::StepTooLarge;
    output->control.node = result.limiting_node;
    return;
  }
  *input.failure = cin_advance::NoFailure;
}
void Device::Run(bool frozen) {
  if (frozen) {
    FrozenValue<<<1,1>>>(input, device_header_);
    Check(cudaGetLastError());
  } else Check(tree::Launch(input, nullptr));
  Check(cudaDeviceSynchronize());
  Check(cudaMemcpy(&header, device_header_, sizeof(header), cudaMemcpyDeviceToHost));
}
void SameOutcome(const Device& a, const Device& b) {
  EXPECT_EQ(a.header.control.status, b.header.control.status);
  EXPECT_EQ(a.header.control.node, b.header.control.node);
  Exact(a.header.control.limit.dt, b.header.control.limit.dt);
  EXPECT_EQ(a.header.key, b.header.key);
}
}
TEST(CinParallelScreenCuda, FullSourceMatrixAcrossMaximumBlocksMatchesFrozenCudaAndHost) {
  for (unsigned count : {8u, 127u, 128u, 129u, 263u, 33001u}) {
    SCOPED_TRACE(count);
    Fixture f(count);
    for (unsigned mode = 0; mode < 3; ++mode) {
      if (mode == 1) {
        std::fill(f.translation.begin(), f.translation.end(), 0);
        std::fill(f.rotation.begin(), f.rotation.end(), 0);
      }
      if (mode == 2) {
        f.translation[0] = 8;
        f.translation[count-1] = 8;
        f.mass[count-1] = 2;
        f.fixed[count-1] = 0;
        f.fixed_rotation[count-1] = 1;
      }
      Device serial(f, .8), parallel(f, .8);
      serial.Run(true);
      parallel.Run(false);
      SameOutcome(parallel, serial);
      ASSERT_EQ(parallel.header.control.status, NodalStatus::Ok);
      const auto expected = Reduce(f.View(), .8);
      const auto actual = parallel.Summary();
      Exact(actual.minimum_dt, expected.minimum_dt);
      EXPECT_EQ(actual.limiting_node, expected.limiting_node);
      EXPECT_EQ(actual.invalid_node, expected.invalid_node);
      auto result = dt::Result{};
      std::uint32_t invalid = UINT32_MAX;
      ASSERT_TRUE(Frozen(f.View(), .8, result, invalid));
      Same(serial.header.result, result);
    }
  }
}
TEST(CinParallelScreenCuda, CoefficientAndGroupErrorOrderKeepsControlLimitAndMotionKey) {
  for (unsigned fault = 0; fault < 8; ++fault) {
    SCOPED_TRACE(fault);
    Fixture f;
    if (fault == 0) { f.inertia[1] = NAN; f.mass[128] = -1; }
    if (fault == 1) { f.groups[0].count = 1; f.rotation[200] = -1; }
    if (fault == 2) f.groups[0].count = 1;
    if (fault == 3) f.groups[1].offset = 999;
    if (fault == 4) f.members[3].node = f.nodes;
    if (fault == 5) f.groups[1].principal_inertia.z = 0;
    if (fault == 6) f.previous = -1;
    if (fault == 7) f.mass[128] = std::numeric_limits<double>::max();
    Device serial(f, .8), parallel(f, .8);
    serial.Run(true);
    parallel.Run(false);
    SameOutcome(parallel, serial);
    EXPECT_EQ(parallel.header.control.status, NodalStatus::InvalidOutput);
    EXPECT_EQ(parallel.header.key, 9876u);
    if (fault == 2) EXPECT_EQ(parallel.header.control.node, f.nodes-1);
    if (fault == 3) EXPECT_EQ(parallel.header.control.node, 3u);
    if (fault == 6) Exact(parallel.Summary().minimum_dt, -7); // Header rejected before writes.
  }
}
TEST(CinParallelScreenCuda, ExactScreenBoundaryRetainsLimitAndDoesNotInitializeRejectedMotionKey) {
  Fixture f;
  dt::Result result;
  std::uint32_t invalid = UINT32_MAX;
  ASSERT_TRUE(Frozen(f.View(), .8, result, invalid));
  for (double duration : {result.minimum_dt, ::nextafter(result.minimum_dt, INFINITY)}) {
    Device serial(f, .8, duration), parallel(f, .8, duration);
    serial.Run(true);
    parallel.Run(false);
    SameOutcome(parallel, serial);
    if (duration == result.minimum_dt) EXPECT_EQ(parallel.header.control.status, NodalStatus::Ok);
    else {
      EXPECT_EQ(parallel.header.control.status, NodalStatus::StepTooLarge);
      EXPECT_EQ(parallel.header.key, 9876u);
    }
  }
}
} // namespace tl::fea::cin_screen_test
