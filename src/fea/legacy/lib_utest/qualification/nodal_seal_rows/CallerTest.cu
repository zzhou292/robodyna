#include "Compare.h"
#include "FrozenTypes.h"
#include "lib_src/solvers/nodal_seal/Validation.cuh"
#include <algorithm>
#include <limits>
#include <vector>

namespace tl::fea::seal_test {
namespace {
class SealRowsCuda : public ::testing::Test {
  void SetUp() override {
    int count = 0;
    ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
    ASSERT_GT(count, 0);
  }
};
struct Packet {
  nodal_detail::Control* control = nullptr;
  double* scratch = nullptr;
  cudaStream_t stream = nullptr;
  explicit Packet(std::uint32_t n) {
    EXPECT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
    EXPECT_EQ(cudaMalloc(reinterpret_cast<void**>(&control),
        nodal_seal::ControlBytes(sizeof(*control), n)), cudaSuccess);
    EXPECT_EQ(cudaMalloc(reinterpret_cast<void**>(&scratch), std::size_t(11) * n * sizeof(double)), cudaSuccess);
  }
  ~Packet() {
    if (stream) cudaStreamSynchronize(stream);
    if (control) cudaFree(control);
    if (scratch) cudaFree(scratch);
    if (stream) cudaStreamDestroy(stream);
  }
  void Upload(std::uint32_t n, const std::vector<double>& values, unsigned fault) {
    nodal_detail::Control initial;
    initial.rows = {scratch + 6 * n, scratch + 7 * n, n, n, 0, 7, true, true, false};
    initial.assembly.base_epoch = 0; initial.assembly.attempt = 7;
    initial.limit.dt = 42; initial.limit.base_epoch = 91;
    if (fault == 1) initial.rows.valid = false;
    if (fault == 2) initial.rows.sealed = true;
    if (fault == 3) initial.rows.capacity = n - 1;
    if (fault == 4) initial.rows.stiffness = nullptr;
    if (fault == 5) initial.rows.stiffness = scratch + 8 * n; // valid foreign layout
    if (fault == 6) initial.rows.node_count = n - 1;
    if (fault == 7) ++initial.assembly.base_epoch;
    if (fault == 8 || fault == 7) {
      initial.assembly.status = tlfea::contact::Status::kNonFiniteResult;
      initial.assembly.node = 17;
    }
    if (fault == 9) initial.status = NodalStatus::InvalidInput;
    ASSERT_EQ(cudaMemcpyAsync(control, &initial, sizeof(initial), cudaMemcpyHostToDevice, stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(scratch, values.data(), values.size() * sizeof(double),
                             cudaMemcpyHostToDevice, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  }
  nodal_detail::Control Read(std::vector<double>& values) {
    nodal_detail::Control result;
    EXPECT_EQ(cudaMemcpyAsync(&result, control, sizeof(result), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(values.data(), scratch, values.size() * sizeof(double),
                             cudaMemcpyDeviceToHost, stream), cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    return result;
  }
};
void ComparePacket(const nodal_detail::Control& a, const nodal_detail::Control& b,
    const Packet& pa, const Packet& pb) {
  SameRows(a.rows, b.rows); SameLimit(a.limit, b.limit);
  // Actual raw pointer identity must survive even the malformed-layout fallback.
  const auto offset = [](const double* value, const Packet& packet) {
    return value ? reinterpret_cast<std::uintptr_t>(value) -
        reinterpret_cast<std::uintptr_t>(packet.scratch) : UINTPTR_MAX;
  };
  EXPECT_EQ(offset(a.rows.stiffness, pa), offset(b.rows.stiffness, pb));
  EXPECT_EQ(offset(a.rows.damping, pa), offset(b.rows.damping, pb));
  EXPECT_EQ(a.status, b.status); EXPECT_EQ(a.node, b.node);
  EXPECT_EQ(a.assembly.status, b.assembly.status);
  EXPECT_EQ(a.assembly.node, b.assembly.node);
  EXPECT_EQ(a.assembly.base_epoch, b.assembly.base_epoch);
  EXPECT_EQ(a.assembly.attempt, b.assembly.attempt);
}
void CompareCase(std::uint32_t n, unsigned fault, std::vector<double> values,
    double safety, double minimum, bool rotations, unsigned invalid_tail = 0) {
  Packet actual(n), old(n);
  actual.Upload(n, values, fault); old.Upload(n, values, fault);
  auto tail = nodal_seal::ControlTail(actual.control, n);
  if (invalid_tail == 1) ++tail.blocks;
  if (invalid_tail == 2) tail.summaries = reinterpret_cast<nodal_seal::RowSummary*>(actual.scratch);
  nodal_seal::Launch(actual.control, actual.scratch, n, 0, 7, safety, minimum, 1,
                     rotations, actual.stream, tail);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  FrozenLaunch(old.control, old.scratch, n, safety, minimum, 1, rotations, old.stream);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  auto av = values, bv = values;
  const auto a = actual.Read(av), b = old.Read(bv);
  ComparePacket(a, b, actual, old);
  EXPECT_EQ(std::memcmp(av.data(), values.data(), values.size() * sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(bv.data(), values.data(), values.size() * sizeof(double)), 0);
}
}
TEST_F(SealRowsCuda, CompleteFrozenCallerMatchesAcrossBlocksAndStrides) {
  for (std::uint32_t n : {1u, 255u, 256u, 257u, 513u, 65537u, 131073u}) {
    SCOPED_TRACE(n);
    std::vector<double> values(std::size_t(11) * n);
    std::fill(values.begin() + 7 * n, values.begin() + 8 * n, -0.);
    CompareCase(n, 0, values, .8, 1e-12, true);
    for (auto i : {0u, n / 2, n - 1}) {
      values[6 * n + i] = 4; values[7 * n + i] = 8;
    }
    CompareCase(n, 0, values, std::nextafter(1., 0.), 1e-12, true);
    CompareCase(n, 0, values, .8, .5, true);
  }
}
TEST_F(SealRowsCuda, FullLegacyFallbackPreservesErrorPriorityAndClearedLimit) {
  constexpr std::uint32_t n = 513;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> values(11 * n);
  values[6 * n + 512] = -1;
  values[7 * n + 256] = nan;
  for (unsigned fault = 0; fault != 10; ++fault) {
    SCOPED_TRACE(fault);
    CompareCase(n, fault, values, .8, 1e-12, true);
    auto axis_fault = values;
    axis_fault[3 * n + 257] = 1;
    axis_fault[4 * n + 257] = nan;
    CompareCase(n, fault, axis_fault, .8, 1e-12, false);
  }
  for (double safety : {0., 1., nan}) CompareCase(n, 0, values, safety, 1e-12, true);
  values.assign(11 * n, 0);
  values[6 * n + 512] = std::numeric_limits<double>::max();
  values[7 * n + 256] = std::numeric_limits<double>::max();
  CompareCase(n, 0, values, .8, 1e-12, true);
  CompareCase(n, 0, values, .8, 1e-12, true, 1);
  CompareCase(n, 0, values, .8, 1e-12, true, 2);
  values.assign(11 * n, 0);
  values[7 * n + 512] = std::numeric_limits<double>::denorm_min();
  CompareCase(n, 0, values, .8, 1e-12, true);
}
} // namespace tl::fea::seal_test
