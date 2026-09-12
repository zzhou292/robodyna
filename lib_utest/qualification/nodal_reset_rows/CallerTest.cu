#include "ControlChecks.h"
#include "Frozen.h"
#include "lib_src/solvers/nodal_reset/Reset.cuh"

namespace tl::fea::reset_test {
namespace {
class ResetRowsCuda : public ::testing::Test {
  void SetUp() override {
    int count = 0;
    ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess); ASSERT_GT(count, 0);
  }
};
struct Packet {
  nodal_detail::Control* control = nullptr;
  double* values = nullptr;
  cudaStream_t stream = nullptr;
  explicit Packet(std::size_t count) {
    EXPECT_EQ(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), cudaSuccess);
    EXPECT_EQ(cudaMalloc(reinterpret_cast<void**>(&control), sizeof(*control)), cudaSuccess);
    EXPECT_EQ(cudaMalloc(reinterpret_cast<void**>(&values), count * sizeof(double)), cudaSuccess);
  }
  ~Packet() {
    if (stream) cudaStreamSynchronize(stream);
    if (control) cudaFree(control);
    if (values) cudaFree(values);
    if (stream) cudaStreamDestroy(stream);
  }
  void Upload(const std::vector<double>& input, std::uint32_t n, unsigned fault) {
    nodal_detail::Control seed;
    SeedControl(seed); seed.rows = Rows(values, n, fault);
    ASSERT_EQ(cudaMemcpyAsync(control, &seed, sizeof(seed), cudaMemcpyHostToDevice, stream), cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(values, input.data(), input.size() * sizeof(double),
                             cudaMemcpyHostToDevice, stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  }
  nodal_detail::Control Read(std::vector<double>& output) const {
    nodal_detail::Control result;
    EXPECT_EQ(cudaMemcpyAsync(&result, control, sizeof(result), cudaMemcpyDeviceToHost, stream), cudaSuccess);
    EXPECT_EQ(cudaMemcpyAsync(output.data(), values, output.size() * sizeof(double),
                             cudaMemcpyDeviceToHost, stream), cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
    return result;
  }
};
void Compare(std::uint32_t n, unsigned fault, std::uint64_t attempt) {
  const auto seed = Seed(n);
  Packet actual(seed.size()), old(seed.size());
  actual.Upload(seed, n, fault); old.Upload(seed, n, fault);
  nodal_reset::ResetTrial<<<1,nodal_reset::Threads,0,actual.stream>>>(actual.control, 7, attempt);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  FrozenLaunch(old.control, 7, attempt, old.stream);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  auto av = seed, bv = seed;
  const auto a = actual.Read(av), b = old.Read(bv);
  EXPECT_EQ(a.status, b.status); SameRows(a.rows, b.rows); SameBytes(av, bv);
  ClearedControl(a, 7, attempt, b.status); ClearedControl(b, 7, attempt, b.status);
  const auto original = Rows(actual.values, n, fault);
  EXPECT_EQ(a.rows.stiffness, original.stiffness); EXPECT_EQ(a.rows.damping, original.damping);
  CheckCleared(seed, av, n, a.status == NodalStatus::Ok);
  // A rejected header must not prevent a subsequent valid reset on the same
  // allocation and stream; no stale shared result may survive the next launch.
  actual.Upload(seed, n, 0); old.Upload(seed, n, 0);
  nodal_reset::ResetTrial<<<1,nodal_reset::Threads,0,actual.stream>>>(actual.control, 8, 1);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  FrozenLaunch(old.control, 8, 1, old.stream);
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  const auto retry = actual.Read(av), prior = old.Read(bv);
  SameRows(retry.rows, prior.rows); SameBytes(av, bv);
  ClearedControl(retry, 8, 1, NodalStatus::Ok); CheckCleared(seed, av, n, true);
}
}
TEST_F(ResetRowsCuda, CompleteFrozenKernelAcrossBlockBoundariesAndRepeatedStrides) {
  for (std::uint32_t n : {1u, 255u, 256u, 257u, 513u, 65537u, 1048579u}) {
    SCOPED_TRACE(n); Compare(n, 0, 10);
  }
}
TEST_F(ResetRowsCuda, InvalidAndStaleHeadersPreservePartialEffectsAndUniformBarrierExit) {
  for (std::uint32_t n : {1u, 257u}) {
    for (unsigned fault = 0; fault != 12; ++fault) {
      for (std::uint64_t attempt : {0ull, 10ull}) {
        SCOPED_TRACE(n);
        SCOPED_TRACE(fault);
        SCOPED_TRACE(attempt);
        Compare(n, fault, attempt);
      }
    }
  }
}
} // namespace tl::fea::reset_test
