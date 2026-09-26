#pragma once
#include "../tied_cin_runtime/OwnerFixture.h"
#include "../radioss_type25_friction/CudaFixture.h"
#include <cstring>
#include <stdexcept>

namespace accepted_mass_test {
namespace fe = tl::fea;
using Status = fe::NodalStatus;
using View = fe::NodalAcceptedRawMassView;
inline void Check(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
inline std::vector<double> Read(const View& view) {
  std::vector<double> values(view.node_count);
  type25_friction_test::Drain drain{view.stream};
  Check(cudaMemcpyAsync(values.data(), view.mass_kg, values.size()*sizeof(double),
      cudaMemcpyDeviceToHost, view.stream));
  Check(cudaStreamSynchronize(view.stream));
  return values;
}
inline void Same(const View& a, const View& b) {
  EXPECT_EQ(a.mass_kg,b.mass_kg);
  EXPECT_EQ(a.node_count,b.node_count);
  EXPECT_EQ(a.owner_id,b.owner_id);
  EXPECT_EQ(a.base_epoch,b.base_epoch);
  EXPECT_EQ(a.attempt,b.attempt);
  EXPECT_EQ(a.qualification_id,b.qualification_id);
  EXPECT_EQ(a.stream,b.stream);
}
inline void SameBits(const std::vector<double>& a, const std::vector<double>& b) {
  ASSERT_EQ(a.size(),b.size());
  EXPECT_EQ(std::memcmp(a.data(),b.data(),a.size()*sizeof(double)),0);
}
class Cuda : public ::testing::Test {
  void SetUp() override {
    int count=0;
    ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
    ASSERT_GT(count,0);
    ASSERT_EQ(cudaSetDevice(0),cudaSuccess);
  }
};
} // namespace accepted_mass_test
