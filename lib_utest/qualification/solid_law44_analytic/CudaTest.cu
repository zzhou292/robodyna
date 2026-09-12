// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cuda_runtime.h>
namespace law44_analytic_test {
namespace {
struct Packet {
  law::Parameters material;
  law::Result constructor, accepted, rejected, retry;
  law::Status status[4]{};
};
__global__ void Advance(Packet* packet, double invalid) {
  auto& p = *packet;
  auto input = Motion(0); input.dt_s = 0;
  p.status[0] = law::Initialize(p.material, input, p.constructor);
  law::History history;
  for (unsigned step = 0; step < 320; ++step) {
    p.status[1] = law::Update(p.material, history, Motion(step), p.accepted);
    if (p.status[1] != law::Status::Ok) return;
    history = p.accepted.history;
  }
  p.rejected = p.accepted;
  input = Motion(320); input.engineering_rate_per_s[5] = invalid;
  p.status[2] = law::Update(p.material, history, input, p.rejected);
  p.status[3] = law::Update(p.material, history, Motion(320), p.retry);
}
}
TEST(SolidLaw44AnalyticCuda, NativeEmptyCurveConstructorTrajectoryAndLateRetry) {
  for (const auto material : {Airbag(), Capped(.6)}) {
    Packet host; host.material = material;
    Packet* device = nullptr;
    ASSERT_EQ(cudaMalloc(&device, sizeof(Packet)), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(device, &host, sizeof(host), cudaMemcpyHostToDevice), cudaSuccess);
    Advance<<<1, 1>>>(device, INFINITY);
    ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&host, device, sizeof(host), cudaMemcpyDeviceToHost), cudaSuccess);
    ASSERT_EQ(cudaFree(device), cudaSuccess);
    ASSERT_EQ(host.status[0], law::Status::Ok); ASSERT_EQ(host.status[1], law::Status::Ok);
    EXPECT_EQ(host.status[2], law::Status::InvalidInput); ASSERT_EQ(host.status[3], law::Status::Ok);
    Same(host.accepted, host.rejected);
    auto input = Motion(0); input.dt_s = 0;
    law44_solid_test::Compare(host.constructor, Native(material, {}, input, true).result, material, {}, input);
    law::History native, previous;
    law::Result result;
    for (unsigned step = 0; step < 320; ++step) {
      previous = native; result = Native(material, native, Motion(step)).result; native = result.history;
    }
    law44_solid_test::Compare(host.accepted, result, material, previous, Motion(319));
    law44_solid_test::Compare(host.retry, Native(material, native, Motion(320)).result, material, native, Motion(320));
  }
}
} // namespace law44_analytic_test
