// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"

namespace tl::fea::solids::batch_detail {
void LaunchFinalizeTest(Storage*, unsigned, unsigned, NodalPreparedView, BatchDiagnostics);
namespace frozen {
void LaunchInitialize(Storage*, cudaStream_t);
void LaunchCandidate(Storage*, unsigned, unsigned, NodalPreparedView, BatchDiagnostics);
void LaunchFinalizeTest(Storage*, unsigned, unsigned, NodalPreparedView, BatchDiagnostics);
}
}
namespace solid_validation_test {
inline bool Cuda(cudaError_t error) {
  EXPECT_EQ(error, cudaSuccess) << cudaGetErrorString(error);
  return error == cudaSuccess;
}
struct DeviceRig {
  HostRig host;
  d::Storage* device = nullptr;
  cudaStream_t stream = nullptr;
  double* positions = nullptr;
  std::size_t nodes = 0;
  double accepted_time = 0;
  std::vector<unsigned char> before, parallel, serial;
  ~DeviceRig() {
    if (device) cudaFree(device);
    if (positions) cudaFree(positions);
    if (stream) cudaStreamDestroy(stream);
  }
  bool Initialize(int flag = 1, bool analytic = false) {
    if (!host.Initialize(flag, analytic)) return false;
    nodes = host.config.owner.node_count;
    if (!Cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking)) ||
        !Cuda(cudaMalloc(reinterpret_cast<void**>(&device), host.layout.bytes)) ||
        !Cuda(cudaMalloc(reinterpret_cast<void**>(&positions), 12 * nodes * sizeof(double)))) return false;
    if (!d::RebaseCurves(host.model, host.layout, device, host.state)) return false;
    auto header = d::RebasedHeader(device, host.layout);
    header.config = host.config;
    header.source_instance_id = host.model.source_instance_id();
    *tl::util::ArenaPointer<d::Storage>(host.arena.data(), host.layout.header) = header;
    before.assign(static_cast<unsigned char*>(host.arena.data()),
        static_cast<unsigned char*>(host.arena.data()) + host.layout.bytes);
    if (!Restore(before)) return false;
    d::LaunchInitialize(device, stream);
    if (!Read(parallel) || !Restore(before)) return false;
    old::LaunchInitialize(device, stream);
    if (!Read(serial)) return false;
    Compare();
    return Header(serial).control.status == s::BatchStatus::Success;
  }
  bool Read(std::vector<unsigned char>& output) {
    output.resize(host.layout.bytes);
    return Cuda(cudaGetLastError()) && Cuda(cudaMemcpyAsync(output.data(), device,
        output.size(), cudaMemcpyDeviceToHost, stream)) && Cuda(cudaStreamSynchronize(stream));
  }
  bool Restore(const std::vector<unsigned char>& input) {
    return Cuda(cudaMemcpyAsync(device, input.data(), input.size(), cudaMemcpyHostToDevice, stream)) &&
        Cuda(cudaStreamSynchronize(stream));
  }
  static const d::Storage& Header(const std::vector<unsigned char>& bytes) {
    return *reinterpret_cast<const d::Storage*>(bytes.data());
  }
  template<class Traits> void CompareFamily(const d::FamilyLayout& layout) {
    for (unsigned slab = 0; slab < 2; ++slab) {
      const auto* a = tl::util::ArenaPointer<d::State<Traits>>(parallel.data(), layout.slab[slab]);
      const auto* b = tl::util::ArenaPointer<d::State<Traits>>(serial.data(), layout.slab[slab]);
      for (std::size_t p = 0; p < layout.parents.count; ++p) {
        const auto x = Traits::Read(a[p].history, a[p].cache);
        const auto y = Traits::Read(b[p].history, b[p].cache);
        EXPECT_EQ(std::memcmp(&x.history, &y.history, sizeof(x.history)), 0) << unsigned(Traits::family);
        EXPECT_EQ(std::memcmp(&x.cache, &y.cache, sizeof(x.cache)), 0) << unsigned(Traits::family);
        EXPECT_EQ(x.stamp.time_s, y.stamp.time_s);
        EXPECT_EQ(x.stamp.sample_index, y.stamp.sample_index);
      }
    }
    EXPECT_EQ(std::memcmp(parallel.data() + layout.status.offset,
        serial.data() + layout.status.offset, layout.status.bytes), 0);
  }
  void Compare() {
    SameControl(Header(parallel).control, Header(serial).control);
    CompareFamily<d::Traits18>(host.layout.solid18);
    CompareFamily<d::Traits24>(host.layout.solid24);
    CompareFamily<d::Traits6z>(host.layout.solid6z);
    CompareFamily<d::Traits18Law44>(host.layout.solid18_law44);
    CompareFamily<d::Traits18Law90>(host.layout.solid18_law90);
  }
  fe::NodalPreparedView View(unsigned epoch, bool invalid = false) {
    std::vector<double> values(12 * nodes);
    for (std::size_t n = 0; n < nodes; ++n) {
      const auto x = host.model.domain()->nodes()[n].position;
      const double xyz[] {x.x, x.y, x.z};
      for (unsigned k = 0; k < 3; ++k) {
        values[3 * n + k] = xyz[k] * (1 + .0001 * (epoch + 1));
        values[3 * nodes + 3 * n + k] = xyz[k] * (1 + .0001 * epoch);
        values[6 * nodes + 3 * n + k] = (k == 1 ? -.002 : .001) * xyz[k];
        values[9 * nodes + 3 * n + k] = .003 * xyz[k];
      }
    }
    if (invalid) values[0] = NAN;
    Cuda(cudaMemcpyAsync(positions, values.data(), values.size() * sizeof(double), cudaMemcpyHostToDevice, stream));
    Cuda(cudaStreamSynchronize(stream));
    fe::NodalPreparedView view;
    view.stream = stream;
    view.base_time = accepted_time;
    view.proposed_time = view.base_time + host.config.owner.fixed_dt;
    view.kick_dt = epoch ? host.config.owner.fixed_dt : .5 * host.config.owner.fixed_dt;
    view.kinematics.base_epoch = epoch;
    view.kinematics.position_xyz = positions;
    view.base_kinematics.position_xyz = positions + 3 * nodes;
    view.kinematics.velocity_xyz = positions + 6 * nodes;
    view.base_kinematics.velocity_xyz = positions + 9 * nodes;
    return view;
  }
  s::BatchDiagnostics Identity(const fe::NodalPreparedView& view, unsigned epoch, unsigned attempt) {
    s::BatchDiagnostics result;
    result.time = view.proposed_time;
    result.epoch = epoch + 1;
    result.base_time = view.base_time;
    result.base_epoch = epoch;
    result.attempt = attempt;
    result.phase = s::BatchPhase::Prepared;
    result.has_completed_interval = true;
    return result;
  }
  bool Candidate(unsigned epoch, unsigned attempt, bool invalid = false) {
    const auto view = View(epoch, invalid);
    const auto identity = Identity(view, epoch, attempt);
    if (!Read(before)) return false;
    // Both launches start with a deliberately unrelated old diagnostic stamp.
    auto& old_control = reinterpret_cast<d::Storage*>(before.data())->control;
    old_control.diagnostics.time = 123;
    old_control.diagnostics.epoch = 991;
    if (!Restore(before)) return false;
    d::LaunchCandidate(device, epoch % 2, 1 - epoch % 2, view, identity);
    if (!Read(parallel) || !Restore(before)) return false;
    old::LaunchCandidate(device, epoch % 2, 1 - epoch % 2, view, identity);
    if (!Read(serial)) return false;
    Compare();
    if (Header(serial).control.status == s::BatchStatus::Success)
      accepted_time = view.proposed_time;
    return true;
  }
};
class SolidCandidateCuda : public ::testing::Test {
  void SetUp() override {
    int devices = 0;
    ASSERT_EQ(cudaGetDeviceCount(&devices), cudaSuccess);
    ASSERT_GT(devices, 0);
  }
};
} // namespace solid_validation_test
