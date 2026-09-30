#include "Fixture.h"
#include <new>
#include "lib_src/collision/nodal_wall_mapped/IntervalReduction.cuh"
#include "../mapped_wall_evaluation/Fixture.h"
namespace wall_interval_test {
__global__ void FrozenInterval(device::Storage* storage, tl::fea::NodalPreparedView view) {
  if (storage->control.status == Code::Ok) c::wall_interval_frozen::MeasureInterval(*storage, view);
}
void Sync() {
  ASSERT_EQ(cudaGetLastError(), cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
}
struct Packet {
  device::Storage storage;
  tl::fea::NodalPreparedView view;
  m::IntervalSummary interval[m::ObserverMaxBlocks];
  bool tree_used = false;
};
struct DeviceFixture {
  Packet* packet = nullptr;
  c::NodalWallPointResult *base = nullptr, *current = nullptr;
  double *base_x = nullptr, *x = nullptr, *base_v = nullptr, *v = nullptr, *error = nullptr;
  explicit DeviceFixture(Fixture& host) {
    const auto count = host.base.nodes.size();
    Allocate(packet, 1);
    Allocate(base, count);
    Allocate(current, count);
    Allocate(base_x, 3*count);
    Allocate(x, 3*count);
    Allocate(base_v, 3*count);
    Allocate(v, 3*count);
    Allocate(error, count);
    *packet = {};
    packet->storage = host.Storage();
    packet->storage.control = {};
    packet->storage.result.diagnostics = host.seed;
    packet->storage.base.nodes = base;
    packet->storage.result.nodes = current;
    packet->storage.addition_error = error;
    packet->view = host.view;
    packet->view.base_kinematics.position_xyz = base_x;
    packet->view.base_kinematics.velocity_xyz = base_v;
    packet->view.kinematics.position_xyz = x;
    packet->view.kinematics.velocity_xyz = v;
    std::copy(host.base.nodes.begin(), host.base.nodes.end(), base);
    std::copy(host.current.nodes.begin(), host.current.nodes.end(), current);
    std::copy(host.base.positions.begin(), host.base.positions.end(), base_x);
    std::copy(host.current.positions.begin(), host.current.positions.end(), x);
    std::copy(host.base_velocity.begin(), host.base_velocity.end(), base_v);
    std::copy(host.velocity.begin(), host.velocity.end(), v);
    std::copy(host.addition.begin(), host.addition.end(), error);
  }
  template<class T> static void Allocate(T*& pointer, std::size_t count) {
    ASSERT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&pointer), count*sizeof(T)), cudaSuccess);
    for (std::size_t i = 0; i < count; ++i) ::new (static_cast<void*>(pointer+i)) T{};
  }
  ~DeviceFixture() {
    cudaFree(error); cudaFree(v); cudaFree(base_v); cudaFree(x); cudaFree(base_x);
    cudaFree(current); cudaFree(base); cudaFree(packet);
  }
  void Run(bool frozen = false, bool scratch = true) {
    if (frozen) FrozenInterval<<<1, 1>>>(&packet->storage, packet->view);
    else m::parallel::MeasureInterval(&packet->storage, packet->view,
        packet->storage.model.node_count, nullptr,
        scratch ? m::IntervalScratch{packet->interval, m::ObserverMaxBlocks} : m::IntervalScratch{},
        &packet->tree_used);
    Sync();
  }
};
TEST(MappedWallIntervalCuda, FixedTreeMatchesHostGroupingAndIndependentPhysicalCertificates) {
  for (unsigned count : {1u, 127u, 128u, 129u, 263u, 33001u}) {
    SCOPED_TRACE(count);
    Fixture host(count);
    host.seed.kick_work = .125;
    host.seed.drift_work = -.25;
    ASSERT_TRUE(host.Staged());
    const auto expected = host.Storage().result.diagnostics;
    DeviceFixture actual(host);
    actual.Run();
    ASSERT_EQ(actual.packet->storage.control.status, Code::Ok);
    EXPECT_TRUE(actual.packet->tree_used);
    ExactDiagnostics(actual.packet->storage.result.diagnostics, expected);
    CheckPhysicalTruth(actual.packet->storage, actual.packet->view);
    CheckRoundedWork(host, actual.packet->storage.result.diagnostics);
    for (unsigned n = 0; n < count; ++n) {
      check::Exact(actual.base[n], host.base.nodes[n]);
      check::Exact(actual.current[n], host.current.nodes[n]);
    }
    ExactDiagnostics(actual.packet->storage.base.diagnostics, host.Storage().base.diagnostics);
  }
}
TEST(MappedWallIntervalCuda, OriginalFailurePriorityPartialDiagnosticsAndNoScratchFallback) {
  for (unsigned fault = 0; fault < 7; ++fault) {
    SCOPED_TRACE(fault);
    Fixture host;
    host.seed.wall_kick_moment_error = {3, 4, 5};
    if (fault == 0) {
      host.velocity[3*host.base.nodes[7].node] = NAN;
      host.base.nodes[7].force_world.x = 0;
      host.velocity[3*host.base.nodes[180].node] = INFINITY;
    }
    if (fault == 1) host.seed.drift_work = NAN;
    if (fault == 2) host.view.kick_dt = -.001;
    if (fault == 3) host.Storage().base.diagnostics.wall_moment.y = DBL_MAX/2;
    if (fault == 4) host.base.nodes[30].force.upper = INFINITY;
    if (fault == 5) host.addition[200] = NAN;
    if (fault == 6) {
      for (unsigned n = 0; n < host.base.nodes.size(); ++n) {
        host.base.nodes[n].force = {1, 1, 1, 0};
        host.base.nodes[n].force_world = {};
        host.base.nodes[n].wall_point = {};
      }
      host.base.nodes[0].wall_point.z = 1;
      host.base.nodes[1].wall_point.z = -1;
      host.base.nodes[2].wall_point.z = std::numeric_limits<double>::denorm_min();
      host.view.kick_dt = .5;
    }
    DeviceFixture serial(host), actual(host), optional(host);
    serial.Run(true);
    actual.packet->tree_used = optional.packet->tree_used = true;
    actual.Run();
    optional.Run(false, false);
    EXPECT_FALSE(actual.packet->tree_used);
    EXPECT_FALSE(optional.packet->tree_used);
    for (auto* result : {&actual, &optional}) {
      const auto& got = result->packet->storage;
      const auto& old = serial.packet->storage;
      EXPECT_EQ(got.control.status, old.control.status);
      EXPECT_EQ(got.control.node, old.control.node);
      EXPECT_EQ(got.control.parent, old.control.parent);
      ExactDiagnostics(got.result.diagnostics, old.result.diagnostics);
      ExactDiagnostics(got.base.diagnostics, old.base.diagnostics);
    }
  }
}
namespace physical = wall_evaluation_test;
struct PhysicalExtra {
  double base_x[3*physical::Nodes], base_v[3*physical::Nodes];
  m::IntervalSummary interval[m::ObserverMaxBlocks];
};
struct PhysicalFixture : physical::Fixture {
  PhysicalExtra* extra = nullptr;
  PhysicalFixture() { DeviceFixture::Allocate(extra, 1); *extra = {}; }
  ~PhysicalFixture() { cudaFree(extra); }
};
__global__ void CopyBase(device::Storage* storage) {
  device::CopyBase(*storage, blockIdx.x*blockDim.x+threadIdx.x, gridDim.x*blockDim.x);
}
TEST(MappedWallIntervalCuda, ActualMixedPhysicalPacketsKeepSameMaskBaseAndRetryTruth) {
  PhysicalFixture f;
  for (unsigned mask = 0; mask < 3; ++mask) {
    SCOPED_TRACE(mask);
    f.Restore();
    for (unsigned p = 0; p < physical::Parents; ++p)
      f.input->activity[p] = mask == 0 ? 1 : mask == 1 ? p%3 != 0 : 0;
    m::parallel::Evaluate(f.storage, f.Side(), f.Kinematics(), f.Identity(),
        physical::Nodes, physical::Parents, nullptr, false);
    Sync();
    ASSERT_EQ(f.storage->control.status, Code::Ok);
    std::copy_n(f.input->x, 3*physical::Nodes, f.extra->base_x);
    std::copy_n(f.input->velocity, 3*physical::Nodes, f.extra->base_v);
    f.storage->result.diagnostics.valid = true;
    CopyBase<<<m::parallel::Blocks(physical::Parents), device::Workers>>>(f.storage);
    Sync();
    std::fill_n(f.storage->addition_error, physical::Nodes, 0.);
    const auto accepted = f.Read(f.storage->base);
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
      f.storage->control = {};
      for (unsigned n = 0; n < physical::Nodes; ++n) {
        f.input->x[3*n] = f.extra->base_x[3*n]+.00001;
        f.input->velocity[3*n] = f.extra->base_v[3*n]+.0003;
      }
      m::parallel::Evaluate(f.storage, f.Side(), f.Kinematics(), f.Identity(),
          physical::Nodes, physical::Parents, nullptr, false);
      Sync();
      ASSERT_EQ(f.storage->control.status, Code::Ok);
      // Inject after candidate mechanics: this exercises the interval's own
      // consumed-x failure, rather than an earlier contact-point validator.
      if (!attempt) f.input->velocity[3*(physical::Nodes-1)] = NAN;
      tl::fea::NodalPreparedView view;
      view.kick_dt = .001;
      view.kinematics = f.Kinematics();
      view.base_kinematics = f.Kinematics();
      view.base_kinematics.position_xyz = f.extra->base_x;
      view.base_kinematics.velocity_xyz = f.extra->base_v;
      m::parallel::MeasureInterval(f.storage, view, physical::Nodes, nullptr,
          {f.extra->interval, m::ObserverMaxBlocks});
      Sync();
      if (!attempt) EXPECT_EQ(f.storage->control.status, Code::NonFiniteArithmetic);
      else {
        ASSERT_EQ(f.storage->control.status, Code::Ok);
        CheckPhysicalTruth(*f.storage, view);
      }
      const auto after = f.Read(f.storage->base);
      ExactDiagnostics(after.diagnostics, accepted.diagnostics);
      for (unsigned n = 0; n < physical::Nodes; ++n) check::Exact(after.nodes[n], accepted.nodes[n]);
      for (unsigned p = 0; p < physical::Parents; ++p) check::Exact(after.parents[p], accepted.parents[p]);
      EXPECT_EQ(after.faces, accepted.faces);
    }
  }
}
} // namespace wall_interval_test
