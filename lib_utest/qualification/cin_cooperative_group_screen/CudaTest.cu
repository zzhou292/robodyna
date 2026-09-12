// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include <stdexcept>

namespace tl::fea::cooperative_test {
namespace {
std::vector<groups::Report> captured;
void Check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
template<bool Frozen>
cudaError_t WithReports(const cin_advance::Input& original, cudaStream_t stream) {
  // Qualification scratch only; production uses the existing owner report arena.
  const auto count = original.groups.group_count;
  groups::Report* storage = nullptr;
  Check(cudaMalloc(reinterpret_cast<void**>(&storage), (count+2)*sizeof(*storage)));
  try {
    Check(cudaMemsetAsync(storage, 0xa5, (count+2)*sizeof(*storage), stream));
    auto input = original;
    input.group_reports = storage+1;
    Check(Frozen ? frozen_screen::Launch(input, stream) : screen::Launch(input, stream));
    Check(cudaStreamSynchronize(stream));
    std::vector<unsigned char> bytes((count+2)*sizeof(*storage));
    Check(cudaMemcpy(bytes.data(), storage, bytes.size(), cudaMemcpyDeviceToHost));
    for (unsigned a = 0; a < sizeof(*storage); ++a) {
      EXPECT_EQ(bytes[a], 0xa5);
      EXPECT_EQ(bytes[(count+1)*sizeof(*storage)+a], 0xa5);
    }
    nodal_detail::Control control;
    screen::Summary summary;
    Check(cudaMemcpy(&control, input.control, sizeof(control), cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(&summary, input.screen, sizeof(summary), cudaMemcpyDeviceToHost));
    captured.clear();
    if (original.groups.group_count && summary.invalid_node == UINT32_MAX &&
        control.status != NodalStatus::DeviceFailure) {
      captured.resize(count);
      std::memcpy(captured.data(), bytes.data()+sizeof(*storage), count*sizeof(*storage));
    }
    Check(cudaFree(storage));
  } catch (...) {
    cudaFree(storage);
    throw;
  }
  return cudaSuccess;
}
void SameWitness(const cin_limiter::Witness& a, const cin_limiter::Witness& b) {
  EXPECT_EQ(a.epoch, b.epoch);
  EXPECT_EQ(a.attempt, b.attempt);
  const auto& x = a.values;
  const auto& y = b.values;
  EXPECT_EQ(x.kind, y.kind);
  EXPECT_EQ(x.node, y.node);
  EXPECT_EQ(x.group, y.group);
  EXPECT_EQ(x.translation_fixed_bits, y.translation_fixed_bits);
  EXPECT_EQ(x.rotation_fixed, y.rotation_fixed);
  EXPECT_EQ(x.rotation_present, y.rotation_present);
  Exact(x.minimum_dt_s, y.minimum_dt_s);
  Exact(x.factor, y.factor);
  Exact(x.mass_kg, y.mass_kg);
  Exact(x.inertia_kg_m2, y.inertia_kg_m2);
  Exact(x.translation_stiffness_n_per_m, y.translation_stiffness_n_per_m);
  Exact(x.rotation_stiffness_nm, y.rotation_stiffness_nm);
  Exact(x.principal_inertia_kg_m2.x, y.principal_inertia_kg_m2.x);
  Exact(x.principal_inertia_kg_m2.y, y.principal_inertia_kg_m2.y);
  Exact(x.principal_inertia_kg_m2.z, y.principal_inertia_kg_m2.z);
  Exact(x.trace_upper_per_s2, y.trace_upper_per_s2);
}
void CompareCuda(packet::Packet p, NodalStatus expected = NodalStatus::Ok) {
  auto old = p, now = p;
  packet::DevicePacket a(old, false, true), b(now, false, true);
  a.RunWith(WithReports<true>);
  const auto old_reports = captured;
  b.RunWith(WithReports<false>);
  ASSERT_EQ(old_reports.size(), captured.size());
  for (unsigned g = 0; g < captured.size(); ++g) Same(old_reports[g], captured[g]);
  a.Download(old);
  b.Download(now);
  EXPECT_EQ(old.control.status, expected) << old.control.node;
  packet::SameControl(old.control, now.control);
  SameWitness(old.control.structural_limiter, now.control.structural_limiter);
  EXPECT_EQ(old.failure, now.failure);
  packet::SameDoubles(old.accepted, p.accepted);
  packet::SameDoubles(now.accepted, p.accepted);
  packet::SameDoubles(old.trial, p.trial);
  packet::SameDoubles(now.trial, p.trial);
  packet::SameDoubles(old.loads, now.loads);
  packet::SameDoubles(old.work, now.work);
  packet::SameDoubles(old.capture, now.capture);
}
}
TEST(CinCooperativeCuda, CompleteFrozenScreenMultiTileCurrentFrameAndLimiter) {
  for (bool capture : {false, true}) {
    for (unsigned count : {2, 3, 63, 64, 65, 127, 128, 129, 260}) {
      SCOPED_TRACE(count);
      auto p = Members({count});
      p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, capture};
      for (auto previous : {0., 1e-6, .125}) {
        p.durations.previous_drift_dt = previous;
        CompareCuda(p);
        std::reverse(p.members.begin(), p.members.end());
        CompareCuda(p);
      }
      std::fill(p.work.begin(), p.work.begin()+2*packet::Nodes, -0.);
      CompareCuda(p);
    }
    auto p = Members({65, 129, 2});
    p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, capture};
    cin_group_test::ReverseGroups(p);
    CompareCuda(p);
    p.groups[1].offset = p.groups[0].offset;
    p.groups[1].count = p.groups[0].count;
    CompareCuda(p);
  }
}
TEST(CinCooperativeCuda, EveryEarlyLateFailurePreservesOutputsAndRetry) {
  for (unsigned fault = 0; fault < 14; ++fault) {
    SCOPED_TRACE(fault);
    auto p = Members({129, 3});
    p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true};
    const auto first = p.members[0].node;
    const auto late = p.members[128].node;
    if (fault == 0) p.groups[0].count = 1;
    if (fault == 1) p.groups[1].count = 1;
    if (fault == 2) p.groups[0].offset = UINT32_MAX;
    if (fault == 3) p.groups[1].count = UINT32_MAX;
    if (fault == 4) p.groups[0].mass = 0;
    if (fault == 5) p.members[128].node = packet::Nodes;
    if (fault == 6) p.dependent[late] = 1;
    if (fault == 7) p.member_nodes[late] = 0;
    if (fault == 8) p.accepted[3*late] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 9) {
      p.work[first] = std::numeric_limits<double>::max();
      p.accepted[3*late] = std::numeric_limits<double>::quiet_NaN();
    }
    if (fault == 10) {
      p.groups[0].mass = 0;
      p.work[12] = -1;
    }
    if (fault == 11) p.work[packet::Nodes+first] = std::numeric_limits<double>::max();
    if (fault >= 12) {
      p.accepted[3*first] = std::numeric_limits<double>::quiet_NaN();
      p.members[fault == 12 ? 63 : 65].node = UINT32_MAX;
    }
    CompareCuda(p, NodalStatus::InvalidOutput);
    auto retry = Members({129, 3});
    retry.Begin(2);
    retry.structural = p.structural;
    CompareCuda(retry);
  }
}
TEST(CinCooperativeCuda, PendingControlAndTooLargeStepPreserveOriginalPriority) {
  auto p = Members({129, 3});
  p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true};
  p.control.status = NodalStatus::DeviceFailure;
  p.groups[0].offset = UINT32_MAX;
  CompareCuda(p, NodalStatus::DeviceFailure);
  p = Members({129, 3});
  p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true};
  p.durations.drift_dt = 100;
  CompareCuda(p, NodalStatus::StepTooLarge);
}
} // namespace tl::fea::cooperative_test
