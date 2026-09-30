// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeCompare.h"
#include "PublicationPeer.h"

namespace type13_resident_test {
struct Snapshot {
  std::vector<double> x, v, w, q, reaction, couple;
  explicit Snapshot(std::size_t n)
      :x(3*n), v(3*n), w(3*n), q(4*n), reaction(3*n), couple(3*n) {}
  fe::NodalSnapshotBuffer Buffer() {
    return {x.data(), v.data(), x.size()/3, q.data(), w.data(), reaction.data(), couple.data()};
  }
};
struct Rig {
  Source source;
  fe::FENodalState owner;
  t::Batch batch;
  t::BatchConfig config;
  std::vector<double> mass, inertia, loads;
  std::vector<t::Evaluation> native;
  bool Initialize(const t::ModelInput&, bool moving = false);
  bool Begin(fe::NodalTrialToken&, fe::NodalAssemblyView&, double acceleration = 0);
  bool Prepare(const fe::NodalTrialToken&, const fe::NodalAssemblyView&, fe::NodalPreparedView&);
  bool Read(std::vector<t::Evaluation>&, t::BatchDiagnostics&);
  void CompareAssembly(const fe::NodalAssemblyView&);
  void ComparePrepared(const fe::NodalTrialToken&, const fe::NodalPreparedView&,
                       const std::vector<t::Evaluation>&, std::vector<t::Evaluation>& next);
  void Discard() {
    owner.Discard();
    batch.DiscardTrial();
  }
};
class Type13ResidentCuda : public ::testing::Test {
  void SetUp() override {
    int count = 0;
    ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess);
    ASSERT_GT(count, 0);
  }
};
inline bool Good(t::BatchReport report) {
  EXPECT_TRUE(report) << report.message << " element " << report.element;
  return bool(report);
}
inline bool Good(fe::NodalReport report) {
  EXPECT_EQ(report.status, fe::NodalStatus::Ok) << report.message;
  return report.status == fe::NodalStatus::Ok;
}
} // namespace type13_resident_test
