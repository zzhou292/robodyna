// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../physical_publication/OwnerFixture.h"
#include "lib_src/elements/publication/PhysicalActivitySnapshot.h"
namespace physical_activity_test {
namespace fe = tl::fea; namespace p = physical_publication_test;
using Status = fe::PhysicalActivityStatus;
inline bool Good(const fe::PhysicalActivityReport& r) {
  EXPECT_EQ(r.status, Status::Ok) << r.message << " family=" << int(r.family)
      << " stage=" << int(r.stage) << " index=" << r.family_index;
  return r.status == Status::Ok;
}
struct Fixture {
  explicit Fixture(double failure = 2.5) : rig(false, failure) {}
  p::Rig rig;
  fe::PhysicalActivitySnapshot snapshot;
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics common;
  fe::PhysicalAcceptedActivityReceipt accepted;
  fe::PhysicalPreparedActivityReceipt candidate;
  bool Initialize() {
    return rig.Initialize() && Good(snapshot.Initialize(rig.owner, rig.publication,
        rig.fixture.physical, rig.Participants(), rig.fixture.Identity()));
  }
  bool Begin() {
    return rig.Begin(token, assembly) && Good(snapshot.CaptureAccepted(rig.owner, token, assembly, &accepted));
  }
  bool PreparePhysical() {
    fe::ShellPhysicalDiagnostics material;
    return rig.Advance(token, assembly, prepared) && rig.Evaluate(token, prepared, material) &&
        p::Good(rig.publication.PreparePhysical(rig.owner, token,
          {&material.qeph, &material.t3, &material.qbat, &material.type25, &material.type13, &material.solids}, &common));
  }
  bool CapturePrepared() {
    return Good(snapshot.CapturePrepared(rig.owner, token, common, prepared, accepted, &candidate));
  }
  bool Commit() {
    return p::Good(rig.publication.CommitPhysical(rig.owner, token, common,
        {prepared.owner_id, prepared.kinematics.base_epoch, prepared.attempt, p::Qualification, true}));
  }
  void Discard() { rig.owner.Discard(); rig.publication.DiscardTrial(); snapshot.DiscardTrial(); }
};
inline std::vector<std::uint8_t> Read(const std::uint8_t* values, std::size_t count, cudaStream_t stream) {
  std::vector<std::uint8_t> out(count);
  EXPECT_EQ(cudaMemcpyAsync(out.data(), values, count, cudaMemcpyDeviceToHost, stream), cudaSuccess);
  EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess); return out;
}
inline void ComparePreparedReadbacks(Fixture& f, const fe::PhysicalActivityDeviceView& v) {
  std::vector<std::uint8_t> q(v.qeph.summary.count), t(v.t3.summary.count);
  ASSERT_TRUE(p::Good(f.rig.qeph.CopyPreparedParentActivity(f.rig.owner, f.token, f.common.qeph, q.data(), q.size())));
  ASSERT_TRUE(p::Good(f.rig.t3.CopyPreparedParentActivity(f.rig.owner, f.token, f.common.t3, t.data(), t.size())));
  EXPECT_EQ(Read(v.qeph.current, q.size(), v.stream), q);
  EXPECT_EQ(Read(v.t3.current, t.size(), v.stream), t);
}
void ArmCopyFailure();
std::size_t ActivityReadbackCount();
} // namespace physical_activity_test
