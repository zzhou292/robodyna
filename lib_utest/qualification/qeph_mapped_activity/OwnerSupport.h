// SPDX-License-Identifier: MIT
#pragma once
#include "Oracle.h"
#include "Transfers.h"
#include "../physical_publication/OwnerFixture.h"

namespace qeph_activity_test {
using Rig = physical_publication_test::Rig;
inline void ReadOracle(Rig& rig, Oracle& oracle, const q::BatchDiagnostics* prepared = nullptr) {
  oracle.physical = &rig.fixture.physical;
  oracle.accepted_stamp = rig.owner.accepted();
  oracle.config.owner = oracle.accepted_stamp;
  oracle.config.element_count = rig.fixture.physical.shells()->qeph_count();
  const auto n = oracle.config.element_count;
  oracle.staging.resize(n);
  oracle.histories.sections.resize(n);
  oracle.histories.failures.resize(n);
  q::BatchDiagnostics diagnostics;
  Watch(n);
  if (prepared) {
    ASSERT_EQ(rig.qeph.CopyPreparedResults(*prepared,oracle.staging.data(),n).status,q::BatchStatus::Success);
    ASSERT_EQ(rig.qeph.CopyPreparedLayeredSectionHistory(*prepared,oracle.histories.sections.data(),n).status,
        q::BatchStatus::Success);
    ASSERT_EQ(rig.qeph.CopyPreparedFailureHistory(*prepared,oracle.histories.failures.data(),n).status,
        q::BatchStatus::Success);
  } else {
    ASSERT_EQ(rig.qeph.CopyAcceptedResults(oracle.accepted_stamp,oracle.staging.data(),n,&diagnostics).status,
        q::BatchStatus::Success);
    ASSERT_EQ(rig.qeph.CopyAcceptedLayeredSectionHistory(oracle.accepted_stamp,
        oracle.histories.sections.data(),n,&diagnostics).status,q::BatchStatus::Success);
    ASSERT_EQ(rig.qeph.CopyAcceptedFailureHistory(oracle.accepted_stamp,
        oracle.histories.failures.data(),n,&diagnostics).status,q::BatchStatus::Success);
  }
  transfers.enabled = false;
}
inline void CheckTransfer() {
  EXPECT_EQ(transfers.force_calls,0u);
  EXPECT_EQ(transfers.compact_calls,2u);
  EXPECT_EQ(transfers.failure_calls,0u);
  EXPECT_EQ(transfers.calls,4u);
  const auto n = transfers.parents;
  EXPECT_EQ(transfers.bytes,n*(sizeof(fe::ShellBatchSectionState)+
      sizeof(fe::sections::ShellLayeredLaw1History)) + 2*m::ActivityBytes(n));
}
inline void CheckFlags(const std::vector<std::uint8_t>& flags,const Oracle& oracle) {
  ASSERT_EQ(flags.size(),oracle.histories.failures.size());
  for (std::size_t parent = 0; parent < flags.size(); ++parent) {
    EXPECT_EQ(flags[parent],oracle.histories.failures[parent].active ? 1 : 0);
  }
}
inline bool Commit(Rig& rig,const fe::NodalTrialToken& token,const fe::NodalPreparedView& view,
    const fe::ShellPhysicalDiagnostics& candidate) {
  return physical_publication_test::Good(rig.publication.CommitPhysical(rig.owner,token,candidate,
      {view.owner_id,view.kinematics.base_epoch,view.attempt,physical_publication_test::Qualification,true}));
}
inline void Discard(Rig& rig) {
  rig.owner.Discard();
  rig.publication.DiscardTrial();
}
} // namespace qeph_activity_test
