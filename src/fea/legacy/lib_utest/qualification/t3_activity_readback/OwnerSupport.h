// SPDX-License-Identifier: MIT
#pragma once
#include "SerialFlow.h"
#include "Transfers.h"
#include "../physical_publication/OwnerFixture.h"
#include <array>
#include <cstring>
#include <limits>

namespace t3_readback_test {
namespace fe = tl::fea;
namespace t3 = fe::t3;
using Rig = physical_publication_test::Rig;
template<class T> std::array<unsigned char, sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  return bytes;
}
struct ObservedHistories {
  const fe::ShellBatchPlasticityBinding* catalog = nullptr;
  std::vector<fe::ShellBatchLayeredSection> sections;
  std::vector<fe::ShellBatchFailureState> failures;
  bool one_point_sections() const { return true; }
  const auto* section_catalog() const { return catalog; }
  const auto* section_staging() const { return sections.data(); }
  const auto* failure_staging() const { return failures.data(); }
  auto ReadSections(unsigned, std::size_t, cudaStream_t, double) {
    return fe::shell_batch_plasticity_detail::SetupReport{
        fe::shell_batch_plasticity_detail::SetupStatus::Success, "OK"};
  }
};
// Complete frozen validation call flow, fed observations from fresh public
// typed APIs. The transport seam never claims to be another owner or cache.
struct Oracle {
  const fe::ShellPhysicalBinding* physical = nullptr;
  const fe::ShellBatchBinding* joined_binding = nullptr;
  fe::NodalStamp accepted_stamp;
  t3::T3BatchConfig config;
  struct Storage { int slab[2]{}; } header;
  Storage* storage = &header;
  cudaStream_t stream = nullptr;
  std::vector<t3::ForceTrial> staging;
  ObservedHistories histories;
  ObservedHistories* plasticity = &histories;
  unsigned AcceptedSlabIndex() const { return 0; }
  t3::BatchReport PendingError() { return {t3::BatchStatus::Success, "OK"}; }
  t3::BatchReport Runtime(cudaError_t, const char* message) { return {t3::BatchStatus::DeviceFailure, message}; }
  t3::BatchReport ReadResults(const int* slab) {
    return serial::ValidateMappedResults(*this, slab == &header.slab[0] ? 0 : 1);
  }
  t3::BatchReport ValidateMappedSections(unsigned slab) { return serial::ValidateMappedSections(*this, slab); }
  t3::BatchReport ValidateOnePointReadback(unsigned slab, double time, std::uint64_t epoch) {
    return serial::OnePoint(*this, slab, time, epoch);
  }
};
inline void ReadOracle(Rig& rig, Oracle& oracle, const t3::BatchDiagnostics* prepared = nullptr) {
  transfers.enabled = false;
  oracle.physical = &rig.fixture.physical;
  oracle.joined_binding = rig.fixture.physical.shells();
  oracle.accepted_stamp = rig.owner.accepted();
  oracle.config.owner = oracle.accepted_stamp;
  oracle.config.element_count = oracle.joined_binding->t3_count();
  const auto n = oracle.config.element_count;
  oracle.staging.resize(n);
  oracle.histories.sections.resize(n);
  oracle.histories.failures.resize(n);
  oracle.histories.catalog = oracle.physical->catalog();
  t3::BatchDiagnostics diagnostics;
  if (prepared) {
    ASSERT_EQ(rig.t3.CopyPreparedResults(*prepared, oracle.staging.data(), n).status, t3::BatchStatus::Success);
    ASSERT_EQ(rig.t3.CopyPreparedLayeredSectionHistory(*prepared, oracle.histories.sections.data(), n).status,
        t3::BatchStatus::Success);
  } else {
    ASSERT_EQ(rig.t3.CopyAcceptedResults(oracle.accepted_stamp, oracle.staging.data(), n, &diagnostics).status,
        t3::BatchStatus::Success);
    ASSERT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(oracle.accepted_stamp,
        oracle.histories.sections.data(), n, &diagnostics).status, t3::BatchStatus::Success);
  }
  // This fixture has true NIP1 only; the unavailable reserved NIP3 sidecar
  // is not part of any oracle activity or point-history observation.
  ASSERT_EQ(n, 1u);
  ASSERT_NE(oracle.histories.sections[0].one_point(), nullptr);
}
inline void SameReport(const t3::BatchReport& actual, const t3::BatchReport& expected) {
  EXPECT_EQ(actual.status, expected.status);
  EXPECT_EQ(actual.element, expected.element);
  EXPECT_EQ(actual.node, expected.node);
  EXPECT_EQ(actual.element_status, expected.element_status);
  EXPECT_STREQ(actual.message, expected.message);
}
inline void CheckTransfers() {
  const auto n = transfers.parents;
  EXPECT_EQ(transfers.copies, 5u);
  EXPECT_EQ(transfers.syncs, 4u);
  EXPECT_EQ(transfers.force_copies, 1u);
  EXPECT_EQ(transfers.bytes, n * (sizeof(fe::ShellBatchOnePointSectionState) +
      sizeof(fe::ShellBatchSectionState) + sizeof(fe::sections::ShellLayeredLaw1History) +
      sizeof(fe::ShellBatchFailureState) + sizeof(t3::ForceTrial)));
}
inline void Discard(Rig& rig) { rig.owner.Discard(); rig.publication.DiscardTrial(); }
inline void Commit(Rig& rig, const fe::NodalTrialToken& token, const fe::NodalPreparedView& view,
    const fe::ShellPhysicalDiagnostics& candidate) {
  ASSERT_EQ(rig.publication.CommitPhysical(rig.owner, token, candidate,
      {view.owner_id, view.kinematics.base_epoch, view.attempt, physical_publication_test::Qualification, true}).status,
      fe::ShellPublicationStatus::Success);
}
} // namespace t3_readback_test
