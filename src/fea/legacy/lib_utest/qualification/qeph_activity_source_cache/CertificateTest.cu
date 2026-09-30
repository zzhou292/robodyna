// SPDX-License-Identifier: MIT
#include "../qeph_mapped_activity/OwnerSupport.h"
#include "lib_src/elements/ShellBatchPlasticityStorage.h"

namespace qeph_activity_test {
namespace pd=fe::shell_batch_plasticity_detail;
TEST(QephSourceCertificateCuda,OwnedCatalogSurvivesCallerAndWarmShapeChecksStayLive) {
  pd::HostStorage owned;
  const fe::ShellBatchPlasticityBinding* catalog=nullptr;
  std::size_t count=0;
  {
    qt_mapped_test::Fixture source;
    count=source.physical.shells()->qeph_count();
    const auto initialized=owned.InitializeMappedCollection(source.physical,fe::ShellBindingFamily::Qeph,
        count,32u<<20,32u<<20,{});
    ASSERT_EQ(initialized.status,pd::SetupStatus::Success)<<initialized.message;
    catalog=owned.section_catalog();
    ASSERT_NE(catalog,nullptr);
    // The failure handle, not the caller's original catalog object, owns it.
    EXPECT_NE(catalog,&source.catalog);
    ASSERT_EQ(owned.CheckActivitySectionSources(0,count).status,pd::SetupStatus::Success);
    ASSERT_EQ(owned.CheckActivitySectionSources(1,count).status,pd::SetupStatus::Success);
    const auto rejected=owned.InitializeMappedCollection(source.physical,fe::ShellBindingFamily::Qeph,
        count,32u<<20,32u<<20,{});
    EXPECT_EQ(rejected.status,pd::SetupStatus::InvalidInput);
    EXPECT_EQ(owned.section_catalog(),catalog);
  }
  ASSERT_EQ(owned.CheckActivitySectionSources(0,count).status,pd::SetupStatus::Success);
  for (const auto bad: {owned.CheckActivitySectionSources(2,count),
                        owned.CheckActivitySectionSources(0,count-1)}) {
    EXPECT_EQ(bad.status,pd::SetupStatus::InvalidInput);
    EXPECT_STREQ(bad.message,"Mixed section readback shape is invalid");
  }
  ASSERT_EQ(owned.CheckActivitySectionSources(1,count).status,pd::SetupStatus::Success);
  pd::HostStorage independent;
  const auto unavailable=independent.CheckActivitySectionSources(0,count);
  EXPECT_EQ(unavailable.status,pd::SetupStatus::InvalidInput);
  EXPECT_STREQ(unavailable.message,"No explicit mixed section history");
  qt_mapped_test::Fixture second_source;
  ASSERT_EQ(independent.InitializeMappedCollection(second_source.physical,fe::ShellBindingFamily::Qeph,
      count,32u<<20,32u<<20,{}).status,pd::SetupStatus::Success);
  EXPECT_NE(independent.section_catalog(),catalog);
  ASSERT_EQ(independent.CheckActivitySectionSources(0,count).status,pd::SetupStatus::Success);
}

TEST(QephSourceCertificateCuda,WarmCertificateStillRejectsEachFreshPhaseAtSameEpochAndRetries) {
  for (std::size_t phase: {1u,2u,3u}) {
    Rig rig;
    ASSERT_TRUE(rig.Initialize());
    const auto count=rig.fixture.physical.shells()->qeph_count();
    const auto stamp=rig.owner.accepted();
    std::vector<std::uint8_t> flags(count,19);
    q::BatchDiagnostics diagnostics;
    ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),count,&diagnostics).status,
        q::BatchStatus::Success);
    const auto expected=flags;
    flags.assign(count,19);
    const auto unchanged=qt_mapped_test::Bytes(diagnostics);
    Watch(count);
    transfers.corrupt_compact_call=phase;
    transfers.corrupt_flag=255;
    const auto rejected=rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),count,&diagnostics);
    EXPECT_NE(rejected.status,q::BatchStatus::Success);
    EXPECT_EQ(transfers.calls,phase);
    EXPECT_EQ(transfers.compact_calls,phase);
    transfers.enabled=false;
    EXPECT_EQ(flags,std::vector<std::uint8_t>(count,19));
    EXPECT_EQ(qt_mapped_test::Bytes(diagnostics),unchanged);
    EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
    Watch(count);
    ASSERT_EQ(rig.qeph.CopyAcceptedParentActivity(stamp,flags.data(),count,&diagnostics).status,
        q::BatchStatus::Success);
    CheckTransfer();
    transfers.enabled=false;
    EXPECT_EQ(flags,expected);
  }
}
} // namespace qeph_activity_test
