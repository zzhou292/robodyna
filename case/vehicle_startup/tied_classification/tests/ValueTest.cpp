#include "../Internal.h"
#include "../../tied_finalization/tests/Fixture.h"

namespace crash::cases::vehicle_startup::classification_test {
TEST(TiedClassificationValues, CompactedRowsKeepSourceIdentityAndObservedNativeFields) {
    finalization_test::Fixture fixture;
    fixture.result.rows.back().choice.projection.s = 1.6;
    const auto packed = fixture.Inputs();
    native_search::FinalizedSearch finalized;
    ASSERT_TRUE(native_search::FinalizeSearch(packed.View(fixture.geometry),&finalized));
    auto input = tied_classification_detail::Pack(fixture.canonical,fixture.declaration,
        fixture.geometry,*finalized.data(),fixture.Receipt());
    ASSERT_EQ(input.slaves.size()+1,fixture.declaration.slave_nodes.size());
    for (std::size_t s = 0; s < input.slaves.size(); ++s)
        EXPECT_EQ(input.nodes[input.slaves[s]].source_id,fixture.declaration.slave_nodes[finalized.data()->slaves[s]].id);
    native_search::ClassificationResult result;
    ASSERT_TRUE(native_search::Classify(input.View(),&result));
    const auto observed = tied_classification_detail::Observed(result,fixture.declaration,*finalized.data());
    EXPECT_EQ(observed.slaves.size(),input.slaves.size());
    EXPECT_EQ(observed.cin_count+observed.penalty_count,observed.slaves.size());
    for (std::size_t s = 0; s < observed.slaves.size(); ++s) {
        EXPECT_EQ(observed.slaves[s].original_nsv_row,finalized.data()->slaves[s]);
        EXPECT_EQ(observed.slaves[s].irupt,result.irupt().data[s]);
        EXPECT_EQ(observed.slaves[s].kinematics.conditions,result.nodes().data[input.slaves[s]].kinematics.conditions);
    }
    for (std::size_t code = 0; code < 8192; ++code)
        EXPECT_EQ(observed.interface_decode[code],result.interface_decode().data[code]);
    EXPECT_EQ(observed.phase,native_search::ClassificationPhase::InterfaceTaggedBeforeKinChk);
}
TEST(TiedClassificationValues, LastAssociationAndPhaseFailuresPreservePreviousNativeResultAndRetry) {
    finalization_test::Fixture fixture;
    const auto packed = fixture.Inputs();
    native_search::FinalizedSearch finalized;
    ASSERT_TRUE(native_search::FinalizeSearch(packed.View(fixture.geometry),&finalized));
    auto good = tied_classification_detail::Pack(fixture.canonical,fixture.declaration,
        fixture.geometry,*finalized.data(),fixture.Receipt());
    native_search::ClassificationResult saved;
    ASSERT_TRUE(native_search::Classify(good.View(),&saved));
    const auto* prior = saved.nodes().data;
    auto bad = fixture.geometry;
    bad.secondary_working_nodes.back() = UINT32_MAX;
    EXPECT_THROW(tied_classification_detail::Pack(fixture.canonical,fixture.declaration,
        bad,*finalized.data(),fixture.Receipt()),std::exception);
    auto phase = fixture.Receipt();
    phase.prior_connection_count = 1;
    EXPECT_THROW(tied_classification_detail::Pack(fixture.canonical,fixture.declaration,
        fixture.geometry,*finalized.data(),phase),std::exception);
    native_search::ClassificationLimits cap;
    cap.max_host_bytes = 1;
    EXPECT_FALSE(native_search::Classify(good.View(),&saved,cap));
    EXPECT_EQ(saved.nodes().data,prior);
    auto retry = tied_classification_detail::Pack(fixture.canonical,fixture.declaration,
        fixture.geometry,*finalized.data(),fixture.Receipt());
    EXPECT_EQ(retry.slaves,good.slaves);
    EXPECT_EQ(retry.masters,good.masters);
    ASSERT_TRUE(native_search::Classify(retry.View(),&saved));
}
}
