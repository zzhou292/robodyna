#include "Fixture.h"
namespace crash::cases::vehicle_startup::post_kinchk_test {
TEST(TiedPostKinChkValues, ExactSourceRowsSurviveAndInterfaceLineIsNotNativeOrdinal) {
    Fixture f;
    const auto packed = post_kinchk_detail::Pack(f.classified,f.receipt);
    native_search::PostKinChkResult out;
    ASSERT_TRUE(native_search::PostKinChk(packed.View(f.classified,f.receipt),&out));
    EXPECT_EQ(f.receipt.native_interface_ordinal,1u);
    EXPECT_EQ(out.source_interface_id(),f.declaration.sources[f.declaration.contact_source].block.first_line);
    for (std::size_t s = 0; s < f.classified.slaves.size(); ++s) {
        EXPECT_EQ(out.slaves().data[s].before.source_id,f.classified.slaves[s].source_node_id);
        EXPECT_EQ(out.slaves().data[s].before.irupt,f.classified.slaves[s].irupt);
        EXPECT_EQ(out.slaves().data[s].kinet,f.classified.slaves[s].kinematics.conditions);
    }
}
TEST(TiedPostKinChkValues, LateRowsWallReceiptUnknownRoleAndPhaseRejectWithoutPublication) {
    Fixture f;
    const auto packed = post_kinchk_detail::Pack(f.classified,f.receipt);
    native_search::PostKinChkResult saved;
    ASSERT_TRUE(native_search::PostKinChk(packed.View(f.classified,f.receipt),&saved));
    const auto* prior = saved.slaves().data;
    auto bad = f.classified;
    bad.slaves.back().original_nsv_row = f.receipt.observed_slaves;
    EXPECT_THROW(post_kinchk_detail::Pack(bad,f.receipt),std::exception);
    bad = f.classified;
    bad.slaves.back().original_nsv_row = bad.slaves.front().original_nsv_row;
    EXPECT_THROW(post_kinchk_detail::Pack(bad,f.receipt),std::exception);
    bad = f.classified;
    bad.phase = native_search::ClassificationPhase::Empty;
    EXPECT_THROW(post_kinchk_detail::Pack(bad,f.receipt),std::exception);
    auto context = f.context;
    context.wall.node_ids.clear();
    EXPECT_THROW(post_kinchk_detail::Receipt(f.canonical,f.declaration,f.Receipt(),context),std::exception);
    context = f.context;
    context.roles.back().role = static_cast<tied::ClassificationSourceRole>(999);
    EXPECT_THROW(post_kinchk_detail::Receipt(f.canonical,f.declaration,f.Receipt(),context),std::exception);
    auto receipt = f.receipt;
    receipt.rbe2_roles = 1;
    EXPECT_THROW(post_kinchk_detail::Pack(f.classified,receipt),std::exception);
    EXPECT_EQ(saved.slaves().data,prior);
    EXPECT_NO_THROW(post_kinchk_detail::Pack(f.classified,f.receipt));
}
TEST(TiedPostKinChkValues, ForecastReusesRetainedSourceOnceAndExcludesRetiredNativeScratch) {
    TiedClassificationForecast before;
    before.source_context_reservation_bytes = 408609922;
    before.distinct_finalized_payload_bytes = 20000000;
    before.result_payload_bytes = 400000;
    before.input_staging_bytes = 30000000;
    before.native_reservation_bytes = 67108864;
    const auto out = post_kinchk_detail::Budget(before,11165,4096,{});
    EXPECT_EQ(out.retained_classification_reservation_bytes,429009922u);
    TiedPostKinChkLimits limits;
    limits.host_bytes = out.total_host_bytes-1;
    EXPECT_THROW(post_kinchk_detail::Budget(before,11165,4096,limits),std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(post_kinchk_detail::Budget(before,11165,4096,limits).total_host_bytes,limits.host_bytes);
}
}
