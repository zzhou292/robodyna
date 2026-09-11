#include "../Internal.h"
#include "../../tied_classification/tests/ActualClassification.h"
#include "lib_utest/qualification/tied_post_kinchk/NativeOracle.h"
#include <gtest/gtest.h>
#include <iostream>
namespace crash::cases::vehicle_startup::post_kinchk_actual_test {
namespace original = classification_actual_test;
const TiedSearchPostKinChk& Prepared() {
    static const auto value = [] {
        const auto forecast = TiedSearchPostKinChk::Forecast(original::Prepared());
        std::cout << "Post-KINCHK preflight " << forecast.total_host_bytes << " bytes\n";
        return TiedSearchPostKinChk::Prepare(original::Prepared());
    }();
    return value;
}
TEST(TiedPostKinChkActual, AllOriginalCinRowsMatchCompleteNativeKinetWithoutGlobalRegistry) {
    const auto& value = Prepared();
    const auto& classified = value.classification().data();
    const auto packed = post_kinchk_detail::Pack(classified,value.receipt());
    const auto native = ::kinchk_test::Native(packed.View(classified,value.receipt()));
    ASSERT_EQ(classified.slaves.size(),11165u);
    ASSERT_EQ(value.result().slaves().count,11165u);
    EXPECT_EQ(classified.cin_count,11165u);
    EXPECT_EQ(classified.penalty_count,0u);
    EXPECT_EQ(native.statistics[1],0);
    EXPECT_EQ(native.statistics[3],1);
    EXPECT_EQ(native.statistics[4],1);
    EXPECT_EQ(&value.classification().data(),&original::Prepared().data());
    for (std::size_t s = 0; s < classified.slaves.size(); ++s) {
        const auto& out = value.result().slaves().data[s];
        const auto& old = classified.slaves[s];
        const auto n = classified.slaves.size();
        EXPECT_EQ(out.before.source_id,old.source_node_id);
        EXPECT_EQ(out.before.irupt,old.irupt); // Native does not read this field.
        EXPECT_EQ(out.before.kinematics.conditions,native.five[s]);
        EXPECT_EQ(out.before.kinematics.translation,native.five[n+s]);
        EXPECT_EQ(out.before.kinematics.rotation,native.five[2*n+s]);
        EXPECT_EQ(out.before.kinematics.duplicate_conditions,native.five[3*n+s]);
        EXPECT_EQ(out.before.kinematics.incompatible_conditions,native.five[4*n+s]);
        EXPECT_EQ(out.kinet,native.kinet[s]);
        EXPECT_FALSE(out.repeated_condition);
        EXPECT_FALSE(out.mixed_incompatible_conditions);
    }
    EXPECT_EQ(native.decode,classified.interface_decode);
    EXPECT_EQ(value.receipt().replaced_wall_part,1001u);
    EXPECT_EQ(value.receipt().native_interface_ordinal,1u);
    EXPECT_EQ(value.phase(),TiedPostKinChkPhase::ObservedKinetAfterKinChk);
    RecordProperty("post_kinchk_forecast_bytes",std::to_string(value.forecast().total_host_bytes));
    RecordProperty("post_kinchk_native_owned_bytes",std::to_string(value.result().forecast().owned_payload_bytes));
    RecordProperty("observed_cin_count",std::to_string(classified.cin_count));
}
TEST(TiedPostKinChkActual, ExactBudgetImmutableLifetimeAndRetry) {
    const auto& saved = Prepared();
    auto copy = saved;
    auto moved = std::move(copy);
    EXPECT_EQ(&moved.result(),&copy.result());
    TiedPostKinChkLimits limits;
    limits.host_bytes = saved.forecast().total_host_bytes-1;
    EXPECT_THROW(TiedSearchPostKinChk::Prepare(original::Prepared(),limits),std::exception);
    ++limits.host_bytes;
    const auto retry = TiedSearchPostKinChk::Prepare(original::Prepared(),limits);
    EXPECT_EQ(retry.result().slaves().count,saved.result().slaves().count);
    EXPECT_EQ(retry.result().slaves().data[11164].kinet,saved.result().slaves().data[11164].kinet);
    limits.native.max_slaves = 11164;
    EXPECT_THROW(TiedSearchPostKinChk::Prepare(original::Prepared(),limits),std::exception);
}
}
