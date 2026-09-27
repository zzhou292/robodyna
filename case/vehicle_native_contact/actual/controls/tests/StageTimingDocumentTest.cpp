#include "case/vehicle_run/StageTimingDocument.h"
#include "FrozenTimingDocument.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_run::test {
namespace {
std::string Json(const output::Document& document) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
    return {buffer.GetString(), buffer.GetSize()};
}
vehicle_dynamics::StepTimingSnapshot Populated(bool enabled) {
    vehicle_dynamics::StepTimingSnapshot snapshot;
    snapshot.enabled = enabled;
    snapshot.counter_saturated = true;
    snapshot.clock_failures = UINT64_MAX;
    snapshot.backward_samples = 37;
    for (std::size_t i = 0; i < snapshot.total.size(); ++i) {
        snapshot.total[i] = {UINT64_MAX - i, UINT64_MAX - 2*i, UINT64_MAX - 3*i,
                             UINT64_MAX - 4*i, UINT64_MAX - 5*i};
        snapshot.last_step[i] = {i+1, i%2, i, 1000+i, 500+i};
    }
    return snapshot;
}
}
TEST(VehicleStageTimingDocument, ExtractionPreservesTheEntireOriginalSerializedDocument) {
    for (bool enabled : {false, true}) {
        const auto snapshot = Populated(enabled);
        EXPECT_EQ(Json(detail::StageTimingDocument(snapshot)), Json(timing_frozen::Timings(snapshot)));
    }
    EXPECT_EQ(Json(detail::StageTimingDocument({})), Json(timing_frozen::Timings({})));
}
TEST(VehicleStageTimingDocument, EveryNamedCounterRetainsExactUnsignedPrecisionAndAttemptScope) {
    const auto snapshot = Populated(true);
    const auto document = detail::StageTimingDocument(snapshot);
    EXPECT_TRUE(document["enabled"].GetBool());
    EXPECT_TRUE(document["counter_saturated"].GetBool());
    EXPECT_EQ(document["clock_failures"].GetUint64(), UINT64_MAX);
    EXPECT_EQ(document["backward_samples"].GetUint64(), 37u);
    for (bool last : {false, true}) {
        const auto& rows = document[last ? "last_attempt" : "total"];
        const auto& counters = last ? snapshot.last_step : snapshot.total;
        ASSERT_EQ(rows.Size(), counters.size());
        for (std::size_t i = 0; i < counters.size(); ++i) {
            const auto& row = rows[static_cast<rapidjson::SizeType>(i)];
            EXPECT_STREQ(row["stage"].GetString(), vehicle_dynamics::StepStageNames[i]);
            EXPECT_EQ(row["calls"].GetUint64(), counters[i].calls);
            EXPECT_EQ(row["failures"].GetUint64(), counters[i].failures);
            EXPECT_EQ(row["valid_samples"].GetUint64(), counters[i].valid_samples);
            EXPECT_EQ(row["wall_ns"].GetUint64(), counters[i].wall_ns);
            EXPECT_EQ(row["maximum_ns"].GetUint64(), counters[i].maximum_ns);
        }
    }
}
TEST(VehicleStageTimingDocument, DisabledDefaultPublishesNoInventedSamplesOrElapsedTime) {
    const auto document = detail::StageTimingDocument({});
    EXPECT_FALSE(document["enabled"].GetBool());
    EXPECT_FALSE(document["counter_saturated"].GetBool());
    EXPECT_EQ(document["clock_failures"].GetUint64(), 0u);
    EXPECT_EQ(document["backward_samples"].GetUint64(), 0u);
    for (const auto* table : {"total", "last_attempt"})
        for (const auto& row : document[table].GetArray())
            for (const auto* key : {"calls", "failures", "valid_samples", "wall_ns", "maximum_ns"})
                EXPECT_EQ(row[key].GetUint64(), 0u);
}
} // namespace crash::cases::vehicle_run::test
