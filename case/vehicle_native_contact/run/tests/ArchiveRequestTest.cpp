#include "../ArchiveRequest.h"
#include "../Horizon.h"
#include "output/physical_run/tests/Support.h"
#include "output/physical_run/ReplayBudget.h"
#include "output/full_shell/FixedStepHorizon.h"
namespace crash::cases::vehicle_native_contact::test {
namespace {
namespace records = output::full_shell;
namespace physical = output::physical_run;
physical::Profile NativeGroup() {
    physical::Profile profile;
    profile.type45 = profile.structural_limit = profile.beam18 = profile.native_group = true;
    return profile;
}
vehicle_run::Horizon LongHorizon() {
    vehicle_run::Horizon horizon;
    horizon.fixed_dt_s = 1.5e-7;
    horizon.requested_duration_s = .03;
    EXPECT_TRUE(records::PlanFixedStepHorizon(horizon.fixed_dt_s,
        horizon.requested_duration_s, horizon.intervals));
    EXPECT_EQ(horizon.intervals, 200001u);
    return horizon;
}
records::Context VehicleRecordShape() {
    // Count-only allocation fixture. Original vehicle topology/materials are
    // authenticated separately by the actual source-bundle roundtrip gate.
    constexpr std::size_t Parents = 349645, Points = 956346;
    std::vector<records::ParentPoints> parents(Parents);
    for (std::size_t i = 0; i < Parents; ++i)
        parents[i] = {i + 1, 201, 2, 1, 2 + unsigned(i < Points - 2 * Parents),
            records::PlasticField::NativeEquivalentPlasticStrain};
    return records::Context::Create(records::test::Id(), 359785, parents.data(), parents.size(), 1.5e-7);
}
physical::Configuration Configuration(const records::Context& context,
    const records::source::BundleRequest& request) {
    return {context.identity(), NativeGroup(), request.archive, context.point_layout_sha256(), false, true};
}
}
TEST(NativeArchiveRequest, DefaultPreservesExistingEnvironmentRequestAndConfiguration) {
    const auto context = physical::test::Context();
    const vehicle_run::Horizon horizon{4, .5, .125, .5L};
    const auto old_request = physical::MakeEnvironmentRequest(context, 4, .5, 3, records::FullRunByteCap);
    const auto new_request = run_detail::MakeArchiveRequest(context, horizon, 3, records::FullRunByteCap);
    EXPECT_EQ(new_request.archive.file_byte_cap, 32u << 20);
    const auto old_document = physical::ConfigurationDocument(Configuration(context, old_request));
    const auto new_document = physical::ConfigurationDocument(Configuration(context, new_request));
    EXPECT_TRUE(old_document == new_document);
}
TEST(NativeArchiveRequest, InvalidByteLimitsRejectBeforeProducingAnArchiveRequest) {
    const auto context = physical::test::Context();
    const vehicle_run::Horizon horizon{4, .5, .125, .5L};
    for (const std::size_t bytes : {std::size_t{0}, records::IntervalCoreBytes - 1,
                                  output::kArtifactFileCap + 1, SIZE_MAX}) {
        SCOPED_TRACE(bytes);
        EXPECT_THROW(run_detail::MakeArchiveRequest(context, horizon, 3,
            records::FullRunByteCap, bytes), std::exception);
    }
}
TEST(NativeArchiveRequest, VehicleThirtyMillisecondShapeUsesBoundedNormalReplay) {
    const auto context = VehicleRecordShape();
    ASSERT_EQ(context.points(), 956346u);
    const auto horizon = LongHorizon();
    const auto old_request = run_detail::MakeArchiveRequest(context, horizon, 121, records::FullRunByteCap);
    const auto request = run_detail::MakeArchiveRequest(context, horizon, 121,
        records::FullRunByteCap, 24u << 20);
    const auto old_config = Configuration(context, old_request);
    const auto config = Configuration(context, request);
    try {
        (void)physical::Replay::Preflight(context, old_config);
        FAIL() << "The original long-run output must reproduce the actual pre-owner rejection";
    } catch (const std::runtime_error& error) {
        EXPECT_STREQ(error.what(), "Physical replay retained/peak buffers exceed host cap");
    }
    EXPECT_LE(physical::Replay::Preflight(context, config), 512u << 20);
    EXPECT_EQ(physical::IntervalReadStagingBytes(config.profile, horizon.intervals, 24u << 20), 69689376u);
    const auto plan = records::activity::PlanWithActivity(context, request.archive, "parent-activity.json");
    EXPECT_EQ(plan.archive.rows_per_chunk, 80659u);
    EXPECT_EQ(plan.archive.interval_chunks, 3u);
    EXPECT_EQ(plan.archive.forecast_bytes, 2257902792u);
    ASSERT_EQ(plan.archive.frame_epochs.size(), 121u);
    EXPECT_EQ(plan.archive.frame_epochs.front(), 0u);
    EXPECT_EQ(plan.archive.frame_epochs.back(), 200001u);
    EXPECT_EQ(plan.archive.frame_epochs,
        records::activity::PlanWithActivity(context, old_request.archive, "parent-activity.json").archive.frame_epochs);
    const auto decoded = physical::ReadConfiguration(physical::ConfigurationDocument(config));
    EXPECT_EQ(decoded.request.file_byte_cap, 24u << 20);
    EXPECT_EQ(decoded.request.intervals, horizon.intervals);
    EXPECT_EQ(output::Bits(decoded.request.fixed_dt), output::Bits(horizon.fixed_dt_s));
    EXPECT_TRUE(physical::SameProfile(decoded.profile, config.profile));
}
TEST(NativeArchiveRequest, HundredMillisecondVehicleFitsExistingArchiveAndReplayBounds) {
    const auto context = VehicleRecordShape();
    vehicle_run::Horizon horizon;
    horizon.fixed_dt_s = 1.5e-7;
    horizon.requested_duration_s = .1;
    ASSERT_TRUE(run_detail::PlanOutputHorizon(horizon.fixed_dt_s,
        horizon.requested_duration_s, horizon.intervals));
    ASSERT_EQ(horizon.intervals, 666667u);
    const auto request = run_detail::MakeArchiveRequest(context, horizon, 301,
        records::FullRunByteCap, 24u << 20);
    const auto config = Configuration(context, request);
    const auto plan = records::activity::PlanWithActivity(context, request.archive, "parent-activity.json");
    EXPECT_EQ(request.archive.total_byte_cap, UINT64_C(6442450944));
    EXPECT_EQ(plan.archive.forecast_bytes, UINT64_C(5345729304));
    EXPECT_LT(plan.archive.forecast_bytes, request.archive.total_byte_cap);
    EXPECT_EQ(plan.archive.rows_per_chunk, 80659u);
    EXPECT_EQ(plan.archive.interval_chunks, 9u);
    EXPECT_EQ(plan.archive.frame_capacity, 302u); // One off-cadence accepted-prefix reserve.
    ASSERT_EQ(plan.archive.frame_epochs.size(), 301u);
    EXPECT_EQ(plan.archive.frame_epochs.front(), 0u);
    EXPECT_EQ(plan.archive.frame_epochs.back(), horizon.intervals);
    for (std::size_t i = 1; i < plan.archive.frame_epochs.size(); ++i) {
        const auto gap = plan.archive.frame_epochs[i] - plan.archive.frame_epochs[i - 1];
        EXPECT_TRUE(gap == 2222u || gap == 2223u);
    }
    const auto thirty = LongHorizon();
    const auto thirty_request = run_detail::MakeArchiveRequest(context, thirty, 121,
        records::FullRunByteCap, 24u << 20);
    EXPECT_EQ(physical::IntervalReadStagingBytes(config.profile, horizon.intervals, 24u << 20), 69689376u);
    EXPECT_EQ(physical::Replay::Preflight(context, config),
        physical::Replay::Preflight(context, Configuration(context, thirty_request)));
    EXPECT_LE(physical::Replay::Preflight(context, config), 512u << 20);
    const auto decoded = physical::ReadConfiguration(physical::ConfigurationDocument(config));
    EXPECT_EQ(decoded.request.intervals, horizon.intervals);
    EXPECT_EQ(decoded.request.frames, 301u);
    EXPECT_EQ(output::Bits(decoded.request.requested_duration), output::Bits(.1));
    EXPECT_EQ(output::Bits(decoded.request.fixed_dt), output::Bits(horizon.fixed_dt_s));
    EXPECT_TRUE(physical::SameProfile(decoded.profile, config.profile));
    // Keeping the old 0.25ms visual cadence would exceed the unchanged 6GiB cap.
    EXPECT_THROW(run_detail::MakeArchiveRequest(context, horizon, 401,
        records::FullRunByteCap, 24u << 20), std::exception);
}
} // namespace crash::cases::vehicle_native_contact::test
