#include "../SampledShellPlasticity.h"
#include "output/physical_frames/tests/FieldsFixture.h"
#include "output/physical_frames/ArchiveState.h"
#include "output/full_shell/tests/FileWriteLimit.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <sstream>

namespace crash::cases::vehicle_run::test {
namespace records = output::full_shell;
namespace frames = output::physical_frames;
using Fixture = frames::test::Fixture;

namespace {
records::FrameStamp Stamp(std::uint64_t epoch, std::uint64_t attempt) {
    if (!epoch) return {};
    const double base = (epoch - 1) * .125;
    return {epoch, epoch - 1, attempt, base + .125, base, base + .0625, epoch == 1 ? .0625 : .125};
}

void ZeroPlasticHistory(Fixture& fixture) {
    fixture.q[1] = frames::test::f::ShellBatchLayeredSection::Plastic({});
    fixture.t[0] = frames::test::f::ShellBatchLayeredSection::OnePoint({});
    fixture.b[0] = {};
}

void Capture(Fixture& fixture, records::FrameStamp stamp = {}) {
    fixture.Stage();
    fixture.buffers.Finish(fixture.context, stamp);
}

const records::FrameRecord& Frame(const Fixture& fixture) {
    return fixture.buffers.frames[fixture.buffers.selected];
}

// The production Session calls RunArchive::Sample. Its component write path is
// exercised here without loading the complete original source bundle or GPU.
// Content/path checks and frame-before-activity ordering are the same operations
// used by physical_frames::Archive::Write.
void WritePair(const std::filesystem::path& directory, const char* stem,
    const Fixture& fixture, bool fail_activity = false) {
    const auto& frame = Frame(fixture);
    const auto& activity = *fixture.buffers.activity[fixture.buffers.selected];
    frames::detail::CheckPair(fixture.context, frame, activity);
    frames::detail::FrameDestinations(directory, stem);
    records::WriteFrame(directory, stem, fixture.context, records::test::View(frame));
    if (fail_activity) {
        records::test::FileSizeLimit limit(1);
        records::activity::WriteActivity(directory, stem, activity);
    } else {
        records::activity::WriteActivity(directory, stem, activity);
    }
}

void SamePublished(const SampledShellPlasticityTotals& actual, const SampledShellPlasticityTotals& expected) {
    EXPECT_EQ(actual.available, expected.available);
    EXPECT_EQ(actual.native_fields_available, expected.native_fields_available);
    EXPECT_EQ(actual.positive_sample_observed, expected.positive_sample_observed);
    EXPECT_EQ(actual.saved_samples, expected.saved_samples);
    EXPECT_EQ(actual.last_epoch, expected.last_epoch);
    EXPECT_EQ(actual.last_attempt, expected.last_attempt);
    EXPECT_EQ(output::Bits(actual.last_time_s), output::Bits(expected.last_time_s));
    EXPECT_EQ(actual.native_points, expected.native_points);
    EXPECT_EQ(actual.last_positive_points, expected.last_positive_points);
    EXPECT_EQ(output::Bits(actual.last_max_native_equivalent_plastic_strain),
        output::Bits(expected.last_max_native_equivalent_plastic_strain));
    EXPECT_EQ(output::Bits(actual.peak_saved_native_equivalent_plastic_strain),
        output::Bits(expected.peak_saved_native_equivalent_plastic_strain));
    EXPECT_EQ(actual.first_positive_saved_epoch, expected.first_positive_saved_epoch);
    EXPECT_EQ(output::Bits(actual.first_positive_saved_time_s), output::Bits(expected.first_positive_saved_time_s));
}
} // namespace

TEST(VehicleRunSampledShell, ActualZeroOneThreeFourPointFieldsIncludeInactiveHistoryAtSavedTimesOnly) {
    Fixture initial;
    ZeroPlasticHistory(initial);
    Capture(initial);
    auto totals = detail::SummarizeShellSample({}, initial.context, Frame(initial));
    EXPECT_TRUE(totals.available);
    EXPECT_TRUE(totals.native_fields_available);
    EXPECT_EQ(totals.native_points, 8u);
    EXPECT_EQ(totals.last_positive_points, 0u);
    EXPECT_FALSE(totals.positive_sample_observed);

    Fixture positive;
    Capture(positive, Stamp(7, 9));
    ASSERT_FALSE(positive.buffers.activity[positive.buffers.selected]->active(4));
    totals = detail::SummarizeShellSample(totals, positive.context, Frame(positive));
    EXPECT_EQ(totals.saved_samples, 2u);
    EXPECT_EQ(totals.last_epoch, 7u);
    EXPECT_EQ(totals.last_attempt, 9u);
    EXPECT_EQ(totals.last_time_s, .875);
    EXPECT_EQ(totals.last_positive_points, 8u);
    EXPECT_EQ(totals.last_max_native_equivalent_plastic_strain, .875);
    EXPECT_EQ(totals.first_positive_saved_epoch, 7u);
    EXPECT_EQ(totals.first_positive_saved_time_s, .875);

    // A later saved state may have lower values (for example a different seek
    // fixture); the observer reports what was saved, never imposes a force law.
    Capture(initial, Stamp(10, 14));
    totals = detail::SummarizeShellSample(totals, initial.context, Frame(initial));
    EXPECT_EQ(totals.saved_samples, 3u);
    EXPECT_EQ(totals.last_positive_points, 0u);
    EXPECT_EQ(totals.last_max_native_equivalent_plastic_strain, 0);
    EXPECT_EQ(totals.peak_saved_native_equivalent_plastic_strain, .875);
    EXPECT_EQ(totals.first_positive_saved_epoch, 7u);
}

TEST(VehicleRunSampledShell, PositiveInitialSampleAndUnavailableFieldsHaveDistinctRepresentations) {
    Fixture fixture;
    Capture(fixture);
    const auto initial = detail::SummarizeShellSample({}, fixture.context, Frame(fixture));
    EXPECT_TRUE(initial.positive_sample_observed);
    EXPECT_EQ(initial.first_positive_saved_epoch, 0u);
    EXPECT_EQ(initial.first_positive_saved_time_s, 0);

    const records::ParentPoints parent{105, 10, 2, frames::QephFamily, 0, records::PlasticField::NotApplicable};
    const auto context = records::Context::Create(records::test::Id(), 3, &parent, 1, .125);
    const records::FrameRecord frame{{}, std::vector<double>(9), {}};
    const auto absent = detail::SummarizeShellSample({}, context, frame);
    EXPECT_TRUE(absent.available);
    EXPECT_FALSE(absent.native_fields_available);
    EXPECT_EQ(absent.saved_samples, 1u);
    const auto document = detail::SampledShellPlasticityDocument(absent);
    EXPECT_FALSE(document["native_fields_available"].GetBool());
    EXPECT_FALSE(document.HasMember("last_saved_max_native_equivalent_plastic_strain"));
    EXPECT_FALSE(document.HasMember("first_positive_saved_epoch"));
}

TEST(VehicleRunSampledShell, InvalidLateFieldsAndStaleAcceptedPhaseRejectBeforeWriterAndRetry) {
    Fixture fixture;
    Capture(fixture);
    auto totals = detail::SummarizeShellSample({}, fixture.context, Frame(fixture));
    const auto before = totals;
    Capture(fixture, Stamp(2, 4));
    const auto valid = Frame(fixture);
    unsigned writes = 0;
    for (unsigned fault = 0; fault < 7; ++fault) {
        auto bad = valid;
        if (fault == 0) bad.plastic_points.back() = std::numeric_limits<double>::quiet_NaN();
        if (fault == 1) bad.plastic_points.back() = -.1;
        if (fault == 2) bad.position_xyz.back() = std::numeric_limits<double>::infinity();
        if (fault == 3) bad.plastic_points.pop_back();
        if (fault == 4) bad.stamp.kick_dt = .0625;
        if (fault == 5) bad.stamp = {};
        if (fault == 6) bad.stamp.attempt = 0;
        EXPECT_THROW(detail::SaveShellSample(totals, fixture.context, bad, [&] { ++writes; }), std::exception);
        SamePublished(totals, before);
    }
    EXPECT_EQ(writes, 0u);
    detail::SaveShellSample(totals, fixture.context, valid, [&] { ++writes; });
    EXPECT_EQ(writes, 1u);
    EXPECT_EQ(totals.last_epoch, 2u);
    const auto published = totals;
    EXPECT_THROW(detail::SaveShellSample(totals, fixture.context, valid, [&] { ++writes; }), std::exception);
    SamePublished(totals, published);
}

TEST(VehicleRunSampledShell, LateActivityWriteFailureKeepsPriorTotalsAndIncompleteFileEvidence) {
    records::test::Directory directory;
    Fixture fixture;
    ZeroPlasticHistory(fixture);
    Capture(fixture);
    SampledShellPlasticityTotals totals;
    detail::SaveShellSample(totals, fixture.context, Frame(fixture), [&] {
        EXPECT_FALSE(totals.available);
        WritePair(directory.path, "initial", fixture);
    });
    const auto before = totals;

    Fixture positive;
    Capture(positive, Stamp(2, 4));
    EXPECT_THROW(detail::SaveShellSample(totals, positive.context, Frame(positive), [&] {
        SamePublished(totals, before);
        WritePair(directory.path, "failed", positive, true);
    }), std::exception);
    SamePublished(totals, before);
    EXPECT_TRUE(std::filesystem::exists(directory.path / "failed.frame.json"));
    EXPECT_EQ(std::filesystem::file_size(directory.path / "failed.activity.bin"), 1u);
    EXPECT_FALSE(std::filesystem::exists(directory.path / "failed.activity.json"));

    // A partial archive is not retried. Only a new destination can supply a
    // separate successful component-write control; production stops the run.
    records::test::Directory fresh;
    detail::SaveShellSample(totals, positive.context, Frame(positive), [&] {
        WritePair(fresh.path, "saved", positive);
    });
    EXPECT_TRUE(totals.positive_sample_observed);
    EXPECT_EQ(totals.first_positive_saved_epoch, 2u);
}

TEST(VehicleRunSampledShell, MismatchedActivityAndLateOccupiedDestinationDoNotPublishOrWriteFrame) {
    records::test::Directory directory;
    Fixture fixture;
    Capture(fixture);
    SampledShellPlasticityTotals totals;
    const auto wrong_activity = *fixture.buffers.activity[fixture.buffers.selected];
    Capture(fixture, Stamp(2, 4));
    auto frame = Frame(fixture);
    // Establish epoch zero, then try a valid later frame with a stale activity.
    frame.stamp = {};
    totals = detail::SummarizeShellSample({}, fixture.context, frame);
    const auto initial = totals;
    EXPECT_THROW(detail::SaveShellSample(totals, fixture.context, Frame(fixture), [&] {
        frames::detail::CheckPair(fixture.context, Frame(fixture), wrong_activity);
    }), std::exception);
    SamePublished(totals, initial);
    output::WriteBytes(directory.path / "occupied.activity.json", "existing");
    EXPECT_THROW(detail::SaveShellSample(totals, fixture.context, Frame(fixture), [&] {
        WritePair(directory.path, "occupied", fixture);
    }), std::exception);
    SamePublished(totals, initial);
    EXPECT_FALSE(std::filesystem::exists(directory.path / "occupied.positions.bin"));
}

TEST(VehicleRunSampledShell, BoundedScalarDocumentAndProgressClearlyLabelSavedObservations) {
    Fixture fixture;
    Capture(fixture);
    const auto totals = detail::SummarizeShellSample({}, fixture.context, Frame(fixture));
    const auto document = detail::SampledShellPlasticityDocument(totals);
    EXPECT_STREQ(document["schema"].GetString(), "robo_dyna.sampled_shell_plasticity.v1");
    EXPECT_NE(std::string(document["scope"].GetString()).find("not continuous first yield"), std::string::npos);
    EXPECT_EQ(document["first_positive_saved_epoch"].GetUint64(), 0u);
    EXPECT_EQ(document["last_saved_positive_points"].GetUint64(), 8u);
    std::ostringstream stream;
    detail::WriteSampledShellPlasticityProgress(stream, totals);
    EXPECT_NE(stream.str().find("shell_plasticity_scope=saved_frames"), std::string::npos);
    EXPECT_NE(stream.str().find("first_positive_saved_shell_epoch=0"), std::string::npos);
    EXPECT_EQ(stream.str().find("first_yield"), std::string::npos);
    std::ostringstream unavailable;
    detail::WriteSampledShellPlasticityProgress(unavailable, {});
    EXPECT_EQ(unavailable.str().find("last_saved_shell_max"), std::string::npos);
    records::test::Directory directory;
    output::WriteJson(directory.path / "sample.json", document);
    EXPECT_LT(std::filesystem::file_size(directory.path / "sample.json"), 4096u);
    EXPECT_LE(8 * sizeof(totals), 2048u);
}
} // namespace crash::cases::vehicle_run::test
