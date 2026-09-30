#include "ElasticCouponArtifacts.h"
#include "ElasticCouponFields.h"
#include "output/ArtifactIO.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
using namespace crash::case_data;
namespace out = crash::output;
namespace fs = std::filesystem;
namespace ref = crash::reference;
namespace shell = tl::fea::reissner;

class TempRoot {
  public:
    TempRoot() {
        const auto pattern = (fs::temp_directory_path() / "elastic-coupon-artifacts-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(), pattern.end()); buffer.push_back(0);
        const char* made = ::mkdtemp(buffer.data());
        if (!made) throw std::runtime_error("Could not create isolated artifact test directory");
        path = made;
    }
    ~TempRoot() { std::error_code error; fs::remove_all(path, error); }
    fs::path path;
};
class ElasticCouponArtifactsCheck : public ::testing::Test {
  protected:
    void SetUp() override {
        int count = 0; ASSERT_EQ(cudaGetDeviceCount(&count), cudaSuccess); ASSERT_GT(count, 0);
    }
};
rapidjson::Document Json(const fs::path& path) {
    const auto bytes = out::ReadBounded(path, 1024 * 1024);
    rapidjson::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.c_str());
    out::Require(!doc.HasParseError() && doc.IsObject(), "Invalid test JSON artifact");
    return doc;
}
std::vector<std::string> Split(const std::string& line) {
    std::istringstream stream(line); std::vector<std::string> result; std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    return result;
}
unsigned FrameEvery(const ElasticCouponCase& run) {
    // The writer is intentionally left incomplete after a short prefix; allow
    // startup/final output within its declared full-horizon frame budget.
    return static_cast<unsigned>(run.metrics()->required_steps);
}
template<std::size_t N>
void ExpectArrayBits(const rapidjson::Value& value, const std::array<double, N>& expected) {
    ASSERT_TRUE(value.IsArray()); ASSERT_EQ(value.Size(), N);
    for (std::size_t i = 0; i < N; ++i) EXPECT_EQ(out::Bits(value[i].GetDouble()), out::Bits(expected[i]));
}
void ExpectFields(const rapidjson::Document& doc, const ElasticCouponFrame& frame) {
    EXPECT_STREQ(doc["schema"].GetString(), "robo_dyna.elastic_coupon_fields.v1");
    EXPECT_EQ(doc["owner_id"].GetUint64(), frame.stamp.owner_id);
    EXPECT_EQ(doc["accepted_epoch"].GetUint64(), frame.stamp.epoch);
    EXPECT_EQ(out::Bits(doc["accepted_time_s"].GetDouble()), out::Bits(frame.stamp.time));
    EXPECT_EQ(doc["reaction_base_epoch"].GetUint64(), frame.stamp.reaction_base_epoch);
    EXPECT_EQ(out::Bits(doc["reaction_time_s"].GetDouble()), out::Bits(frame.stamp.reaction_time));
    EXPECT_EQ(doc["reactions_valid"].GetBool(), frame.stamp.reactions_valid);
    EXPECT_EQ(doc["element_evaluation_base_epoch"].GetUint64(), frame.element_association.base_epoch);
    EXPECT_EQ(doc["element_evaluation_attempt"].GetUint64(), frame.element_association.attempt);
    EXPECT_EQ(doc["qualification_id"].GetUint64(), frame.element_association.configuration_id);
    ExpectArrayBits(doc["position_xyz_m"], frame.position);
    ExpectArrayBits(doc["orientation_wxyz"], frame.rotation);
    ExpectArrayBits(doc["velocity_xyz_m_per_s"], frame.velocity);
    ExpectArrayBits(doc["omega_world_xyz_rad_per_s"], frame.omega);
    ExpectArrayBits(doc["reaction_force_xyz_N_at_base"], frame.reaction_force);
    ExpectArrayBits(doc["reaction_couple_world_xyz_Nm_at_base"], frame.reaction_couple);
    ASSERT_EQ(doc["elements"].Size(), ref::kCouponElements);
    for (std::size_t e = 0; e < ref::kCouponElements; ++e) {
        const auto& actual = doc["elements"][e]; const auto& expected = frame.element[e];
        EXPECT_EQ(actual["parent_element"].GetUint(), e + 1);
        EXPECT_EQ(out::Bits(actual["elastic_energy_J"].GetDouble()), out::Bits(expected.energy));
        EXPECT_EQ(out::Bits(actual["bending_energy_J"].GetDouble()), out::Bits(expected.bending_energy));
        for (std::size_t n = 0; n < 4; ++n) {
            for (std::size_t c = 0; c < 3; ++c) {
                EXPECT_EQ(out::Bits(actual["force_world_N"][n][c].GetDouble()),
                          out::Bits(shell::detail::Component(expected.force[n], c)));
                EXPECT_EQ(out::Bits(actual["couple_world_Nm"][n][c].GetDouble()),
                          out::Bits(shell::detail::Component(expected.couple[n], c)));
            }
            for (std::size_t c = 0; c < 12; ++c) {
                EXPECT_EQ(out::Bits(actual["gauss_strain"][n][c].GetDouble()), out::Bits(expected.strain[n][c]));
                EXPECT_EQ(out::Bits(actual["gauss_resultant"][n][c].GetDouble()), out::Bits(expected.resultant[n][c]));
            }
        }
    }
}

TEST_F(ElasticCouponArtifactsCheck, ShortActualRunRoundtripsFieldsMeshAndSourceBinding) {
    ElasticCouponCase run;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    TempRoot root; const auto directory = root.path / "prefix";
    ElasticCouponArtifacts writer(directory.string(), run, FrameEvery(run));
    ElasticCouponFrame initial, accepted;
    report = run.Capture(initial); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ASSERT_NO_THROW(writer.WriteFrame(run));
    const auto base = run.metrics()->stamp;
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ASSERT_NO_THROW(writer.RecordInterval(base, *run.metrics()));
    report = run.Capture(accepted); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ASSERT_NO_THROW(writer.WriteFrame(run));
    const auto configuration = Json(directory / "configuration.json");
    ASSERT_EQ(configuration["vertex_binding"].Size(), 6u);
    ASSERT_EQ(configuration["triangle_binding"].Size(), 4u);
    EXPECT_EQ(configuration["required_steps"].GetUint64(), run.metrics()->required_steps);
    EXPECT_EQ(out::Bits(configuration["fixed_dt_s"].GetDouble()), out::Bits(accepted.stamp.fixed_dt));
    for (unsigned n = 0; n < 6; ++n) {
        const auto& binding = configuration["vertex_binding"][n]; ASSERT_EQ(binding.Size(), 4u);
        EXPECT_EQ(binding[0u].GetUint64(), n); EXPECT_EQ(binding[1u].GetUint64(), 1u);
        EXPECT_EQ(binding[2u].GetUint64(), 1u); EXPECT_EQ(binding[3u].GetUint64(), n + 1);
    }
    for (unsigned triangle = 0; triangle < 4; ++triangle) {
        const auto& binding = configuration["triangle_binding"][triangle]; ASSERT_EQ(binding.Size(), 9u);
        EXPECT_EQ(binding[5u].GetUint64(), triangle / 2 + 1);
        EXPECT_EQ(binding[6u].GetUint64(), 1u); EXPECT_EQ(binding[7u].GetUint64(), 0u);
        EXPECT_EQ(binding[8u].GetUint64(), triangle % 2);
    }
    for (unsigned epoch = 0; epoch < 2; ++epoch) {
        const auto& expected = epoch ? accepted : initial;
        const std::string stem = epoch ? "accepted-000001" : "accepted-000000";
        const auto fields = Json(directory / (stem + ".fields.json")); ExpectFields(fields, expected);
        EXPECT_STREQ(fields["element_evaluation_phase"].GetString(),
                     epoch ? "prepared_candidate_subsequently_committed" : "accepted_base");
        chrono::ChTriangleMeshConnected mesh;
        std::ifstream file(directory / (stem + ".mesh.json")); chrono::ChArchiveInJSON archive(file, true);
        archive >> chrono::make_ChNameValue("mesh", mesh);
        ASSERT_EQ(mesh.GetCoordsVertices().size(), 6u); ASSERT_EQ(mesh.GetIndicesVertices().size(), 4u);
        for (unsigned n = 0; n < 6; ++n)
            for (unsigned c = 0; c < 3; ++c)
                EXPECT_EQ(out::Bits(mesh.GetCoordsVertices()[n][c]), out::Bits(expected.position[3*n+c]));
        for (unsigned triangle = 0; triangle < 4; ++triangle)
            for (unsigned c = 0; c < 3; ++c)
                EXPECT_EQ(mesh.GetIndicesVertices()[triangle][c], configuration["triangle_binding"][triangle][c].GetUint64());
        const auto obj = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile((directory / (stem + ".obj")).string(), false, false);
        ASSERT_TRUE(obj); EXPECT_EQ(obj->GetNumVertices(), 6u); EXPECT_EQ(obj->GetNumTriangles(), 4u);
    }
    std::istringstream csv(out::ReadBounded(directory / "accepted-intervals.csv", 1024 * 1024));
    std::string line; ASSERT_TRUE(bool(std::getline(csv, line))); const auto headers = Split(line);
    std::map<std::string, std::size_t> column;
    for (std::size_t i = 0; i < headers.size(); ++i) column.emplace(headers[i], i);
    ASSERT_TRUE(bool(std::getline(csv, line))); const auto row = Split(line); ASSERT_EQ(row.size(), headers.size());
    EXPECT_EQ(std::stoull(row.at(column.at("base_epoch"))), 0u);
    EXPECT_EQ(std::stoull(row.at(column.at("accepted_epoch"))), 1u);
    EXPECT_EQ(out::Bits(std::stod(row.at(column.at("force_eval_time_s")))), out::Bits(base.time));
    EXPECT_EQ(out::Bits(std::stod(row.at(column.at("accepted_time_s")))), out::Bits(accepted.stamp.time));
    EXPECT_EQ(out::Bits(std::stod(row.at(column.at("elastic_energy_J")))), out::Bits(accepted.metrics.diagnostics.elastic_energy));
    EXPECT_FALSE(bool(std::getline(csv, line)));
    EXPECT_FALSE(fs::exists(directory / "manifest.json")); EXPECT_FALSE(fs::exists(directory / "final-metrics.json"));
}

TEST_F(ElasticCouponArtifactsCheck, IncompleteFinishAndExplicitFailureNeverClaimCompletion) {
    ElasticCouponCase run;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    TempRoot root; const auto directory = root.path / "failed";
    ElasticCouponArtifacts writer(directory.string(), run, FrameEvery(run));
    writer.WriteFrame(run); const auto base = run.metrics()->stamp;
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    writer.RecordInterval(base, *run.metrics()); writer.WriteFrame(run);
    EXPECT_THROW(writer.Finish(run, .1), std::runtime_error);
    EXPECT_FALSE(fs::exists(directory / "manifest.json")); EXPECT_FALSE(fs::exists(directory / "final-metrics.json"));
    writer.Fail("Intentional short-prefix stop"); const auto failure = Json(directory / "failure.json");
    EXPECT_STREQ(failure["status"].GetString(), "failed");
    EXPECT_STREQ(failure["message"].GetString(), "Intentional short-prefix stop");
    EXPECT_EQ(failure["last_recorded_epoch"].GetUint64(), 1u);
    const auto retained = out::ReadBounded(directory / "failure.json", 1024 * 1024);
    EXPECT_THROW(writer.RecordInterval(base, *run.metrics()), std::runtime_error);
    EXPECT_THROW(writer.WriteFrame(run), std::runtime_error);
    EXPECT_THROW(writer.Finish(run, .1), std::runtime_error);
    writer.Fail("A later message cannot replace the original failure");
    EXPECT_EQ(out::ReadBounded(directory / "failure.json", 1024 * 1024), retained);
    EXPECT_FALSE(fs::exists(directory / "manifest.json")); EXPECT_EQ(run.metrics()->stamp.epoch, 1u);
}

TEST_F(ElasticCouponArtifactsCheck, ForeignStaleSkippedIntervalsAndDuplicateFramesAreRejected) {
    ElasticCouponCase run, foreign;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = foreign.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    TempRoot root; const auto directory = root.path / "association";
    ElasticCouponArtifacts writer(directory.string(), run, FrameEvery(run));
    writer.WriteFrame(run);
    const auto before = out::ReadBounded(directory / "accepted-frames.csv", 1024 * 1024);
    EXPECT_THROW(writer.WriteFrame(run), std::runtime_error);
    EXPECT_THROW(writer.WriteFrame(foreign), std::runtime_error);
    EXPECT_EQ(out::ReadBounded(directory / "accepted-frames.csv", 1024 * 1024), before);
    const auto zero = run.metrics()->stamp;
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    const auto first = *run.metrics();
    EXPECT_THROW(writer.WriteFrame(run), std::runtime_error);  // Interval must be recorded first.
    auto wrong = first; ++wrong.diagnostics.base_epoch;
    EXPECT_THROW(writer.RecordInterval(zero, wrong), std::runtime_error);
    const auto foreign_base = foreign.metrics()->stamp;
    report = foreign.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    EXPECT_THROW(writer.RecordInterval(foreign_base, *foreign.metrics()), std::runtime_error);
    const auto one = run.metrics()->stamp;
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    EXPECT_THROW(writer.RecordInterval(one, *run.metrics()), std::runtime_error);  // Skipped first interval.
    EXPECT_FALSE(fs::exists(directory / "accepted-000002.mesh.json"));
    ASSERT_NO_THROW(writer.RecordInterval(zero, first));
    EXPECT_THROW(writer.RecordInterval(zero, first), std::runtime_error);  // Duplicate/stale.
    ASSERT_NO_THROW(writer.RecordInterval(one, *run.metrics()));
    ASSERT_NO_THROW(writer.WriteFrame(run));
    EXPECT_THROW(writer.WriteFrame(run), std::runtime_error);
    EXPECT_THROW(writer.Finish(foreign, .1), std::runtime_error);
    EXPECT_FALSE(fs::exists(directory / "manifest.json"));
}

TEST_F(ElasticCouponArtifactsCheck, ExistingDirectoryAndUninitializedCaseArePreserved) {
    TempRoot root; const auto existing = root.path / "existing";
    ASSERT_TRUE(fs::create_directory(existing)); out::WriteBytes(existing / "sentinel", "Preserve these existing bytes\n");
    ElasticCouponCase empty;
    EXPECT_THROW(ElasticCouponArtifacts((root.path / "uninitialized").string(), empty, 1), std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path / "uninitialized"));
    ElasticCouponCase run;
    const auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    EXPECT_THROW(ElasticCouponArtifacts(existing.string(), run, FrameEvery(run)), std::runtime_error);
    EXPECT_EQ(out::ReadBounded(existing / "sentinel", 1024), "Preserve these existing bytes\n");
    EXPECT_EQ(std::distance(fs::directory_iterator(existing), fs::directory_iterator{}), 1);
    EXPECT_THROW(ElasticCouponArtifacts((root.path / "bad-cadence").string(), run, 0), std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path / "bad-cadence"));
}

TEST_F(ElasticCouponArtifactsCheck, FieldPhaseTracksAcceptedRecoveryAndRejectsInvalidAssociation) {
    ElasticCouponCase run;
    auto report = run.Initialize(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    report = run.Step(); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    ElasticCouponFrame frame;
    report = run.Capture(frame); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    EXPECT_STREQ(CouponFrameFields(frame)["element_evaluation_phase"].GetString(), "prepared_candidate_subsequently_committed");
    report = run.Step({1e-9}); ASSERT_EQ(report.status, CouponStatus::AdmissionFailure) << report.diagnostic;
    report = run.Capture(frame); ASSERT_EQ(report.status, CouponStatus::Ok) << report.diagnostic;
    const auto recovered = CouponFrameFields(frame);
    EXPECT_STREQ(recovered["element_evaluation_phase"].GetString(), "accepted_base");
    EXPECT_EQ(recovered["element_evaluation_base_epoch"].GetUint64(), frame.stamp.epoch);
    EXPECT_EQ(recovered["reaction_base_epoch"].GetUint64(), frame.stamp.epoch - 1);
    EXPECT_THROW(CouponFrameFields(ElasticCouponFrame{}), std::runtime_error);
    for (unsigned variant = 0; variant < 10; ++variant) {
        auto invalid = frame;
        if (variant == 0) invalid.element_association.valid = false;
        if (variant == 1) ++invalid.element_association.owner_id;
        if (variant == 2) ++invalid.element_association.base_epoch;
        if (variant == 3) invalid.element_association.phase = shell::ShellBatchPhase::kUnspecified;
        if (variant == 4) invalid.element[1].energy = std::numeric_limits<double>::quiet_NaN();
        if (variant == 5) invalid.element[1].bending_energy = std::numeric_limits<double>::infinity();
        if (variant == 6) invalid.stamp.temporal_scheme = tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
        if (variant == 7) invalid.stamp.velocity_phase = tl::fea::NodalVelocityPhase::PreviousMidpoint;
        if (variant == 8) invalid.metrics.stamp.temporal_scheme = tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
        if (variant == 9) invalid.metrics.stamp.velocity_phase = tl::fea::NodalVelocityPhase::PreviousMidpoint;
        EXPECT_THROW(CouponFrameFields(invalid), std::runtime_error) << "variant " << variant;
    }
    ExpectFields(recovered, frame);
}
}  // namespace
