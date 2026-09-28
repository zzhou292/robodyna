#include "case/vehicle_dynamics/Reports.h"
#include "case/vehicle_run/Loop.h"
#include <gtest/gtest.h>
#include <vector>
namespace crash::cases::vehicle_dynamics::test {
namespace {
// Existing loop seam; this checks diagnostic propagation and prefix closure,
// not a material evaluation, device transfer or numerical owner rollback.
class QephFailureOperations final : public vehicle_run::detail::Operations {
  public:
    vehicle_run::Endpoint Accepted() const noexcept override { return accepted; }
    void Prepare() override {
        ++prepared_calls;
        pending = true;
        tl::fea::qeph::BatchReport report;
        report.status = tl::fea::qeph::BatchStatus::Success;
        if (accepted.epoch == 2) {
            report.status = tl::fea::qeph::BatchStatus::ElementFailure;
            report.message = "QEPH device validation failed";
            report.element = 12;
            report.element_status = tl::fea::qeph::Status::kUnsupportedGeometry;
        }
        detail::Require(report, "QEPH candidate");
    }
    void Commit() override {
        EXPECT_TRUE(pending);
        ++accepted.epoch;
        accepted.time_s += .001;
        pending = false;
    }
    void Discard() noexcept override { ++discarded; pending = false; }
    void Append() override { logged = accepted.epoch; }
    void Capture() override { captured.push_back(accepted.epoch); }
    void SaveSample() override { saved = accepted.epoch; }
    void Finish(bool complete, const std::string& message) override {
        EXPECT_FALSE(complete);
        EXPECT_FALSE(pending);
        ++finished;
        reason = message;
    }
    vehicle_run::Endpoint accepted;
    bool pending = false;
    unsigned prepared_calls = 0, discarded = 0, finished = 0;
    std::uint64_t logged = 0, saved = 0;
    std::vector<std::uint64_t> captured;
    std::string reason;
};
}
TEST(QephStageReports, LateFailurePreservesAcceptedClockAndClosesItsPrefixWithExactDetail) {
    QephFailureOperations operations;
    vehicle_run::Config config;
    config.fixed_dt_s = .001;
    config.samples = 2;
    const auto result = vehicle_run::detail::RunLoop(operations, vehicle_run::Plan(config),
        {0, 5}, {}, [] { return 0.; });
    EXPECT_EQ(result.kind, vehicle_run::StopKind::PhysicsRejected);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(result.progress.accepted.epoch, 2u);
    EXPECT_EQ(result.progress.accepted.time_s, .002);
    EXPECT_EQ(operations.prepared_calls, 3u);
    EXPECT_EQ(operations.discarded, 1u);
    EXPECT_EQ(operations.logged, 2u);
    EXPECT_EQ(operations.saved, 2u);
    EXPECT_EQ(operations.captured, (std::vector<std::uint64_t>{0, 2}));
    EXPECT_EQ(operations.finished, 1u);
    EXPECT_EQ(operations.reason, result.reason);
    EXPECT_EQ(result.reason, "QEPH candidate: QEPH device validation failed"
        " [participant=qeph element_index=12 node_index=unavailable"
        " batch_status=8 element_status=2 nodal_status=0]");
}
} // namespace crash::cases::vehicle_dynamics::test
