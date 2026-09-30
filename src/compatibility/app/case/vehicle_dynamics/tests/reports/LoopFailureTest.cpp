#include "case/vehicle_dynamics/Reports.h"
#include "case/vehicle_run/Loop.h"
#include <gtest/gtest.h>
#include <vector>

namespace crash::cases::vehicle_dynamics::test {
namespace run = vehicle_run;

// Fault injection uses the existing loop seam. This proves exception/prefix
// orchestration, not a GPU material evaluation or the owner's numerical rollback.
class LateFailure final : public run::detail::Operations {
  public:
    explicit LateFailure(bool beam) : fail_beam_(beam) {}

    run::Endpoint Accepted() const noexcept override { return accepted_; }

    void Prepare() override {
        ++prepared_calls;
        pending_ = true;
        tl::fea::solids::BatchReport solid;
        if (accepted_.epoch == 2 && !fail_beam_) {
            solid.status = tl::fea::solids::BatchStatus::ElementFailure;
            solid.message = "Late solid failure";
            solid.family = tl::fea::solids::Family::Solid18Law90;
            solid.parent = 1344;
            solid.element_status = 7;
        }
        detail::Require(solid, "Solid candidate");
        ++completed_solid_stages;

        tl::fea::beam18::BatchReport beam;
        if (accepted_.epoch == 2 && fail_beam_) {
            beam.status = tl::fea::beam18::BatchStatus::ElementFailure;
            beam.message = "Late beam failure";
            beam.parent = 141;
            beam.element_status = -1;
        }
        detail::Require(beam, "Structural beam candidate");
    }

    void Commit() override {
        EXPECT_TRUE(pending_);
        ++accepted_.epoch;
        accepted_.time_s += .001;
        pending_ = false;
    }

    void Discard() noexcept override {
        ++discarded;
        pending_ = false;
    }

    void Append() override { logged = accepted_.epoch; }
    void Capture() override { captured.push_back(accepted_.epoch); }
    void SaveSample() override { saved = accepted_.epoch; }

    void Finish(bool complete, const std::string& reason) override {
        EXPECT_FALSE(complete);
        EXPECT_FALSE(pending_);
        EXPECT_EQ(saved, logged);
        ++finished;
        prefix_reason = reason;
    }

    unsigned prepared_calls = 0;
    unsigned completed_solid_stages = 0;
    unsigned discarded = 0;
    unsigned finished = 0;
    std::uint64_t saved = UINT64_MAX;
    std::uint64_t logged = 0;
    std::vector<std::uint64_t> captured;
    std::string prefix_reason;

  private:
    bool fail_beam_;
    bool pending_ = false;
    run::Endpoint accepted_;
};

void CheckLateFailure(bool beam) {
    LateFailure operations(beam);
    run::Config config;
    config.fixed_dt_s = .001;
    config.samples = 2;
    // Epoch 2 is off cadence: the failed attempt must export exactly that prefix.
    const auto result = run::detail::RunLoop(operations, run::Plan(config), {0, 5}, {}, [] { return 0.; });
    EXPECT_EQ(result.kind, run::StopKind::PhysicsRejected);
    EXPECT_TRUE(result.valid_manifest);
    EXPECT_EQ(result.progress.accepted.epoch, 2u);
    EXPECT_EQ(result.progress.accepted.time_s, .002);
    EXPECT_EQ(operations.Accepted().epoch, 2u);
    EXPECT_EQ(operations.Accepted().time_s, .002);
    EXPECT_EQ(operations.prepared_calls, 3u);
    EXPECT_EQ(operations.completed_solid_stages, beam ? 3u : 2u);
    EXPECT_EQ(operations.discarded, 1u);
    EXPECT_EQ(operations.logged, 2u);
    EXPECT_EQ(operations.saved, 2u);
    EXPECT_EQ(operations.captured, (std::vector<std::uint64_t>{0, 2}));
    EXPECT_EQ(operations.finished, 1u);
    EXPECT_EQ(operations.prefix_reason, result.reason);
    const std::string expected = beam
        ? "Structural beam candidate: Late beam failure [participant=beam18 parent_index=141"
          " node_index=unavailable batch_status=6 element_status=-1 nodal_status=0]"
        : "Solid candidate: Late solid failure [participant=solids family=Solid18Law90 parent_index=1344"
          " node_index=unavailable batch_status=6 element_status=7 nodal_status=0]";
    EXPECT_EQ(result.reason, expected);
}

TEST(NativeStageReports, LateSolidFailureDiscardsAndExportsOnlyAcceptedPrefix) {
    CheckLateFailure(false);
}

TEST(NativeStageReports, LateBeamFailureAfterSolidsKeepsAcceptedClockAndPrefix) {
    CheckLateFailure(true);
}

} // namespace crash::cases::vehicle_dynamics::test
