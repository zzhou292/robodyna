#include "case/vehicle_dynamics/Reports.h"
#include <gtest/gtest.h>
#include <cstring>
#include <type_traits>

namespace crash::cases::vehicle_dynamics::test {
namespace solids = tl::fea::solids;
namespace beam18 = tl::fea::beam18;

// The production timer invokes Require on a deduced call result in this way.
template<class Call>
void CheckStage(Call&& call, const char* stage) {
    detail::Require(call(), stage);
}

TEST(NativeStageReports, EverySolidFamilyPreservesNativeFieldsAndLabelsOnlyIndices) {
    static_assert(std::is_trivially_copyable_v<SolidFailure>);
    const solids::Family families[] = {solids::Family::Solid18, solids::Family::Solid24,
        solids::Family::Solid6z, solids::Family::Solid18Law44, solids::Family::Solid18Law90};
    const char* names[] = {"Solid18", "Solid24", "Solid6z", "Solid18Law44", "Solid18Law90"};
    for (unsigned index = 0; index < 5; ++index) {
        solids::BatchReport report;
        report.status = solids::BatchStatus::ElementFailure;
        report.message = "Native solid candidate rejected";
        report.family = families[index];
        report.parent = 1344;
        report.node = 376929;
        report.element_status = 7;
        report.nodal_status = tl::fea::NodalStatus::ContributorFailure;
        try {
            CheckStage([&] { return report; }, "Solid candidate");
            FAIL() << "Expected a typed native stage error";
        } catch (const NativeStageError& error) {
            ASSERT_TRUE(std::holds_alternative<SolidFailure>(error.failure()));
            const auto& failure = std::get<SolidFailure>(error.failure());
            EXPECT_EQ(failure.status, report.status);
            EXPECT_EQ(failure.family, report.family);
            EXPECT_EQ(failure.parent_index, report.parent);
            EXPECT_EQ(failure.node_index, report.node);
            EXPECT_EQ(failure.element_status, report.element_status);
            EXPECT_EQ(failure.nodal_status, report.nodal_status);
            EXPECT_EQ(std::string(error.what()),
                "Solid candidate: Native solid candidate rejected [participant=solids family=" +
                std::string(names[index]) + " parent_index=1344 node_index=376929"
                " batch_status=6 element_status=7 nodal_status=6]");
        }
    }
}

TEST(NativeStageReports, BeamCacheFailureKeepsSignedStatusAndUnavailableNode) {
    static_assert(std::is_trivially_copyable_v<StructuralBeamFailure>);
    beam18::BatchReport report;
    report.status = beam18::BatchStatus::NonfiniteResult;
    report.message = "Result cache rejected";
    report.parent = 0;
    report.element_status = -1;
    try {
        CheckStage([&] { return report; }, "Structural beam candidate");
        FAIL() << "Expected a typed native stage error";
    } catch (const NativeStageError& error) {
        ASSERT_TRUE(std::holds_alternative<StructuralBeamFailure>(error.failure()));
        const auto& failure = std::get<StructuralBeamFailure>(error.failure());
        EXPECT_EQ(failure.status, report.status);
        EXPECT_EQ(failure.parent_index, 0u);
        EXPECT_EQ(failure.node_index, SIZE_MAX);
        EXPECT_EQ(failure.element_status, -1);
        EXPECT_EQ(failure.nodal_status, tl::fea::NodalStatus::Ok);
        EXPECT_EQ(std::string(error.what()), "Structural beam candidate: Result cache rejected"
            " [participant=beam18 parent_index=0 node_index=unavailable"
            " batch_status=8 element_status=-1 nodal_status=0]");
    }
}

TEST(NativeStageReports, OwnedExceptionSurvivesReportAndMessageLifetime) {
    char native_message[] = "late material failure";
    solids::BatchReport report;
    report.status = solids::BatchStatus::ElementFailure;
    report.family = solids::Family::Solid18Law44;
    report.message = native_message;
    report.parent = 385;
    report.element_status = -1;
    const NativeStageError original(report, "Solid candidate");
    const NativeStageError copied = original;
    std::memset(native_message, 'x', sizeof(native_message) - 1);
    report = {};
    EXPECT_EQ(std::string(copied.what()), std::string(original.what()));
    EXPECT_NE(std::string(copied.what()).find("late material failure"), std::string::npos);
    const auto& failure = std::get<SolidFailure>(copied.failure());
    EXPECT_EQ(failure.family, solids::Family::Solid18Law44);
    EXPECT_EQ(failure.parent_index, 385u);
    EXPECT_EQ(failure.element_status, -1);
}

TEST(NativeStageReports, MissingContextAndUnknownFamilyRemainExplicit) {
    solids::BatchReport report;
    report.status = solids::BatchStatus::DeviceFailure;
    report.message = nullptr;
    report.family = static_cast<solids::Family>(97);
    const NativeStageError error(report, nullptr);
    EXPECT_EQ(std::get<SolidFailure>(error.failure()).family, report.family);
    EXPECT_EQ(std::string(error.what()), "Physical stage: Native operation rejected"
        " [participant=solids family=unknown(97) parent_index=unavailable"
        " node_index=unavailable batch_status=9 element_status=0 nodal_status=0]");
}

TEST(NativeStageReports, SuccessDoesNotReadUnusedMessageOrFailureContext) {
    solids::BatchReport solid;
    solid.message = nullptr;
    solid.family = static_cast<solids::Family>(97);
    solid.element_status = -1;
    beam18::BatchReport beam;
    beam.message = nullptr;
    beam.element_status = -1;
    EXPECT_NO_THROW(CheckStage([&] { return solid; }, nullptr));
    EXPECT_NO_THROW(CheckStage([&] { return beam; }, nullptr));
}

TEST(NativeStageReports, GenericReportRetainsOriginalExceptionAndMessage) {
    tl::fea::NodalReport report;
    report.status = tl::fea::NodalStatus::Ok;
    report.message = nullptr;
    EXPECT_NO_THROW(CheckStage([&] { return report; }, nullptr));
    report.status = tl::fea::NodalStatus::WrongPhase;
    report.message = "Actual owner phase rejected";
    try {
        CheckStage([&] { return report; }, "Borrow owner");
        FAIL() << "Expected the existing generic runtime error";
    } catch (const NativeStageError&) {
        FAIL() << "A generic owner report must not acquire a material family";
    } catch (const std::runtime_error& error) {
        EXPECT_STREQ(error.what(), "Borrow owner: Actual owner phase rejected");
    }
}

} // namespace crash::cases::vehicle_dynamics::test
