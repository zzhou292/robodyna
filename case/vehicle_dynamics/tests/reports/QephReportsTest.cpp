#include "case/vehicle_dynamics/Reports.h"
#include <gtest/gtest.h>
#include <cstring>
#include <type_traits>
namespace crash::cases::vehicle_dynamics::test {
namespace qeph = tl::fea::qeph;
namespace {
template<class Call> void QephStage(Call&& call, const char* stage) {
    // Same deduced-return expression as the production Timed wrapper.
    detail::Require(call(), stage);
}
}
TEST(QephStageReports, ElementFailuresRetainTheExactBatchAndElementStatuses) {
    static_assert(std::is_trivially_copyable_v<QephFailure>);
    for (auto status : {qeph::Status::kInvalidInput, qeph::Status::kUnsupportedGeometry,
                        qeph::Status::kNonfiniteResult, qeph::Status::kInvalidReference}) {
        qeph::BatchReport report;
        report.status = qeph::BatchStatus::ElementFailure;
        report.message = "QEPH device validation failed";
        report.element = 324094;
        report.node = 376929;
        report.element_status = status;
        report.nodal_status = tl::fea::NodalStatus::ContributorFailure;
        try {
            QephStage([&] { return report; }, "QEPH candidate");
            FAIL() << "Expected the typed QEPH failure";
        } catch (const NativeStageError& error) {
            ASSERT_TRUE(std::holds_alternative<QephFailure>(error.failure()));
            const auto& value = std::get<QephFailure>(error.failure());
            EXPECT_EQ(value.status, report.status);
            EXPECT_EQ(value.element_index, report.element);
            EXPECT_EQ(value.node_index, report.node);
            EXPECT_EQ(value.element_status, report.element_status);
            EXPECT_EQ(value.nodal_status, report.nodal_status);
            EXPECT_EQ(std::string(error.what()), "QEPH candidate: QEPH device validation failed"
                " [participant=qeph element_index=324094 node_index=376929 batch_status=8 element_status=" +
                std::to_string(static_cast<int>(status)) + " nodal_status=6]");
        }
    }
}
TEST(QephStageReports, Uint32UnavailableIndicesAreNeverWidenedIntoRealIndices) {
    qeph::BatchReport report;
    report.status = qeph::BatchStatus::NonfiniteResult;
    report.message = "QEPH device validation failed";
    const NativeStageError error(report, "QEPH candidate");
    const auto& value = std::get<QephFailure>(error.failure());
    EXPECT_EQ(value.element_index, UINT32_MAX);
    EXPECT_EQ(value.node_index, UINT32_MAX);
    EXPECT_EQ(std::string(error.what()), "QEPH candidate: QEPH device validation failed"
        " [participant=qeph element_index=unavailable node_index=unavailable"
        " batch_status=10 element_status=0 nodal_status=0]");
    report.element = report.node = 0;
    const NativeStageError zero(report, "QEPH candidate");
    EXPECT_NE(std::string(zero.what()).find("element_index=0 node_index=0"), std::string::npos);
    report.element = UINT32_MAX - 1;
    const NativeStageError last(report, "QEPH candidate");
    EXPECT_NE(std::string(last.what()).find("element_index=4294967294"), std::string::npos);
}
TEST(QephStageReports, OwnedCopySurvivesReportAndMessageDestruction) {
    char message[] = "temporary QEPH detail";
    qeph::BatchReport report;
    report.status = qeph::BatchStatus::ElementFailure;
    report.element = 12;
    report.element_status = qeph::Status::kUnsupportedGeometry;
    report.message = message;
    const NativeStageError original(report, "QEPH candidate");
    const NativeStageError copy = original;
    std::memset(message, 'x', sizeof(message) - 1);
    report = {};
    EXPECT_EQ(std::string(copy.what()), std::string(original.what()));
    EXPECT_NE(std::string(copy.what()).find("temporary QEPH detail"), std::string::npos);
    EXPECT_EQ(std::get<QephFailure>(copy.failure()).element_index, 12u);
    EXPECT_EQ(std::get<QephFailure>(copy.failure()).element_status, qeph::Status::kUnsupportedGeometry);
}
TEST(QephStageReports, MissingMessageAndFutureEnumValuesRemainExplicit) {
    qeph::BatchReport report;
    report.status = static_cast<qeph::BatchStatus>(97);
    report.element_status = static_cast<qeph::Status>(98);
    report.nodal_status = static_cast<tl::fea::NodalStatus>(99);
    report.message = nullptr;
    const NativeStageError error(report, nullptr);
    const auto& value = std::get<QephFailure>(error.failure());
    EXPECT_EQ(value.status, report.status);
    EXPECT_EQ(value.element_status, report.element_status);
    EXPECT_EQ(value.nodal_status, report.nodal_status);
    EXPECT_EQ(std::string(error.what()), "Physical stage: Native operation rejected"
        " [participant=qeph element_index=unavailable node_index=unavailable"
        " batch_status=97 element_status=98 nodal_status=99]");
}
TEST(QephStageReports, SuccessDoesNotInspectUnusedMessageOrFailureFields) {
    qeph::BatchReport report;
    report.status = qeph::BatchStatus::Success;
    report.message = nullptr;
    report.element = UINT32_MAX - 1;
    report.element_status = static_cast<qeph::Status>(98);
    EXPECT_NO_THROW(QephStage([&] { return report; }, nullptr));
}
} // namespace crash::cases::vehicle_dynamics::test
