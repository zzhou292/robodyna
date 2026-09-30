#include "Fixture.h"
#include "../Export.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::test {
TEST(QephRejectedExport, RunCompanionFailuresStayInsideTheNonthrowingDiagnosticBoundary) {
    static_assert(std::is_nothrow_move_constructible_v<ExportResult>);
    static_assert(std::is_nothrow_move_assignable_v<ExportResult>);
    output::full_shell::test::Directory directory;
    const auto captured = Captured();
    const auto failed = ExportForRun(captured, directory.path / "absent-run");
    EXPECT_EQ(failed.status, ExportStatus::ExportFailed);
    EXPECT_FALSE(failed.manifest);
    EXPECT_EQ(captured.original.element_status, native::Status::kInvalidInput);
    const auto published = ExportForRun(captured, directory.path);
    EXPECT_EQ(published.status, ExportStatus::Captured) << published.error.data();
    EXPECT_TRUE(std::filesystem::exists(directory.path / "qeph-rejection/failure.json"));
}
TEST(QephRejectedExport, NoFailureCreatesNoCompanionAndCapturedWordsAreReadBackExactly) {
    output::full_shell::test::Directory directory;
    const auto path = directory.path / "qeph-rejection";
    CaptureState absent;
    EXPECT_EQ(Export(absent, path).status, ExportStatus::NoRejectedCandidate);
    EXPECT_FALSE(std::filesystem::exists(path));
    const auto captured = Captured();
    const auto result = Export(captured, path);
    ASSERT_EQ(result.status, ExportStatus::Captured) << result.error.data();
    ASSERT_TRUE(result.manifest);
    EXPECT_TRUE(std::filesystem::exists(path / "input.words.bin"));
    const auto bytes = output::ReadBounded(path / "failure.json", 32u << 10);
    output::Document document; document.Parse(bytes.data(), bytes.size());
    ASSERT_FALSE(document.HasParseError());
    EXPECT_TRUE(document["input_available"].GetBool());
    EXPECT_TRUE(document["host_reproduces_original_rejection"].GetBool());
    EXPECT_EQ(document["requested_source"]["parent_id"].GetUint64(), captured.input.source.parent);
    EXPECT_EQ(document["captured_source"]["parent_id"].GetUint64(), captured.input.source.parent);
    const auto second = Export(captured, path);
    EXPECT_EQ(second.status, ExportStatus::ExportFailed);
    EXPECT_FALSE(second.manifest);
}
TEST(QephRejectedExport, UnsupportedFailurePublishesMetadataWithoutInventingAnInputPacket) {
    output::full_shell::test::Directory directory;
    auto captured = Captured();
    captured.complete = false;
    captured.status = native::RejectedCaptureStatus::UnsupportedFailure;
    const auto result = Export(captured, directory.path / "evidence");
    ASSERT_EQ(result.status, ExportStatus::CaptureIncomplete) << result.error.data();
    EXPECT_TRUE(result.manifest);
    EXPECT_TRUE(std::filesystem::exists(directory.path / "evidence/metadata.words.bin"));
    EXPECT_FALSE(std::filesystem::exists(directory.path / "evidence/input.words.bin"));
    EXPECT_EQ(captured.original.element_status, native::Status::kInvalidInput);
}
TEST(QephRejectedExport, InvalidCaptureSerializationDoesNotReplaceItsOriginalFailure) {
    output::full_shell::test::Directory directory;
    auto captured = Captured();
    captured.input.element.reference.input.thickness *= 2;
    const auto result = Export(captured, directory.path / "partial");
    EXPECT_EQ(result.status, ExportStatus::ExportFailed);
    EXPECT_FALSE(result.manifest);
    EXPECT_NE(result.error[0], 0);
    EXPECT_EQ(captured.original.status, native::BatchStatus::ElementFailure);
    EXPECT_EQ(captured.original.element_status, native::Status::kInvalidInput);
    EXPECT_TRUE(std::filesystem::exists(directory.path / "partial/metadata.words.bin"));
}
}
