#include "viewer/physical_run/Options.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
namespace crash::viewer::physical_run::test {
namespace fs=std::filesystem;
TEST(PhysicalViewerValues, OneReceiptInputStrictControlsAndDefaultPhysicalScale) {
    const char* args[]{"viewer","run","--color","part-id","--fps","24","--view","wall-side"};
    auto options=Parse(8,const_cast<char**>(args));
    EXPECT_EQ(options.input,"run");EXPECT_EQ(options.frames_per_second,24);
    EXPECT_EQ(options.scene.colors,visual::ReplayColorMode::PartId);
    EXPECT_EQ(options.scene.view,visual::ReplayView::WallSide);
    EXPECT_EQ(options.capture_bytes,2ull<<30);
    const char* duplicate[]{"viewer","run","--fps","24","--fps","24"};
    EXPECT_THROW(Parse(6,const_cast<char**>(duplicate)),std::exception);
    const char* interpolate[]{"viewer","run","--interpolate","true"};
    EXPECT_THROW(Parse(4,const_cast<char**>(interpolate)),std::exception);
    const char* future[]{"viewer","run","--require-frames","18446744073709551615"};
    EXPECT_THROW(Parse(4,const_cast<char**>(future)),std::exception);
}
TEST(PhysicalViewerValues, ExplicitCaptureDiskBudgetsPreserveStrictAdmission) {
    const char* values[]{"2", "6", "10"};
    const std::size_t budgets[]{2ull<<30, 6ull<<30, 10ull<<30};
    for (std::size_t i=0; i<3; ++i) {
        const char* args[]{"viewer", "run", "--capture-cap-gib", values[i]};
        EXPECT_EQ(Parse(4,const_cast<char**>(args)).capture_bytes,budgets[i]);
    }
    const char* invalid[]{"0", "1", "4", "8", "12", "-10", "+10", "010", "10.0", "10x"};
    for (const auto* value : invalid) {
        const char* args[]{"viewer", "run", "--capture-cap-gib", value};
        EXPECT_THROW(Parse(4,const_cast<char**>(args)),std::exception);
    }
}
TEST(PhysicalViewerValues, TenGiBDiskBudgetFits301ImagesWithoutWeakeningImageBound) {
    constexpr std::size_t per_image=32ull<<20;
    constexpr std::size_t reserve=4ull<<20;
    EXPECT_EQ(CaptureForecast(301,10ull<<30),301*per_image+reserve);
    EXPECT_THROW(CaptureForecast(301,6ull<<30),std::exception);
    EXPECT_EQ(CaptureForecast(319,10ull<<30),319*per_image+reserve);
    EXPECT_THROW(CaptureForecast(320,10ull<<30),std::exception);
    EXPECT_EQ(CaptureForecast(63,2ull<<30),63*per_image+reserve);
    EXPECT_THROW(CaptureForecast(64,2ull<<30),std::exception);
    EXPECT_EQ(CaptureForecast(191,6ull<<30),191*per_image+reserve);
    EXPECT_THROW(CaptureForecast(192,6ull<<30),std::exception);
    EXPECT_THROW(CaptureForecast(0,10ull<<30),std::exception);
    EXPECT_THROW(CaptureForecast(SIZE_MAX,10ull<<30),std::exception);
    EXPECT_THROW(CaptureForecast(1,8ull<<30),std::exception);
    EXPECT_THROW(CaptureForecast(1,SIZE_MAX),std::exception);
}
TEST(PhysicalViewerValues, ControllerDirectoryAndReceiptResolveTheSameExactAuthority) {
    output::full_shell::test::Directory directory;
    output::physical_run::ViewerInput input;
    input.archive_directory="archive";input.mapping_sha256=output::Sha256("mapping");
    input.manifest={"manifest.json",output::Sha256("manifest"),8};
    input.source.canonical_manifest={"canonical.json",output::Sha256("canonical"),9};
    input.source.scope_report={"scope.json",output::Sha256("scope"),5};
    input.source.source_member={"source.key",output::Sha256("source"),6};
    input.source.tire_policy="fixture_selection";input.source.units={"t","mm","s",1000,.001,1};
    const auto file=output::physical_run::WriteViewerInput(directory.path,"viewer-input.json",input);
    Options options;options.input=directory.path;
    const auto from_directory=ReadInput(options);
    options.input=directory.path/file.file;
    const auto from_file=ReadInput(options);
    EXPECT_EQ(from_file.receipt.sha256,from_directory.receipt.sha256);
    EXPECT_EQ(from_file.values.mapping_sha256,input.mapping_sha256);
    EXPECT_EQ(from_file.values.manifest.sha256,input.manifest.sha256);
    options.expected_receipt_sha256=output::Sha256("foreign");
    EXPECT_THROW(ReadInput(options),std::exception);
    options.expected_receipt_sha256=file.sha256;
    EXPECT_NO_THROW(ReadInput(options));
}
TEST(PhysicalViewerValues, ExplicitRecoveryDescriptorAndReceiptHash) {
    const char* args[]{"viewer","--recovered","recovered-samples.json","--require-frames","100",
        "--capture-cap-gib","6"};
    const auto options=Parse(7,const_cast<char**>(args));
    EXPECT_TRUE(options.recovered);EXPECT_EQ(options.require_frames,100u);
    EXPECT_EQ(options.capture_bytes,6ull<<30);
    const char* missing[]{"viewer","--recovered"};
    EXPECT_THROW(Parse(2,const_cast<char**>(missing)),std::exception);
    output::full_shell::test::Directory directory;
    output::WriteBytes(directory.path/"recovered-samples.json","{}\n");
    auto selected=options;selected.input=directory.path/"recovered-samples.json";
    const auto input=ReadInput(selected);
    EXPECT_TRUE(input.recovered);EXPECT_EQ(input.receipt.sha256,output::Sha256("{}\n"));
    EXPECT_TRUE(input.values.manifest.file.empty());
    // Selecting bytes alone cannot turn them into a valid recovered reader.
    EXPECT_THROW(OpenSamples(input),std::exception);
    selected.expected_receipt_sha256=output::Sha256("foreign");
    EXPECT_THROW(ReadInput(selected),std::exception);
    selected.expected_receipt_sha256.clear();selected.input=directory.path;
    EXPECT_THROW(ReadInput(selected),std::exception);
}
TEST(PhysicalViewerValues, BoundedCaptureCountAndImmutableInputDestination) {
    EXPECT_LT(CaptureForecast(60,2ull<<30),2ull<<30);
    EXPECT_THROW(CaptureForecast(64,2ull<<30),std::exception);
    EXPECT_GT(CaptureForecast(180,6ull<<30),4ull<<30);
    EXPECT_THROW(CaptureForecast(SIZE_MAX,6ull<<30),std::exception);
    output::full_shell::test::Directory directory;
    fs::create_directory(directory.path/"input");
    EXPECT_THROW(CheckCaptureDestination(directory.path/"input",directory.path/"input"/"render"),std::exception);
    EXPECT_NO_THROW(CheckCaptureDestination(directory.path/"input",directory.path/"render"));
    fs::create_directory(directory.path/"render");
    EXPECT_THROW(CheckCaptureDestination(directory.path/"input",directory.path/"render"),std::exception);
}
} // namespace crash::viewer::physical_run::test
