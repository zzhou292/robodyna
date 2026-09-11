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
