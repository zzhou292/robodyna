#include <gtest/gtest.h>
#include "viewer/ReplayAssets.h"
#include "viewer/physical_run/Options.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace {
class ReplayAssets : public ::testing::Test {
  protected:
    void SetUp() override {
        const auto* temp = std::getenv("TEST_TMPDIR");
        ASSERT_NE(temp, nullptr);
        directory = std::filesystem::path(temp) / "replay-assets";
        std::filesystem::create_directories(directory / "vsg/fonts");
        std::ofstream(directory / "logo_chrono_alpha.png") << "test fixture";
        std::ofstream(directory / "vsg/fonts/OpenSans-Bold.vsgb") << "test fixture";
    }
    void TearDown() override { std::filesystem::remove_all(directory); }
    std::filesystem::path directory;
};
}

TEST_F(ReplayAssets, ExplicitDirectoryTakesPrecedenceAndResolvesCanonically) {
    EXPECT_EQ(crash::viewer::ResolveReplayAssets(directory, directory / "missing"),
              std::filesystem::canonical(directory));
    EXPECT_EQ(crash::viewer::ResolveReplayAssets({}, directory), std::filesystem::canonical(directory));
    EXPECT_THROW(crash::viewer::ResolveReplayAssets(directory / "missing", directory), std::exception);
}

TEST_F(ReplayAssets, MissingAssetsAndImplicitCwdFallbackAreRejected) {
    EXPECT_THROW(crash::viewer::ResolveReplayAssets({}), std::exception);
    std::filesystem::remove(directory / "vsg/fonts/OpenSans-Bold.vsgb");
    EXPECT_THROW(crash::viewer::ResolveReplayAssets(directory), std::exception);
}

TEST(ReplayAssetOptions, FrontendPassesAnExplicitDirectoryWithoutChangingCaptureControls) {
    const char* args[]{"viewer", "run", "--chrono-data", "/declared/assets", "--capture-cap-gib", "10"};
    const auto options = crash::viewer::physical_run::Parse(6, const_cast<char**>(args));
    EXPECT_EQ(options.chrono_data, "/declared/assets");
    EXPECT_EQ(options.capture_bytes, 10ull << 30);
    const char* duplicate[]{"viewer", "run", "--chrono-data", "one", "--chrono-data", "two"};
    EXPECT_THROW(crash::viewer::physical_run::Parse(6, const_cast<char**>(duplicate)), std::exception);
    const char* empty[]{"viewer", "run", "--chrono-data", ""};
    EXPECT_THROW(crash::viewer::physical_run::Parse(4, const_cast<char**>(empty)), std::exception);
}
