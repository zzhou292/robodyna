#include "robodyna/io/RbPaths.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>

namespace {
namespace io = robodyna::io;
namespace fs = std::filesystem;

static_assert(std::is_same_v<decltype(io::GetDataPath()), const std::string&>);
static_assert(std::is_same_v<decltype(io::GetOutputPath()), const std::string&>);
static_assert(std::is_same_v<decltype(io::GetTestOutputPath()), const std::string&>);

// This binary owns its process-global path stores and runs cases sequentially.
// Reading the inherited output getters itself creates directories, so even the
// initial saved values must be read only after entering the test scratch directory.
class Paths : public ::testing::Test {
  protected:
    void SetUp() override {
        const auto* test_tmp = std::getenv("TEST_TMPDIR");
        ASSERT_NE(test_tmp, nullptr) << "Run via Bazel, or provide an isolated TEST_TMPDIR";
        previous_cwd = fs::current_path();
        scratch = fs::absolute(fs::path(test_tmp)) / ::testing::UnitTest::GetInstance()->current_test_info()->name();
        ASSERT_TRUE(fs::create_directory(scratch)) << "Refusing to reuse an existing directory";
        owns_scratch = true;
        fs::current_path(scratch);
        saved_data = chrono::GetChronoDataPath();
        saved_output = chrono::GetChronoOutputPath();
        saved_test_output = chrono::GetChronoTestOutputPath();
        saved_stores = true;
    }

    void TearDown() override {
        if (!owns_scratch)
            return;
        if (saved_stores) {
            chrono::SetChronoDataPath(saved_data);
            chrono::SetChronoOutputPath(saved_output);
            chrono::SetChronoTestOutputPath(saved_test_output);
        }
        fs::current_path(previous_cwd);
        fs::remove_all(scratch);
    }

    fs::path previous_cwd;
    fs::path scratch;
    std::string saved_data;
    std::string saved_output;
    std::string saved_test_output;
    bool owns_scratch = false;
    bool saved_stores = false;
};

TEST_F(Paths, BothNamesShareThreeDistinctStoresAndReferenceIdentity) {
    chrono::SetChronoDataPath("assets/old/");
    chrono::SetChronoOutputPath("demo-old");
    chrono::SetChronoTestOutputPath("test-old");
    const auto* data_store = &chrono::GetChronoDataPath();
    const auto* output_store = &chrono::GetChronoOutputPath();
    const auto* test_store = &chrono::GetChronoTestOutputPath();
    EXPECT_EQ(&io::GetDataPath(), data_store);
    EXPECT_EQ(&io::GetOutputPath(), output_store);
    EXPECT_EQ(&io::GetTestOutputPath(), test_store);
    EXPECT_NE(data_store, output_store);
    EXPECT_NE(data_store, test_store);
    EXPECT_NE(output_store, test_store);

    io::SetDataPath("assets/new/");
    EXPECT_EQ(chrono::GetChronoDataPath(), "assets/new/");
    EXPECT_EQ(*data_store, "assets/new/");
    EXPECT_EQ(*output_store, "demo-old");
    EXPECT_EQ(*test_store, "test-old");
    io::SetOutputPath("demo-new");
    EXPECT_EQ(chrono::GetChronoOutputPath(), "demo-new");
    EXPECT_EQ(*output_store, "demo-new");
    EXPECT_EQ(*test_store, "test-old");
    io::SetTestOutputPath("test-new");
    EXPECT_EQ(chrono::GetChronoTestOutputPath(), "test-new");
    EXPECT_EQ(*test_store, "test-new");
    EXPECT_EQ(&io::GetDataPath(), data_store);
    EXPECT_EQ(&io::GetOutputPath(), output_store);
    EXPECT_EQ(&io::GetTestOutputPath(), test_store);
}

TEST_F(Paths, DataFileRetainsLiteralStringConcatenation) {
    io::SetDataPath("assets");
    EXPECT_EQ(io::GetDataFile("mesh.obj"), "assetsmesh.obj");
    EXPECT_EQ(chrono::GetChronoDataFile("mesh.obj"), io::GetDataFile("mesh.obj"));
    chrono::SetChronoDataPath("assets/");
    EXPECT_EQ(io::GetDataFile("mesh.obj"), "assets/mesh.obj");
    EXPECT_EQ(io::GetDataFile("/absolute-looking"), "assets//absolute-looking");
    EXPECT_EQ(io::GetDataFile("../mesh.obj"), "assets/../mesh.obj");
    io::SetDataPath("");
    EXPECT_EQ(io::GetDataFile("mesh.obj"), "mesh.obj");
    EXPECT_FALSE(fs::exists("assets"));
}

TEST_F(Paths, SettersDoNotCreateDirectoriesButOutputGettersDo) {
    io::SetDataPath("data-unused");
    io::SetOutputPath("demo-new");
    io::SetTestOutputPath("test-new");
    EXPECT_FALSE(fs::exists("data-unused"));
    EXPECT_FALSE(fs::exists("demo-new"));
    EXPECT_FALSE(fs::exists("test-new"));
    EXPECT_EQ(io::GetDataPath(), "data-unused");
    EXPECT_FALSE(fs::exists("data-unused"));
    EXPECT_EQ(io::GetOutputPath(), "demo-new");
    EXPECT_TRUE(fs::is_directory("demo-new"));
    EXPECT_FALSE(fs::exists("test-new"));
    EXPECT_EQ(io::GetTestOutputPath(), "test-new");
    EXPECT_TRUE(fs::is_directory("test-new"));
    EXPECT_EQ(&io::GetOutputPath(), &chrono::GetChronoOutputPath());
}

TEST_F(Paths, BothDirectoryOverloadsAndExistingErrorBehaviorRemainAvailable) {
    auto path_overload = static_cast<bool (*)(const fs::path&)>(&io::CreateOutputDirectory);
    auto string_overload = static_cast<bool (*)(const std::string&)>(&io::CreateOutputDirectory);
    EXPECT_EQ(path_overload, static_cast<bool (*)(const fs::path&)>(&chrono::CreateOutputDirectory));
    EXPECT_EQ(string_overload, static_cast<bool (*)(const std::string&)>(&chrono::CreateOutputDirectory));
    EXPECT_TRUE(path_overload(fs::path("path-dir")));
    EXPECT_TRUE(string_overload(std::string("string-dir")));
    EXPECT_TRUE(path_overload(fs::path("path-dir")));
    EXPECT_THROW(path_overload(fs::path("missing-parent/child")), fs::filesystem_error);
    EXPECT_FALSE(fs::exists("missing-parent"));

    { std::ofstream file("occupied"); file << "keep existing file"; }
    io::SetOutputPath("occupied");
    EXPECT_THROW(io::GetOutputPath(), fs::filesystem_error);
    EXPECT_THROW(chrono::GetChronoOutputPath(), fs::filesystem_error);
    io::SetTestOutputPath("occupied");
    EXPECT_THROW(io::GetTestOutputPath(), fs::filesystem_error);
    EXPECT_THROW(chrono::GetChronoTestOutputPath(), fs::filesystem_error);
    EXPECT_EQ(fs::file_size("occupied"), 18u);
}

}  // namespace
