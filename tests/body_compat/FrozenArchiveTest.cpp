#include "tests/body_compat/BodyFixture.h"
#include "tests/archive_support/StreamArchive.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include <gtest/gtest.h>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

namespace robodyna::body_compat {
namespace {
std::filesystem::path fixtures;

std::string ReadFrozen(const char* extension) {
    const auto path = fixtures / (std::string("baseline.") + extension);
    if (std::filesystem::file_size(path) > 1024 * 1024)
        throw std::runtime_error("body archive exceeds the qualified 1 MiB fixture bound");
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
        throw std::runtime_error("cannot read frozen body archive: " + path.string());
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

template <class Reader>
void CheckReader(const char* extension) {
    Fixture loaded;
    ASSERT_NO_THROW(archive_test::Read<Reader>(ReadFrozen(extension), loaded));
    // Includes exact dynamic types/tags, BodyEasyBox constructor recovery and
    // inertia, marker relative pose, raw parent pointers and shared child owners.
    EXPECT_NO_THROW(Check(loaded));
}

template <class Writer>
void CheckWriter(const char* extension) {
    Fixture current;
    Populate(current);
    EXPECT_EQ(archive_test::Write<Writer>(current), ReadFrozen(extension));
}

TEST(FrozenBodyArchive, ReadsPreRenameJson) { CheckReader<chrono::ChArchiveInJSON>("json"); }
TEST(FrozenBodyArchive, ReadsPreRenameXml) { CheckReader<chrono::ChArchiveInXML>("xml"); }
TEST(FrozenBodyArchive, ReadsPreRenameBinary) { CheckReader<chrono::ChArchiveInBinary>("bin"); }
TEST(FrozenBodyArchive, WritesPreRenameJsonExactly) { CheckWriter<chrono::ChArchiveOutJSON>("json"); }
TEST(FrozenBodyArchive, WritesPreRenameXmlExactly) { CheckWriter<chrono::ChArchiveOutXML>("xml"); }
TEST(FrozenBodyArchive, WritesPreRenameBinaryExactly) { CheckWriter<chrono::ChArchiveOutBinary>("bin"); }
}  // namespace
}  // namespace robodyna::body_compat

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2)
        return 2;
    robodyna::body_compat::fixtures = std::filesystem::path(argv[1]).parent_path();
    return RUN_ALL_TESTS();
}
