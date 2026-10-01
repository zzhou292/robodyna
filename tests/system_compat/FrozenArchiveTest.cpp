#include "tests/system_compat/SystemFixture.h"
#include "tests/archive_support/StreamArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
std::filesystem::path fixtures;
std::unique_ptr<robodyna::system_compat::Fixture> current;

std::string ReadFrozen(const char* extension) {
    const auto path = fixtures / (std::string("baseline.") + extension);
    if (std::filesystem::file_size(path) > 1024 * 1024)
        throw std::runtime_error("system archive exceeds the 1 MiB bound");
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
        throw std::runtime_error("cannot read frozen system archive");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

template <class Reader>
void ReadCheck(const char* extension) {
    robodyna::system_compat::Fixture loaded;
    ASSERT_NO_THROW(robodyna::archive_test::Read<Reader>(ReadFrozen(extension), loaded));
    EXPECT_NO_THROW(robodyna::system_compat::Check(loaded, true));
}

TEST(FrozenSystemArchive, ReadsPreRenameJson) { ReadCheck<chrono::ChArchiveInJSON>("json"); }
TEST(FrozenSystemArchive, ReadsPreRenameXml) { ReadCheck<chrono::ChArchiveInXML>("xml"); }
TEST(FrozenSystemArchive, ReadsPreRenameBinary) { ReadCheck<chrono::ChArchiveInBinary>("bin"); }
TEST(FrozenSystemArchive, WritesPreRenameJsonExactly) {
    EXPECT_EQ(robodyna::archive_test::Write<chrono::ChArchiveOutJSON>(*current), ReadFrozen("json"));
}
TEST(FrozenSystemArchive, WritesPreRenameXmlExactly) {
    EXPECT_EQ(robodyna::archive_test::Write<chrono::ChArchiveOutXML>(*current), ReadFrozen("xml"));
}
TEST(FrozenSystemArchive, WritesPreRenameBinaryExactly) {
    EXPECT_EQ(robodyna::archive_test::Write<chrono::ChArchiveOutBinary>(*current), ReadFrozen("bin"));
}
}  // namespace

int main(int argc, char** argv) {
    // Match the producer's first graph construction, before tests allocate any
    // other ChObj. Serialized inherited assembly names remain untouched.
    current = std::make_unique<robodyna::system_compat::Fixture>();
    robodyna::system_compat::Populate(*current);
    robodyna::system_compat::Check(*current, false);
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2)
        return 2;
    fixtures = std::filesystem::path(argv[1]).parent_path();
    return RUN_ALL_TESTS();
}
