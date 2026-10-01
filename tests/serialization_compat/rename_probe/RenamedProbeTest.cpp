#include "tests/serialization_compat/rename_probe/RenamedTypes.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

namespace robodyna::serialization_compat {
using chrono::make_ChNameValue;

namespace {
std::filesystem::path fixtures;

std::string ReadBytes(const char* extension) {
    std::ifstream input(fixtures / (std::string("baseline.") + extension), std::ios::binary);
    if (!input)
        throw std::runtime_error("missing frozen legacy probe archive");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

template <class In>
void CheckFrozenRead(const char* extension) {
    std::istringstream stream(ReadBytes(extension), std::ios::in | std::ios::binary);
    std::shared_ptr<RbRenameProbe> object;
    std::shared_ptr<RbRenameProbeBase> base_alias;
    std::shared_ptr<RbRenameProbe> repeated;
    std::shared_ptr<RbRenameProbe> null_object;
    {
        In archive(stream);
        archive >> CHNVP(object) >> CHNVP(base_alias) >> CHNVP(repeated) >> CHNVP(null_object);
    }
    ASSERT_NE(object, nullptr);
    EXPECT_EQ(object->base_value, 17);
    EXPECT_DOUBLE_EQ(object->value, 2.5);
    EXPECT_EQ(object->Kind(), 41);
    EXPECT_EQ(object->observed_version, 5);
    EXPECT_EQ(object->observed_base_version, 3);
    EXPECT_EQ(object, repeated);
    EXPECT_EQ(base_alias.get(), static_cast<RbRenameProbeBase*>(object.get()));
    EXPECT_NE(static_cast<void*>(object.get()), static_cast<void*>(base_alias.get()));
    EXPECT_FALSE(object.owner_before(base_alias));
    EXPECT_FALSE(base_alias.owner_before(object));
    EXPECT_FALSE(null_object);
}

template <class Out>
void CheckFrozenWrite(const char* extension) {
    std::ostringstream stream(std::ios::out | std::ios::binary);
    auto object = std::make_shared<RbRenameProbe>();
    std::shared_ptr<RbRenameProbeBase> base_alias = object;
    auto repeated = object;
    std::shared_ptr<RbRenameProbe> null_object;
    {
        Out archive(stream);
        archive << CHNVP(object) << CHNVP(base_alias) << CHNVP(repeated) << CHNVP(null_object);
    }
    EXPECT_EQ(stream.str(), ReadBytes(extension));
}

TEST(RenamedArchiveProbe, ReadsLegacyJson) { CheckFrozenRead<chrono::ChArchiveInJSON>("json"); }
TEST(RenamedArchiveProbe, ReadsLegacyXml) { CheckFrozenRead<chrono::ChArchiveInXML>("xml"); }
TEST(RenamedArchiveProbe, ReadsLegacyBinary) { CheckFrozenRead<chrono::ChArchiveInBinary>("bin"); }
TEST(RenamedArchiveProbe, WritesLegacyJsonExactly) { CheckFrozenWrite<chrono::ChArchiveOutJSON>("json"); }
TEST(RenamedArchiveProbe, WritesLegacyXmlExactly) { CheckFrozenWrite<chrono::ChArchiveOutXML>("xml"); }
TEST(RenamedArchiveProbe, WritesLegacyBinaryExactly) { CheckFrozenWrite<chrono::ChArchiveOutBinary>("bin"); }

TEST(RenamedArchiveProbe, OneCanonicalFactoryTagAndAdjustedAbstractBase) {
    EXPECT_NE(std::string(typeid(RbRenameProbeBase).name()), "N6chrono17ChRenameProbeBaseE");
    EXPECT_NE(std::string(typeid(RbRenameProbe).name()), "N6chrono13ChRenameProbeE");
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(RbRenameProbe)), "ChRenameProbe");
    EXPECT_FALSE(chrono::ChClassFactory::IsClassRegistered(std::string("RbRenameProbe")));
    EXPECT_FALSE(chrono::ChClassFactory::IsClassRegistered(typeid(RbRenameProbeBase)));
    RbRenameProbeBase* raw = nullptr;
    ASSERT_NO_THROW(chrono::ChClassFactory::create("ChRenameProbe", &raw));
    std::unique_ptr<RbRenameProbeBase> base(raw);
    ASSERT_NE(base, nullptr);
    ASSERT_NE(dynamic_cast<RbRenameProbe*>(base.get()), nullptr);
    EXPECT_EQ(base->Kind(), 41);
    EXPECT_EQ(chrono::ChCastingMap::GetClassnameFromPtrTypeindex(typeid(RbRenameProbeBase*)), "ChRenameProbeBase");
}
}  // namespace
}  // namespace robodyna::serialization_compat

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2)
        return 2;
    robodyna::serialization_compat::fixtures = std::filesystem::path(argv[1]).parent_path();
    return RUN_ALL_TESTS();
}
