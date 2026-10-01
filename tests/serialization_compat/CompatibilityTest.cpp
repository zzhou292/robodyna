#include "tests/serialization_compat/ArchiveFormats.h"

#include <filesystem>
#include <memory>
#include <string>
#include <typeindex>

#include <gtest/gtest.h>

namespace robodyna::serialization_compat {
namespace {
std::filesystem::path fixtures;

class FrozenArchive : public ::testing::TestWithParam<Format> {};

TEST_P(FrozenArchive, ReadsPreRenameStateAndSharedGraph) {
    const auto bytes = ReadBytes((fixtures / (std::string("baseline.") + Extension(GetParam()))).string());
    FixtureModel loaded;
    ASSERT_NO_THROW(ReadFixture(GetParam(), bytes, loaded));
    EXPECT_NO_THROW(CheckFixture(loaded));
}

TEST_P(FrozenArchive, WriterRetainsPreRenameBytes) {
    FixtureModel fixture;
    PopulateFixture(fixture);
    const auto expected = ReadBytes((fixtures / (std::string("baseline.") + Extension(GetParam()))).string());
    EXPECT_EQ(WriteFixture(GetParam(), fixture), expected);
}

INSTANTIATE_TEST_SUITE_P(JsonXmlBinary, FrozenArchive,
                        ::testing::Values(Format::Json, Format::Xml, Format::Binary),
                        [](const ::testing::TestParamInfo<Format>& info) { return Extension(info.param); });

TEST(ArchiveFactoryCompatibility, LegacyTagsCreateActualDerivedType) {
    chrono::ChBody* raw = nullptr;
    ASSERT_NO_THROW(chrono::ChClassFactory::create("ChBodyAuxRef", &raw));
    std::unique_ptr<chrono::ChBody> body(raw);
    ASSERT_NE(body, nullptr);
    EXPECT_NE(dynamic_cast<chrono::ChBodyAuxRef*>(body.get()), nullptr);
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(*body)), "ChBodyAuxRef");
    chrono::ChBody* raw_plain = nullptr;
    ASSERT_NO_THROW(chrono::ChClassFactory::create("ChBody", &raw_plain));
    std::unique_ptr<chrono::ChBody> plain(raw_plain);
    ASSERT_NE(plain, nullptr);
    EXPECT_EQ(typeid(*plain), typeid(chrono::ChBody));
}

TEST(ArchiveFactoryCompatibility, MultipleInheritanceAdjustsRawAndSharedPointers) {
    auto body = std::make_shared<chrono::ChBodyAuxRef>();
    void* raw = body.get();
    auto* expected_frame = static_cast<chrono::ChBodyFrame*>(body.get());
    auto* expected_contact = static_cast<chrono::ChContactable*>(body.get());
    // Ensure this really exercises a nonzero base offset on the supported ABI.
    ASSERT_NE(static_cast<void*>(expected_frame), raw);
    ASSERT_NE(static_cast<void*>(expected_contact), raw);
    EXPECT_EQ(chrono::ChCastingMap::Convert("ChBodyAuxRef", "ChBodyFrame", raw), expected_frame);
    EXPECT_EQ(chrono::ChCastingMap::Convert("ChBodyAuxRef", "ChContactable", raw), expected_contact);
    EXPECT_EQ(chrono::ChCastingMap::Convert(typeid(chrono::ChBodyAuxRef*), typeid(chrono::ChBodyFrame*), raw),
              expected_frame);
    const auto converted = chrono::ChCastingMap::Convert("ChBodyAuxRef", "ChBodyFrame", std::static_pointer_cast<void>(body));
    EXPECT_EQ(converted.get(), expected_frame);
    EXPECT_FALSE(body.owner_before(converted));
    EXPECT_FALSE(converted.owner_before(body));
}

TEST(ArchiveFactoryCompatibility, FrozenTextPinsUnregisteredVersionKeys) {
    // Literal names intentionally freeze GCC/Itanium identities, rather than
    // deriving the expected names from whatever renamed type is under test.
    const auto json = ReadBytes((fixtures / "baseline.json").string());
    const auto xml = ReadBytes((fixtures / "baseline.xml").string());
    for (const std::string key : {"_version_N6chrono9ChVector3IdEE", "_version_N6chrono7ChFrameIdEE",
                                  "_version_N6chrono13ChFrameMovingIdEE", "_version_N6chrono11ChBodyFrameE",
                                  "_version_ChBody", "_version_ChBodyAuxRef"}) {
        EXPECT_NE(json.find(key), std::string::npos) << key;
        EXPECT_NE(xml.find(key), std::string::npos) << key;
    }
    EXPECT_FALSE(chrono::ChClassFactory::IsClassRegistered(typeid(chrono::ChBodyFrame)));
    EXPECT_FALSE(chrono::ChClassFactory::IsClassRegistered(typeid(chrono::ChFramed)));
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
