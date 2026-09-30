#include "ActualMappingSupport.h"
#include "../../tests/FileWriteLimit.h"

namespace crash::output::full_shell::source::test {
namespace {
RecordFile DocumentFile(const std::filesystem::path& root, const std::string& name, const Document& doc) {
    WriteJson(root / name, doc);
    const auto bytes = ReadBounded(root / name, MappingMetadataByteCap);
    return {name, Sha256(bytes), bytes.size()};
}
} // namespace
TEST(SourceMappingActual, RehashedWrongMaterialAndTruncatedTailNeverReplaceVisibleMapping) {
    if (!std::getenv("ROBO_STATIC_CANONICAL")) GTEST_SKIP() << "Explicit original-source fixture not configured";
    const auto& source = ActualSource();
    const auto& mapping = ActualMapping();
    full_shell::test::Directory dir;
    const auto plan = PlanMappingRecord(mapping, "good");
    const auto file = WriteMappingRecord(dir.path, mapping, "good");
    auto visible = std::make_unique<PreparedSourceMapping>(ReadMappingRecord(dir.path, source, file, mapping.digest()));
    const auto* before = visible.get();
    auto changed = mapping.arrays();
    auto& ids = changed[detail::ParentIds];
    const auto new_mid = source.data().parts.back().material;
    const auto offset = 8 * (4 * (mapping.parents().size() - 1) + 2);
    for (unsigned j = 0; j < 8; ++j) ids.bytes[offset + j] = static_cast<char>((new_mid >> (8 * j)) & 255);
    ids.descriptor.sha256 = Sha256(ids.bytes);
    const auto bad_digest = MappingDigest(changed);
    ASSERT_NE(bad_digest, mapping.digest());
    auto descriptors = plan.arrays;
    descriptors[detail::ParentIds] = arrays::WriteBytes(dir.path, "bad-parent-ids.bin", ids.descriptor.layout, ids.bytes);
    const auto bad = DocumentFile(dir.path, "bad.mapping.json", detail::MappingDocument(source, bad_digest, descriptors));
    EXPECT_THROW(visible = std::make_unique<PreparedSourceMapping>(
        ReadMappingRecord(dir.path, source, bad, bad_digest)), std::exception);
    EXPECT_EQ(visible.get(), before);
    auto units = detail::MappingDocument(source, mapping.digest(), plan.arrays);
    units["source_authority"]["time_to_s"].SetDouble(2);
    const auto bad_units = DocumentFile(dir.path, "bad-units.mapping.json", units);
    EXPECT_THROW(ReadMappingRecord(dir.path, source, bad_units, mapping.digest()), std::exception);
    const auto& tail = mapping.arrays().back().bytes;
    full_shell::test::Overwrite(dir.path / plan.arrays.back().file, tail.substr(0, tail.size() - 1));
    EXPECT_THROW(visible = std::make_unique<PreparedSourceMapping>(
        ReadMappingRecord(dir.path, source, file, mapping.digest())), std::exception);
    EXPECT_EQ(visible.get(), before);
    full_shell::test::Overwrite(dir.path / plan.arrays.back().file, tail);
    EXPECT_NO_THROW(visible = std::make_unique<PreparedSourceMapping>(
        ReadMappingRecord(dir.path, source, file, mapping.digest())));
}
TEST(SourceMappingActual, PreexistingTailAndActualPartialWriteLeaveOnlyIncompleteEvidence) {
    if (!std::getenv("ROBO_STATIC_CANONICAL")) GTEST_SKIP() << "Explicit original-source fixture not configured";
    const auto& mapping = ActualMapping();
    full_shell::test::Directory dir;
    const auto blocked = PlanMappingRecord(mapping, "blocked");
    WriteBytes(dir.path / blocked.arrays.back().file, "existing");
    EXPECT_THROW(WriteMappingRecord(dir.path, mapping, "blocked"), std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path / blocked.arrays.front().file));
    const auto partial = PlanMappingRecord(mapping, "partial");
    {
        full_shell::test::FileSizeLimit limit;
        EXPECT_THROW(WriteMappingRecord(dir.path, mapping, "partial"), std::exception);
    }
    EXPECT_FALSE(std::filesystem::exists(dir.path / partial.description.file));
    EXPECT_LT(std::filesystem::file_size(dir.path / partial.arrays.front().file), partial.arrays.front().bytes);
    EXPECT_THROW(WriteMappingRecord(dir.path, mapping, "partial"), std::exception);
}
} // namespace crash::output::full_shell::source::test
