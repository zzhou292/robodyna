#include "SourceBundleTestSupport.h"

namespace crash::output::full_shell::source::test {
TEST(SourceBundleActual,RehashedChunkSequenceAndContentCorruptionPreserveVisibleMapping) {
    const auto& bundle = ActualBundle();
    ft::Directory directory;
    BundleDirectories(directory.path);
    const auto record = WriteSourceBundle(directory.path, bundle);
    const auto original = ReadBounded(directory.path / record.file, BundleMetadataByteCap);
    auto visible = std::make_unique<PreparedSourceMapping>(ActualMapping());
    const auto* identity = visible.get();
    const auto read = [&](const RecordFile& file) {
        visible = std::make_unique<PreparedSourceMapping>(ReadSourceBundle(directory.path, file,
            ActualInputs(), ActualMapping().digest()));
    };
    auto doc = array_json::Parse(original, BundleMetadataByteCap);
    doc["member_chunks"][1]["offset"].SetUint64(33554433);
    auto changed = RewriteBundle(directory.path, record, doc);
    EXPECT_THROW(read(changed), std::exception);
    EXPECT_EQ(visible.get(), identity);
    doc = array_json::Parse(original, BundleMetadataByteCap);
    const auto& tail = bundle.description().chunks.back().record;
    auto tail_bytes = ReadBounded(directory.path / tail.file, tail.bytes);
    tail_bytes.back() ^= 1;
    ft::Overwrite(directory.path / tail.file, tail_bytes);
    const auto wrong_hash = Sha256(tail_bytes);
    for (auto& f : doc["files"].GetArray()) {
        if (std::string(f["file"].GetString()) == tail.file)
            f["sha256"].SetString(wrong_hash.c_str(), doc.GetAllocator());
    }
    changed = RewriteBundle(directory.path, record, doc);
    EXPECT_THROW(read(changed), std::exception); // Per-chunk hashes pass; complete original key rejects.
    EXPECT_EQ(visible.get(), identity);
    tail_bytes.back() ^= 1;
    ft::Overwrite(directory.path / tail.file, tail_bytes);
    ft::Overwrite(directory.path / record.file, original);
    auto wrong = record; wrong.sha256[0] = wrong.sha256[0] == 'a' ? 'b' : 'a';
    EXPECT_THROW(read(wrong), std::exception);
    EXPECT_EQ(visible.get(), identity);
    read(record);
    EXPECT_EQ(visible->digest(), ActualMapping().digest());
}
TEST(SourceBundleActual,LateTruncationUnitsAndInventoryRejectWithoutPublishingPartialRead) {
    const auto& bundle = ActualBundle();
    ft::Directory directory;
    BundleDirectories(directory.path);
    const auto record = WriteSourceBundle(directory.path, bundle);
    const auto original = ReadBounded(directory.path / record.file, BundleMetadataByteCap);
    auto doc = array_json::Parse(original, BundleMetadataByteCap);
    doc["source_authority"]["length_to_m"].SetDouble(1.);
    auto changed = RewriteBundle(directory.path, record, doc);
    EXPECT_THROW(ReadSourceBundle(directory.path, changed, ActualInputs(), ActualMapping().digest()), std::exception);
    doc = array_json::Parse(original, BundleMetadataByteCap);
    doc["files"].Erase(doc["files"].End() - 2); // Delete the final mapping array from declared inventory only.
    changed = RewriteBundle(directory.path, record, doc);
    EXPECT_THROW(ReadSourceBundle(directory.path, changed, ActualInputs(), ActualMapping().digest()), std::exception);
    ft::Overwrite(directory.path / record.file, original);
    const auto mapping_plan = PlanMappingRecord(ActualMapping(), "source-mapping");
    const auto& tail = mapping_plan.arrays.back();
    const auto tail_bytes = ReadBounded(directory.path / tail.file, tail.bytes);
    std::filesystem::resize_file(directory.path / tail.file, tail.bytes - 1);
    EXPECT_THROW(ReadSourceBundle(directory.path, record, ActualInputs(), ActualMapping().digest()), std::exception);
    ft::Overwrite(directory.path / tail.file, tail_bytes);
    EXPECT_NO_THROW(ReadSourceBundle(directory.path, record, ActualInputs(), ActualMapping().digest()));
}
TEST(SourceBundleActual,ExistingTailAndLatePartialKeyCopyCannotPublishOrOverwrite) {
    const auto& bundle = ActualBundle();
    const auto& d = bundle.description();
    ft::Directory existing;
    BundleDirectories(existing.path);
    WriteBytes(existing.path / bundle.descriptor().file, "preexisting");
    EXPECT_THROW(WriteSourceBundle(existing.path, bundle), std::exception);
    EXPECT_FALSE(std::filesystem::exists(existing.path / d.canonical_manifest.file));
    EXPECT_EQ(ReadBounded(existing.path / bundle.descriptor().file, 100), "preexisting");
    ft::Directory partial;
    BundleDirectories(partial.path);
    {
        ft::FileSizeLimit limit(20 * 1024 * 1024);
        EXPECT_THROW(WriteSourceBundle(partial.path, bundle), std::exception);
    }
    EXPECT_TRUE(std::filesystem::exists(partial.path / d.canonical_manifest.file));
    EXPECT_EQ(std::filesystem::file_size(partial.path / d.chunks.front().record.file), 20u * 1024 * 1024);
    EXPECT_FALSE(std::filesystem::exists(partial.path / d.mapping.file));
    EXPECT_FALSE(std::filesystem::exists(partial.path / bundle.descriptor().file));
    EXPECT_THROW(WriteSourceBundle(partial.path, bundle), std::exception);
}
TEST(SourceBundleActual,SharedCanonicalBytePathPreservesAuthorityAndRejectsMalformedMember) {
    const auto in = ActualInputs();
    auto bytes = detail::ReadFile(in.member_root, in.source_member, 64 * 1024 * 1024);
    const auto source = CanonicalSource::ReadWithMemberBytes(in, bytes);
    EXPECT_EQ(source.data().scope_bytes, ActualSource().data().scope_bytes);
    EXPECT_EQ(source.data().arrays.back().bytes, ActualSource().data().arrays.back().bytes);
    bytes.back() ^= 1;
    EXPECT_THROW(CanonicalSource::ReadWithMemberBytes(in, bytes), std::exception);
    bytes.pop_back();
    EXPECT_THROW(CanonicalSource::ReadWithMemberBytes(in, bytes), std::exception);
}
} // namespace crash::output::full_shell::source::test
