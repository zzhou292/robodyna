#include "output/full_shell/static_bundle/tests/SourceBundleTestSupport.h"
namespace crash::output::full_shell::source::test {
TEST(NativeArchiveOriginalSource, SmallerArtifactsPreserveEveryOriginalSourceAndMappingByte) {
    auto request = BundleOptions();
    request.archive.file_byte_cap = 24u << 20;
    const auto bundle = PreparedSourceBundle::Prepare(ActualMapping(), request);
    const auto& description = bundle.description();
    ASSERT_EQ(description.chunks.size(), 2u);
    EXPECT_EQ(description.chunks[0].record.bytes, 24u << 20);
    EXPECT_EQ(description.chunks[1].offset, 24u << 20);
    EXPECT_EQ(description.chunks[1].record.bytes, 17680929u);
    for (const auto& file : bundle.reservations()) EXPECT_LE(file.bytes, 24u << 20);
    ft::Directory directory;
    BundleDirectories(directory.path);
    const auto descriptor = WriteSourceBundle(directory.path, bundle);
    auto authority = ActualInputs();
    authority.canonical_root = authority.scope_root = authority.member_root = "/not-used-by-bundle-reader";
    const auto restored = ReadSourceBundle(directory.path, descriptor, authority, bundle.mapping().digest());
    EXPECT_EQ(restored.digest(), bundle.mapping().digest());
    EXPECT_EQ(restored.source().data().canonical_bytes, ActualSource().data().canonical_bytes);
    EXPECT_EQ(restored.source().data().scope_bytes, ActualSource().data().scope_bytes);
    ASSERT_EQ(restored.source().data().arrays.size(), ActualSource().data().arrays.size());
    for (std::size_t i = 0; i < restored.source().data().arrays.size(); ++i)
        EXPECT_EQ(restored.source().data().arrays[i].bytes, ActualSource().data().arrays[i].bytes);
    ASSERT_EQ(restored.arrays().size(), ActualMapping().arrays().size());
    for (std::size_t i = 0; i < restored.arrays().size(); ++i)
        EXPECT_EQ(restored.arrays()[i].bytes, ActualMapping().arrays()[i].bytes);
    request.archive.file_byte_cap = 16u << 20;
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
}
}
