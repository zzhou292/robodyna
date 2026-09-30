#include "../SourceBundleInternal.h"
#include "../../tests/TestSupport.h"
#include <limits>

namespace crash::output::full_shell::source::test {
TEST(SourceBundleValues,ChunkLayoutIsExactOrderedBoundedAndSourceSizeAware) {
    const std::string bytes("abc\0defghijk", 12);
    const auto good = detail::DescribeChunks("source", bytes, 5);
    ASSERT_EQ(good.size(), 3u);
    EXPECT_EQ(good[0].record.bytes, 5u);
    EXPECT_EQ(good[1].offset, 5u);
    EXPECT_EQ(good[2].offset, 10u);
    EXPECT_EQ(good[2].record.bytes, 2u);
    EXPECT_EQ(good[0].record.sha256, Sha256(bytes.substr(0, 5)));
    auto bad = good;
    ++bad.back().offset;
    EXPECT_THROW(detail::CheckChunks(bad, bytes.size(), 5), std::exception);
    bad = good;
    --bad.back().record.bytes;
    EXPECT_THROW(detail::CheckChunks(bad, bytes.size(), 5), std::exception);
    bad = good;
    bad.back().record.file = bad.front().record.file;
    EXPECT_THROW(detail::CheckChunks(bad, bytes.size(), 5), std::exception);
    EXPECT_THROW(detail::CheckChunks(good, SIZE_MAX, 5), std::exception);
    EXPECT_THROW(detail::DescribeChunks("source", bytes, 0), std::exception);
    EXPECT_THROW(detail::DescribeChunks("../source", bytes, 5), std::exception);
}
TEST(SourceBundleValues,ReaderCapacityRejectsBeforeMissingDescriptorAccess) {
    SourceInputs in;
    in.canonical_manifest = {"absent", std::string(64, 'a'), 1};
    in.scope_report = {"absent", std::string(64, 'b'), 1};
    in.source_member = {"absent", std::string(64, 'c'), 1};
    in.tire_policy = "retain_all";
    in.units = {"kg", "m", "s", 1, 1, 1};
    SourceLimits limit; limit.host_bytes = 1;
    try {
        ReadSourceBundle("/missing-source-bundle", {"absent", std::string(64, 'd'), 1},
            in, std::string(64, 'e'), limit);
        FAIL();
    } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "Static source byte capacity exceeded");
    }
}
} // namespace crash::output::full_shell::source::test
