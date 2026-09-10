#include "../MappingArrays.h"
#include "../MappingRecords.h"
#include "../../tests/TestSupport.h"

namespace crash::output::full_shell::source::test {
TEST(SourceMappingValues, DigestBindsCompleteContentButExcludesFilePaths) {
    std::array<NamedArray, 8> entries;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& a = entries[i];
        a.name = detail::MappingSpecs()[i].name;
        const auto layout = detail::MappingLayout(i, 4, 1, 2);
        if (layout.scalar == arrays::Scalar::UInt64) {
            std::vector<std::uint64_t> values(layout.rows * layout.columns, UINT64_C(9007199254740993) + i);
            a.bytes = arrays::Encode(layout, values.data(), values.size());
        } else {
            std::vector<std::uint32_t> values(layout.rows * layout.columns, static_cast<std::uint32_t>(i));
            a.bytes = arrays::Encode(layout, values.data(), values.size());
        }
        a.descriptor = {a.name + ".bin", layout, a.bytes.size(), Sha256(a.bytes)};
    }
    const auto digest = MappingDigest(entries);
    // Independent Python struct.pack('<Q'/'<I') + hashlib oracle, using the
    // documented domain, explicit field names and values above.
    EXPECT_EQ(digest, "d6bc9b035da76036cb7f00e547c3ffad14da0d18339efccd6fbb706e987b08d4");
    for (auto& a : entries) a.descriptor.file = "renamed/" + a.descriptor.file;
    EXPECT_EQ(MappingDigest(entries), digest);
    auto bad = entries;
    bad.back().bytes.back() ^= 1;
    EXPECT_THROW(MappingDigest(bad), std::exception);
    bad.back().descriptor.sha256 = Sha256(bad.back().bytes);
    EXPECT_NE(MappingDigest(bad), digest);
    bad = entries; bad.back().descriptor.layout.fields[0] = "wrong";
    EXPECT_THROW(MappingDigest(bad), std::exception);
    bad = entries; bad[detail::ParentIds].descriptor.layout.rows = UINT64_MAX;
    EXPECT_THROW(MappingDigest(bad), std::exception);
}
TEST(SourceMappingValues, SourceBudgetRejectsBeforeMissingInputFileRead) {
    SourceInputs in;
    in.canonical_root = in.scope_root = in.member_root = "/nonexistent-source-mapping-test";
    in.canonical_manifest = {"manifest.json", std::string(64, 'a'), 1};
    in.scope_report = {"scope.json", std::string(64, 'b'), 1};
    in.source_member = {"source.key", std::string(64, 'c'), 1};
    in.tire_policy = "retain_all";
    in.units = {"t", "mm", "s", 1000, .001, 1};
    SourceLimits limits; limits.host_bytes = 1;
    try {
        CanonicalSource::Read(in, limits);
        FAIL() << "Invalid capacity accepted";
    } catch (const std::exception& e) {
        EXPECT_NE(std::string(e.what()).find("capacity"), std::string::npos);
    }
    auto changed = in.units; changed.time_to_s = 2;
    EXPECT_FALSE(SameUnits(in.units, changed));
    changed = in.units; changed.length_to_m = 0;
    EXPECT_THROW(CheckUnits(changed), std::exception);
}
} // namespace crash::output::full_shell::source::test
