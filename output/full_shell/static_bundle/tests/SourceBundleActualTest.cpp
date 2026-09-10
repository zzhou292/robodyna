#include "SourceBundleTestSupport.h"

namespace crash::output::full_shell::source::test {
TEST(SourceBundleActual,ExactOriginalSourceAndMappingRoundtripWithinStaticAndRunPlans) {
    const auto& bundle = ActualBundle();
    const auto& d = bundle.description();
    ASSERT_EQ(d.chunks.size(), 2u);
    EXPECT_EQ(d.chunks[0].offset, 0u);
    EXPECT_EQ(d.chunks[0].record.bytes, 33554432u);
    EXPECT_EQ(d.chunks[1].offset, 33554432u);
    EXPECT_EQ(d.chunks[1].record.bytes, 9292321u);
    EXPECT_EQ(bundle.reservations().size(), 31u);
    std::size_t total = 0;
    for (const auto& f : bundle.reservations()) {
        EXPECT_LE(f.bytes, kArtifactFileCap);
        total += f.bytes;
    }
    EXPECT_EQ(total, d.static_bytes);
    EXPECT_EQ(total, 144409535u + d.mapping.bytes + bundle.descriptor().bytes);
    EXPECT_LT(total, StaticReserveBytes);
    const auto request = BundleOptions();
    std::size_t external = 0;
    for (const auto& f : request.archive.static_files) external += f.bytes;
    EXPECT_EQ(bundle.archive_plan().static_declared_bytes, total + external);
    EXPECT_LT(bundle.archive_plan().forecast_bytes, TotalByteCap);
    ft::Directory directory;
    BundleDirectories(directory.path);
    const auto descriptor = WriteSourceBundle(directory.path, bundle);
    EXPECT_TRUE(detail::SameFile(descriptor, bundle.descriptor()));
    auto authority = ActualInputs();
    authority.canonical_root = authority.scope_root = authority.member_root = "/not-used-by-bundle-reader";
    const auto restored = ReadSourceBundle(directory.path, descriptor, authority, bundle.mapping().digest());
    EXPECT_EQ(restored.digest(), bundle.mapping().digest());
    EXPECT_EQ(restored.parents().size(), 349645u);
    EXPECT_EQ(restored.nodes(), 359785u);
    EXPECT_EQ(restored.triangles(), 677989u);
    for (std::size_t i = 0; i < 17; ++i)
        EXPECT_EQ(restored.source().data().arrays[i].bytes, bundle.mapping().source().data().arrays[i].bytes);
    for (std::size_t i = 0; i < 8; ++i)
        EXPECT_EQ(restored.arrays()[i].bytes, bundle.mapping().arrays()[i].bytes);
    EXPECT_EQ(restored.source().data().scope_bytes, ActualSource().data().scope_bytes);
    EXPECT_EQ(restored.source().data().canonical_bytes, ActualSource().data().canonical_bytes);
    std::size_t measured = 0;
    for (const auto& f : bundle.reservations()) measured += std::filesystem::file_size(directory.path / f.file);
    EXPECT_EQ(measured, total);
    RecordProperty("static_payload_bytes", std::to_string(total));
    RecordProperty("whole_run_forecast_bytes", std::to_string(bundle.archive_plan().forecast_bytes));
}
TEST(SourceBundleActual,ExactLimitsAndUnreservedFutureObligationsRejectBeforeWriting) {
    auto request = BundleOptions();
    request.archive.total_byte_cap = ActualBundle().archive_plan().forecast_bytes - 1;
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
    request = BundleOptions();
    request.archive.static_byte_reserve = ActualBundle().archive_plan().static_declared_bytes +
        ActualBundle().archive_plan().frame_capacity * FrameMetadataByteCap - 1;
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
    request = BundleOptions();
    request.archive.static_files.erase(request.archive.static_files.begin());
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
    request = BundleOptions();
    request.archive.static_files.push_back({ActualSource().data().arrays.front().descriptor.file, 1});
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
    request = BundleOptions();
    request.archive.plastic_points = 3 * request.archive.parents;
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
    request = BundleOptions();
    request.archive.frames = 250;
    EXPECT_THROW(PreparedSourceBundle::Prepare(ActualMapping(), request), std::exception);
}
} // namespace crash::output::full_shell::source::test
