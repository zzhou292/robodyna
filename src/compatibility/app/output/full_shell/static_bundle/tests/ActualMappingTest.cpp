#include "ActualMappingSupport.h"

namespace crash::output::full_shell::source::test {
TEST(SourceMappingActual, CompleteCanonicalAndReorderedMappingRoundtripRetainIdsAndApplicability) {
    if (!std::getenv("ROBO_STATIC_CANONICAL")) GTEST_SKIP() << "Explicit original-source fixture not configured";
    const auto& source = ActualSource();
    const auto& mapping = ActualMapping();
    const auto& d = source.data();
    EXPECT_EQ(d.arrays.size(), 17u);
    EXPECT_EQ(d.selected_parts.size(), 867u);
    EXPECT_EQ(d.excluded_parts.size(), 8u);
    EXPECT_EQ(mapping.nodes(), 359785u);
    EXPECT_EQ(mapping.parents().size(), 349645u);
    EXPECT_EQ(mapping.triangles(), 677989u);
    EXPECT_EQ(mapping.payload_bytes(), 40339684u);
    full_shell::test::Directory dir;
    std::filesystem::create_directory(dir.path / "arrays");
    std::size_t copied = 0;
    for (const auto& a : d.arrays) {
        const auto out = arrays::WriteBytes(dir.path, a.descriptor.file, a.descriptor.layout, a.bytes);
        EXPECT_EQ(out.sha256, a.descriptor.sha256);
        EXPECT_EQ(arrays::ReadBytes(dir.path, out), a.bytes);
        copied += out.bytes;
    }
    EXPECT_EQ(copied, 45415582u);
    const auto plan = PlanMappingRecord(mapping, "mapping");
    EXPECT_EQ(plan.bytes, mapping.payload_bytes() + plan.description.bytes);
    const auto short_plan = PlanMappingRecord(mapping, "short");
    EXPECT_THROW(WriteMappingRecord(dir.path, mapping, "short", short_plan.bytes - 1), std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path / "short-node_source_ids.bin"));
    const auto file = WriteMappingRecord(dir.path, mapping, "mapping");
    const auto restored = ReadMappingRecord(dir.path, source, file, mapping.digest());
    EXPECT_EQ(restored.digest(), mapping.digest());
    for (std::size_t i = 0; i < mapping.arrays().size(); ++i)
        EXPECT_EQ(restored.arrays()[i].bytes, mapping.arrays()[i].bytes);
    Identity id; id.owner = 1; id.run = 2; id.topology = 3; id.source_instance = 4; id.configuration = 5; id.qualification = 6;
    const auto frame = restored.MakeFrameContext(id, 0x1p-26);
    EXPECT_EQ(frame.identity().source_mapping_sha256, mapping.digest());
    EXPECT_EQ(frame.points(), 0u); // No invented constitutive points in this formatting-only fixture.
    for (std::size_t i = 0; i < frame.parents().size(); ++i) {
        EXPECT_EQ(frame.parents()[i].source_element, mapping.parents()[i].source_element);
        EXPECT_EQ(frame.parents()[i].source_part, mapping.parents()[i].source_part);
        EXPECT_EQ(frame.parents()[i].native_points, 0u);
        EXPECT_EQ(frame.parents()[i].plastic, PlasticField::Unavailable);
    }
    id.source_mapping_sha256 = std::string(64, 'a');
    EXPECT_THROW(restored.MakeFrameContext(id, 0x1p-26), std::exception);
}
TEST(SourceMappingActual, LateSourceOrderingAndNativeDeclarationFailuresPreserveSharedInput) {
    if (!std::getenv("ROBO_STATIC_CANONICAL")) GTEST_SKIP() << "Explicit original-source fixture not configured";
    const auto& source = ActualSource();
    const auto& mapping = ActualMapping();
    const auto digest = mapping.digest();
    ActualOrder order(source);
    auto input = order.View(); input.node_count = SIZE_MAX;
    input.canonical_nodes = reinterpret_cast<const std::uint32_t*>(1);
    EXPECT_THROW(PreparedSourceMapping::Prepare(source, input), std::exception);
    const auto last_node = order.nodes.back(); order.nodes.back() = order.nodes.front();
    EXPECT_THROW(PreparedSourceMapping::Prepare(source, order.View()), std::exception);
    order.nodes.back() = last_node;
    const auto last_parent = order.parents.back(); order.parents.back() = order.parents.front();
    EXPECT_THROW(PreparedSourceMapping::Prepare(source, order.View()), std::exception);
    order.parents.back() = last_parent; order.parents.back().plastic = PlasticField::NativeEquivalentPlasticStrain;
    EXPECT_THROW(PreparedSourceMapping::Prepare(source, order.View()), std::exception);
    order.parents.back().plastic = static_cast<PlasticField>(99);
    EXPECT_THROW(PreparedSourceMapping::Prepare(source, order.View()), std::exception);
    order.parents.back() = last_parent;
    const auto retried = PreparedSourceMapping::Prepare(source, order.View());
    EXPECT_EQ(retried.digest(), digest);
    EXPECT_EQ(mapping.digest(), digest);
    auto wrong_units = ActualInputs(); wrong_units.units.length_to_m = .01;
    EXPECT_THROW(CanonicalSource::Read(wrong_units), std::exception);
}
} // namespace crash::output::full_shell::source::test
