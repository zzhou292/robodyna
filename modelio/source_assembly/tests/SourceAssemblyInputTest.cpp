#include "AssemblyTestSupport.h"
#include <type_traits>

namespace crash::modelio::assembly::test {
static_assert(!std::is_copy_assignable_v<SourceAssemblyShellInput> && !std::is_move_assignable_v<SourceAssemblyShellInput>);
static_assert(!std::is_copy_assignable_v<SourceAssemblyMaterialInput> && !std::is_move_assignable_v<SourceAssemblyMaterialInput>);
TEST(SourceAssemblyInputs,EveryNativeShellInputKeepsExactGeometryAndSourceMaterial) {
    const SourceAssemblyShellInput adapter(Load()); const auto input = adapter.input(); const auto& d = adapter.source().data();
    ASSERT_EQ(input.qeph_count, 804u); ASSERT_EQ(input.t3_count, 111u); EXPECT_EQ(input.node_count, 1030u);
    const auto check = [&](const auto& native, std::size_t source_parent, unsigned arity) {
        const auto& parent = d.parents.at(source_parent);
        EXPECT_EQ(native.source_parent_id, parent.source_id); EXPECT_EQ(parent.arity, arity);
        const auto& material = d.materials.at(parent.material_index); const auto& section = d.sections.at(parent.section_index);
        SameBits(native.reference.density, material.density_kg_m3); SameBits(native.reference.young_modulus, material.young_pa);
        SameBits(native.reference.poisson_ratio, material.poisson_ratio); SameBits(native.reference.thickness, section.thickness_m[0]);
        for (unsigned n = 0; n < arity; ++n) {
            EXPECT_EQ(native.nodes[n], parent.nodes[n]); const auto& node = d.nodes.at(native.nodes[n]);
            EXPECT_EQ(native.reference.node_ids[n], node.source_id);
            SameBits(native.reference.position[n].x, node.position_m.x);
            SameBits(native.reference.position[n].y, node.position_m.y);
            SameBits(native.reference.position[n].z, node.position_m.z);
        }
    };
    for (std::size_t i = 0; i < input.qeph_count; ++i) check(input.qeph[i], adapter.qeph_source_parents()[i], 4);
    for (std::size_t i = 0; i < input.t3_count; ++i) check(input.t3[i], adapter.t3_source_parents()[i], 3);
}
TEST(SourceAssemblyInputs,MaterialRangesKeepEveryCurveAndExactParentAssociation) {
    const SourceAssemblyMaterialInput adapter(Load(), MaterialRatePolicy::OpenRadiossDirectImportDefault);
    const auto input = adapter.input(); const auto& d = adapter.source().data();
    ASSERT_EQ(input.curve_count, 2u); ASSERT_EQ(input.material_count, 6u); ASSERT_EQ(input.section_count, 6u); ASSERT_EQ(input.parent_count, 915u);
    for (std::size_t i = 0; i < input.curve_count; ++i) {
        EXPECT_EQ(input.curves[i].curve_id, d.curves[i].id); ASSERT_EQ(input.curves[i].curve.count, d.curves[i].plastic_strain.size());
        for (std::size_t n = 0; n < input.curves[i].curve.count; ++n) {
            SameBits(input.curves[i].curve.plastic_strain[n], d.curves[i].plastic_strain[n]);
            SameBits(input.curves[i].curve.yield_stress_pa[n], d.curves[i].stress_pa[n]);
        }
    }
    for (std::size_t i = 0; i < input.material_count; ++i) {
        const auto& m = input.materials[i]; const auto& original = d.materials[i];
        EXPECT_EQ(m.material_id, original.id); EXPECT_EQ(m.curve_id, original.curve_id);
        SameBits(m.density_kg_m3, original.density_kg_m3); SameBits(m.young_pa, original.young_pa);
        SameBits(m.poisson_ratio, original.poisson_ratio); EXPECT_TRUE(m.rate.enabled);
        SameBits(m.rate.cowper_symonds_c_per_s, original.rate_c_per_s); SameBits(m.rate.cowper_symonds_p, original.rate_p);
        SameBits(m.rate.cutoff_hz, 10000);
        EXPECT_EQ(m.continuation, tl::material::ShellPlasticityCurveContinuation::NativeLastSegment);
    }
    for (std::size_t i = 0; i < input.parent_count; ++i) {
        const auto& p = input.parents[i]; const auto& original = d.parents[i];
        EXPECT_EQ(p.source_parent_id, original.source_id); EXPECT_EQ(p.source_part_id, original.part_id);
        EXPECT_EQ(p.material_id, original.material_id); EXPECT_EQ(p.section_id, original.section_id);
        EXPECT_EQ(p.family_index, original.family_index);
        EXPECT_EQ(p.family, original.arity == 4 ? tl::fea::ShellBindingFamily::Qeph : tl::fea::ShellBindingFamily::T3);
    }
}
TEST(SourceAssemblyInputs,CopiedAndMovedAdaptersRetainTheirOwnBorrowedViews) {
    auto original = std::make_unique<SourceAssemblyShellInput>(Load());
    auto copy = *original;
    EXPECT_NE(copy.input().qeph, original->input().qeph);
    original.reset(); const auto first_id = copy.input().qeph[0].source_parent_id;
    auto moved = std::move(copy); EXPECT_EQ(moved.input().qeph[0].source_parent_id, first_id);
    auto first_material = std::make_unique<SourceAssemblyMaterialInput>(Load(), MaterialRatePolicy::OpenRadiossDirectImportDefault);
    auto copied_material = *first_material;
    EXPECT_NE(copied_material.input().curves, first_material->input().curves);
    const auto curve_values = copied_material.input().curves[1].curve.yield_stress_pa;
    first_material.reset(); auto moved_material = std::move(copied_material);
    EXPECT_EQ(moved_material.input().curves[1].curve.yield_stress_pa, curve_values);
    SameBits(curve_values[45], 362e6);
}
TEST(SourceAssemblyInputs,UnrecognizedRatePolicyIsRejectedWithoutChangingSource) {
    const auto source = Load(); const auto* nodes = source.data().nodes.data();
    EXPECT_THROW(SourceAssemblyMaterialInput(source, static_cast<MaterialRatePolicy>(123)), std::runtime_error);
    EXPECT_EQ(source.data().nodes.data(), nodes); EXPECT_EQ(source.data().identity.sha256, PinnedYarisSixPartInventory().sha256);
}
}  // namespace crash::modelio::assembly::test
