#include "TestSupport.h"
#include "modelio/source_assembly/NativeMaterialInput.h"
#include <map>

namespace crash::modelio::vehicle::test {
TEST(VehicleSectionResolution, OrdinaryFailZeroTableKeepsItsSourceAndUsesNativeContinuation) {
    // Regression source for the captured Yaris failure. Selection in production
    // is by native declaration type, never this part or element ID.
    const auto& resolution = Resolution();
    const auto p = PartIndex(2000141);
    ASSERT_EQ(resolution.parts()[p].status, SectionDisposition::Existing);
    ASSERT_EQ(resolution.material(p), Plan().material(p));
    const auto& source = *resolution.material(p);
    const auto& native = *resolution.native_material(p);
    EXPECT_EQ(source.hardening, assembly::MaterialHardening::TabulatedLaw44);
    ASSERT_NE(source.curve_id, 0u);
    EXPECT_EQ(native.curve_id, source.curve_id);
    EXPECT_EQ(native.continuation, tl::material::ShellPlasticityCurveContinuation::NativeLastSegment);
    EXPECT_EQ(native.rate.policy, tl::material::ShellPlasticityRatePolicy::Legacy);
    EXPECT_EQ(output::Bits(native.rate.cowper_symonds_c_per_s), output::Bits(source.rate_c_per_s));
    EXPECT_EQ(output::Bits(native.rate.cowper_symonds_p), output::Bits(source.rate_p));
    std::size_t parents = 0;
    for (std::size_t i = 0; i < resolution.parents().size(); ++i) {
        if (resolution.parents()[i].part_index != p) continue;
        const auto* row = resolution.native_parent(i);
        ASSERT_NE(row, nullptr);
        EXPECT_EQ(row->policy, tl::fea::ShellFailurePolicy::None);
        EXPECT_EQ(row->source.source_part_id, 2000141u);
        ++parents;
    }
    EXPECT_EQ(parents, Plan().parts()[p].shell_count);
    EXPECT_GT(parents, 0u);
}

TEST(VehicleSectionResolution, CompleteOriginalScopeResolvesOnlyOrdinaryConstantFailure) {
    const auto& r = Resolution();
    const auto& count = r.counts();
    EXPECT_EQ(count.parts, 867);
    EXPECT_EQ(count.shells, 349645);
    EXPECT_EQ(count.existing_parts, 823);
    EXPECT_EQ(count.existing_shells, 278301);
    EXPECT_EQ(count.failure_parts, 12);
    EXPECT_EQ(count.failure_shells, 47781);
    EXPECT_EQ(count.unresolved_parts, 32);
    EXPECT_EQ(count.unresolved_shells, 23563);
    const std::map<std::uint64_t, std::pair<std::size_t, double>> expected = {
        {2000003, {5596, 3.5}}, {2000037, {3237, 3.5}}, {2000173, {9234, 1}},
        {2000190, {3112, 1}}, {2000191, {1546, 1}}, {2000319, {9491, 1}},
        {2000322, {3271, 1}}, {2000323, {2976, 1}}, {2000345, {3455, 1}},
        {2000348, {2830, 1}}, {2000352, {1521, 1}}, {2000357, {1512, .25}}};
    std::size_t failure_parts = 0, analytic = 0, table = 0;
    for (std::size_t p = 0; p < r.parts().size(); ++p) {
        const auto& original = Plan().parts()[p];
        const auto& part = r.parts()[p];
        if (part.status == SectionDisposition::Unresolved) {
            EXPECT_EQ(r.material(p), nullptr);
            EXPECT_EQ(r.section(p), nullptr);
            EXPECT_EQ(r.native_material(p), nullptr);
            continue;
        }
        ASSERT_NE(r.native_material(p), nullptr);
        if (part.status == SectionDisposition::Existing) {
            EXPECT_EQ(r.material(p), Plan().material(p));
            EXPECT_EQ(r.section(p), Plan().section(p));
            const auto expected = r.material(p)->curve_id
                ? tl::material::ShellPlasticityCurveContinuation::NativeLastSegment
                : tl::material::ShellPlasticityCurveContinuation::StrictDomain;
            EXPECT_EQ(r.native_material(p)->continuation, expected);
            continue;
        }
        ++failure_parts;
        const auto found = expected.find(original.part_id);
        ASSERT_NE(found, expected.end());
        EXPECT_EQ(original.shell_count, found->second.first);
        EXPECT_EQ(output::Bits(part.failure_strain), output::Bits(found->second.second));
        const auto& material = *r.material(p);
        const auto& native = *r.native_material(p);
        EXPECT_EQ(material.source.raw_text, original.unresolved_sources[2].raw_text);
        EXPECT_EQ(r.section(p)->source.raw_text, original.unresolved_sources[1].raw_text);
        EXPECT_EQ(output::Bits(native.young_pa), output::Bits(material.young_pa));
        EXPECT_EQ(output::Bits(native.density_kg_m3), output::Bits(material.density_kg_m3));
        EXPECT_EQ(output::Bits(native.poisson_ratio), output::Bits(material.poisson_ratio));
        EXPECT_DOUBLE_EQ(native.rate.cutoff_hz, 10000);
        if (material.curve_id) {
            table += original.shell_count;
            ASSERT_TRUE(material.supplied_etan_pa);
            EXPECT_EQ(output::Bits(*material.supplied_etan_pa), output::Bits(0.0));
            EXPECT_EQ(native.continuation, tl::material::ShellPlasticityCurveContinuation::NativeLastSegment);
        } else {
            analytic += original.shell_count;
            EXPECT_EQ(native.hardening, tl::material::ShellPlasticityHardeningKind::LinearLaw44);
            EXPECT_EQ(output::Bits(native.linear.tangent_modulus_pa), output::Bits(*material.supplied_etan_pa));
        }
    }
    EXPECT_EQ(failure_parts, expected.size());
    EXPECT_EQ(analytic, 10345);
    EXPECT_EQ(table, 37436);
    EXPECT_EQ(r.parts()[PartIndex(2000524)].status, SectionDisposition::Unresolved);
    EXPECT_EQ(r.failure_curves().size(), 3);
    EXPECT_EQ(Plan().counts().supported_parents, 278301);
    EXPECT_LT(r.startup_budget_bytes(), 512 * 1024 * 1024);
}

TEST(VehicleSectionResolution, FullOriginalTopologyAndNativePolicyOrderRetainSharedSourceLifetime) {
    const auto& r = Resolution();
    auto copy = r;
    auto moved = std::move(copy);
    EXPECT_EQ(copy.parents().data(), r.parents().data());
    EXPECT_EQ(moved.parents().data(), r.parents().data());
    EXPECT_EQ(&r.source().canonical().data(), &Canonical().data());
    const auto& array = source::FindArray(Canonical().data(), "shells_records");
    const auto records = output::arrays::Decode<std::uint64_t>(array.descriptor, array.bytes);
    std::size_t q4 = 0, t3 = 0, failures = 0, unresolved = 0;
    for (std::size_t e = 0; e < r.parents().size(); ++e) {
        const auto& parent = r.parents()[e];
        ASSERT_EQ(parent.canonical_parent, Plan().parents()[e].canonical_parent);
        ASSERT_EQ(parent.part_index, Plan().parents()[e].part_index);
        const auto* row = records.data() + 6 * parent.canonical_parent;
        ASSERT_EQ(parent.source_parent_id, row[0]);
        const bool quad = row[4] != row[5];
        ASSERT_EQ(parent.topology, quad ? SourceShellTopology::Q4 : SourceShellTopology::T3);
        ASSERT_EQ(parent.topology_index, quad ? q4++ : t3++);
        const auto* native = r.native_parent(e);
        const auto status = r.parts()[parent.part_index].status;
        if (status == SectionDisposition::Unresolved) {
            ASSERT_EQ(native, nullptr);
            ++unresolved;
            continue;
        }
        ASSERT_NE(native, nullptr);
        ASSERT_EQ(native->source.family_index, parent.topology_index);
        ASSERT_EQ(native->source.source_parent_id, row[0]);
        ASSERT_EQ(native->source.source_part_id, row[1]);
        ASSERT_EQ(native->source.material_id, Plan().parts()[parent.part_index].material_id);
        ASSERT_EQ(native->source.section_id, Plan().parts()[parent.part_index].section_id);
        const bool failed = status == SectionDisposition::ConstantFailure;
        ASSERT_EQ(native->policy, failed ? tl::fea::ShellFailurePolicy::ConstantAllPoints : tl::fea::ShellFailurePolicy::None);
        if (failed) {
            ASSERT_EQ(output::Bits(native->constant.failure_strain), output::Bits(r.parts()[parent.part_index].failure_strain));
            ++failures;
        }
    }
    EXPECT_EQ(q4, 328344);
    EXPECT_EQ(t3, 21301);
    EXPECT_EQ(failures, 47781);
    EXPECT_EQ(unresolved, 23563);
    EXPECT_EQ(r.native_parent(r.parents().size()), nullptr);
}
} // namespace crash::modelio::vehicle::test
