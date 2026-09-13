#include "Source.h"
#include "lib_src/collision/FixedContactFacetBinding.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {

TEST(VehicleSelfContactSource,
    OriginalContactSetIntersectionAndFixedFacetWarpInventory) {
    const auto& original = OriginalContactSelection().data();
    const auto& physical = Execution().physical();
    const auto& selected = ContactInventory();
    const auto& surface = ContactSurface();
    ASSERT_EQ(original.counts.selected_parts, 861u);
    ASSERT_EQ(original.counts.retained_shells, 337092u);
    ASSERT_EQ(original.counts.solids, 2952u);
    ASSERT_EQ(original.counts.beams, 0u);
    ASSERT_EQ(selected.centered.size() + selected.excluded.size(),
        original.counts.retained_shells);
    ASSERT_EQ(surface.parents().size(), selected.centered.size());
    ASSERT_TRUE(surface.MatchesPhysical(physical));

    std::size_t q4 = 0, t3 = 0;
    std::set<std::uint64_t> centered_parts, offset_parts;
    for (const auto& parent : surface.parents()) {
        q4 += parent.arity == 4;
        t3 += parent.arity == 3;
        ASSERT_TRUE(parent.arity == 3 || parent.arity == 4);
        centered_parts.insert(parent.source.source_part_id);
    }
    for (const auto& parent : selected.excluded)
        offset_parts.insert(parent.source_part_id);
    EXPECT_EQ(q4 + t3, selected.centered.size());

    std::vector<double> positions;
    positions.reserve(3 * physical.domain()->node_count());
    for (const auto& node : physical.domain()->nodes()) {
        positions.push_back(node.position.x);
        positions.push_back(node.position.y);
        positions.push_back(node.position.z);
    }
    const contact::VectorView view{positions.data(),
        static_cast<std::uint32_t>(physical.domain()->node_count()), 3, 1};
    ASSERT_TRUE(view.valid());

    for (unsigned level = 0; level <= 2; ++level) {
        contact::FixedContactFacetConfig config;
        config.level = level;
        const auto limits = contact::FixedContactFacetLimits::Vehicle();
        const auto preflight =
            contact::FixedContactFacetBinding::Preflight(
                surface, config, limits);
        ASSERT_EQ(preflight.report.status,
            contact::FixedContactFacetStatus::Ok);
        const auto n = std::size_t{1} << level;
        const auto expected_facets = q4 * 2 * n * n + t3 * n * n;
        ASSERT_EQ(preflight.forecast.parents, surface.parents().size());
        ASSERT_EQ(preflight.forecast.facets, expected_facets);
        ASSERT_LE(expected_facets, limits.max_facets);

        contact::FixedContactFacetBinding facets;
        ASSERT_EQ(facets.Initialize(surface, config, limits).status,
            contact::FixedContactFacetStatus::Ok);
        ASSERT_EQ(facets.forecast().facets, expected_facets);
        contact::FacetApproximationSummary approximation;
        ASSERT_EQ(facets.SummarizeApproximation(view, &approximation),
            contact::Status::kOk);
        ASSERT_EQ(approximation.parents, surface.parents().size());
        ASSERT_TRUE(std::isfinite(
            approximation.maximum_bilinear_error_upper_m));
        ASSERT_TRUE(std::isfinite(
            approximation.maximum_vertex_roundoff_upper_m));
        ASSERT_TRUE(std::isfinite(
            approximation.maximum_total_error_upper_m));
        ASSERT_GE(approximation.maximum_total_error_upper_m,
            approximation.maximum_bilinear_error_upper_m);
        const auto maximum_parent =
            approximation.maximum_total_parent == SIZE_MAX ? 0 :
            surface.parents()[approximation.maximum_total_parent]
                .source.source_parent_id;

        // Complete descriptor algebra is covered by the focused facet gate.
        // This source-scale check samples both ends and regular parent strides
        // while the exact complete count comes from the immutable templates.
        std::size_t described = 0;
        std::set<std::size_t> descriptor_parents{
            0, surface.parents().size() - 1};
        for (std::size_t parent = 0;
             parent < surface.parents().size(); parent += 4096)
            descriptor_parents.insert(parent);
        for (std::size_t parent = 0;
             parent < surface.parents().size(); ++parent) {
            if (surface.parents()[parent].arity == 3) {
                descriptor_parents.insert(parent);
                break;
            }
        }
        for (const auto parent : descriptor_parents) {
            const auto count = facets.facet_count(parent);
            ASSERT_EQ(count,
                surface.parents()[parent].arity == 4 ? 2 * n * n : n * n);
            for (unsigned local = 0; local < count; ++local) {
                contact::FixedContactFacet descriptor;
                ASSERT_EQ(facets.Describe(parent, local, &descriptor).status,
                    contact::FixedContactFacetStatus::Ok);
                ASSERT_EQ(descriptor.parent_index, parent);
                ASSERT_EQ(descriptor.level, level);
                ASSERT_EQ(descriptor.local_facet, local);
                ASSERT_EQ(descriptor.source.source_parent_id,
                    surface.parents()[parent].source.source_parent_id);
                ++described;
            }
        }
        EXPECT_GT(described, 0u);
        std::cout << "Original fixed facets level=" << level
                  << " parents=" << surface.parents().size()
                  << " facets=" << expected_facets
                  << " positive_bilinear="
                  << approximation.positive_bilinear_parents
                  << " positive_roundoff="
                  << approximation.positive_vertex_roundoff_parents
                  << " max_bilinear_m="
                  << approximation.maximum_bilinear_error_upper_m
                  << " max_roundoff_m="
                  << approximation.maximum_vertex_roundoff_upper_m
                  << " max_total_m="
                  << approximation.maximum_total_error_upper_m
                  << " max_parent_eid=" << maximum_parent << '\n';
        RecordProperty("level_" + std::to_string(level) + "_facets",
            std::to_string(expected_facets));
        RecordProperty("level_" + std::to_string(level) +
                "_positive_bilinear_parents",
            std::to_string(approximation.positive_bilinear_parents));
        RecordProperty("level_" + std::to_string(level) +
                "_positive_roundoff_parents",
            std::to_string(
                approximation.positive_vertex_roundoff_parents));
    }
    std::cout << "Original contact physical intersection centered="
              << selected.centered.size()
              << " offsets=" << selected.excluded.size()
              << " q4=" << q4 << " t3=" << t3
              << " centered_parts=" << centered_parts.size()
              << " offset_parts=" << offset_parts.size() << '\n';
    RecordProperty("selected_centered_parents",
        std::to_string(selected.centered.size()));
    RecordProperty("selected_offset_parents",
        std::to_string(selected.excluded.size()));
    RecordProperty("selected_centered_parts",
        std::to_string(centered_parts.size()));
    RecordProperty("selected_offset_parts",
        std::to_string(offset_parts.size()));
    RecordProperty("selected_centered_q4", std::to_string(q4));
    RecordProperty("selected_centered_t3", std::to_string(t3));
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
