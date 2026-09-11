#include "ReferenceSourceSupport.h"
#include "modelio/vehicle_sections/tests/GlassSourceSupport.h"

namespace crash::cases::vehicle_startup::test {
TEST(VehicleGlassReference, CompleteNativeAssessmentPreservesPriorRowsAndBindsPlacement) {
    const auto& resolution = source_test::GlassResolution();
    const auto limits = ReferenceLimits::ResolvedSections();
    const auto forecast = ForecastReferences(resolution,limits);
    RecordProperty("forecast_bytes",std::to_string(forecast.total_bytes));
    RecordProperty("source_bound_bytes",std::to_string(forecast.source_bound_bytes));
    RecordProperty("reference_capacity_bytes",std::to_string(forecast.reference_capacity_bytes));
    const auto references = VehicleShellReferences::Prepare(resolution,limits);
    ASSERT_EQ(references.rows().size(),349645);
    EXPECT_EQ(references.counts().attempted,340292);
    EXPECT_EQ(references.counts().unresolved,9353);
    EXPECT_EQ(references.counts().rejected,0);
    EXPECT_EQ(references.counts().succeeded,340292);
    EXPECT_EQ(references.first_error(),nullptr);
    ASSERT_NE(references.resolution(),nullptr);
    EXPECT_EQ(references.resolution()->parents().data(),resolution.parents().data());
    EXPECT_EQ(&references.source().canonical().data(),&resolution.source().canonical().data());
    const auto prior = VehicleShellReferences::Prepare(source_test::Resolution());
    const detail::Geometry geometry(references.source().canonical().data());
    std::size_t glass = 0,placed = 0;
    for (std::size_t e = 0; e < references.rows().size(); ++e) {
        const auto& row = references.rows()[e];
        const auto& old = prior.rows()[e];
        ASSERT_EQ(row.element_id,old.element_id);
        ASSERT_EQ(row.part_id,old.part_id);
        ASSERT_EQ(row.material_id,old.material_id);
        ASSERT_EQ(row.section_id,old.section_id);
        ASSERT_EQ(row.source_line,old.source_line);
        ASSERT_EQ(row.canonical_parent,old.canonical_parent);
        ASSERT_EQ(row.part_index,old.part_index);
        const auto& part = resolution.parts()[row.part_index];
        if (part.status != modelio::vehicle::SectionDisposition::GlassTab1) {
            ASSERT_EQ(row.status,old.status);
            ASSERT_EQ(row.family,old.family);
            if (const auto* q = prior.qeph(e)) Same(*references.qeph(e),*q);
            if (const auto* t = prior.t3(e)) Same(*references.t3(e),*t);
            continue;
        }
        ++glass;
        if (part.placement != tl::fea::ShellReferencePlacement::Centered) ++placed;
        ASSERT_EQ(old.status,ReferenceStatus::UnresolvedDeclaration);
        const auto& material = *resolution.material(row.part_index);
        const auto& section = *resolution.section(row.part_index);
        const auto check_input = [&](const auto& input) {
            EXPECT_EQ(input.placement,part.placement);
            Same(input.young_modulus,material.young_pa);
            Same(input.poisson_ratio,material.poisson_ratio);
            Same(input.density,material.density_kg_m3);
            Same(input.thickness,section.thickness_m[0]);
            for (std::size_t n = 0; n < std::extent_v<decltype(input.position)>; ++n) {
                const auto global = geometry.connections[4*row.canonical_parent+n];
                EXPECT_EQ(input.node_ids[n],geometry.node_ids[global]);
                Same(input.position[n],tl::math::Vec3{geometry.positions[3*global],
                    geometry.positions[3*global+1],geometry.positions[3*global+2]});
            }
        };
        if (const auto* q = references.qeph(e)) {
            check_input(q->input);
            CheckNative(*q);
        } else {
            ASSERT_NE(references.t3(e),nullptr);
            check_input(references.t3(e)->input);
            CheckNative(*references.t3(e));
        }
    }
    EXPECT_EQ(glass,14210);
    EXPECT_EQ(placed,8502);
    auto short_limits = limits;
    short_limits.host_bytes = forecast.total_bytes-1;
    EXPECT_THROW(VehicleShellReferences::Prepare(resolution,short_limits),std::runtime_error);
    short_limits = limits;
    short_limits.parents = 349644;
    EXPECT_THROW(VehicleShellReferences::Prepare(resolution,short_limits),std::runtime_error);
    EXPECT_EQ(references.counts().succeeded,340292);
    auto exact = limits;
    exact.parents = 349645;
    exact.nodes = resolution.source().counts().nodes;
    exact.host_bytes = forecast.total_bytes;
    // Retry admission without allocating a redundant third full reference set.
    EXPECT_EQ(ForecastReferences(resolution,exact).total_bytes,forecast.total_bytes);
    RecordProperty("resolution_sha256",resolution.identity().sha256);
    RecordProperty("native_succeeded",std::to_string(references.counts().succeeded));
}
} // namespace crash::cases::vehicle_startup::test
