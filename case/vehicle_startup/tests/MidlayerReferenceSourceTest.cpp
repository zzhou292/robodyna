#include "ReferenceComparison.h"
#include "modelio/vehicle_sections/tests/GlassSourceSupport.h"
#include "lib_src/elements/qbat/QbatReference.h"
#include "lib_src/elements/t3/T3Startup.h"
#include "lib_utest/qualification/qbat/source_fixture/YarisQbatSourceFixture.h"
#include <algorithm>

namespace crash::cases::vehicle_startup::test {
namespace original=yaris_qbat_source_fixture;
namespace source_test=modelio::vehicle::test;
TEST(VehicleMidlayerReferenceSource, EveryOriginalReferenceAndPriorNamedValueAgrees) {
    const auto& base=source_test::GlassResolution();
    const auto resolution=VehicleSectionResolution::ResolveOriginalMidlayer(base,
        modelio::vehicle::ResolutionProfile::OriginalMidlayerV1);
    const auto forecast=ForecastReferences(resolution);
    const auto current=VehicleShellReferences::Prepare(resolution);
    const auto prior=VehicleShellReferences::Prepare(base);
    ASSERT_EQ(current.rows().size(),349645);
    EXPECT_EQ(current.counts().succeeded,344543);
    EXPECT_EQ(current.counts().rejected,0);
    EXPECT_EQ(current.counts().unresolved,5102);
    EXPECT_EQ(current.counts().qbat_succeeded,4250);
    ASSERT_NE(current.resolution(),nullptr);
    EXPECT_TRUE(current.resolution()->resolution_key()==resolution.resolution_key());
    std::size_t qi=0,triangles=0,old_count=0;
    for(std::size_t e=0;e<current.rows().size();++e) {
        const auto& row=current.rows()[e];
        const auto& old=prior.rows()[e];
        ASSERT_EQ(row.element_id,old.element_id);
        ASSERT_EQ(row.part_id,old.part_id);
        ASSERT_EQ(row.material_id,old.material_id);
        ASSERT_EQ(row.section_id,old.section_id);
        ASSERT_EQ(row.canonical_parent,old.canonical_parent);
        ASSERT_EQ(row.source_line,old.source_line);
        ASSERT_EQ(row.part_index,old.part_index);
        if(row.part_id!=2000524) {
            ASSERT_EQ(row.family,old.family);
            ASSERT_EQ(row.status,old.status);
            if(const auto* q=prior.qeph(e)) { Same(*current.qeph(e),*q); ++old_count; }
            if(const auto* t=prior.t3(e)) { Same(*current.t3(e),*t); ++old_count; }
            continue;
        }
        ASSERT_EQ(old.status,ReferenceStatus::UnresolvedDeclaration);
        const auto& material=*resolution.material(row.part_index);
        const auto& section=*resolution.section(row.part_index);
        if(const auto* q=current.qbat(e)) {
            ASSERT_LT(qi,std::size(original::quads));
            const auto& source=original::quads[qi++];
            EXPECT_EQ(row.element_id,source.id);
            EXPECT_EQ(row.canonical_parent,source.canonical_index);
            EXPECT_EQ(row.source_line,source.source_line);
            tl::fea::qbat::ReferenceInput expected;
            // Independent original fixture geometry. Coefficients use the
            // actual app source SI operations, not the fixture's rounded rho1000.
            for(unsigned n=0;n<4;++n) {
                const auto& node=original::nodes[source.node[n]];
                expected.quadrilateral.node_ids[n]=node.id;
                expected.quadrilateral.position[n]={node.position_m[0],node.position_m[1],node.position_m[2]};
            }
            expected.quadrilateral.density=material.density_kg_m3;
            expected.quadrilateral.young_modulus=material.young_pa;
            expected.quadrilateral.poisson_ratio=material.poisson_ratio;
            expected.quadrilateral.thickness=section.thickness_m[0];
            expected.initial_a11_pa=material.young_pa/(1-material.poisson_ratio*material.poisson_ratio);
            SameInput(q->input().quadrilateral,expected.quadrilateral);
            tl::fea::qbat::Reference native;
            ASSERT_EQ(tl::fea::qbat::InitializeReference(expected,native),tl::fea::qbat::Status::kSuccess);
            Same(q->quadrilateral(),native.quadrilateral());
            const auto& a=q->coefficients();const auto& b=native.coefficients();
            Same(a.characteristic_length_m,b.characteristic_length_m);
            Same(a.sound_speed_m_s,b.sound_speed_m_s);
            Same(a.viscosity_timestep_factor,b.viscosity_timestep_factor);
            Same(a.unscaled_element_dt_s,b.unscaled_element_dt_s);
            Same(a.nodal_translation_stiffness_n_m,b.nodal_translation_stiffness_n_m);
            Same(a.nodal_rotation_stiffness_nm,b.nodal_rotation_stiffness_nm);
            EXPECT_EQ(current.qeph(e),nullptr);
            EXPECT_EQ(current.t3(e),nullptr);
        } else {
            ++triangles;
            ASSERT_EQ(row.element_id,2357656);
            ASSERT_NE(current.t3(e),nullptr);
            tl::fea::t3::ReferenceInput input;
            input.density=material.density_kg_m3;
            input.young_modulus=material.young_pa;
            input.poisson_ratio=material.poisson_ratio;
            input.thickness=section.thickness_m[0];
            constexpr std::uint64_t ids[]{2300357,2300138,2300139};
            for(unsigned n=0;n<3;++n) {
                const auto node=std::find_if(std::begin(original::nodes),std::end(original::nodes),
                    [&](const auto& value) { return value.id==ids[n]; });
                ASSERT_NE(node,std::end(original::nodes));
                input.node_ids[n]=ids[n];
                input.position[n]={node->position_m[0],node->position_m[1],node->position_m[2]};
            }
            SameInput(current.t3(e)->input,input);
            tl::fea::t3::ReferenceData native;
            ASSERT_EQ(tl::fea::t3::InitializeReference(input,native),tl::fea::t3::Status::kSuccess);
            Same(*current.t3(e),native);
        }
    }
    EXPECT_EQ(qi,4250);
    EXPECT_EQ(triangles,1);
    EXPECT_EQ(old_count,340292);
    auto limits=ReferenceLimits::ResolvedSections();
    limits.host_bytes=forecast.total_bytes-1;
    EXPECT_THROW(VehicleShellReferences::Prepare(resolution,limits),std::runtime_error);
    limits.host_bytes=forecast.total_bytes;
    EXPECT_EQ(ForecastReferences(resolution,limits).total_bytes,forecast.total_bytes);
    EXPECT_EQ(current.counts().succeeded,344543);
    RecordProperty("forecast_bytes",std::to_string(forecast.total_bytes));
    RecordProperty("source_bound_bytes",std::to_string(forecast.source_bound_bytes));
    RecordProperty("reference_capacity_bytes",std::to_string(forecast.reference_capacity_bytes));
}
} // namespace crash::cases::vehicle_startup::test
