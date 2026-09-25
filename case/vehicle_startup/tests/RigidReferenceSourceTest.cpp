#include "ReferenceComparison.h"
#include "modelio/vehicle_sections/tests/RigidSourceSupport.h"
#include "../ReferenceStorage.h"
namespace crash::cases::vehicle_startup::test {
namespace source_test=modelio::vehicle::test;
namespace {
void SameQbat(const tl::fea::qbat::Reference& a,const tl::fea::qbat::Reference& b) {
    EXPECT_EQ(a.prepared(),b.prepared());Same(a.quadrilateral(),b.quadrilateral());
    SameInput(a.input().quadrilateral,b.input().quadrilateral);Same(a.input().initial_a11_pa,b.input().initial_a11_pa);
    const auto& x=a.input().options;const auto& y=b.input().options;
    EXPECT_EQ(x.ihbe,y.ihbe);EXPECT_EQ(x.irep,y.irep);EXPECT_EQ(x.ismstr,y.ismstr);
    EXPECT_EQ(x.nptr,y.nptr);EXPECT_EQ(x.npts,y.npts);EXPECT_EQ(x.nptt,y.nptt);EXPECT_EQ(x.layers,y.layers);
    EXPECT_EQ(x.idrill,y.idrill);EXPECT_EQ(x.npinch,y.npinch);EXPECT_EQ(x.material_law,y.material_law);
    EXPECT_EQ(x.property_type,y.property_type);EXPECT_EQ(x.ithick,y.ithick);EXPECT_EQ(x.iplas,y.iplas);
    Same(x.offset_ratio,y.offset_ratio);Same(x.inertia_denominator_override,y.inertia_denominator_override);
    Same(x.membrane_viscosity,y.membrane_viscosity);Same(x.numerical_viscosity,y.numerical_viscosity);
    const auto& c=a.coefficients();const auto& d=b.coefficients();
    Same(c.characteristic_length_m,d.characteristic_length_m);Same(c.sound_speed_m_s,d.sound_speed_m_s);
    Same(c.viscosity_timestep_factor,d.viscosity_timestep_factor);Same(c.unscaled_element_dt_s,d.unscaled_element_dt_s);
    Same(c.nodal_translation_stiffness_n_m,d.nodal_translation_stiffness_n_m);
    Same(c.nodal_rotation_stiffness_nm,d.nodal_rotation_stiffness_nm);
}
template<class Input> void CheckRigidInput(const Input& input,const ReferenceRow& row,
    const detail::Geometry& geometry,const VehicleSectionResolution& resolution) {
    const auto& material=*resolution.material(row.part_index);const auto& section=*resolution.section(row.part_index);
    Same(input.density,material.density_kg_m3);Same(input.young_modulus,material.young_pa);
    Same(input.poisson_ratio,material.poisson_ratio);Same(input.thickness,section.thickness_m[0]);
    EXPECT_EQ(input.placement,tl::fea::ShellReferencePlacement::Centered);
    for(std::size_t n=0;n<std::extent_v<decltype(input.node_ids)>;++n) {
        const auto global=geometry.connections[4*row.canonical_parent+n];
        EXPECT_EQ(input.node_ids[n],geometry.node_ids[global]);
        Same(input.position[n],tl::math::Vec3{geometry.positions[3*global],geometry.positions[3*global+1],geometry.positions[3*global+2]});
    }
}
}
TEST(VehicleRigidReferenceSource, CompleteOriginalRolesAndAllPriorReferenceBitsArePreserved) {
    const auto& resolution=source_test::RigidResolution();
    const auto prior=VehicleShellReferences::Prepare(source_test::MidlayerResolution());
    const auto current=VehicleShellReferences::Prepare(resolution,ReferenceLimits::CompleteRigidOverlay());
    const detail::Geometry geometry(resolution.source().canonical().data());
    ASSERT_EQ(current.rows().size(),349645);EXPECT_EQ(current.counts().unresolved,0);
    EXPECT_EQ(current.counts().succeeded,349645);EXPECT_EQ(current.counts().rejected,0);
    EXPECT_EQ(current.counts().qbat_succeeded,4250);
    std::size_t old_count=0,rigid_count=0;
    for(std::size_t e=0;e<current.rows().size();++e) {
        const auto& row=current.rows()[e];const auto& old=prior.rows()[e];
        ASSERT_EQ(row.element_id,old.element_id);ASSERT_EQ(row.part_id,old.part_id);
        ASSERT_EQ(row.material_id,old.material_id);ASSERT_EQ(row.section_id,old.section_id);
        ASSERT_EQ(row.source_line,old.source_line);ASSERT_EQ(row.canonical_parent,old.canonical_parent);
        ASSERT_EQ(row.part_index,old.part_index);
        if(old.status==ReferenceStatus::Success) {
            ++old_count;EXPECT_EQ(row.role,modelio::vehicle::SourceShellRole::ConstitutiveShell);
            EXPECT_EQ(row.rigid_root_index,SIZE_MAX);ASSERT_EQ(row.family,old.family);
            if(const auto* q=prior.qeph(e)) Same(*current.qeph(e),*q);
            if(const auto* t=prior.t3(e)) Same(*current.t3(e),*t);
            if(const auto* b=prior.qbat(e)) SameQbat(*current.qbat(e),*b);
        } else {
            ++rigid_count;EXPECT_EQ(row.role,modelio::vehicle::SourceShellRole::OriginalRigidPart);
            EXPECT_EQ(row.rigid_root_index,resolution.rigid_root_index(row.part_index));
            ASSERT_EQ(row.status,ReferenceStatus::Success);
            if(const auto* q=current.qeph(e)) CheckRigidInput(q->input,row,geometry,resolution);
            else {ASSERT_NE(current.t3(e),nullptr);CheckRigidInput(current.t3(e)->input,row,geometry,resolution);}
        }
    }
    EXPECT_EQ(old_count,344543);EXPECT_EQ(rigid_count,5102);
    const auto forecast=ForecastReferences(resolution,ReferenceLimits::CompleteRigidOverlay());
    EXPECT_EQ(forecast.qbat_capacity,4250);
    auto limits=ReferenceLimits::CompleteRigidOverlay();limits.host_bytes=forecast.total_bytes-1;
    EXPECT_THROW(VehicleShellReferences::Prepare(resolution,limits),std::runtime_error);
    limits.host_bytes=forecast.total_bytes;EXPECT_EQ(ForecastReferences(resolution,limits).total_bytes,forecast.total_bytes);
    RecordProperty("forecast_bytes",std::to_string(forecast.total_bytes));
    RecordProperty("source_bound_bytes",std::to_string(forecast.source_bound_bytes));
}
}
