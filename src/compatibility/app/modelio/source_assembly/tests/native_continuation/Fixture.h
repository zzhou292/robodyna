#pragma once
#include "modelio/source_assembly/NativeMaterialInput.h"
#include "lib_utest/qualification/plasticity_binding/PlasticityBindingFixture.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
namespace crash::modelio::assembly::continuation_test {
namespace fe = tl::fea;
namespace mat = tl::material;
using Policy = mat::ShellPlasticityCurveContinuation;
inline Material Source(const fe::ShellPlasticityMaterialInput& native) {
    Material source;
    source.id = native.material_id; source.curve_id = native.curve_id;
    source.young_pa = native.young_pa; source.poisson_ratio = native.poisson_ratio;
    source.density_kg_m3 = native.density_kg_m3;
    source.rate_c_per_s = 8000; source.rate_p = 8;
    return source;
}
struct Fixture : plasticity_binding_test::Fixture {
    fe::ShellBatchBinding geometry;
    fe::ShellBatchPlasticityBinding binding;
    fe::ShellBatchFailureBinding failure;
    Fixture() {
        for (auto& material : materials) material = detail::NativeMaterial(Source(material));
        EXPECT_EQ(geometry.Initialize(collection()).status, fe::ShellBindingStatus::Success);
        EXPECT_EQ(binding.InitializeSections(geometry, catalog()).status, fe::ShellPlasticityBindingStatus::Success);
        fe::ShellFailureParentInput rows[2];
        for (unsigned i = 0; i < 2; ++i) rows[i].source = parents[i];
        rows[0].policy = fe::ShellFailurePolicy::ConstantAllPoints;
        rows[0].constant.failure_strain = 1.;
        EXPECT_EQ(failure.Initialize(binding, rows, 2).status, fe::ShellPlasticityBindingStatus::Success);
    }
    fe::sections::PointParameters Parameters(fe::ShellBindingFamily family) const {
        fe::sections::PointParameters parameters;
        EXPECT_TRUE(binding.Parameters(family, 0, &parameters));
        return parameters;
    }
};
inline void Exact(const fe::sections::ShellLayeredJ2Result& a, const fe::sections::ShellLayeredJ2Result& b) {
    const auto same = [](double x, double y) { EXPECT_EQ(output::Bits(x), output::Bits(y)); };
    for (unsigned p = 0; p < 3; ++p) {
        for (unsigned c = 0; c < 5; ++c) same(a.history.point[p].stress[c], b.history.point[p].stress[c]);
        same(a.history.point[p].plastic_strain, b.history.point[p].plastic_strain);
        same(a.history.point[p].filtered_rate_per_s, b.history.point[p].filtered_rate_per_s);
    }
    for (unsigned c = 0; c < 5; ++c) same(a.material_stress[c], b.material_stress[c]);
    for (unsigned c = 0; c < 3; ++c) same(a.bending_stress[c], b.bending_stress[c]);
    same(a.reported_thickness, b.reported_thickness);
    const auto& x = a.diagnostics; const auto& y = b.diagnostics;
    same(x.plastic_work_density_increment,y.plastic_work_density_increment);
    same(x.maximum_plastic_strain,y.maximum_plastic_strain); same(x.mean_plastic_strain,y.mean_plastic_strain);
    same(x.minimum_tangent_ratio,y.minimum_tangent_ratio); same(x.mean_tangent_ratio,y.mean_tangent_ratio);
    same(x.mean_yield_before_pa,y.mean_yield_before_pa); same(x.last_point_yield_before_pa,y.last_point_yield_before_pa);
}
}
