#pragma once
#include "ReferenceSourceSupport.h"
#include "modelio/vehicle_sections/tests/TestSupport.h"

namespace crash::cases::vehicle_startup::test {
inline const VehicleShellReferences& ResolvedReferences() {
    static const auto value=VehicleShellReferences::Prepare(source_test::Resolution());
    return value;
}
template<class Input>
void CheckResolvedInput(const Input& input,const ReferenceRow& row,const detail::Geometry& geometry) {
    const auto& resolution=source_test::Resolution();
    const auto* material=resolution.material(row.part_index);
    const auto* section=resolution.section(row.part_index);
    ASSERT_NE(material,nullptr);
    ASSERT_NE(section,nullptr);
    EXPECT_EQ(row.material_id,material->id);
    EXPECT_EQ(row.section_id,section->id);
    Same(input.young_modulus,material->young_pa);
    Same(input.poisson_ratio,material->poisson_ratio);
    Same(input.density,material->density_kg_m3);
    Same(input.thickness,section->thickness_m[0]);
    for (std::size_t n=0;n<std::extent_v<decltype(input.position)>;++n) {
        const auto global=geometry.connections[4*row.canonical_parent+n];
        EXPECT_EQ(input.node_ids[n],geometry.node_ids[global]);
        Same(input.position[n],tl::math::Vec3{geometry.positions[3*global],
            geometry.positions[3*global+1],geometry.positions[3*global+2]});
    }
}
} // namespace crash::cases::vehicle_startup::test
