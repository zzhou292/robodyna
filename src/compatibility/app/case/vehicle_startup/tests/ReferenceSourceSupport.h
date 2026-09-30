#pragma once
#include "../ReferenceStorage.h"
#include "ReferenceComparison.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include <cmath>

namespace crash::cases::vehicle_startup::test {
namespace source_test=modelio::vehicle::test;
inline const VehicleShellReferences& References() {
    static const auto value=VehicleShellReferences::Prepare(source_test::Plan());return value;
}
template<class Input> void CheckSourceInput(const Input& input,const ReferenceRow& row,const detail::Geometry& g) {
    const auto& source=References().source();const auto* m=source.material(row.part_index);const auto* s=source.section(row.part_index);
    ASSERT_NE(m,nullptr);ASSERT_NE(s,nullptr);
    Same(input.density,m->density_kg_m3);Same(input.young_modulus,m->young_pa);
    Same(input.poisson_ratio,m->poisson_ratio);Same(input.thickness,s->thickness_m[0]);
    for(std::size_t n=0;n<std::extent_v<decltype(input.position)>;++n) {
        const auto global=g.connections[4*row.canonical_parent+n];
        EXPECT_EQ(input.node_ids[n],g.node_ids[global]);
        Same(input.position[n],tl::math::Vec3{g.positions[3*global],g.positions[3*global+1],g.positions[3*global+2]});
    }
}
template<class Reference> void CheckNative(const Reference& ref) {
    ASSERT_TRUE(ref.prepared);ASSERT_GT(ref.area,0);ASSERT_TRUE(std::isfinite(ref.area));
    for(std::size_t n=0;n<std::extent_v<decltype(ref.nodal_mass)>;++n) {
        EXPECT_GT(ref.nodal_mass[n],0);EXPECT_TRUE(std::isfinite(ref.nodal_mass[n]));
        EXPECT_GT(ref.physical_inertia[n],0);EXPECT_TRUE(std::isfinite(ref.physical_inertia[n]));
        EXPECT_GT(ref.added_inertia[n],0);EXPECT_TRUE(std::isfinite(ref.added_inertia[n]));
        EXPECT_GT(ref.isotropic_inertia[n],0);EXPECT_TRUE(std::isfinite(ref.isotropic_inertia[n]));
    }
}
} // namespace crash::cases::vehicle_startup::test
