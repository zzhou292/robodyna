#pragma once
#include "../VehicleShellReferences.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <type_traits>

namespace crash::cases::vehicle_startup::test {
inline void Same(double a,double b) {EXPECT_EQ(output::Bits(a),output::Bits(b));}
inline void Same(const tl::math::Vec3& a,const tl::math::Vec3& b) {Same(a.x,b.x);Same(a.y,b.y);Same(a.z,b.z);}
template<class T,std::size_t N> void Same(const T (&a)[N],const T (&b)[N]) {
    for(std::size_t i=0;i<N;++i)Same(a[i],b[i]);
}
template<class Input> void SameInput(const Input& a,const Input& b) {
    EXPECT_EQ(a.placement,b.placement);
    Same(a.position,b.position);
    for(std::size_t i=0;i<std::extent_v<decltype(a.position)>;++i)EXPECT_EQ(a.node_ids[i],b.node_ids[i]);
    Same(a.density,b.density);Same(a.thickness,b.thickness);Same(a.young_modulus,b.young_modulus);Same(a.poisson_ratio,b.poisson_ratio);
}
template<class Reference> void Common(const Reference& a,const Reference& b) {
    EXPECT_EQ(a.prepared,b.prepared);SameInput(a.input,b.input);Same(a.frame.v,b.frame.v);Same(a.area,b.area);
    Same(a.local_position,b.local_position);Same(a.nodal_mass,b.nodal_mass);Same(a.physical_inertia,b.physical_inertia);
    Same(a.added_inertia,b.added_inertia);Same(a.isotropic_inertia,b.isotropic_inertia);
}
inline void Same(const tl::fea::qeph::ReferenceData& a,const tl::fea::qeph::ReferenceData& b) {
    Common(a,b);Same(a.derivative_x,b.derivative_x);Same(a.derivative_y,b.derivative_y);
}
inline void Same(const tl::fea::t3::ReferenceData& a,const tl::fea::t3::ReferenceData& b) {
    Common(a,b);Same(a.angle_cosine,b.angle_cosine);Same(a.angle_weight,b.angle_weight);
    Same(a.element_mass,b.element_mass);Same(a.element_isotropic_inertia,b.element_isotropic_inertia);
    Same(a.element_physical_inertia,b.element_physical_inertia);Same(a.element_added_inertia,b.element_added_inertia);
    Same(a.characteristic_length,b.characteristic_length);Same(a.startup_derivative,b.startup_derivative);
}
} // namespace crash::cases::vehicle_startup::test
