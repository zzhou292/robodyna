#pragma once
#include "lib_src/collision/NodalWallContactDiagnosticIdentity.h"
#include <gtest/gtest.h>
#include <cstring>
namespace wall_observer_test {
namespace c=tlfea::contact;
inline std::uint64_t Raw(double x) {std::uint64_t result;std::memcpy(&result,&x,8);return result;}
inline void Exact(double a,double b) {EXPECT_EQ(Raw(a),Raw(b));}
inline void Exact(c::Vec3 a,c::Vec3 b) {Exact(a.x,b.x);Exact(a.y,b.y);Exact(a.z,b.z);}
inline void Exact(c::Q4CertifiedIntegral a,c::Q4CertifiedIntegral b) {
  Exact(a.value,b.value);Exact(a.lower,b.lower);Exact(a.upper,b.upper);Exact(a.error,b.error);
}
inline void Exact(const c::NodalWallPointResult& a,const c::NodalWallPointResult& b) {
  EXPECT_EQ(a.node,b.node);EXPECT_EQ(a.fixed,b.fixed);EXPECT_EQ(a.valid,b.valid);
  EXPECT_EQ(a.touching_or_penetrating,b.touching_or_penetrating);EXPECT_EQ(a.base_epoch,b.base_epoch);EXPECT_EQ(a.attempt,b.attempt);
  Exact(a.force,b.force);Exact(a.potential,b.potential);Exact(a.stiffness,b.stiffness);
  Exact(a.force_world,b.force_world);Exact(a.wall_point,b.wall_point);Exact(a.wall_reaction,b.wall_reaction);Exact(a.wall_moment,b.wall_moment);
  Exact(a.surface_power,b.surface_power);Exact(a.local_velocity_first_timestep,b.local_velocity_first_timestep);
  EXPECT_EQ(a.row.count,b.row.count);EXPECT_EQ(a.row.valid,b.row.valid);EXPECT_EQ(a.row.base_epoch,b.row.base_epoch);EXPECT_EQ(a.row.attempt,b.row.attempt);
  for(unsigned i=0;i<c::kMaxNormalNodes;++i) {
    EXPECT_EQ(a.row.nodes[i],b.row.nodes[i]);Exact(a.row.stiffness[i],b.row.stiffness[i]);Exact(a.row.damping[i],b.row.damping[i]);
  }
}
inline void Exact(const c::NodalWallParentResult& a,const c::NodalWallParentResult& b) {
  EXPECT_EQ(a.parent_element_id,b.parent_element_id);EXPECT_EQ(a.parent_face_id,b.parent_face_id);EXPECT_EQ(a.feature_id,b.feature_id);
  EXPECT_EQ(a.arity,b.arity);EXPECT_EQ(a.family,b.family);EXPECT_EQ(a.valid,b.valid);
  Exact(a.resultant,b.resultant);Exact(a.potential,b.potential);for(unsigned i=0;i<4;++i)Exact(a.force[i],b.force[i]);
}
inline void ExactDiagnostics(c::NodalWallDiagnostics a,c::NodalWallDiagnostics b) {
  // Named scalar checks handle NaN/signed-zero partial failures; never compare
  // unrelated compiler padding as evidence of a successful result.
  Exact(a.resultant,b.resultant);Exact(a.potential,b.potential);Exact(a.wall_reaction,b.wall_reaction);
  Exact(a.wall_moment,b.wall_moment);Exact(a.surface_power,b.surface_power);Exact(a.maximum_penetration,b.maximum_penetration);
  a.resultant=b.resultant={};a.potential=b.potential={};a.wall_reaction=b.wall_reaction={};
  a.wall_moment=b.wall_moment={};a.surface_power=b.surface_power=0;a.maximum_penetration=b.maximum_penetration=0;
  EXPECT_EQ(a.valid,b.valid);a.valid=b.valid=true;
  EXPECT_TRUE(c::nodal_wall_device_detail::SameDiagnostics(a,b));
}
inline void ExactNonObserverDiagnostics(c::NodalWallDiagnostics a,c::NodalWallDiagnostics b) {
  a.resultant=b.resultant={};a.potential=b.potential={};a.wall_reaction=b.wall_reaction={};
  a.wall_moment=b.wall_moment={};a.surface_power=b.surface_power=0;
  ExactDiagnostics(a,b);
}
} // namespace wall_observer_test
