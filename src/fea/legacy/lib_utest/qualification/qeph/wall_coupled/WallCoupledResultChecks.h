#pragma once
#include "WallCoupledFixture.h"
#include "lib_utest/qualification/surface_contact/NodalWallOwnerFixture.h"

namespace qeph_wall_test {
inline void SameNumerics(const sc::NodalWallDeviceResults& a,const sc::NodalWallDeviceResults& b) {
  // Retag a copied bounded result to compare all physical/result fields through
  // the existing host/device oracle. Only attempt identity is different.
  sc::NodalWallResult host;
  host.valid=b.diagnostics.valid; host.node_count=b.diagnostics.node_count; host.parent_count=b.diagnostics.parent_count;
  host.resultant=b.diagnostics.resultant; host.potential=b.diagnostics.potential;
  host.wall_reaction=b.diagnostics.wall_reaction; host.wall_moment=b.diagnostics.wall_moment;
  host.surface_power=b.diagnostics.surface_power;
  for(unsigned n=0;n<host.node_count;++n) {
    EXPECT_EQ(a.nodes[n].attempt,a.diagnostics.attempt); EXPECT_EQ(a.nodes[n].row.attempt,a.diagnostics.attempt);
    EXPECT_EQ(b.nodes[n].attempt,b.diagnostics.attempt); EXPECT_EQ(b.nodes[n].row.attempt,b.diagnostics.attempt);
    host.nodes[n]=b.nodes[n]; host.nodes[n].attempt=a.nodes[n].attempt;
    host.nodes[n].row.attempt=a.nodes[n].row.attempt;
  }
  for(unsigned e=0;e<host.parent_count;++e) host.parents[e]=b.parents[e];
  ContactAgreement(a,host);
  for(unsigned n=0;n<host.node_count;++n) EXPECT_EQ(a.wall_face[n],b.wall_face[n]);
  const auto& x=a.diagnostics; const auto& y=b.diagnostics;
  EXPECT_EQ(x.base_potential,y.base_potential); EXPECT_EQ(x.base_potential_error,y.base_potential_error);
  EXPECT_EQ(x.potential_increment,y.potential_increment); EXPECT_EQ(x.kick_work,y.kick_work);
  EXPECT_EQ(x.kick_work_roundoff,y.kick_work_roundoff); EXPECT_EQ(x.drift_work,y.drift_work);
  EXPECT_EQ(x.drift_work_roundoff,y.drift_work_roundoff); EXPECT_EQ(x.conservative_defect,y.conservative_defect);
  EXPECT_EQ(x.work_uncertainty,y.work_uncertainty); EXPECT_EQ(x.quadratic_work_upper,y.quadratic_work_upper);
  EXPECT_EQ(x.wall_kick_impulse,y.wall_kick_impulse); EXPECT_EQ(x.wall_kick_impulse_error,y.wall_kick_impulse_error);
  nodal_wall_owner_test::Same(x.wall_kick_moment,y.wall_kick_moment);
  nodal_wall_owner_test::Same(x.wall_kick_moment_error,y.wall_kick_moment_error);
  EXPECT_EQ(x.maximum_penetration,y.maximum_penetration); EXPECT_EQ(x.stiffness_rate_bound,y.stiffness_rate_bound);
}
} // namespace qeph_wall_test
