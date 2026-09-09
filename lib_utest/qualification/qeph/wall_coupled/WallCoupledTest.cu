#include "WallCoupledFixture.h"

namespace qeph_wall_test {
using WallCoupledCuda=tl_test::nodal_temporal::NodalTemporalCuda;
TEST_F(WallCoupledCuda, SameTiltedReferenceHasIndependentMassAndNonuniformContactPreloadAtRest) {
  for(unsigned cells:{1u,2u}) {
    SCOPED_TRACE(cells);
    WallRig w(cells); ASSERT_TRUE(w.Initialize(H0)); auto& r=w.shell;
    NativeSequence native; ASSERT_TRUE(native.Initialize(r));
    Snapshot initial; ASSERT_TRUE(Read(r.owner,initial)); OwnerAgreement(r,initial,native.state);
    const auto contact=w.Host(initial,0,1); ASSERT_TRUE(contact.valid);
    PortResults cache{}; q::BatchDiagnostics d; ASSERT_TRUE(ReadAccepted(r,cache,d));
    EXPECT_FALSE(d.has_completed_interval); EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    double minimum=DepthCap,maximum=0,total_mass=0;
    for(unsigned n=0;n<r.n;++n) {
      minimum=std::min(minimum,r.x[3*n]); maximum=std::max(maximum,r.x[3*n]); total_mass+=r.mass[n];
      for(unsigned a=0;a<3;++a) { EXPECT_EQ(initial.v[3*n+a],0); EXPECT_EQ(initial.omega[3*n+a],0); }
      EXPECT_EQ(initial.q[4*n],1);
    }
    EXPECT_GT(minimum,0); EXPECT_LT(maximum,DepthCap); EXPECT_GT(maximum-minimum,.0001);
    const long double area=static_cast<long double>(Side)*Side*cells;
    EXPECT_NEAR(total_mass,static_cast<double>(area*Density*Thickness),2e-12*total_mass);
    EXPECT_NEAR(w.weights.total_area().value,static_cast<double>(area),2e-12*area);
    for(unsigned e=0;e<cells;++e) {
      EXPECT_EQ(cache[e].proposed_history.data().internal_work[0],0);
      EXPECT_EQ(cache[e].proposed_history.data().internal_work[1],0);
      EXPECT_EQ(cache[e].proposed_history.data().hourglass_viscous_work,0);
      for(unsigned i=0;i<4;++i) {
        EXPECT_EQ(qeph_startup_test::Length(cache[e].internal_force[i]),0);
        EXPECT_EQ(qeph_startup_test::Length(cache[e].internal_couple[i]),0);
        const auto n=r.element[e].nodes[i]; const auto x=r.element[e].reference.input.position[i];
        EXPECT_EQ(x.x,initial.x[3*n]); EXPECT_EQ(x.y,initial.x[3*n+1]); EXPECT_EQ(x.z,initial.x[3*n+2]);
      }
    }
    EXPECT_GT(contact.potential.lower,0); EXPECT_GT(contact.resultant.lower,0);
    for(unsigned j=0;j<contact.node_count;++j) {
      const auto& point=contact.nodes[j];
      EXPECT_TRUE(point.touching_or_penetrating); EXPECT_FALSE(point.fixed);
      EXPECT_EQ(point.surface_power,0); EXPECT_EQ(point.force_world.y,0); EXPECT_EQ(point.force_world.z,0);
    }
    const auto suffix="_cells_"+std::to_string(cells);
    Property("minimum_represented_preload_m"+suffix,minimum);
    Property("maximum_represented_preload_m"+suffix,maximum);
    Property("initial_contact_potential_J"+suffix,contact.potential.value);
    Property("total_native_mass_kg"+suffix,total_mass);
    Property("contact_rate_diagnostic_s2"+suffix,w.wall.stiffness_rate_bound());
  }
}
TEST_F(WallCoupledCuda, OneCellContactAndInternalCacheCloseNativeRecurrenceAndLedgers) {
  ASSERT_NO_FATAL_FAILURE(Prefix(1));
}
TEST_F(WallCoupledCuda, SharedTwoCellContactAndInternalCacheCloseNativeRecurrenceAndLedgers) {
  ASSERT_NO_FATAL_FAILURE(Prefix(2));
}
} // namespace qeph_wall_test
