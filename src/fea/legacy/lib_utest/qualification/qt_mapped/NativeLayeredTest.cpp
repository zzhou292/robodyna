// SPDX-License-Identifier: MIT
#include "../shell_placement_force/native/NativePacket.h"
#include "lib_src/elements/qeph/mapped/Stiffness.h"
#include "lib_src/elements/t3/mapped/Stiffness.h"
#include "NativeScatter.h"

namespace qt_mapped_test {
namespace placed=placement_force_test;
namespace fe=tl::fea;
template<class Family> void Layered() {
  constexpr bool quad=std::is_same_v<Family,placed::Q>;
  constexpr unsigned nodes=quad?4:3;
  constexpr unsigned sti=quad?146:87;
  constexpr unsigned force_begin=quad?118:64;
  constexpr unsigned couple_begin=quad?130:73;
  for (auto plane:placed::Planes) {
    placed::Fixture<Family> source(plane);
    ASSERT_EQ(InitializeLayeredTab1History(source.reference,source.material,source.failure,{},source.accepted),Family::Status::kSuccess);
    auto oracle=placed::native::Seed(source.accepted);
    auto stationary=placed::Interval(source.reference,0);
    for (unsigned slot=0;slot<nodes;++slot) {
      if constexpr(quad) {
        stationary.position_endpoint[slot]=source.reference.input.position[slot];
        stationary.velocity_midpoint[slot]={};
        stationary.omega_midpoint[slot]={};
      } else {
        stationary.position[slot]=source.reference.input.position[slot];
        stationary.velocity[slot]={};
        stationary.angular_velocity[slot]={};
      }
    }
    // Independent full caller observes virgin coefficients. Production only
    // constructs a coefficient value; it does not advance an initial history.
    placed::native::Advance(source.reference,stationary,source.material,source.failure,oracle);
    fe::shell_nodal_stiffness::Packet<nodes> initial;
    if constexpr(quad) ASSERT_TRUE(fe::qeph::mapped::InitialStiffness(source.reference,fe::ShellSectionLaw::LayeredLaw44Nip3,initial));
    else ASSERT_TRUE(fe::t3::mapped::InitialStiffness(source.reference,fe::ShellSectionLaw::LayeredLaw44Nip3,initial));
    for (unsigned slot=0;slot<nodes;++slot) {
      const double factor=quad?oracle.force[12+slot%2]:1;
      Near(initial.translation[slot],oracle.force[sti]*factor);
      Near(initial.rotation[slot],oracle.force[sti+1]*factor);
    }
    bool removed=false;
    for (unsigned mask: {0u,7u}) {
      placed::Fixture<Family> path(plane,mask);
      auto native=placed::native::Seed(path.accepted);
      for (unsigned step=0;step<32;++step) {
        SCOPED_TRACE(step);
        typename Family::Trial trial;
        ASSERT_EQ(path.Evaluate(step,trial),Family::Status::kSuccess);
        placed::native::Advance(path.reference,placed::Interval(path.reference,step),path.material,path.failure,native);
        fe::shell_nodal_stiffness::Packet<nodes> packet;
        if constexpr(quad) ASSERT_TRUE(fe::qeph::mapped::AcceptedStiffness(trial.force,packet));
        else ASSERT_TRUE(fe::t3::mapped::AcceptedStiffness(trial.force,packet));
        const double coefficients[]{native.force[sti],native.force[sti+1],native.force[12],native.force[13]};
        Scatter(trial.force.internal_force,trial.force.internal_couple,packet,
            native.force.data()+force_begin,native.force.data()+couple_begin,coefficients);
        EXPECT_EQ(trial.force.proposed_history.data().active,native.history[quad?37:25]);
        removed|=!trial.section.history.element_active;
        path.Accept(trial);
      }
    }
    EXPECT_TRUE(removed);
  }
}
TEST(QtMappedNative, QephVirginPlacedCurrentRemovedCoefficientsAndScatter) { Layered<placed::Q>(); }
TEST(QtMappedNative, T3VirginPlacedCurrentRemovedCoefficientsAndScatter) { Layered<placed::T>(); }
} // namespace qt_mapped_test
