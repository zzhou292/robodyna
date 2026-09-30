// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "../mapped_shell_observer/Truth.h"
namespace qeph_observer_test {
using namespace mapped_observer_truth;
inline std::array<TruthSum,m::ObserverChannels> Truth(Fixture& f,unsigned epoch=1,bool assembled=true) {
  std::array<TruthSum,m::ObserverChannels> truth{};
  const auto view=f.View(epoch);
  for(unsigned p=0;p<f.roles.size();++p) {
    if(f.roles[p]==fe::ShellSectionLaw::RigidSkin)continue;
    const auto& r=f.host->slab[1].element[p];const auto& h=r.proposed_history.data();
    for(unsigned c=0;c<2;++c) {truth[c].Add(h.internal_work[c]);truth[c+2].Add(r.diagnostics.internal_work_increment[c]);}
    truth[4].Add(h.hourglass_viscous_work);truth[5].Add(r.diagnostics.hourglass_viscous_work_increment);
    if(!assembled)continue;
    const auto& old=f.host->slab[0].element[p];
    for(unsigned s=0;s<4;++s) {
      const auto n=f.host->model.element[p].nodes[s];
      double v[3],w[3],dx[3],rotation[3];
      for(unsigned a=0;a<3;++a) {
        const auto i=3*n+a;
        v[a]=(view.base_kinematics.velocity_xyz[i]+view.kinematics.velocity_xyz[i])*.5;
        w[a]=(view.base_kinematics.angular_velocity_xyz[i]+view.kinematics.angular_velocity_xyz[i])*.5;
        dx[a]=view.kinematics.position_xyz[i]-view.base_kinematics.position_xyz[i];
        rotation[a]=f.host->model.config.owner.fixed_dt*view.kinematics.angular_velocity_xyz[i];
      }
      const double force[]={old.internal_force[s].x,old.internal_force[s].y,old.internal_force[s].z};
      const double couple[]={old.internal_couple[s].x,old.internal_couple[s].y,old.internal_couple[s].z};
      truth[6].Add(-(view.kick_dt*(Dot3(force,v)+Dot3(couple,w))));
      truth[7].Add(-(Dot3(force,dx)+Dot3(couple,rotation)));
    }
  }
  return truth;
}
inline void CheckTruth(Fixture& fixture,const b::Control& actual,unsigned epoch=1,bool assembled=true) {
  const auto truth=Truth(fixture,epoch,assembled);const auto actual_sums=Sums(actual.diagnostics);
  const auto blocks=m::ObserverBlocks(fixture.roles.size(),Nodes);
  for(unsigned c=0;c<m::ObserverChannels;++c) {
    const High bound=truth[c].Bound(fixture.roles.size(),blocks);
    EXPECT_TRUE(Abs(High(actual_sums[c])-truth[c].value)<=bound)<<"channel "<<c;
    // A representable corruption demonstrably outside this very packet's
    // bound must be distinguishable; no global NearlyEqual relaxation exists.
    const double corrupted=std::nextafter(static_cast<double>(truth[c].value+4*bound+High(1)),INFINITY);
    EXPECT_GT(Abs(High(corrupted)-truth[c].value),bound)<<"control channel "<<c;
  }
}
} // namespace qeph_observer_test
