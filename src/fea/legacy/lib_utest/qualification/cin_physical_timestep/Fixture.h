// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_timestep/Screen.h"
#include <algorithm>
#include <array>
namespace cin_step_test {
namespace fe = tl::fea;
namespace dt = fe::cin_timestep;
struct Fixture {
  static constexpr unsigned N = 8;
  std::array<double,19*N+36> accepted{};
  std::array<double,N> mass{2,0,0,1,0,1,1,0};
  std::array<double,N> inertia{3,0,0,1,0,1,0,0};
  std::array<double,N> translation{4,0,11,13,100,200,5,0};
  std::array<double,N> rotation{6,0,2,3,10,20,0,0};
  std::array<std::uint8_t,N> fixed{0,0,0,0,0,0,0,7};
  std::array<std::uint8_t,N> fixed_rotation{0,0,0,0,0,0,0,1};
  std::array<std::uint8_t,N> present{1,1,1,1,1,1,0,1};
  std::array<std::uint8_t,N> secondary{0,1,0,0,0,0,0,0};
  std::array<std::uint8_t,N> member{0,0,2,2,3,3,0,0};
  std::array<fe::rigid::GroupRange,2> groups{{{0,2,5,{2,4,6},true},{2,2,3,{.25,.5,1},true}}};
  std::array<fe::rigid::MemberMetric,4> members{{{2,0,0},{3,1,1},{4,0,0},{5,1,1}}};
  Fixture() {
    const double x[]{0,0,0, 0,0,0, -1,0,0, 1,0,0, 2,.5,1, 2,.5,-2, 0,1,0, 0,0,0};
    std::copy(x,x+3*N,accepted.begin());
    fe::NodalRigidGroupState state;
    state.principal_axes={{1,0,0,0,1,0,0,0,1}};
    fe::rigid::WriteGroupState(accepted.data()+19*N,state);
    state.center={2,.5,-.5};
    fe::rigid::WriteGroupState(accepted.data()+19*N+18,state);
  }
  dt::Sources View() const {
    return {accepted.data(),mass.data(),inertia.data(),translation.data(),rotation.data(),
      fixed.data(),fixed_rotation.data(),present.data(),secondary.data(),
      {groups.data(),members.data(),member.data(),2,4,.001},N,0};
  }
};
} // namespace cin_step_test
