// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
namespace type45_native {
struct State {
  std::array<double,39> uvar{};
  std::array<double,13> history{}; // dx3, rotation3, local force3, couple3, EINT
};
struct Startup {
  std::array<double,13> values{}; // RINI K/Kr/C/Cr; auto K/Kr/limits/flag; cached M1/M2/J1/J2
};
struct Step {
  std::array<double,25> values{}; // endpoint {F3,M3,K,Kr}x2; XKM/XKR/XCM/XCR; XL3; mass/inertia
};
extern "C" void type45_native_startup(const int* kind,const double* property,
  const double* initial_and_main_xyz,const int* roles,const double* damping_m_j,
  const double* main_m_j_k_kr,const double* dt,double* uvar,double* observations,int* status);
extern "C" void type45_native_step(const int* kind,const double* property,
  const double* current_xyz,const double* angular_velocity,const double* time,
  const double* dt,const int* cycle,double* uvar,double* history,double* observations,int* status);
} // namespace type45_native
