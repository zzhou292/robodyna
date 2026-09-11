#pragma once
#include "lib_src/elements/ShellPlacementCoefficients.h"
#include "lib_src/elements/qeph/QephStiffnessDiagnostics.h"
#include "lib_src/elements/t3/T3StiffnessDiagnostics.h"
#include "lib_utest/qualification/shell_tab1_glass/Tab1Fixture.h"
namespace placement_test {
using namespace tab1_test;
using Placement=tl::fea::ShellReferencePlacement;
constexpr Placement Planes[]{Placement::Centered,Placement::TopReferencePlane,
    Placement::BottomReferencePlane};
inline int NativeIpos(Placement p) {
  return p==Placement::TopReferencePlane?3:p==Placement::BottomReferencePlane?4:0;
}
inline sec::ShellLayeredJ2Input Input(unsigned step,double thickness,Placement p) {
  auto input=Increment(step,thickness);
  input.placement=p;
  input.strain_curvature_increment[0]=step<26?.001:-.00001;
  return input;
}
inline double Viscosity(const sec::ShellLayeredJ2Input& in,double area,double dm) {
  const double speed=std::sqrt(70e9/(1-.22*.22)/2500.);
  return (1.+4./10.+1./100.+4./1000.)*dm*2500.*speed*
      std::sqrt(area)*(in.dt/std::max(in.dt*in.dt,1.e-20));
}
} // namespace placement_test
