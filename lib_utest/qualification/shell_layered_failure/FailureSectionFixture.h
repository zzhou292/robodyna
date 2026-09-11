#pragma once
#include "lib_src/elements/sections/ShellLayeredJ2FailureWork.h"
#include <algorithm>
#include <stdexcept>

namespace layered_failure_test {
namespace sec=tl::fea::sections;
namespace mat=tl::material;
inline constexpr double strains[]{0,.002,.01,.1,2.};
inline constexpr double yields[]{220e6,250e6,300e6,400e6,500e6};
inline sec::PointParameters Parameters(bool analytic=false,bool rate=false) {
  sec::PointParameters p;
  mat::TabulatedShellPlasticityRate r;
  if(rate) r={true,40.,5.,100.};
  const auto status=analytic?
      mat::PrepareLinearLaw44ShellPlasticity(200e9,.3,7890,{220e6,1e9},r,p):
      mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{strains,yields,5},r,p);
  if(status!=sec::PointStatus::Ok) throw std::runtime_error("fixture material");
  return p;
}
inline sec::ShellLayeredJ2Input Input(const sec::PointParameters& p) {
  sec::ShellLayeredJ2Input input;
  input.reference_thickness=input.reported_thickness=.002;
  input.transverse_shear_modulus=p.shear_modulus*(5./6.);
  input.dt=1.e-6;
  return input;
}
struct WorkHistory {
  double strain_curvature[8]{},material_stress[5]{},stress[5]{},bending_stress[3]{};
  double internal_work[2]{},thickness=.002;
};
inline sec::ShellLayeredJ2FailureHistory Seed(unsigned mask) {
  sec::ShellLayeredJ2FailureHistory h;
  for(unsigned p=0;p<3;++p) if(mask&(1u<<p)) {
    h.failure[p].damage=1;
    h.failure[p].point_active=false;
    h.failure[p].failure_time_s=1.e-6;
  }
  h.element_active=mask!=7;
  return h;
}
} // namespace layered_failure_test
