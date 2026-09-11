#include "NativeOracle.h"

extern "C" {
void tl_tied_patch_force(const double*, const double*, const double*, const int*, double*, double*);
void tl_tied_patch_motion(const double*, const double*, const double*, const double*, const int*, double*);
void tl_tied_patch_coefficients(const double*, const double*, const double*, const int*, double*);
}
namespace tied_patch_test {
namespace {
void Pack(Vec3 v, double* output) { output[0]=v.x; output[1]=v.y; output[2]=v.z; }
}
NativeResult Native(const tie::PatchInput& input, const tie::SecondaryLoad& load,
    const tie::MasterMotion& motion, bool repeated_node) {
  double x[15], v[12], a[12], force[3], couple[3];
  for (unsigned i = 0; i < 4; ++i) {
    Pack(input.master_position[i],x+3*i);
    Pack(motion.velocity[i],v+3*i);
    Pack(motion.acceleration[i],a+3*i);
  }
  Pack(input.secondary_position,x+12); Pack(load.force,force); Pack(load.couple,couple);
  NativeResult output;
  const int repeated = repeated_node ? 1 : 0;
  tl_tied_patch_force(x,force,couple,&repeated,output.cofactor.data(),output.values.data());
  tl_tied_patch_motion(x,output.cofactor.data(),v,a,&repeated,output.values.data()+12);
  return output;
}
std::array<double,24> NativeCoefficients(const tie::PatchInput& geometry,
    const tie::CoefficientInput& input, bool repeated_node) {
  double x[15];
  for (unsigned i = 0; i < 4; ++i) Pack(geometry.master_position[i],x+3*i);
  Pack(geometry.secondary_position,x+12);
  const auto& s = input.secondary;
  const double secondary[] = {s.mass,s.inertia,s.translational_stiffness,s.rotational_stiffness};
  const int repeated = repeated_node ? 1 : 0;
  std::array<double,24> output{};
  tl_tied_patch_coefficients(x,secondary,input.initial_master_inertia,&repeated,output.data());
  return output;
}
} // namespace tied_patch_test
