#include "AssemblyNativeOracle.h"
#include <vector>

extern "C" {
void rigid_part_native_raw(int,int,const double*,const double*,const double*,
    const double*,double,double,double*);
void rigid_part_native_merge(const double*,const double*,double*);
void rigid_part_native_point_mass(double,double,double,double*);
}
namespace rigid_assembly_test {
NativeValues NativeRaw(const r::AssemblyBodyInput& in) {
  const auto count=in.part_count+in.extra_count;
  std::vector<double> x(3*count),mass(count),inertia(count);
  for(std::size_t i=0;i<count;++i) {
    const auto& p=i<in.part_count?in.part[i]:in.extra[i-in.part_count];
    x[3*i]=p.position.x;x[3*i+1]=p.position.y;x[3*i+2]=p.position.z;
    mass[i]=p.mass;inertia[i]=p.inertia;
  }
  const double primary[]{in.primary.position.x,in.primary.position.y,in.primary.position.z};
  NativeValues out{};
  rigid_part_native_raw(static_cast<int>(in.part_count),static_cast<int>(in.extra_count),
    x.data(),mass.data(),inertia.data(),primary,in.primary.mass,in.primary.inertia,out.data());
  return out;
}
NativeValues NativeMerge(const NativeValues& parent,const NativeValues& child) {
  NativeValues out{};rigid_part_native_merge(parent.data(),child.data(),out.data());return out;
}
std::array<double,3> NativePointMass(double mass,double inertia,double added) {
  std::array<double,3> out{};rigid_part_native_point_mass(mass,inertia,added,out.data());return out;
}
} // namespace rigid_assembly_test
