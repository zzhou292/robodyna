#pragma once
#include "lib_src/elements/sections/ShellLayeredTab1Work.h"
#include "lib_src/elements/sections/ShellLayeredJ2FailureWork.h"
#include <array>
#include <cstring>
#include <gtest/gtest.h>
namespace tab1_test {
namespace mat=tl::material;
namespace fail=mat::failure;
namespace sec=tl::fea::sections;
using Status=fail::Tab1FailureStatus;
using PointStatus=sec::PointStatus;
inline fail::Tab1ConstantTable Table() { return {{-.3,0,.3},.015}; }
inline sec::PointParameters Material() {
  sec::PointParameters p;
  EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},
      {true,0,1,10000,mat::ShellPlasticityRatePolicy::FilteredZeroC},p),PointStatus::Ok);
  return p;
}
inline sec::ShellLayeredJ2Input Increment(unsigned step,double thickness=.003) {
  sec::ShellLayeredJ2Input in;
  in.reference_thickness=in.reported_thickness=thickness;
  in.transverse_shear_modulus=70e9/(2*(1+.22))*5/6;
  in.dt=1.e-7;
  in.strain_curvature_increment[0]=step<15?.001:-.00001;
  in.strain_curvature_increment[1]=-.15*in.strain_curvature_increment[0];
  in.strain_curvature_increment[2]=.12*in.strain_curvature_increment[0];
  in.strain_curvature_increment[3]=1.e-5;
  in.strain_curvature_increment[4]=-1.e-5;
  in.strain_curvature_increment[5]=.01;
  in.strain_curvature_increment[6]=-.02;
  in.strain_curvature_increment[7]=.01;
  return in;
}
inline sec::ShellLayeredTab1History Seed(unsigned mask) {
  sec::ShellLayeredTab1History h;
  for(unsigned p=0;p<3;++p) {
    h.saved.point[p].filtered_rate_per_s=23.+p;
    if(mask&(1u<<p)) h.failure[p].damage=h.failure[p].maximum_damage=.999;
  }
  return h;
}
struct Work {
  double stress[5]{},material_stress[5]{},bending_stress[3]{},strain_curvature[8]{},
      thickness=.003,internal_work[2]{};
};
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes;
  std::memcpy(bytes.data(),&value,sizeof value);
  return bytes;
}
inline void SameFailureHistory(const fail::Tab1ConstantFailureHistory& a,
    const fail::Tab1ConstantFailureHistory& b) {
  EXPECT_EQ(Bytes(a.damage),Bytes(b.damage));
  EXPECT_EQ(Bytes(a.maximum_damage),Bytes(b.maximum_damage));
  EXPECT_EQ(Bytes(a.failure_time_s),Bytes(b.failure_time_s));
  EXPECT_EQ(a.table_segment,b.table_segment);
  EXPECT_EQ(a.point_active,b.point_active);
}
} // namespace tab1_test
