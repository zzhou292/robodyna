#pragma once
#include "lib_src/elements/sections/ShellLayeredLaw1.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace layered_law1_test {
namespace mat=tl::material;namespace sec=tl::fea::sections;
using Parameters=mat::ShellElasticLaw1PointParameters;
using PointHistory=mat::ShellElasticLaw1PointHistory;
using PointInput=mat::ShellElasticLaw1PointInput;
using PointResult=mat::ShellElasticLaw1PointResult;
using History=sec::ShellLayeredLaw1History;using Input=sec::ShellLayeredLaw1Input;using Result=sec::ShellLayeredLaw1Result;
template<class T> auto Bytes(const T& value) {std::array<unsigned char,sizeof(T)> b{};std::memcpy(b.data(),&value,sizeof(T));return b;}
inline Parameters Material(double e=200e9,double nu=.3,double rho=7890.) {
  Parameters p;if(!mat::PrepareShellElasticLaw1Point(e,nu,rho,p))throw std::runtime_error("bad test material");return p;
}
inline Input Step(unsigned i,double h,double gs) {
  Input in;in.reference_thickness=in.reported_thickness=h;in.transverse_shear_modulus=gs;
  const double sign=i<64?1.:(i<96?0.:-1.);
  in.strain_curvature_increment[0]=sign*3.e-4;
  in.strain_curvature_increment[1]=sign*-7.e-5;
  in.strain_curvature_increment[2]=sign*9.e-5;
  in.strain_curvature_increment[3]=sign*4.e-5;in.strain_curvature_increment[4]=sign*-2.e-5;
  in.strain_curvature_increment[5]=sign*.025;in.strain_curvature_increment[6]=sign*-.007;
  in.strain_curvature_increment[7]=sign*.013;return in;
}
inline void Near(double value,long double expected,long double scale=1.L) {
  EXPECT_LE(std::abs(static_cast<long double>(value)-expected),2.e-12L*(scale+std::abs(expected)));
}
inline void NativeClose(double a,double b) {EXPECT_LE(std::abs(a-b),1.e-8+2.e-11*std::max(std::abs(a),std::abs(b)));}
inline void ThicknessClose(double a,double b) {EXPECT_LE(std::abs(a-b),2.e-14*std::max(std::abs(a),std::abs(b))+1.e-18);}
} // namespace layered_law1_test
