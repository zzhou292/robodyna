#pragma once
#include "lib_src/materials/SolidLaw36Point.h"
namespace law36_test {
inline constexpr double E = 1887.0 * 1e6;
inline constexpr double Nu = .417;
inline constexpr double Rho = 1.07e-9 * 1e12;
inline constexpr double X[] = {0x0.0p+0, 0x1.de69ad42c3c9fp-8, 0x1.5e9e1b089a027p-7, 0x1.d495182a9930cp-7, 0x1.288ce703afb7fp-6, 0x1.6bb98c7e28241p-6, 0x1.b71758e219653p-6, 0x1.eb851eb851eb8p-6};
inline constexpr double Y[] = {0x1.326d50ccccccdp+23, 0x1.6dd83fccccccdp+23, 0x1.023d5fccccccdp+24, 0x1.42efdf999999ap+24, 0x1.79809fccccccdp+24, 0x1.a2f9d03333333p+24, 0x1.bebf2fe666667p+24, 0x1.bee63fe666666p+24};
inline constexpr tl::material::law36::Curve Curve{X,Y,8};
} // namespace law36_test
