// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type13/Type13Startup.h"

namespace type13_test {
namespace t=tl::fea::type13;
// Resolved original PID2000486 property, t/mm/s. Literal fixtures deliberately
// contain no MAT100 conversion implementation; app owns that separate gate.
struct Fixture {
  t::CurvePoint points[4][5]{
    {{-0x1.0189374bc6a7fp+0,-0x1.968141b3d3515p+16}, {-0x1.89374bc6a7efap-8,-0x1.7027c7945ec74p+12}, {0x0.0p+0,0x0.0p+0}, {0x1.89374bc6a7efap-8,0x1.7027c7945ec74p+12}, {0x1.0189374bc6a7fp+0,0x1.968141b3d3515p+16}},
    {{-0x1.024834e59c2cbp+0,-0x1.09fb373ce848cp+15}, {-0x1.241a72ce165a3p-7,-0x1.a4bfbf84fe9a9p+11}, {0x0.0p+0,0x0.0p+0}, {0x1.241a72ce165a3p-7,0x1.a4bfbf84fe9a9p+11}, {0x1.024834e59c2cbp+0,0x1.09fb373ce848cp+15}},
    {{-0x1.cf9096bb98c7ep-1,-0x1.9be026592fac2p+16}, {-0x1.61e4f765fd8aep-8,-0x1.7504f7424827fp+12}, {0x0.0p+0,0x0.0p+0}, {0x1.61e4f765fd8aep-8,0x1.7504f7424827fp+12}, {0x1.cf9096bb98c7ep-1,0x1.9be026592fac2p+16}},
    {{-0x1.4445ce6276b0ep-1,-0x1.fc219220c825bp+16}, {-0x1.496b7c53c5c80p-8,-0x1.cc31b97976791p+12}, {0x0.0p+0,0x0.0p+0}, {0x1.496b7c53c5c80p-8,0x1.cc31b97976791p+12}, {0x1.4445ce6276b0ep-1,0x1.fc219220c825bp+16}}
  };
  TL_TYPE13_HD t::PropertyInput Input() const {
    t::PropertyInput in;
    in.units={1000,.001,1};
    in.mass_per_length=0x1.48e48e269521dp-23;
    in.inertia_per_length=0x1.00f28f0e24827p-21;
    in.controls={1,1,0,0,0};
    const unsigned curve[6]={0,1,1,2,3,3};
    const double stiffness[6]={0x1.df5e768930be1p+19,0x1.70bed155d6b99p+18,0x1.70bed155d6b99p+18,0x1.0dd5aa90d5f96p+20,0x1.65a0bc0000000p+20,0x1.65a0bc0000000p+20};
    for(unsigned c=0;c<6;++c)in.channels[c]={curve[c],stiffness[c],1,1,0,-2e20,2e20,1,2,1};
    for(unsigned c=0;c<4;++c)in.curves[c]={points[c],5};
    return in;
  }
};
TL_TYPE13_HD inline t::ReferenceInput DenseReference() {
  t::ReferenceInput input;
  input.position[0]={1200,-400,700};
  input.position[1]={1201.25,-398,702.5};
  input.position[2]={0,0,0};
  return input;
}
} // namespace type13_test
