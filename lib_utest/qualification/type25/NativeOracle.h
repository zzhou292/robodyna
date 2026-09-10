// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include "lib_src/elements/type25/Type25Math.h"

namespace type25_test {
extern "C" void type25_native_reference(const double*,const double*,double*);
// Native operations receive source working-unit scalars. This adapter performs
// only dimensional conversion, field packing and radial channel mapping.
spring::Evaluation NativeEvaluate(spring::SourceUnits,const spring::Property&,const spring::Reference&,
    const spring::History&,const spring::EndpointKinematics (&)[2],double);
extern "C" void type25_native_frame(const double*,const double*,const double*,const double*,const double*,double*,double*,double*);
extern "C" void type25_native_deformation(const double*,const double*,const double*,const double*,const double*,const double*,const double*,double*);
extern "C" void type25_native_response(const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const int*,const double*,double*,double*,int*,double*);
extern "C" void type25_native_scatter(const double*,const double*,const double*,const double*,double*);
extern "C" void type25_native_mass(const double*,const double*,double*);
extern "C" void type25_native_dt(const double*,const double*,const double*,const double*,const double*,const double*,double*);
} // namespace type25_test
