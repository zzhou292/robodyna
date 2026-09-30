#pragma once
#include "lib_src/elements/type13/Type13RecurrenceTypes.h"
namespace type13_recurrence_test {
namespace t=tl::fea::type13;
// This oracle does not call the production recurrence. It carries its own
// history through exact Fortran source excerpts and the complete VINTER2.
t::Evaluation NativeEvaluate(const t::Property&,const t::Reference&,const t::NativeHistory&,
    const t::NativeEndpointKinematics (&)[2],double native_dt,bool fresh=false);
extern "C" {
void type13_h1_channel(const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const int*,double*,int*);
void type25_native_frame(const double*,const double*,const double*,const double*,const double*,double*,double*,double*);
void type25_native_scatter(const double*,const double*,const double*,const double*,double*);
void type13_native_deformation(const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const int*,double*,double*);
void type13_native_failure(const double*,const double*,const double*,const double*,const double*,double*);
void type13_native_stability(const double*,const double*,const double*,const double*,const double*,double*);
}
} // namespace type13_recurrence_test
