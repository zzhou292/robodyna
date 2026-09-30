// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
#include <array>
namespace type13_test {
double NativeSlope(const t::CurvePoint (&curve)[5],double stiffness,double scale=1);
std::array<double,3> NativeMass(double mass,double inertia,double length);
struct NativeFrame { double y[3]{},length=0;int branch=0; };
NativeFrame NativeReference(const t::ReferenceInput&);
}
