#pragma once
#include "AssemblyFixture.h"

namespace rigid_assembly_test {
using NativeValues=std::array<double,13>;
NativeValues NativeRaw(const r::AssemblyBodyInput&);
NativeValues NativeMerge(const NativeValues& parent,const NativeValues& child);
std::array<double,3> NativePointMass(double mass,double inertia,double added);
} // namespace rigid_assembly_test
