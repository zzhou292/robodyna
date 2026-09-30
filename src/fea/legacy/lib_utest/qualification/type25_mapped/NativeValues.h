// SPDX-License-Identifier: MIT
#pragma once
#include "../type25/NativeOracle.h"
#include "../type25/EvaluationValues.h"
#include "native/Packet.h"
#include <array>
namespace type25_mapped_test {
std::array<double,4> NativeStiffness(tl::fea::type25::SourceUnits,
    const tl::fea::type25::Property&,double length,bool active,unsigned mass_mode=0);
void CompareNative(const tl::fea::type25::Evaluation&,const tl::fea::type25::Evaluation&,
    tl::fea::type25::SourceUnits,const tl::fea::type25::Property&,
    const tl::fea::type25::EndpointKinematics (&)[2],double dt);
}
