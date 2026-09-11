#pragma once
#include "../PlacementForceFixture.h"
#include "../../shell_tab1_force/native/NativePacket.h"

namespace placement_force_test::native {
using tab1_force_test::native::Packet;
using tab1_force_test::native::Seed;
void Advance(const tl::fea::qeph::ReferenceData&,const tl::fea::qeph::PrescribedInterval&,
    const tl::fea::sections::PointParameters&,const tl::fea::sections::ShellLayeredTab1Parameters&,Packet&);
void Advance(const tl::fea::t3::ReferenceData&,const tl::fea::t3::PrescribedInterval&,
    const tl::fea::sections::PointParameters&,const tl::fea::sections::ShellLayeredTab1Parameters&,Packet&);
} // namespace placement_force_test::native
