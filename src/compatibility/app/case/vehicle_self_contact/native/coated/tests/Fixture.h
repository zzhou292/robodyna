#pragma once
#include "../Values.h"
#include <gtest/gtest.h>
#include <cstring>
namespace crash::cases::vehicle_self_contact::native::coated::test {
inline Inputs Cube() {
    Inputs input;
    input.units = {.001, 1000, 1};
    const n::Vector points[]{{0,0,0},{2,0,0},{2,3,0},{0,3,0},{0,0,4},{2,0,4},{2,3,4},{0,3,4}};
    for (unsigned i = 0; i < 8; ++i) input.nodes.push_back({10+i, i, points[i]});
    Solid solid;
    solid.source_id = 500; solid.part_id = 30; solid.source_line = 19;
    solid.nodes = ReaderSlots(ReaderKind::Hex8, {0,1,2,3,4,5,6,7}, input.nodes);
    input.solids.push_back(solid);
    Shell top;
    top.primary = {100, n::ShellLayout::Quad4, {4,5,6,7}};
    top.part_id = 20; top.source_line = 20; top.contact_selected = true;
    input.shells.push_back(top);
    return input;
}
inline std::uint64_t Bits(double value) {
    std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); return bits;
}
} // namespace crash::cases::vehicle_self_contact::native::coated::test
