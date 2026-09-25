// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace coating_orientation_test {
inline s::NativeCoatingOrientationInput Packet(unsigned count = 8) {
    s::NativeCoatingOrientationInput value;
    value.node_count = count;
    value.segment_slots[0] = 0;
    value.segment_slots[1] = 1;
    value.segment_slots[2] = 2;
    for (unsigned i = 0; i < count; ++i)
        value.positions[i] = {double(i % 3), double((i + 1) % 4), .25 * double(i + 1)};
    value.positions[0] = {-2., -1., 0.};
    value.positions[1] = {3., -1., 0.};
    value.positions[2] = {-2., 4., 0.};
    for (unsigned i = count; i < 20; ++i) {
        const auto poison = std::numeric_limits<double>::quiet_NaN();
        value.positions[i] = {poison, poison, poison};
    }
    return value;
}
inline std::vector<s::NativeCoatingOrientationInput> Cases() {
    std::vector<s::NativeCoatingOrientationInput> result;
    for (unsigned count : {8u, 10u, 16u, 20u}) {
        for (double height : {-1., 0., 1., 1e-200, 1e100}) {
            for (bool reversed : {false, true}) {
                auto packet = Packet(count);
                for (unsigned i = 3; i < count; ++i) packet.positions[i].z *= height;
                if (reversed) std::swap(packet.segment_slots[0], packet.segment_slots[1]);
                result.push_back(packet);
            }
        }
        auto repeated = Packet(count);
        repeated.positions[3] = repeated.positions[2];
        repeated.positions[count-1] = repeated.positions[count-2];
        result.push_back(repeated);
        auto translated = repeated;
        for (unsigned i = 0; i < count; ++i) {
            translated.positions[i].x += 123456789.125;
            translated.positions[i].y -= 987654321.25;
            translated.positions[i].z += 1000.5;
        }
        result.push_back(translated);
        for (double z : {0., -0., std::nextafter(0., 1.), std::nextafter(0., -1.)}) {
            auto boundary = Packet(count);
            for (unsigned i = 0; i < count; ++i) boundary.positions[i].z = z;
            result.push_back(boundary);
        }
    }
    return result;
}
}
