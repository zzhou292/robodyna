// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <stdexcept>
namespace coating_orientation_test {
extern "C" void rd_coating_orientation(const double*, const int*, const int*, int*, double*);
s::NativeCoatingOrientationResult Oracle(const s::NativeCoatingOrientationInput& input) {
    const int count = static_cast<int>(input.node_count);
    double points[60]{};
    for (unsigned i = 0; i < input.node_count; ++i) {
        if (i >= 20) throw std::invalid_argument("Native oracle packet exceeds count");
        points[3*i] = input.positions[i].x;
        points[3*i+1] = input.positions[i].y;
        points[3*i+2] = input.positions[i].z;
    }
    const int corners[]{int(input.segment_slots[0]), int(input.segment_slots[1]), int(input.segment_slots[2])};
    int sign = 0;
    double determinant = 0;
    rd_coating_orientation(points, &count, corners, &sign, &determinant);
    if (sign != 1 && sign != -1) throw std::runtime_error("Native observed inclusion is not defined");
    return {static_cast<s::CoatingOrientation>(sign), determinant};
}
}
