// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#ifdef __FAST_MATH__
#error Coating orientation requires precise production arithmetic
#endif
int main() {
    namespace s = tlfea::contact::radioss_type25::startup;
    s::NativeCoatingOrientationInput input;
    input.node_count = 8;
    input.segment_slots[1] = 1;
    input.segment_slots[2] = 2;
    input.positions[1] = {1., 0., 0.};
    input.positions[2] = {0., 1., 0.};
    s::NativeCoatingOrientationResult result;
    return s::EvaluateNativeCoatingOrientation(input, &result) == s::Status::Ok &&
        result.orientation == s::CoatingOrientation::Forward ? 0 : 1;
}
