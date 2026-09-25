// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Assembly.h"
#include <vector>
namespace type25_assembly_test {
namespace rd = tlfea::contact::radioss_type25;
namespace ass = rd::assembly;
// Qualification only: invokes the independently compiled source loops.
std::vector<ass::NativeNodalValue> NativeAssemble(
    const std::vector<ass::Connectivity>& rows,
    const std::vector<rd::NativeFrictionResult>& responses,
    const std::vector<std::uint32_t>& cohort_ends,
    const std::vector<ass::NativeNodalValue>& incoming);
} // namespace type25_assembly_test
