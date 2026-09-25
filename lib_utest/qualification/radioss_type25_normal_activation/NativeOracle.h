// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/normal_activation/Types.h"
#include <vector>
namespace normal_activation_test {
namespace activation=tlfea::contact::radioss_type25::normal_activation;
struct Result {std::vector<std::uint32_t> main_active,node_tag,free_main_ids;};
// Serial qualification-only complete original FREE_BOUND + TAGN. Source normal
// floats are not read. The supplied free list is not used to produce expected
// masks: the independent native routine derives it from actual source fields.
Result Oracle(const activation::Input&);
} // namespace normal_activation_test
