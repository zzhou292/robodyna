// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25NodalCorrection.h"
#include <vector>
namespace nodal_correction_test {
namespace c=tlfea::contact::radioss_type25::source_nodal::correction;
std::vector<double> Oracle(const c::Input&);
}
