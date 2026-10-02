#pragma once

#include "chrono/utils/ChBodyGeometry.h"

namespace robodyna::verification {

chrono::utils::ChBodyGeometry MakeTransportGeometry();
void CheckTransportGeometry(const chrono::utils::ChBodyGeometry& received);

}  // namespace robodyna::verification
