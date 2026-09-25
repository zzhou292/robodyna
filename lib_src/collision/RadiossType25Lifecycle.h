// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/selection/lifecycle/Admission.h"
// Shared host/device numerical operations live in selection::lifecycle::detail.
// They require a completely validated, immutable Input and disjoint private
// row scratch. Prepare every row, scan/admit the complete count, then Complete.
// The owner authenticates source/accepted-state lifetime, enforces one writer
// per secondary, and reconstructs retained/spatial/sliding order before ASS0.
// This module creates neither a physical receipt nor an independent clock.
