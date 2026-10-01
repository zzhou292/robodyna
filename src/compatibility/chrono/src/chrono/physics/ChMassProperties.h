// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in LICENSES/Chrono-BSD-3-Clause.txt at the Robodyna root and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

// Robodyna compatibility header; canonical definitions have one owning implementation.
#pragma once
#include "robodyna/mechanics/RbMassProperties.h"

namespace chrono {
using ChMassProperties = robodyna::mechanics::RbMassProperties;
using ChInertiaUtils = robodyna::mechanics::RbInertiaUtils;
using CompositeInertia = robodyna::mechanics::CompositeInertia;
}  // namespace chrono
