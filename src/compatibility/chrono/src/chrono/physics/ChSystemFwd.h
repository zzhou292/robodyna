// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in LICENSES/Chrono-BSD-3-Clause.txt at the Robodyna root and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

// Robodyna compatibility aliases for the canonical system family.
#pragma once
#include "robodyna/simulation/RbSystemFwd.h"
namespace chrono {
using ChSystem = ::robodyna::simulation::RbSystem;
using ChSystemNSC = ::robodyna::simulation::RbSystemNSC;
using ChSystemSMC = ::robodyna::simulation::RbSystemSMC;
}
