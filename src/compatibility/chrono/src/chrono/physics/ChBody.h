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

// Robodyna compatibility route; RbBody has one canonical implementation.
#pragma once
#include "chrono/physics/ChBodyFwd.h"
#include "robodyna/mbd/RbBody.h"
namespace chrono {
using ::robodyna::mbd::BODY_DOF;
using ::robodyna::mbd::BODY_QDOF;
using ::robodyna::mbd::BODY_ROT;
}
