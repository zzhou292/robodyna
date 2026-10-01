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

// Robodyna compatibility route; canonical definitions have one compile owner.
#ifndef CHBODYAUXREF_H
#define CHBODYAUXREF_H
#include "chrono/physics/ChBody.h"
#include "robodyna/mbd/RbBodyAuxRef.h"

namespace chrono {
using ChBodyAuxRef = ::robodyna::mbd::RbBodyAuxRef;
}  // namespace chrono

#endif
