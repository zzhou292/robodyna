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
// Authors: Alessandro Tasora, Radu Serban, Arman Pazouki
// =============================================================================
//
// Classes for creating easy-to-use bodies that optionally include contact and
// visualization shapes.
//
// =============================================================================

// Robodyna compatibility route; canonical definitions have one compile owner.
#ifndef CHBODYEASY_H
#define CHBODYEASY_H
#include "chrono/physics/ChBodyAuxRef.h"
#include "robodyna/mbd/RbBodyEasy.h"

namespace chrono {
using ChBodyEasySphere = ::robodyna::mbd::RbBodyEasySphere;
using ChBodyEasyEllipsoid = ::robodyna::mbd::RbBodyEasyEllipsoid;
using ChBodyEasyCylinder = ::robodyna::mbd::RbBodyEasyCylinder;
using ChBodyEasyBox = ::robodyna::mbd::RbBodyEasyBox;
using ChBodyEasyConvexHull = ::robodyna::mbd::RbBodyEasyConvexHull;
using ChBodyEasyConvexHullAuxRef = ::robodyna::mbd::RbBodyEasyConvexHullAuxRef;
using ChBodyEasyMesh = ::robodyna::mbd::RbBodyEasyMesh;
using ChBodyEasyClusterOfSpheres = ::robodyna::mbd::RbBodyEasyClusterOfSpheres;
}  // namespace chrono

#endif
