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
// Robodyna adaptation: canonical system implementation; archive identities retained.
// =============================================================================
//
// Physical system in which contact is modeled using a non-smooth
// (complementarity-based) method.
//
// =============================================================================

#include <algorithm>

#include "robodyna/simulation/RbSystemNSC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
CH_FACTORY_REGISTER(ChSystemNSC)
}

namespace robodyna::simulation {

// Translation-unit-only lookup keeps the inherited numerical expressions intact.
using namespace ::chrono;

// Register into the object factory, to enable run-time dynamic creation and persistence
// Stable registered identity is emitted before the canonical implementation namespace.

RbSystemNSC::RbSystemNSC(const std::string& name) : RbSystem(name) {
    // Set default solver
    SetSolverType(ChSolver::Type::PSOR);

    // Set default contact container
    contact_container = chrono_types::make_shared<ChContactContainerNSC>();
    contact_container->SetSystem(this);

    // Set default collision envelope and margin.
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    ChCollisionModel::SetDefaultSuggestedMargin(0.01);
}

RbSystemNSC::RbSystemNSC(const RbSystemNSC& other) : RbSystem(other) {
    contact_container = chrono_types::make_shared<ChContactContainerNSC>();
    contact_container->SetSystem(this);
}

void RbSystemNSC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerNSC>(container))
        RbSystem::SetContactContainer(container);
}

void RbSystemNSC::SetMinBounceSpeed(double value) {
    std::static_pointer_cast<ChContactContainerNSC>(contact_container)->min_bounce_speed = value;
}

void RbSystemNSC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<RbSystemNSC>();

    // serialize parent class
    RbSystem::ArchiveOut(archive_out);

    // serialize all member data:
}

// Method to allow de serialization of transient data from archives.
void RbSystemNSC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<RbSystemNSC>();

    // deserialize parent class
    RbSystem::ArchiveIn(archive_in);

    // stream in all member data:
}

}  // namespace robodyna::simulation
