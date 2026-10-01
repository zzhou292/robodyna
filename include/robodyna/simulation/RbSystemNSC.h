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
// Robodyna adaptation: canonical system ownership and names; numerical operations unchanged.
// =============================================================================
//
// Physical system in which contact is modeled using a non-smooth
// (complementarity-based) method.
//
// =============================================================================

#ifndef ROBODYNA_SIMULATION_RBSYSTEMNSC_H
#define ROBODYNA_SIMULATION_RBSYSTEMNSC_H

#include "robodyna/simulation/RbSystem.h"

namespace robodyna::simulation {

/// Class for a physical system in which contact is modeled using a non-smooth
/// (complementarity-based) method.
class ChApi RbSystemNSC : public RbSystem {
  public:
    /// Create a physical system.
    RbSystemNSC(const std::string& name = "");

    /// Copy constructor
    RbSystemNSC(const RbSystemNSC& other);

    /// Destructor
    virtual ~RbSystemNSC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual RbSystemNSC* Clone() const override { return new RbSystemNSC(*this); }

    /// Return the contact method supported by this system.
    virtual chrono::ChContactMethod GetContactMethod() const override final { return chrono::ChContactMethod::NSC; }

    /// Replace the contact container.
    virtual void SetContactContainer(std::shared_ptr<chrono::ChContactContainer> container) override;

    /// Minimum rebounce speed for elastic collision (default: 0.15).
    /// For elastic collisions with nonzero restitution coefficient, objects will rebounce only if their relative
    /// colliding speed is above this threshold. Default 0.15 m/s. If this value is set too low, aliasing problems can
    /// happen with small high frequency rebounces and settling to static stacking might be more difficult.
    void SetMinBounceSpeed(double value);

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(chrono::ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(chrono::ChArchiveIn& archive_in) override;
};

// Stable version specialization is declared in the required inherited trait namespace below.

}  // namespace robodyna::simulation

namespace chrono {
CH_CLASS_VERSION(::robodyna::simulation::RbSystemNSC, 0)
}

#endif
