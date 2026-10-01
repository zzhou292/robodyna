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
// Authors: Radu Serban, Alessandro Tasora
// Robodyna adaptation: canonical system implementation; archive identities retained.
// =============================================================================
//
// Physical system in which contact is modeled using a smooth (penalty-based)
// method.
//
// =============================================================================

#include <limits>

#include "robodyna/simulation/RbSystemSMC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"

#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChIterativeSolverLS.h"

namespace chrono {
CH_FACTORY_REGISTER(ChSystemSMC)
}

namespace robodyna::simulation {

// Translation-unit-only lookup keeps the inherited numerical expressions intact.
using namespace ::chrono;

// Register into the object factory, to enable run-time dynamic creation and persistence
// Stable registered identity is emitted before the canonical implementation namespace.

RbSystemSMC::RbSystemSMC(const std::string& name)
    : RbSystem(name),
      m_use_mat_props(true),
      m_contact_model(Hertz),
      m_adhesion_model(AdhesionForceModel::Constant),
      m_tdispl_model(OneStep),
      m_stiff_contact(false),
      m_force_algo(new ChDefaultContactForceTorqueSMC) {
    // Set default solver
    SetSolverType(ChSolver::Type::PSOR);

    // Set default contact container
    contact_container = chrono_types::make_shared<ChContactContainerSMC>();
    contact_container->SetSystem(this);

    // For default SMC there is no need to create contacts 'in advance'
    // when models are closer than the safety envelope, so set default envelope to 0
    ChCollisionModel::SetDefaultSuggestedEnvelope(0);

    m_minSlipVelocity = 1e-4;
    m_characteristicVelocity = 1;
}

RbSystemSMC::RbSystemSMC(const RbSystemSMC& other) : RbSystem(other) {
    m_use_mat_props = other.m_use_mat_props;
    m_contact_model = other.m_contact_model;
    m_adhesion_model = other.m_adhesion_model;
    m_tdispl_model = other.m_tdispl_model;
    m_stiff_contact = other.m_stiff_contact;
    m_minSlipVelocity = other.m_minSlipVelocity;
    m_characteristicVelocity = other.m_characteristicVelocity;

    // Note that the contact container and the force algorithm are not copied, but rather new default ones are created.
    contact_container = chrono_types::make_shared<ChContactContainerSMC>();
    contact_container->SetSystem(this);
    m_force_algo = chrono_types::make_unique<ChDefaultContactForceTorqueSMC>();
}

void RbSystemSMC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerSMC>(container))
        RbSystem::SetContactContainer(container);
}

void RbSystemSMC::SetSlipVelocityThreshold(double vel) {
    m_minSlipVelocity = std::max(vel, std::numeric_limits<double>::epsilon());
}

void RbSystemSMC::SetContactForceTorqueAlgorithm(std::unique_ptr<RbContactForceTorqueSMC>&& algorithm) {
    m_force_algo = std::move(algorithm);
}

// Trick to avoid putting the following mapper macro inside the class definition in .h file:
// enclose macros in local 'ChSystemSMC_Properties_enum_mapper', just to avoid avoiding cluttering of the parent class.
class RbSystemSMCPropertiesMapper : public RbSystemSMC {
  public:
    CH_ENUM_MAPPER_BEGIN(ContactForceModel);
    CH_ENUM_VAL(Hooke);
    CH_ENUM_VAL(Hertz);
    CH_ENUM_VAL(PlainCoulomb);
    CH_ENUM_VAL(Flores)
    CH_ENUM_MAPPER_END(ContactForceModel);

    CH_ENUM_MAPPER_BEGIN(AdhesionForceModel);
    CH_ENUM_VAL(AdhesionForceModel::Constant);
    CH_ENUM_VAL(AdhesionForceModel::DMT);
    CH_ENUM_VAL(AdhesionForceModel::Perko);
    CH_ENUM_MAPPER_END(AdhesionForceModel);

    CH_ENUM_MAPPER_BEGIN(TangentialDisplacementModel);
    CH_ENUM_VAL(None);
    CH_ENUM_VAL(OneStep);
    CH_ENUM_VAL(MultiStep);
    CH_ENUM_MAPPER_END(TangentialDisplacementModel);
};

void RbSystemSMC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<RbSystemSMC>();

    // serialize parent class
    RbSystem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_use_mat_props);
    archive_out << CHNVP(m_minSlipVelocity);
    archive_out << CHNVP(m_characteristicVelocity);
    RbSystemSMCPropertiesMapper::ContactForceModel_mapper mmodel_mapper;
    archive_out << CHNVP(mmodel_mapper(m_contact_model), "contact_model");
    RbSystemSMCPropertiesMapper::AdhesionForceModel_mapper madhesion_mapper;
    archive_out << CHNVP(madhesion_mapper(m_adhesion_model), "adhesion_model");
    RbSystemSMCPropertiesMapper::TangentialDisplacementModel_mapper mtangential_mapper;
    archive_out << CHNVP(mtangential_mapper(m_tdispl_model), "tangential_model");
    //// TODO  complete...
}

/// Method to allow de serialization of transient data from archives.
void RbSystemSMC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<RbSystemSMC>();

    // deserialize parent class
    RbSystem::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(m_use_mat_props);
    archive_in >> CHNVP(m_minSlipVelocity);
    archive_in >> CHNVP(m_characteristicVelocity);
    RbSystemSMCPropertiesMapper::ContactForceModel_mapper mmodel_mapper;
    archive_in >> CHNVP(mmodel_mapper(m_contact_model), "contact_model");
    RbSystemSMCPropertiesMapper::AdhesionForceModel_mapper madhesion_mapper;
    archive_in >> CHNVP(madhesion_mapper(m_adhesion_model), "adhesion_model");
    RbSystemSMCPropertiesMapper::TangentialDisplacementModel_mapper mtangential_mapper;
    archive_in >> CHNVP(mtangential_mapper(m_tdispl_model), "tangential_model");
    //// TODO  complete...

    // Recompute statistics, offsets, etc.
    this->Setup();
}

}  // namespace robodyna::simulation
