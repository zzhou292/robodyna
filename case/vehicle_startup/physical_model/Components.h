#pragma once
#include "Internal.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
// Shared private native value construction. Source factories authenticate their
// actual domain before entering; no owner, clock or public readiness is created.
struct Components {
    Components(const modelio::physical_scope::PhysicalScope& source,
               const fe::NodalNodeDomain& domain, modelio::type25::Declaration declaration)
        : masses(modelio::point_mass::VehiclePointMassSource::Prepare(source, domain)),
          welds(modelio::type25::VehicleType25Source::Prepare(source, domain, declaration)) {}
    Components(const modelio::physical_scope::DomainEmbedding& embedding,
               modelio::type25::Declaration declaration)
        : masses(modelio::point_mass::VehiclePointMassSource::PrepareEmbedded(embedding)),
          welds(modelio::type25::VehicleType25Source::PrepareEmbedded(embedding, declaration)) {}
    modelio::point_mass::VehiclePointMassSource masses;
    modelio::type25::VehicleType25Source welds;
    fe::ShellNodeMap shell_map;
    fe::type13::Model beams;
    fe::solids::Model solids;
    fe::Type13NodeContributions beam_coefficients;
    fe::beam18::Model structural_beams;
    fe::Beam18NodeContributions structural_coefficients;
    fe::NodalCoefficientLedger ledger;
    fe::rigid::NodalRigidPartAssemblyModel parts;
    fe::NodalRigidGroupModel plain;
    fe::NodalRigidAssemblyBinding rigid;
};
void PrepareComponents(const modelio::physical_domain::VehiclePhysicalDomain& original,
    const fe::NodalNodeDomain& actual_domain, const fe::ShellBatchBinding& shells,
    Limits, Components&);
} // namespace crash::cases::vehicle_startup::physical_model::detail
