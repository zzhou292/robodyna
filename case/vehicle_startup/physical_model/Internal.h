#pragma once
#include "VehiclePhysicalModel.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::physical_model::detail {
using output::Require;
namespace fe = tl::fea;
struct ComponentFootprint { std::size_t native_reservation = 0, packing_bytes = 0; };
void ValidateComponentLimits(Limits, bool extended);
ComponentFootprint NativeFootprint(const modelio::physical_domain::VehiclePhysicalDomain&, Limits);
inline modelio::type25::Declaration WeldDeclaration() {
    return {modelio::type25::Policy::OriginalDefaultSpotweldsV1, 0x59415249533235ULL};
}
void PrepareBeams(const modelio::type13::SourceType13&, const fe::NodalNodeDomain&,
                  std::size_t cap, fe::type13::Model&);
void PrepareStructuralBeams(const modelio::beam18::Source&, const fe::NodalNodeDomain&,
                            std::size_t cap, fe::beam18::Model&);
void PrepareSolids(const modelio::solid_source::VehicleSolidSource&, const fe::NodalNodeDomain&,
                   std::size_t cap, fe::solids::Model&);
fe::NodalRigidGroupMember PlainMember(const fe::NodalNodeDomain&, const fe::NodalCoefficientLedger&,
                                     std::uint64_t nid);
void PreparePlain(const modelio::physical_domain::VehiclePhysicalDomain&, const fe::NodalNodeDomain&,
                  const fe::NodalCoefficientLedger&, std::size_t cap, fe::NodalRigidGroupModel&);
void PreparePlain(const modelio::physical_domain::VehiclePhysicalDomain&, const fe::NodalCoefficientLedger&,
                  std::size_t cap, fe::NodalRigidGroupModel&);
} // namespace crash::cases::vehicle_startup::physical_model::detail
