#pragma once
#include "../PhysicalScope.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include <iostream>

namespace crash::modelio::physical_scope::test {
struct OriginalInputs {
    std::string member;
    rigid::RigidPartSource rigid;
    rigid::point_mass::Source masses;
    tied_shell::TiedShellDeclaration tied;
    type13::SourceType13 beams;
    solid_source::VehicleSolidSource solids;
    explicit OriginalInputs(solid_source::Policy policy = solid_source::Policy::OriginalAdhesive18RubberHephS6zV1)
        : member(output::ReadBounded(vehicle::test::Canonical().data().inputs.member_root /
                     vehicle::test::Canonical().data().inputs.source_member.file, 64 * 1024 * 1024)),
          rigid(rigid::RigidPartSource::Prepare(vehicle::test::Plan(), member)),
          masses(rigid::point_mass::Source::Prepare(rigid)),
          tied(tied_shell::TiedShellDeclaration::Prepare(vehicle::test::Canonical(), member)),
          beams(type13::SourceType13::Read(BeamPath(), {5150841,
              "c15fc2096317ac0206397ac50776f8456ddd23495e0c65aeee98e093ebd0b1b1"})),
          solids(solid_source::VehicleSolidSource::Prepare(vehicle::test::Canonical(), member,
              policy, (policy == solid_source::Policy::OriginalExtendedSolidsV4 ||
                         policy == solid_source::Policy::OriginalVehicleSupportsV5)
                  ? solid_source::Limits::ExtendedSolids() : solid_source::Limits{})) {}
    static std::filesystem::path BeamPath() {
        const auto* value = std::getenv("ROBO_DYNA_TYPE13_DECLARATION");
        output::Require(value && *value, "Explicit original TYPE13 source fixture required");
        return value;
    }
};
inline const OriginalInputs& Inputs() { static const OriginalInputs inputs; return inputs; }
inline const PhysicalScope& Actual() {
    static const auto scope = [] {
        const auto& in = Inputs();
        const auto forecast = PhysicalScope::Preflight(in.masses, in.tied, in.beams, in.solids);
        std::cout << "PhysicalScope complete preflight bytes=" << forecast.total_bytes << std::endl;
        return PhysicalScope::Prepare(in.masses, in.tied, in.beams, in.solids);
    }();
    return scope;
}
} // namespace crash::modelio::physical_scope::test
