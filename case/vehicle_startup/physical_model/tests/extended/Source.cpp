#include "Support.h"

namespace crash::cases::vehicle_startup::physical_model::extended_test {
const modelio::physical_scope::test::OriginalInputs& Inputs() {
    static const modelio::physical_scope::test::OriginalInputs value(solid_source::Policy::OriginalExtendedSolidsV4);
    return value;
}
const modelio::physical_scope::PhysicalScope& Scope() {
    static const auto value = [] {
        const auto& input = Inputs();
        return modelio::physical_scope::PhysicalScope::Prepare(input.masses, input.tied, input.beams, input.solids);
    }();
    return value;
}
const domain_source::VehiclePhysicalDomain& Domain() {
    static const auto value = domain_source::VehiclePhysicalDomain::Prepare(Scope(), DomainPolicy);
    return value;
}
const VehiclePhysicalModel& Model() {
    static const auto value = [] {
        std::cout << "Extended physical model complete preflight=" <<
            VehiclePhysicalModel::Preflight(Domain(), test::Shells()).total_bytes << std::endl;
        return VehiclePhysicalModel::Prepare(Domain(), test::Shells());
    }();
    return value;
}
const joint_source::VehicleType45Source& JointSource() {
    static const auto value = joint_source::VehicleType45Source::Prepare(Domain(), JointPolicy);
    return value;
}
const joints::VehicleJointModel& Joints() {
    static const auto value = joints::VehicleJointModel::Prepare(Model(), JointSource());
    return value;
}
} // namespace crash::cases::vehicle_startup::physical_model::extended_test
