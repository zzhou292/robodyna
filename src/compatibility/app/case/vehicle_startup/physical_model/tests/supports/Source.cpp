#include "Support.h"
namespace crash::cases::vehicle_startup::physical_model::supports_test {
const modelio::physical_scope::test::OriginalInputs& Inputs() {
    static const modelio::physical_scope::test::OriginalInputs value(solid_source::Policy::OriginalVehicleSupportsV5);
    return value;
}
const modelio::beam18::Source& Beams() {
    static const auto value=modelio::beam18::Source::Prepare(Inputs().tied.canonical(),Inputs().member,
        modelio::beam18::Policy::OriginalCircularFourPointLaw44V1);
    return value;
}
const modelio::physical_scope::PhysicalScope& Scope() {
    static const auto value=[] {
        const auto& in=Inputs();
        return modelio::physical_scope::PhysicalScope::PrepareVehicleSupports(in.masses,in.tied,in.beams,in.solids,Beams());
    }();
    return value;
}
const domain_source::VehiclePhysicalDomain& Domain() {
    static const auto value=domain_source::VehiclePhysicalDomain::Prepare(Scope(),DomainPolicy);
    return value;
}
const VehiclePhysicalModel& Model() {
    static const auto value=[] {
        std::cout << "Vehicle supports physical complete preflight=" <<
            VehiclePhysicalModel::Preflight(Domain(),test::Shells(),Limits::VehicleSupports()).total_bytes << std::endl;
        return VehiclePhysicalModel::Prepare(Domain(),test::Shells(),Limits::VehicleSupports());
    }();
    return value;
}
const joint_source::VehicleType45Source& JointSource() {
    static const auto value=joint_source::VehicleType45Source::Prepare(Domain(),JointPolicy);
    return value;
}
const joints::VehicleJointModel& Joints() {
    static const auto value=joints::VehicleJointModel::Prepare(Model(),JointSource());
    return value;
}
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
