#pragma once
#include "../Support.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include "modelio/physical_domain/Policy.h"
namespace crash::cases::vehicle_startup::physical_model::supports_test {
namespace domain_source=modelio::physical_domain;
namespace solid_source=modelio::solid_source;
namespace joint_source=modelio::type45;
namespace fe=tl::fea;
using test::Same;
constexpr auto DomainPolicy=domain_source::Policy::RetainedShellAssembliesVehicleSupportsV5;
constexpr auto JointPolicy=joint_source::Policy::OriginalDirectSdiType45VehicleSupportsV5;
const modelio::physical_scope::test::OriginalInputs& Inputs();
const modelio::beam18::Source& Beams();
const modelio::physical_scope::PhysicalScope& Scope();
const domain_source::VehiclePhysicalDomain& Domain();
const VehiclePhysicalModel& Model();
const joint_source::VehicleType45Source& JointSource();
const joints::VehicleJointModel& Joints();
} // namespace crash::cases::vehicle_startup::physical_model::supports_test
