#pragma once
#include "../EnvelopeExecutionSource.h"
#include "case/vehicle_startup/shell_execution/Internal.h"
namespace crash::cases::vehicle_wall::native::execution_detail {
namespace fe = tl::fea;
namespace original = vehicle_startup::shell_execution;
using output::Require;
const vehicle_startup::VehicleSectionResolution& Check(const EnvelopePhysicalSource&);
void AppendWall(original::detail::Packing&, const EnvelopePhysicalSource&,
    const modelio::assembly::Law1ExecutionPolicy&);
// Value packing only; the public factory authenticates the declared source.
void AppendDeclaredWall(original::detail::Packing&,const EnvironmentParent&,const MaterialDeclaration&,
    double working_length_m,const modelio::assembly::Law1ExecutionPolicy&);

}
