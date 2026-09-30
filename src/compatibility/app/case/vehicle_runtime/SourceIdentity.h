#pragma once
#include "case/vehicle_startup/shell_execution/VehicleShellExecution.h"
#include "case/vehicle_startup/physical_attachments/VehiclePhysicalAttachments.h"
#include "lib_src/solvers/NodalCinRuntime.h"
namespace crash::cases::vehicle_runtime {
using Execution = vehicle_startup::shell_execution::VehicleShellExecution;
using Attachments = vehicle_startup::physical_attachments::VehiclePhysicalAttachments;
namespace detail {
void CheckSource(const Execution&,const Attachments&);
tl::fea::NodalCinWitnessSource Witnesses(const Attachments&) noexcept;
} // namespace detail
} // namespace crash::cases::vehicle_runtime
