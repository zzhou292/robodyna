#pragma once
#include "VehicleShellExecution.h"
#include "Packing.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::shell_execution {
struct VehicleShellExecution::Storage {
    Storage(const physical_model::VehiclePhysicalModel& source,Law1ExecutionPolicy policy)
        : model(source),law1_policy(policy) {}
    physical_model::VehiclePhysicalModel model;
    tl::fea::ShellPhysicalBinding physical;
    Forecast forecast;
    Law1ExecutionPolicy law1_policy;
};
namespace detail {
using output::Require;
const VehicleSectionResolution& CheckSource(const physical_model::VehiclePhysicalModel&);
void PackSource(const physical_model::VehiclePhysicalModel&, Packing&);
void PackSource(const physical_model::VehiclePhysicalModel&, Packing&,const Law1ExecutionPolicy&);
Law1ExecutionPolicy ResolvePolicy(const physical_model::VehiclePhysicalModel&,Law1ExecutionProfile);
void CheckLimits(const Limits&);
Forecast ForecastPayload(std::size_t source_bytes, std::size_t fixed_bytes,
    std::size_t parts, std::size_t parents, std::size_t curves, const Limits&);
} // namespace detail
} // namespace crash::cases::vehicle_startup::shell_execution
