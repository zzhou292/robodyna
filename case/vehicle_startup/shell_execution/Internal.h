#pragma once
#include "VehicleShellExecution.h"
#include "Packing.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::shell_execution {
struct VehicleShellExecution::Storage {
    explicit Storage(const physical_model::VehiclePhysicalModel& source) : model(source) {}
    physical_model::VehiclePhysicalModel model;
    tl::fea::ShellPhysicalBinding physical;
    Forecast forecast;
};
namespace detail {
using output::Require;
const VehicleSectionResolution& CheckSource(const physical_model::VehiclePhysicalModel&);
void PackSource(const physical_model::VehiclePhysicalModel&, Packing&);
void CheckLimits(const Limits&);
Forecast ForecastPayload(std::size_t source_bytes, std::size_t fixed_bytes,
    std::size_t parts, std::size_t parents, std::size_t curves, const Limits&);
} // namespace detail
} // namespace crash::cases::vehicle_startup::shell_execution
