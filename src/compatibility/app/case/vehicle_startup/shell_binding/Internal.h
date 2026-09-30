#pragma once
#include "../VehicleShellBinding.h"
#include "modelio/vehicle_sections/VehicleSectionResolution.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_startup::shell_binding_detail {
struct Inputs {
    std::vector<tl::fea::ShellQephBindingInput> qeph;
    std::vector<tl::fea::ShellT3BindingInput> t3;
    std::vector<tl::fea::ShellQbatBindingInput> qbat;
    tl::fea::ShellFormulationCollectionInput Borrow(std::size_t nodes) const noexcept;
};
VehicleShellBindingForecast Forecast(const VehicleShellReferences&,VehicleShellBindingLimits,std::size_t fixed);
Inputs Pack(const VehicleShellReferences&, std::size_t extra_qeph = 0);
}
