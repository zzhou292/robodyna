#pragma once
#include "VehicleWallStartup.h"
namespace crash::cases::vehicle_wall {
struct VehicleWallStartup::Data {
    explicit Data(const VehicleWallSetup& value) : setup(value) {}
    VehicleWallSetup setup;
    RuntimeForecast forecast;
    tl::fea::NodalStamp initial_stamp;
    tlfea::contact::NodalWallMappedContact contact;
};
} // namespace crash::cases::vehicle_wall
