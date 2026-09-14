#pragma once

#include "VehicleSelfContactStartup.h"

namespace crash::cases::vehicle_self_contact {

struct VehicleSelfContactStartup::Data {
    explicit Data(const VehicleSelfContactSetup& value) : setup(value) {}
    VehicleSelfContactSetup setup;
    RuntimeForecast forecast;
    tl::fea::NodalStamp initial_stamp;
    tlfea::contact::SelfContactTransaction transaction;
};

}  // namespace crash::cases::vehicle_self_contact
