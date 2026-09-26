#pragma once
#include "VehicleJointModel.h"
#include "SourceMapping.h"
namespace crash::cases::vehicle_startup::joints {
struct VehicleJointModel::Storage {
    Storage(const Physical& p,const Source& s,Forecast f):physical(p),source(s),forecast(f) {}
    Physical physical;
    Source source;
    Forecast forecast;
    tl::fea::type45::Model model;
    std::vector<std::uint32_t> rows;
};
} // namespace crash::cases::vehicle_startup::joints
