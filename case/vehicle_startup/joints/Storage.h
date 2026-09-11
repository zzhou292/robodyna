#pragma once
#include "VehicleJointModel.h"
namespace crash::cases::vehicle_startup::joints {
struct VehicleJointModel::Storage {
    Storage(const Physical& p,const Source& s,Forecast f):physical(p),source(s),forecast(f) {}
    Physical physical;
    Source source;
    Forecast forecast;
    tl::fea::type45::Model model;
    std::vector<std::uint32_t> rows;
};
namespace detail {
tl::fea::type45::JointInput Pack(const modelio::type45::Row&,const modelio::type45::Data&);
}
} // namespace crash::cases::vehicle_startup::joints
