#pragma once
#include "modelio/type45/VehicleType45Source.h"
#include "lib_src/elements/type45/Model.h"
namespace crash::cases::vehicle_startup::joints::detail {
tl::fea::type45::JointInput Pack(const modelio::type45::Row&,const modelio::type45::Data&);
}
