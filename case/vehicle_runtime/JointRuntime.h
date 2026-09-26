#pragma once
#include "Source.h"
#include "VehiclePhysicalStartup.h"
#include "../vehicle_startup/joints/VehicleJointModel.h"
namespace crash::cases::vehicle_runtime::detail {
tl::fea::type45::BatchConfig ConfigureJoints(const Config&,const Source&,const tl::fea::NodalStamp&);
void ForecastJoints(const Config&,const Source&,Forecast&);
void CheckJointSource(const Execution&,const JointModel&);
tl::fea::type45::BatchConfig ConfigureJoints(const Config&,const Attachments&,const tl::fea::NodalStamp&);
void ForecastJoints(const Config&,const Execution&,const Attachments&,const JointModel&,Forecast&);
} // namespace crash::cases::vehicle_runtime::detail
