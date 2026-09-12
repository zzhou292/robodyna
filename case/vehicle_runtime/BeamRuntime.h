#pragma once
#include "Forecast.h"
namespace crash::cases::vehicle_runtime::detail {
// Confirms exact retained Model/snapshot backing before any shared-source
// discount or resident construction. The source policy is sealed by the model.
void CheckBeamSource(const Execution&);
tl::fea::beam18::BatchConfig ConfigureStructuralBeams(const Config&,const Attachments&,const tl::fea::NodalStamp&);
void ForecastStructuralBeams(const Config&,const Execution&,const Attachments&,Forecast&);
} // namespace crash::cases::vehicle_runtime::detail
