#pragma once
#include "Declaration.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
namespace crash::cases::vehicle_native_contact::activity::values {
using RawControls=vehicle_self_contact::native::initial_controls::RawControls;
native::Controls Self(const RawControls&,tlfea::contact::radioss_type25::startup::SolidErosion);
native::Controls Wall(const RawControls&,int declared_solid_erosion);
}
