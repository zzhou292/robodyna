#include "lib_src/collision/RadiossType25InterfaceSurface.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
int main() {
  namespace n=tlfea::contact::radioss_type25;
  n::surface_interface::Input input;
  n::surface_interface::Forecast forecast;
  const auto status=n::surface_interface::Preflight(input,{},forecast);
  n::startup::Input sides;
  const auto capacity=n::startup::PreflightMixedSides(sides);
  return status.status!=n::surface_interface::Status::Ok &&
      capacity.status==n::startup::Status::UnsupportedProfile ? 0 : 1;
}
