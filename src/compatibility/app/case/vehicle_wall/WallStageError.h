#pragma once
#include "lib_src/collision/NodalWallMappedContact.h"
#include <stdexcept>
#include <string>
namespace crash::cases::vehicle_wall {
// Preserves the actual numerical rejection without inventing a timestep limit
// absent from the native contact report. The dynamics catch still discards all.
class WallStageError : public std::runtime_error {
  public:
    WallStageError(const tlfea::contact::NodalWallDeviceReport& report,const char* stage)
        : std::runtime_error(std::string(stage)+": "+report.message),report_(report) {}
    const tlfea::contact::NodalWallDeviceReport& report() const noexcept { return report_; }
  private:
    tlfea::contact::NodalWallDeviceReport report_;
};
} // namespace crash::cases::vehicle_wall
