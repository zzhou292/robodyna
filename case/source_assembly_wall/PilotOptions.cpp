#include "PilotOptions.h"
#include <stdexcept>

namespace crash::cases::source_assembly_wall {
double PilotFixedStep(const PilotOptions& options) {
    if(options.refinement!=1&&options.refinement!=2&&options.refinement!=4)
        throw std::invalid_argument("Pilot refinement must be 1, 2 or 4");
    if(options.step_multiple!=1&&options.step_multiple!=2&&options.step_multiple!=4&&options.step_multiple!=8)
        throw std::invalid_argument("Pilot step multiple must be 1, 2, 4 or 8");
    return (1./67108864)*options.step_multiple/options.refinement;
}
}
