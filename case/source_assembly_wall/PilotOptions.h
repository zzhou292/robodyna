#pragma once
#include "case/source_assembly_dynamics/StepTiming.h"

namespace crash::cases::source_assembly_wall {
struct PilotOptions {
    unsigned refinement=1,step_multiple=1;
    source_assembly_dynamics::StepTimingOptions timing;
};
// A requested fixed step only. Existing source/contact/deformation admission
// remains authoritative; choosing an option does not prove stability.
double PilotFixedStep(const PilotOptions&);
}
