#pragma once
#include <cstdint>
#include "case/source_assembly_dynamics/StepTiming.h"

namespace crash::cases::source_assembly_wall {
enum class PilotAssembly { SixPart,SevenPartSpotweld };
struct PilotOptions {
    unsigned refinement=1,step_multiple=1;
    bool observe_force_stage=false;
    bool native_rotation_domain=false;
    std::uint64_t observe_qeph_spin_node=0;
    source_assembly_dynamics::StepTimingOptions timing;
    PilotAssembly assembly=PilotAssembly::SixPart;
};
// A requested fixed step only. Existing source/contact/deformation admission
// remains authoritative; choosing an option does not prove stability.
double PilotFixedStep(const PilotOptions&);
}
