#pragma once
#include "PilotOptions.h"
#include <cstdint>
#include <string>

namespace crash::cases::source_assembly_wall {
struct CliOptions {
    std::string inventory,wall,archive,timing_path,spin_path;
    std::uint64_t steps=0;
    unsigned frame_every=0;
    std::uint64_t spin_every=8;
    PilotOptions pilot;
};
// Five required positional values, optional positional refinement, then named
// options in either order. No source reading, filesystem mutation or CUDA.
CliOptions ParseOptions(int argc,const char* const* argv);
}
