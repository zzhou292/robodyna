#pragma once
#include "Replay.h"
namespace crash::output::physical_run::replay_detail {
struct Memory {
    std::size_t interval_workspace=0,frame_workspace=0,environment_workspace=0;
    std::size_t sequential_workspace=0,peak_host_bytes=0;
};
// Retained source/context/metadata coexist. ReadIntervals owns its buffers only
// until it returns; sampled frame/activity validation and environment loading
// follow it sequentially. This is allocation admission, not record validation.
Memory Budget(const records::Context&,const Configuration&,std::size_t source_bytes,
              std::size_t host_cap);
}
