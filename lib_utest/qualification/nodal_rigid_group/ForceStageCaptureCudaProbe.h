#pragma once
#include <cstddef>
namespace force_stage_capture_probe {
// Qualification-only linker observation; never a production failure hook.
std::size_t AllocationCalls() noexcept;
}
