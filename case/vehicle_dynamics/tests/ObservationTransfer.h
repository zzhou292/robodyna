#pragma once
#include <cstddef>
namespace motion_transfer_probe {
struct Counts {std::size_t host_reads=0,host_bytes=0,device_allocations=0;};
void Begin() noexcept;
Counts End() noexcept;
}
