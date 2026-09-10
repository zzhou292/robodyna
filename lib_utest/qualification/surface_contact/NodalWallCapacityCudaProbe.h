#pragma once
// Qualification-only linker wrappers. Production has no test hook or allocator.
namespace nodal_wall_capacity_probe {
void CountAllocations(bool enabled) noexcept;
unsigned AllocationCalls() noexcept;
void FailDeviceReadAfter(int successful_reads) noexcept;
unsigned DeviceReads() noexcept;
} // namespace nodal_wall_capacity_probe
