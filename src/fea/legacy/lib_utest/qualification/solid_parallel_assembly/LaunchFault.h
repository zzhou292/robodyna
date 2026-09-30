#pragma once
namespace solid_parallel_test::launch_fault {
void Arm(unsigned boundary) noexcept;
unsigned Observed() noexcept;
}
