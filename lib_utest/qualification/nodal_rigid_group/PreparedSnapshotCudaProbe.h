#pragma once
// Qualification-only linker interposition; no production failure hook.
namespace prepared_snapshot_probe {
void FailAfterNextDeviceRead() noexcept;
bool CompletedReadBeforeFailure() noexcept;
}
