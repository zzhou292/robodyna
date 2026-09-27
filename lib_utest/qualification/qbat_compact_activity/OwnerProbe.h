// SPDX-License-Identifier: MIT
#pragma once
#include <cstddef>
namespace qbat_activity_test {
enum class ErrorPhase { None, Memset, Launch, Copy, Drain };
void CaptureFullSource(std::size_t parents);
const void* FullSource();
void CapturePacket(std::size_t parents);
void* PacketHost();
std::size_t PacketBytes();
bool ArmError(ErrorPhase);
unsigned ErrorHits();
} // namespace qbat_activity_test
