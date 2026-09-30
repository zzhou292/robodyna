#pragma once
#include "lib_src/elements/qeph/rejected_candidate/Types.h"
#include <cstdint>
#include <vector>
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection {
namespace native = tl::fea::qeph;
inline constexpr std::size_t MaximumWords = 8192;
inline constexpr std::size_t SerializationWorkspaceBytes = 1u << 20;
// Versioned field-order encoding. Doubles are IEEE binary64 bit patterns in
// uint64 words; no C++ padding, borrowed pointer, JSON NaN or restart state.
std::vector<std::uint64_t> Encode(const native::RejectedCandidateInput&);
native::RejectedCandidateInput Decode(const std::vector<std::uint64_t>&);
std::vector<std::uint64_t> EncodeMetadata(const native::RejectedCandidateMetadata&);
}
