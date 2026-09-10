#pragma once
#include <cstddef>
#include <limits>

namespace crash::visual {
struct NodalCaptureLimits {
    std::size_t max_nodes = 128;
    // Two active nodal captures only. Chrono topology/mesh storage has the
    // independent bounds in Binding; CUDA owner storage is not charged here.
    std::size_t max_host_bytes = 32 * 1024;
};
inline constexpr std::size_t MaxNodalCaptureNodes = 2048;
inline constexpr std::size_t MaxNodalCaptureBytes = 512 * 1024;
// No allocation or borrowed data read. Failed preflight leaves bytes unchanged.
inline bool NodalCaptureBytes(std::size_t nodes, bool rotations, NodalCaptureLimits limits,
                              std::size_t& bytes) noexcept {
    if (!nodes || !limits.max_nodes || limits.max_nodes > MaxNodalCaptureNodes ||
        nodes > limits.max_nodes || !limits.max_host_bytes || limits.max_host_bytes > MaxNodalCaptureBytes)
        return false;
    const std::size_t per_node = 2 * (rotations ? 13 : 6) * sizeof(double);
    if (nodes > std::numeric_limits<std::size_t>::max() / per_node ||
        nodes * per_node > limits.max_host_bytes) return false;
    bytes = nodes * per_node;
    return true;
}
} // namespace crash::visual
