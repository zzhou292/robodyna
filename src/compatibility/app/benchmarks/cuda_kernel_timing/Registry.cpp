#include "Timing.h"
#include <cstring>

namespace robo_dyna::cuda_kernel_timing {
namespace {
unsigned Host(const void* host) noexcept {
    const auto& state = Data();
    for (unsigned i = 0; i < state.kernel_count; ++i)
        if (state.kernels[i].active && state.kernels[i].host == host) return i;
    return Unknown;
}
void Retire(unsigned index) noexcept {
    auto& state = Data();
    state.kernels[index].active = false;
    for (std::size_t i = 0; i < state.handle_count; ++i)
        if (state.handles[i].kernel == index) state.handles[i].kernel = Unknown;
}
}
void Register(void** module, const void* host, const char* name) noexcept {
    auto& state = Data();
    Lock lock(state.mutex);
    if (!state.enabled || state.closed) return;
    const auto old = Host(host);
    const auto length = name ? strnlen(name, NameCap) : 0;
    if (old != Unknown) {
        const auto& entry = state.kernels[old];
        if (entry.module == module && length < NameCap && !entry.truncated &&
            std::strcmp(entry.name, name ? name : "") == 0) return;
        Retire(old); // Pointer reuse must never inherit another registration's name.
    }
    if (!host || !length || state.kernel_count == KernelCap) {
        Add(state.registrations_dropped);
        return;
    }
    auto& entry = state.kernels[state.kernel_count++];
    entry.module = module;
    entry.host = host;
    entry.active = true;
    entry.truncated = length == NameCap;
    const auto copied = length < NameCap ? length : NameCap-1;
    if (copied) std::memcpy(entry.name, name, copied);
    entry.name[copied] = 0;
}
void Unregister(void** module) noexcept {
    auto& state = Data();
    Lock lock(state.mutex);
    for (unsigned i = 0; i < state.kernel_count; ++i)
        if (state.kernels[i].active && state.kernels[i].module == module) Retire(i);
}
void Bind(cudaKernel_t handle, const void* host) noexcept {
    auto& state = Data();
    Lock lock(state.mutex);
    if (!state.enabled || state.closed || !handle) return;
    const auto kernel = Host(host);
    for (std::size_t i = 0; i < state.handle_count; ++i) {
        if (state.handles[i].value == handle) {
            state.handles[i].kernel = kernel;
            return;
        }
    }
    if (state.handle_count == HandleCap) {
        Add(state.handles_dropped);
        return;
    }
    state.handles[state.handle_count++] = {handle, kernel};
}
unsigned Find(LaunchKind kind, const void* value) noexcept {
    if (kind == LaunchKind::Public) return Host(value);
    const auto& state = Data();
    for (std::size_t i = 0; i < state.handle_count; ++i)
        if (state.handles[i].value == value) return state.handles[i].kernel;
    return Unknown;
}
} // namespace robo_dyna::cuda_kernel_timing
