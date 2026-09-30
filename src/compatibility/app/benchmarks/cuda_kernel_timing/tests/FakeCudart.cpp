#include "../Runtime.h"
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {
struct Event { unsigned sequence = 0; int device = 0; };
std::atomic<unsigned> launches{0}, creates{0}, destroys{0}, event_calls{0}, registrations{0};
std::atomic<int> early_result{0};
thread_local int current_device = 0;
thread_local unsigned sequence = 0;
thread_local const void* last_key = nullptr;
bool Mode(const char* name) {
    const char* value = std::getenv("FAKE_CUDA_MODE");
    return value && std::strcmp(value, name) == 0;
}
cudaError_t Execute(dim3 grid, dim3 block, void** args, std::size_t shared, cudaStream_t stream) {
    ++launches;
    if (errno != EBUSY || grid.y != 2 || grid.z != 3 || block.x != 4 || block.y != 5 ||
        block.z != 6 || shared != 19 || stream != reinterpret_cast<cudaStream_t>(0x55) || !args) {
        errno = EPROTO;
        return cudaErrorUnknown;
    }
    if (!grid.x) { errno = EAGAIN; return cudaErrorInvalidConfiguration; }
    ++*static_cast<unsigned*>(args[0]);
    errno = E2BIG;
    return cudaSuccess;
}
}
extern "C" unsigned fake_launches() { return launches; }
extern "C" unsigned fake_events() { return event_calls; }
extern "C" unsigned fake_live_events() { return creates-destroys; }
extern "C" unsigned fake_registrations() { return registrations; }
extern "C" const void* fake_last_key() { return last_key; }
extern "C" int fake_early_result() { return early_result; }
extern "C" void fake_device(int device) { current_device = device; }
extern "C" cudaError_t CUDARTAPI cudaLaunchKernel(
    const void* key, dim3 grid, dim3 block, void** args, std::size_t shared, cudaStream_t stream) {
    last_key = key;
    return Execute(grid, block, args, shared, stream);
}
extern "C" cudaError_t CUDARTAPI __cudaLaunchKernel(
    cudaKernel_t kernel, dim3 grid, dim3 block, void** args, std::size_t shared, cudaStream_t stream) {
    last_key = kernel;
    if (Mode("nested")) return cudaLaunchKernel(kernel, grid, block, args, shared, stream);
    return Execute(grid, block, args, shared, stream);
}
extern "C" cudaError_t CUDARTAPI __cudaGetKernel(cudaKernel_t* result, const void* host) {
    if (!result || host == reinterpret_cast<void*>(0xdead)) {
        errno = ENOENT;
        return cudaErrorInvalidDeviceFunction;
    }
    *result = reinterpret_cast<cudaKernel_t>(reinterpret_cast<std::uintptr_t>(host)+0x100000);
    errno = E2BIG;
    return cudaSuccess;
}
extern "C" void CUDARTAPI __cudaRegisterFunction(
    void**, const char*, char*, const char*, int, uint3*, uint3*, dim3*, dim3*, int*) {
    ++registrations;
    errno = E2BIG;
}
extern "C" void CUDARTAPI __cudaUnregisterFatBinary(void**) { errno = E2BIG; }
extern "C" cudaError_t CUDARTAPI cudaStreamIsCapturing(cudaStream_t, cudaStreamCaptureStatus* status) {
    ++event_calls;
    if (Mode("capture_error")) return cudaErrorInvalidResourceHandle;
    *status = Mode("capture") ? cudaStreamCaptureStatusActive : cudaStreamCaptureStatusNone;
    return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaEventCreate(cudaEvent_t* result) {
    ++event_calls;
    if (Mode("create_error")) return cudaErrorMemoryAllocation;
    *result = reinterpret_cast<cudaEvent_t>(new Event{0, current_device});
    ++creates;
    return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaEventRecord(cudaEvent_t event, cudaStream_t) {
    ++event_calls;
    auto& value = *reinterpret_cast<Event*>(event);
    if (value.device != current_device) return cudaErrorInvalidDevice;
    value.sequence = ++sequence;
    if (Mode("start_error") || (Mode("stop_error") && sequence%2 == 0)) return cudaErrorInvalidValue;
    return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaEventSynchronize(cudaEvent_t) {
    ++event_calls;
    return Mode("sync_error") ? cudaErrorLaunchFailure : cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaEventElapsedTime(float* result, cudaEvent_t, cudaEvent_t) {
    ++event_calls;
    if (Mode("elapsed_error")) return cudaErrorInvalidValue;
    *result = Mode("invalid_time") ? NAN : 1.25f;
    return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaEventDestroy(cudaEvent_t event) {
    ++event_calls;
    const auto* value = reinterpret_cast<Event*>(event);
    const bool wrong_device = value->device != current_device;
    delete value;
    ++destroys;
    return wrong_device ? cudaErrorInvalidDevice : Mode("destroy_error") ? cudaErrorUnknown : cudaSuccess;
}
__attribute__((constructor)) static void EarlyRegister() {
    if (!std::getenv("FAKE_CUDA_EARLY")) return;
    __cudaRegisterFunction(reinterpret_cast<void**>(0x77), reinterpret_cast<const char*>(0x123),
                           nullptr, "early_kernel", -1, nullptr, nullptr, nullptr, nullptr, nullptr);
    early_result = errno == E2BIG;
}
