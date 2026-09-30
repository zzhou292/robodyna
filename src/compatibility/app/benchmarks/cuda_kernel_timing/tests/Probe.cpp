#include "../Runtime.h"
#include "../Timing.h"
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

extern "C" unsigned fake_launches();
extern "C" unsigned fake_events();
extern "C" unsigned fake_live_events();
extern "C" int fake_early_result();
extern "C" const void* fake_last_key();
extern "C" void fake_device(int);
namespace {
void** module = reinterpret_cast<void**>(0x11);
const void* host = reinterpret_cast<void*>(0x456);
void Register(const void* pointer, const char* name) {
    __cudaRegisterFunction(module, static_cast<const char*>(pointer), nullptr, name, -1,
                           nullptr, nullptr, nullptr, nullptr, nullptr);
    if (errno != E2BIG) std::abort();
}
bool Launch(cudaKernel_t kernel, bool handle, bool fail, unsigned& effects) {
    void* args[]{&effects};
    errno = EBUSY;
    const auto result = handle
        ? __cudaLaunchKernel(kernel, dim3(fail ? 0 : 1, 2, 3), dim3(4, 5, 6), args, 19,
                             reinterpret_cast<cudaStream_t>(0x55))
        : cudaLaunchKernel(host, dim3(fail ? 0 : 1, 2, 3), dim3(4, 5, 6), args, 19,
                           reinterpret_cast<cudaStream_t>(0x55));
    return result == (fail ? cudaErrorInvalidConfiguration : cudaSuccess) && errno == (fail ? EAGAIN : E2BIG) &&
        fake_last_key() == (handle ? static_cast<const void*>(kernel) : host);
}
}
int main(int argc, char** argv) {
    if (std::getenv("FAKE_CUDA_EARLY") && !fake_early_result()) return 1;
    const bool overflow = argc > 1 && std::strcmp(argv[1], "overflow") == 0;
    const char* mode = std::getenv("FAKE_CUDA_MODE");
    Register(host, mode && std::strcmp(mode, "unnamed") == 0 ? "" : "kernel_\"line\\\n");
    cudaKernel_t kernel = nullptr;
    if (__cudaGetKernel(&kernel, host) != cudaSuccess || errno != E2BIG) return 2;
    const auto saved = kernel;
    if (__cudaGetKernel(&kernel, reinterpret_cast<void*>(0xdead)) != cudaErrorInvalidDeviceFunction ||
        errno != ENOENT || kernel != saved) return 3;
    unsigned effects = 0;
    if (!Launch(kernel, true, false, effects) || !Launch(kernel, false, false, effects) ||
        !Launch(kernel, true, true, effects) || effects != 2) return 4;
    fake_device(1); // Every event pair must be local to the active call/context.
    if (!Launch(kernel, true, false, effects)) return 5;
    fake_device(0);
    std::atomic<unsigned> failures{0};
    std::thread threads[4];
    for (auto& thread : threads) thread = std::thread([&] {
        unsigned local = 0;
        for (unsigned i = 0; i < 10; ++i)
            if (!Launch(kernel, true, false, local)) ++failures;
        if (local != 10) ++failures;
    });
    for (auto& thread : threads) thread.join();
    if (failures) return 6;
    __cudaUnregisterFatBinary(module);
    if (errno != E2BIG || !Launch(kernel, true, false, effects)) return 7;
    Register(host, "replacement_kernel");
    // Successful lookup explicitly rebinds a reused runtime handle.
    if (__cudaGetKernel(&kernel, host) != cudaSuccess || !Launch(kernel, true, false, effects)) return 8;
    if (overflow) {
        using namespace robo_dyna::cuda_kernel_timing;
        char long_name[NameCap+20];
        std::memset(long_name, 'x', sizeof(long_name)-1);
        long_name[sizeof(long_name)-1] = 0;
        Register(reinterpret_cast<void*>(0x900), long_name);
        for (std::size_t i = 0; i < HandleCap+2; ++i) {
            const auto pointer = reinterpret_cast<void*>(0x1000+i);
            Register(pointer, "capacity_kernel");
            cudaKernel_t extra = nullptr;
            if (__cudaGetKernel(&extra, pointer) != cudaSuccess) return 9;
        }
    }
    if (fake_launches() != 46 || effects != 5 || fake_live_events()) return 10;
    const bool enabled = std::getenv("ROBO_DYNA_CUDA_KERNEL_TIMING_OUTPUT") &&
        *std::getenv("ROBO_DYNA_CUDA_KERNEL_TIMING_OUTPUT");
    if (!enabled && fake_events()) return 11;
    std::cout << "fake kernel forwarding passed\n";
}
