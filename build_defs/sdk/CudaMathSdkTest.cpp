#include <cublas_v2.h>
#include <cudss.h>
#include <cusolver_common.h>
#include <cusparse.h>
#include <nvrtc.h>

#include <gtest/gtest.h>
#include <dlfcn.h>
#include <string>

namespace {
TEST(CudaMathSdk, HeaderAndHostLibraryVersionsAgreeWithoutADeviceHandle) {
    int value = 0;
    ASSERT_EQ(cublasGetProperty(MAJOR_VERSION, &value), CUBLAS_STATUS_SUCCESS);
    EXPECT_EQ(value, CUBLAS_VER_MAJOR);
    ASSERT_EQ(cublasGetProperty(MINOR_VERSION, &value), CUBLAS_STATUS_SUCCESS);
    EXPECT_EQ(value, CUBLAS_VER_MINOR);
    ASSERT_EQ(cudssGetProperty(MAJOR_VERSION, &value), CUDSS_STATUS_SUCCESS);
    EXPECT_EQ(value, CUDSS_VERSION_MAJOR);
    ASSERT_EQ(cudssGetProperty(MINOR_VERSION, &value), CUDSS_STATUS_SUCCESS);
    EXPECT_EQ(value, CUDSS_VERSION_MINOR);
    ASSERT_EQ(cusolverGetProperty(MAJOR_VERSION, &value), CUSOLVER_STATUS_SUCCESS);
    EXPECT_EQ(value, CUSOLVER_VER_MAJOR);
    ASSERT_EQ(cusparseGetProperty(MAJOR_VERSION, &value), CUSPARSE_STATUS_SUCCESS);
    EXPECT_EQ(value, CUSPARSE_VER_MAJOR);
    int major = 0, minor = 0;
    ASSERT_EQ(nvrtcVersion(&major, &minor), NVRTC_SUCCESS);
    EXPECT_EQ(major, 13);
    EXPECT_EQ(minor, 2);
}

TEST(CudaMathSdk, EveryAdmittedSharedAbiAndNvrtcCompanionLoads) {
    struct Entry { const char* library; const char* symbol; };
    const Entry entries[] = {
        {"libcublas.so.13", "cublasGetProperty"},
        {"libcublasLt.so.13", "cublasLtGetVersion"},
        {"libcusparse.so.12", "cusparseGetProperty"},
        {"libcusolver.so.12", "cusolverGetProperty"},
        {"libcudss.so.0", "cudssGetProperty"},
        {"libnvJitLink.so.13", nullptr},
        {"libnvrtc.so.13", "nvrtcVersion"},
        {"libnvrtc-builtins.so.13.2", nullptr},
    };
    for (const auto& entry : entries) {
        SCOPED_TRACE(entry.library);
        void* handle = dlopen(entry.library, RTLD_NOW | RTLD_LOCAL);
        ASSERT_NE(handle, nullptr) << dlerror();
        if (entry.symbol) {
            EXPECT_NE(dlsym(handle, entry.symbol), nullptr) << entry.symbol;
        }
        EXPECT_EQ(dlclose(handle), 0);
    }
    // No cudaSetDevice, context, matrix/solver handle, allocation or kernel call.
}

TEST(CudaMathSdk, NvrtcFindsItsBuiltinsAndCompilesWithoutLaunchingAKernel) {
    struct Program {
        nvrtcProgram value = nullptr;
        ~Program() { if (value) nvrtcDestroyProgram(&value); }
    } program;
    const char* source = "extern \"C\" __global__ void sdk_probe(float* x) { x[threadIdx.x] = __sinf(x[threadIdx.x]); }";
    ASSERT_EQ(nvrtcCreateProgram(&program.value, source, "sdk_probe.cu", 0, nullptr, nullptr), NVRTC_SUCCESS);
    const char* options[] = {"--gpu-architecture=compute_75", "--std=c++17"};
    const auto status = nvrtcCompileProgram(program.value, 2, options);
    std::string log;
    if (status != NVRTC_SUCCESS) {
        std::size_t length = 0;
        if (nvrtcGetProgramLogSize(program.value, &length) == NVRTC_SUCCESS && length <= 65536) {
            log.resize(length);
            nvrtcGetProgramLog(program.value, log.data());
        }
    }
    ASSERT_EQ(status, NVRTC_SUCCESS) << log;
    std::size_t ptx_bytes = 0;
    ASSERT_EQ(nvrtcGetPTXSize(program.value, &ptx_bytes), NVRTC_SUCCESS);
    EXPECT_GT(ptx_bytes, 0u);
    // Compilation is a CPU operation. No driver context/module or GPU launch.
}
}  // namespace
