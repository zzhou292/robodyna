#include "chrono_sensor/ChConfigSensor.h"

#include <nvrtc.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {
struct Program {
    nvrtcProgram value = nullptr;
    ~Program() {
        if (value)
            nvrtcDestroyProgram(&value);
    }
};
}  // namespace

TEST(OptixRuntimeCompiler, CompilesOriginalBoxShaderToOptixIrWithoutCreatingADevice) {
    const std::filesystem::path source = std::filesystem::path(CHRONO_SENSOR_SHADER_DIR) / "box.cu";
    std::ifstream input(source);
    ASSERT_TRUE(input.good()) << source;
    const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const char* directories[] = {CUDA_NVRTC_INCLUDE_LIST};
    const char* original_flags[] = {CUDA_NVRTC_FLAG_LIST};
    std::vector<std::string> owned;
    for (const auto* directory : directories) {
        if (!directory)
            break;
        ASSERT_TRUE(std::filesystem::is_directory(directory)) << directory;
        owned.emplace_back(std::string("-I") + directory);
    }
    for (const auto* flag : original_flags) {
        if (!flag)
            break;
        owned.emplace_back(flag);
    }
    // The retained production path selects this same IR on CUDA12+; the pinned
    // CUDA13.2 runtime compiler supports it without an OptiX device/context.
    owned.emplace_back("--optix-ir");
    std::vector<const char*> flags;
    for (const auto& flag : owned)
        flags.push_back(flag.c_str());
    Program program;
    const auto filename = source.string();
    ASSERT_EQ(nvrtcCreateProgram(&program.value, text.c_str(), filename.c_str(), 0, nullptr, nullptr), NVRTC_SUCCESS);
    const auto result = nvrtcCompileProgram(program.value, static_cast<int>(flags.size()), flags.data());
    size_t log_size = 0;
    ASSERT_EQ(nvrtcGetProgramLogSize(program.value, &log_size), NVRTC_SUCCESS);
    std::vector<char> log(log_size ? log_size : 1, '\0');
    ASSERT_EQ(nvrtcGetProgramLog(program.value, log.data()), NVRTC_SUCCESS);
    ASSERT_EQ(result, NVRTC_SUCCESS) << log.data();
    size_t ir_size = 0;
    ASSERT_EQ(nvrtcGetOptiXIRSize(program.value, &ir_size), NVRTC_SUCCESS);
    ASSERT_GT(ir_size, 0);
    std::vector<char> ir(ir_size);
    ASSERT_EQ(nvrtcGetOptiXIR(program.value, ir.data()), NVRTC_SUCCESS);
}
