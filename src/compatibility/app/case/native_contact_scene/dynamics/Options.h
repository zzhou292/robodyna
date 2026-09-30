#pragma once
#include "NativeSceneRun.h"
namespace crash::cases::native_scene::cli {
struct Options {
    std::filesystem::path source,output,source_output,stop_file;
    std::string source_sha256;
    RunConfig config;vehicle_run::Control control;
    bool forecast_only=false;
};
Options Parse(int,const char* const*);
const char* Usage() noexcept;
}
