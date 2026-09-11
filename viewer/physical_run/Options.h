#pragma once
#include "chrono/physical_run/Scene.h"
#include "output/physical_run/ViewerInput.h"
namespace crash::viewer::physical_run {
struct Options {
    std::filesystem::path input,capture;
    std::string expected_receipt_sha256;
    double frames_per_second=10;
    std::size_t require_frames=0,capture_bytes=2ull<<30;
    visual::physical_run::SceneOptions scene;
};
Options Parse(int argc,char** argv);
struct Input {
    std::filesystem::path directory;
    output::full_shell::RecordFile receipt;
    output::physical_run::ViewerInput values;
};
Input ReadInput(const Options&);
std::size_t CaptureForecast(std::size_t samples,std::size_t cap);
void CheckCaptureDestination(const std::filesystem::path& input,const std::filesystem::path& output);
} // namespace crash::viewer::physical_run
