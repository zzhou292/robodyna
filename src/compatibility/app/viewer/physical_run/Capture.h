#pragma once
#include "Options.h"
#include "viewer/ReplayVsg.h"
#include <sstream>
namespace crash::viewer::physical_run {
class Capture {
  public:
    Capture(std::filesystem::path,std::size_t samples,std::size_t byte_cap);
    void Frame(FixedReplayVisual&,const visual::ReplayStamp&);
    void Finish(const Options&,const Input&,const visual::physical_run::Scene&,FixedReplayVisual&,ReplayLighting);
  private:
    std::filesystem::path directory_;
    std::size_t expected_,count_=0,bytes_=0,cap_;
    std::ostringstream index_;
};
output::Document CaptureMetadata(const Options&,const Input&,const visual::physical_run::Scene&,
    FixedReplayVisual&,ReplayLighting,std::size_t image_bytes,const std::string& index);
} // namespace crash::viewer::physical_run
