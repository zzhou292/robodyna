#include "Capture.h"
#include <iomanip>
namespace crash::viewer::physical_run {
Capture::Capture(std::filesystem::path directory,std::size_t samples,std::size_t cap)
    :directory_(std::move(directory)),expected_(samples),cap_(cap) {
    CaptureForecast(samples,cap);
    index_<<"sample,epoch,accepted_time_s,file,bytes,sha256\n"<<std::setprecision(17);
}
void Capture::Frame(FixedReplayVisual& visual,const visual::ReplayStamp& stamp) {
    output::Require(stamp.index==count_ && count_<expected_,"PNG sample order differs from archive");
    output::Require(bytes_<=cap_ && (32u<<20)<=cap_-bytes_,"PNG capture byte cap exhausted");
    std::ostringstream name;name<<"frame-"<<std::setw(6)<<std::setfill('0')<<count_<<".png";
    const auto file=CaptureReplayImage(visual,directory_/name.str());
    bytes_+=file.bytes;
    index_<<stamp.index<<','<<stamp.epoch<<','<<stamp.time<<','<<file.file<<','<<file.bytes<<','<<file.sha256<<'\n';
    ++count_;
}
void Capture::Finish(const Options& options,const Input& input,const visual::physical_run::Scene& scene,
        FixedReplayVisual& visual,ReplayLighting light) {
    output::Require(count_==expected_ && scene.stamp()->index+1==expected_,"Capture ended before all archived samples");
    const auto bytes=index_.str();
    output::Require(bytes.size()<=2u<<20 && bytes_+bytes.size()<=cap_-(2u<<20),"Capture index/metadata cap exhausted");
    output::WriteBytes(directory_/"frames.csv",bytes);
    output::physical_run::WriteDocument(directory_,"manifest.json",
        CaptureMetadata(options,input,scene,visual,light,bytes_,bytes),2u<<20);
}
} // namespace crash::viewer::physical_run
