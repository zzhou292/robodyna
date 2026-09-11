#include "Options.h"
namespace crash::viewer::physical_run {
Input ReadInput(const Options& options) {
    auto file=options.input;
    if(std::filesystem::is_directory(file)) file/="viewer-input.json";
    if(file.parent_path().empty()) file=std::filesystem::current_path()/file;
    const auto bytes=output::ReadBounded(file,output::physical_run::ViewerInputByteCap);
    const auto hash=output::Sha256(bytes);
    output::Require(options.expected_receipt_sha256.empty() || options.expected_receipt_sha256==hash,
        "Viewer receipt differs from explicit external hash");
    output::full_shell::RecordFile receipt{file.filename().string(),hash,bytes.size()};
    auto values=output::physical_run::ReadViewerInput(file.parent_path(),receipt);
    return {file.parent_path(),receipt,std::move(values)};
}
std::size_t CaptureForecast(std::size_t samples,std::size_t cap) {
    // One bounded PNG per actual sample, plus index/metadata. This conservative
    // output cap is separate from the immutable binary input archive's cap.
    constexpr std::size_t per_image=32u<<20,reserve=4u<<20;
    output::Require(samples && (cap==2ull<<30 || cap==6ull<<30) && cap>=reserve &&
        samples<=(cap-reserve)/per_image,"Physical PNG capture forecast exceeds selected output cap");
    return reserve+samples*per_image;
}
void CheckCaptureDestination(const std::filesystem::path& input,const std::filesystem::path& output) {
    const auto parent=std::filesystem::weakly_canonical(input),child=std::filesystem::weakly_canonical(output);
    auto a=parent.begin(),b=child.begin();
    while(a!=parent.end() && b!=child.end() && *a==*b) {++a;++b;}
    output::Require(a!=parent.end(),"Capture directory must be outside the immutable input run");
    output::Require(!std::filesystem::exists(child) && std::filesystem::is_directory(child.parent_path()),
        "Capture directory must be new with an existing parent");
}
} // namespace crash::viewer::physical_run
