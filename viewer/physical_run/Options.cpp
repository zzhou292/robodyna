#include "Options.h"
#include <cmath>
#include <set>
namespace crash::viewer::physical_run {
Options Parse(int argc,char** argv) {
    using output::Require;
    Require(argc>=2,"usage: robo_dyna_physical_replay RUN_DIR_OR_RECEIPT [--capture NEW_DIR] [--fps 1..60] [--color part-id|plastic-strain|uniform] [--view incident-side|wall-side] [--wireframe] [--require-frames N] [--receipt-sha256 SHA] [--capture-cap-gib 2|6]");
    Options out;out.input=argv[1];
    std::set<std::string> seen;
    for(int i=2;i<argc;++i) {
        const std::string name=argv[i];
        Require(seen.insert(name).second,"Duplicate physical viewer option");
        if(name=="--wireframe") {out.scene.wireframe=true;continue;}
        Require(i+1<argc,"Missing physical viewer option value");
        const std::string value=argv[++i];
        if(name=="--capture") out.capture=value;
        else if(name=="--fps") {
            std::size_t end=0;out.frames_per_second=std::stod(value,&end);
            Require(end==value.size() && std::isfinite(out.frames_per_second) && out.frames_per_second>=1 &&
                out.frames_per_second<=60,"Recorded sample playback rate must be 1..60");
        } else if(name=="--color") {
            Require(visual::ParseReplayColorMode(value,out.scene.colors),"Unknown physical color mode");
        } else if(name=="--view") {
            Require(visual::ParseReplayView(value,out.scene.view),"Unknown physical camera view");
        } else if(name=="--require-frames") {
            std::size_t end=0;const auto n=std::stoull(value,&end);
            Require(end==value.size() && n>0 && n<=10000,"Required sample count must be 1..10000");
            out.require_frames=n;
        } else if(name=="--receipt-sha256") {
            output::arrays::CheckHash(value);out.expected_receipt_sha256=value;
        } else if(name=="--capture-cap-gib") {
            Require(value=="2" || value=="6","Capture cap must be explicit 2 or 6 GiB");
            out.capture_bytes=(value=="2"?2ull:6ull)<<30;
        } else throw std::invalid_argument("Unknown physical viewer option: "+name);
    }
    return out;
}
} // namespace crash::viewer::physical_run
