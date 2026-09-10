#include "CliOptions.h"
#include <charconv>
#include <stdexcept>
#include <string_view>

namespace crash::cases::source_assembly_wall {
namespace {
constexpr const char* Usage="usage: robo_dyna_source_assembly_wall INVENTORY WALL STEPS FRAME_EVERY NEW_DIR "
    "[REFINEMENT_1_2_4] [--stage-timing NEW_JSON] [--step-multiple 1_2_4_8] [--observe-force-stage] "
    "[--spin-node SOURCE_NID --spin-output NEW_JSONL [--spin-every INTERVALS]]";
std::uint64_t Count(std::string_view value,std::uint64_t maximum) {
    std::uint64_t count=0;
    const auto parsed=std::from_chars(value.data(),value.data()+value.size(),count);
    if(parsed.ec!=std::errc{}||parsed.ptr!=value.data()+value.size()||!count||count>maximum)
        throw std::invalid_argument("Expected a positive count within the declared limit");
    return count;
}
}
CliOptions ParseOptions(int argc,const char* const* argv) {
    if(argc<6||!argv)throw std::invalid_argument(Usage);
    for(int i=0;i<argc;++i)if(!argv[i]||!*argv[i])throw std::invalid_argument(Usage);
    CliOptions next;next.inventory=argv[1];next.wall=argv[2];next.archive=argv[5];
    next.steps=Count(argv[3],1<<20);next.frame_every=static_cast<unsigned>(Count(argv[4],next.steps));
    int i=6;
    if(i<argc&&std::string_view(argv[i]).substr(0,2)!="--")
        next.pilot.refinement=static_cast<unsigned>(Count(argv[i++],4));
    bool saw_multiple=false,saw_spin_every=false;
    while(i<argc) {
        const std::string_view option=argv[i++];
        if(option=="--observe-force-stage") {
            if(next.pilot.observe_force_stage)throw std::invalid_argument("Repeated force-stage option");
            next.pilot.observe_force_stage=true;continue;
        }
        if(i==argc||std::string_view(argv[i]).substr(0,2)=="--")
            throw std::invalid_argument("Missing assembly option value");
        if(option=="--stage-timing"&&next.timing_path.empty()) {
            next.timing_path=argv[i++];next.pilot.timing.enabled=true;
        } else if(option=="--step-multiple"&&!saw_multiple) {
            next.pilot.step_multiple=static_cast<unsigned>(Count(argv[i++],8));saw_multiple=true;
        } else if(option=="--spin-node"&&!next.pilot.observe_qeph_spin_node) {
            next.pilot.observe_qeph_spin_node=Count(argv[i++],UINT64_MAX);
        } else if(option=="--spin-output"&&next.spin_path.empty()) {
            next.spin_path=argv[i++];
        } else if(option=="--spin-every"&&!saw_spin_every) {
            next.spin_every=Count(argv[i++],next.steps);saw_spin_every=true;
        } else throw std::invalid_argument("Unknown or repeated assembly option");
    }
    if(bool(next.pilot.observe_qeph_spin_node)!=!next.spin_path.empty()||
       (saw_spin_every&&!next.pilot.observe_qeph_spin_node))
        throw std::invalid_argument("Spin trace requires both source node and output path");
    if(!saw_spin_every&&next.spin_every>next.steps)next.spin_every=next.steps;
    PilotFixedStep(next.pilot);return next;
}
}
