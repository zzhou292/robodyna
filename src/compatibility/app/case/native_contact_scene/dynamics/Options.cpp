#include "Options.h"
#include "output/BoundedArrayIO.h"
#include <charconv>
#include <cmath>
#include <set>
namespace crash::cases::native_scene::cli {
namespace {
std::uint64_t Integer(const std::string& s) {std::uint64_t x=0;const auto p=std::from_chars(s.data(),s.data()+s.size(),x);
    if(p.ec!=std::errc{}||p.ptr!=s.data()+s.size())throw std::invalid_argument("Expected bounded unsigned CLI integer");return x;}
double Real(const std::string& s){std::size_t used=0;const auto x=std::stod(s,&used);
    if(used!=s.size()||!std::isfinite(x))throw std::invalid_argument("Expected finite CLI scalar");return x;}
}
Options Parse(int argc,const char* const* argv) {
    if(argc<1||argc>48)throw std::invalid_argument("Native scene argument count exceeds cap");
    Options o;std::set<std::string> seen;
    for(int i=1;i<argc;++i){const std::string key=argv[i];
        if(key.size()>4096||!seen.insert(key).second)throw std::invalid_argument("Duplicate or oversized native scene option");
        if(key=="--forecast-only"){o.forecast_only=true;continue;}
        if(++i==argc)throw std::invalid_argument("Missing native scene option value");
        const std::string value=argv[i];if(value.size()>4096)throw std::invalid_argument("Native scene option value exceeds cap");
        if(key=="--source")o.source=value;else if(key=="--source-sha256")o.source_sha256=value;
        else if(key=="--output")o.output=value;else if(key=="--source-output")o.source_output=value;
        else if(key=="--stop-file")o.stop_file=value;else if(key=="--run-id")o.config.run_id=Integer(value);
        else if(key=="--steps")o.config.steps=Integer(value);else if(key=="--samples")o.config.samples=Integer(value);
        else if(key=="--fixed-dt-s")o.config.dynamics.fixed_dt=Real(value);
        else if(key=="--diagnostic-intervals")o.control.maximum_accepted_intervals=Integer(value);
        else if(key=="--maximum-elapsed-s")o.control.maximum_elapsed_s=Real(value);
        else throw std::invalid_argument("Unknown native scene option");
    }
    if(o.source.empty()||o.output.empty()||o.source_output.empty()||!o.config.run_id)
        throw std::invalid_argument("Explicit authenticated source, fresh source-output/output and run-id are required");
    if(o.config.dynamics.fixed_dt<=0||!o.config.steps||o.config.steps>1000000||o.config.samples<2||o.config.samples>1000||
        o.config.samples-1>o.config.steps||o.control.maximum_elapsed_s<0)
        throw std::invalid_argument("Invalid native scene timestep/count/sample/control domain");
    output::arrays::CheckHash(o.source_sha256);
    if(o.output.lexically_normal()==o.source_output.lexically_normal())throw std::invalid_argument("Static source and physical run directories must differ");
    o.config.dynamics.configuration=0x4e4154495645ULL;o.config.dynamics.qualification=0x5343454e4531ULL;
    return o;
}
const char* Usage() noexcept{return "robo_dyna_native_scene_run --source declared-scene.json --source-sha256 SHA256 --source-output NEW_STATIC_DIR --output EMPTY_RUN_DIR --run-id N [--steps 1000 --fixed-dt-s 3e-7 --samples 31 --forecast-only --diagnostic-intervals N --stop-file PATH --maximum-elapsed-s S]";}
}
