#include "NormalImpactArtifacts.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
double Number(const char* value) {
    char* end=nullptr;const double result=std::strtod(value,&end);
    if(end==value||*end||!std::isfinite(result))throw std::runtime_error("Invalid finite numeric argument");return result;
}
unsigned Integer(const char* value) {
    const double number=Number(value);if(number<1||number>10000||number!=std::floor(number))throw std::runtime_error("Invalid bounded integer argument");
    return static_cast<unsigned>(number);
}
}  // namespace

int main(int argc,char** argv) {
    namespace cd=crash::case_data;
    cd::NormalImpactCase run;std::unique_ptr<cd::NormalImpactArtifacts> artifacts;
    try {
        if(argc<3)throw std::runtime_error("Usage: robo-dyna canonical-manifest.json NEW-output-directory [--dt seconds] [--horizon seconds] [--frame-every steps] [--patch-divisions 1|2|4] [--refine-wall]");
        cd::NormalImpactConfig config;double horizon=.07;unsigned frame_every=10;
        for(int i=3;i<argc;++i) {
            const std::string option=argv[i];if(option=="--refine-wall"){config.refine_wall=true;continue;}
            if(i+1>=argc)throw std::runtime_error("Missing option value");const char* value=argv[++i];
            if(option=="--dt")config.dt=Number(value);
            else if(option=="--horizon")horizon=Number(value);
            else if(option=="--frame-every")frame_every=Integer(value);
            else if(option=="--patch-divisions")config.patch_divisions=Integer(value);
            else throw std::runtime_error("Unknown command-line option");
        }
        if(config.dt<=0||horizon<=0||horizon>.2||horizon/config.dt>10000||horizon/config.dt<1||
            (config.patch_divisions!=1&&config.patch_divisions!=2&&config.patch_divisions!=4))
            throw std::runtime_error("Run exceeds bounded horizon/step/patch limits");
        const auto steps=static_cast<unsigned>(std::llround(horizon/config.dt));
        if(std::fabs(steps*config.dt-horizon)>32*std::numeric_limits<double>::epsilon()*std::max(horizon,config.dt))
            throw std::runtime_error("Horizon must be an integer number of fixed steps");
        if(1+(steps+frame_every-1)/frame_every>1000)throw std::runtime_error("Output exceeds 1000-frame cap including the initial frame");
        const auto bytes=cd::ReadPinnedWallManifest(argv[1]);std::istringstream input(bytes);cd::CanonicalWall wall;
        const auto loaded=wall.Load(input);if(loaded.status!=cd::WallStatus::Ok)throw std::runtime_error(loaded.message);
        if(wall.vertices().size()!=62||wall.triangles().size()!=100||wall.source_quads().size()!=46)
            throw std::runtime_error("Required canonical Yaris wall counts differ");
        artifacts=std::make_unique<cd::NormalImpactArtifacts>(argv[2],bytes,wall,config,horizon,frame_every);
        const auto begin=std::chrono::steady_clock::now();const auto initialized=run.Initialize(wall,config);
        if(initialized.status!=cd::ImpactStatus::Ok)throw std::runtime_error(initialized.message);
        if(!run.metrics()||!run.output())throw std::runtime_error("Initialized impact has no accepted output");
        artifacts->WriteAcceptedFrame(*run.output(),*run.metrics());
        for(unsigned step=1;step<=steps;++step) {
            const auto base=run.metrics()->stamp;const auto advanced=run.Step();
            if(advanced.status!=cd::ImpactStatus::Ok)throw std::runtime_error(advanced.message);
            if(!run.metrics()||!run.last_interval())throw std::runtime_error("Committed step has no matching interval diagnostics");
            artifacts->RecordInterval(base,*run.metrics(),*run.last_interval());
            if(step%frame_every==0||step==steps) {
                const auto published=run.Publish();if(published.status!=crash::visual::Status::Ok)throw std::runtime_error(published.message);
                artifacts->WriteAcceptedFrame(*run.output(),*run.metrics());
            }
        }
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        artifacts->Finish(run,elapsed);
        std::cout<<"Completed restricted normal-contact rig: "<<run.metrics()->stamp.epoch<<" accepted steps, "
                 <<run.metrics()->stamp.time<<" s. Artifacts: "<<argv[2]<<'\n';return 0;
    } catch(const std::exception& error) {
        if(artifacts)artifacts->Fail(error.what(),run.metrics());
        std::cerr<<"Normal-contact run incomplete: "<<error.what()<<'\n';return 1;
    }
}
