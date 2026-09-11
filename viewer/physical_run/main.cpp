#include "Options.h"
#include "Playback.h"
#include "Capture.h"
#include <vsg/core/Exception.h>
#include <chrono>
#include <iostream>
#include <thread>
namespace {
using Clock=std::chrono::steady_clock;
int Failure(const std::filesystem::path& directory,const std::string& message) {
    if(!directory.empty()) {
        try {crash::output::WriteBytes(directory/"failure.txt",message+'\n');} catch(...) {}
    }
    std::cerr<<"robo-dyna physical replay: "<<message<<'\n';return 1;
}
}
int main(int argc,char** argv) {
    namespace app=crash::viewer::physical_run;
    namespace output=crash::output;
    std::filesystem::path capture_directory;
    try {
        const auto options=app::Parse(argc,argv);const auto input=app::ReadInput(options);
        const auto root=output::physical_run::ViewerArchivePath(input.directory,input.values);
        const auto replay=output::physical_run::Replay::Open(root,input.values.manifest,input.values.source,input.values.mapping_sha256);
        output::Require(!options.require_frames || options.require_frames==replay.index().frames.size(),"Required sample count differs");
        const bool capture=!options.capture.empty();
        if(capture) {
            app::CheckCaptureDestination(input.directory,options.capture);
            app::CaptureForecast(replay.index().frames.size(),options.capture_bytes);
        }
        crash::visual::physical_run::Scene scene;
        const auto initialized=scene.Initialize(replay,options.scene);
        output::Require(initialized.status==crash::visual::ReplaySceneStatus::Ok,initialized.message);
        std::cout<<"Physical replay: "<<replay.context().nodes()<<" shell nodes, "<<replay.index().frames.size()
            <<" recorded samples, host forecast "<<scene.forecast()->peak_host_bytes<<" bytes; no physics executed"<<std::endl;
        std::unique_ptr<app::Capture> exporter;
        if(capture) {
            output::Require(std::filesystem::create_directory(options.capture),"Could not create PNG capture directory");
            capture_directory=options.capture;
            exporter=std::make_unique<app::Capture>(capture_directory,replay.index().frames.size(),options.capture_bytes);
        }
        auto visual=std::make_shared<crash::viewer::FixedReplayVisual>();
        app::Playback playback;
        const auto light=crash::viewer::ConfigureReplayVisual(*visual,scene.system(),*scene.camera());
        visual->AddGuiComponent(app::MakeOverlay(scene,playback,capture,options.frames_per_second));
        crash::viewer::InitializeReplayVisual(*visual);
        if(capture) crash::viewer::WarmupReplayCapture(*visual);
        bool first=true;auto next=Clock::now();
        while(visual->Run()) {
            const auto now=Clock::now();
            if(!first && (capture || playback.next || (!playback.paused && now>=next)) &&
                scene.stamp()->index+1<replay.index().frames.size()) {
                const auto report=scene.Publish(scene.stamp()->index+1);
                output::Require(report.status==crash::visual::ReplaySceneStatus::Ok,report.message);
                playback.next=false;
                next=now+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/options.frames_per_second));
            }
            if(capture) {
                exporter->Frame(*visual,*scene.stamp());
                if(scene.stamp()->index+1==replay.index().frames.size()) break;
            } else crash::viewer::RenderReplayFrame(*visual);
            if(first) {
                next=now+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/options.frames_per_second));
                first=false;
            }
            if(playback.close) {visual->Quit();break;}
            if(!capture) std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        if(capture) exporter->Finish(options,input,scene,*visual,light);
        return 0;
    } catch(const vsg::Exception& error) {return Failure(capture_directory,error.message);}
      catch(const std::exception& error) {return Failure(capture_directory,error.what());}
}
