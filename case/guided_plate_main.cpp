#include "GuidedPlateArtifacts.h"
#include "GuidedPlateStudyIO.h"
#include "CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace {
namespace cd=crash::case_data;
namespace io=crash::output;
void Check(const cd::GuidedPlateReport& report) {
    if(report.status!=cd::GuidedPlateStatus::Ok)throw std::runtime_error(report.diagnostic);
}
void NewPath(const std::filesystem::path& path) {
    io::Require(!path.empty()&&!std::filesystem::exists(path),"Output path must be new");
    const auto parent=path.has_parent_path()?path.parent_path():std::filesystem::path(".");
    io::Require(std::filesystem::is_directory(parent),"Output parent directory must exist");
}
int Compare(const char* coarse_path,const char* fine_path,const char* new_report) {
    NewPath(new_report);
    const auto coarse_bytes=io::ReadBounded(coarse_path,cd::kGuidedStudyByteCap);
    const auto fine_bytes=io::ReadBounded(fine_path,cd::kGuidedStudyByteCap);
    const auto coarse=cd::ParseGuidedPlateStudy(coarse_bytes),fine=cd::ParseGuidedPlateStudy(fine_bytes);
    cd::GuidedStudyComparison comparison;std::string error;
    if(!cd::CompareGuidedPlateStudies(coarse,fine,comparison,error))throw std::runtime_error(error);
    cd::WriteGuidedPlateComparison(new_report,comparison,io::Sha256(coarse_bytes),io::Sha256(fine_bytes));
    std::cout<<comparison.diagnostic<<'\n';return comparison.passed?0:2;
}
int Run(const char* wall_path,const char* study_path,const char* refinement,const char* bundle_path) {
    NewPath(study_path);if(bundle_path)NewPath(bundle_path);
    const std::string selection=refinement;
    io::Require(selection=="1"||selection=="2"||selection=="4","Refinement must be 1, 2 or 4");
    cd::GuidedPlateConfig config;config.refinement=static_cast<unsigned>(selection[0]-'0');
    const auto bytes=cd::ReadPinnedWallManifest(wall_path);std::istringstream input(bytes);cd::CanonicalWall wall;
    const auto loaded=wall.Load(input);io::Require(loaded.status==cd::WallStatus::Ok,loaded.message);
    cd::GuidedPlateCase run;cd::GuidedPlateStudy observer;
    std::unique_ptr<cd::GuidedPlateArtifacts> artifacts;
    const auto begin=std::chrono::steady_clock::now();
    try {
        Check(run.Initialize(wall,config));cd::GuidedPlateFrame frame;Check(run.Capture(frame));
        cd::GuidedStudyConfig study_config;std::string error;
        if(!cd::PrepareGuidedStudyConfig(*run.metrics(),*run.model_data(),*run.guided_data(),run.contact_reference(),
                                        config.refinement,study_config,error)||
           !observer.Initialize(study_config,*run.metrics(),frame,error))throw std::runtime_error(error);
        constexpr unsigned frame_every=100;
        if(bundle_path) {
            artifacts=std::make_unique<cd::GuidedPlateArtifacts>(bundle_path,bytes,wall,run,frame_every);
            artifacts->WriteFrame(run);
        }
        const auto steps=run.metrics()->required_steps;
        for(std::uint64_t epoch=1;epoch<=steps;++epoch) {
            const auto base=run.metrics()->stamp;Check(run.Step());
            if(artifacts)artifacts->RecordInterval(base,*run.metrics());
            cd::GuidedPlateFrame* sample=nullptr;
            if(observer.NeedsSample(epoch)) {Check(run.Capture(frame));sample=&frame;}
            if(!observer.Record(*run.metrics(),sample,error))throw std::runtime_error(error);
            if(artifacts&&(epoch%frame_every==0||epoch==steps))artifacts->WriteFrame(run);
            if(epoch%((steps+9)/10)==0||epoch==steps) {
                const auto& s=observer.data()->summary;
                std::cout<<"Accepted "<<epoch<<'/'<<steps<<", time="<<s.accepted_time
                         <<" s, peak depth="<<s.maximum_penetration<<" m, max certified energy error="
                         <<s.maximum_certified_energy_relative_error<<'\n'<<std::flush;
            }
        }
        cd::GuidedStudyData completed;if(!observer.Finish(completed,error))throw std::runtime_error(error);
        cd::WriteGuidedPlateStudy(study_path,completed);
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        if(artifacts)artifacts->Finish(run,elapsed);
        std::cout<<"Guided study completed in "<<elapsed<<" s; report "<<study_path
                 <<"; separated rebound observed="<<completed.summary.separated_rebounding<<'\n';return 0;
    } catch(const std::exception& e) {
        if(artifacts)artifacts->Fail(e.what());throw;
    }
}
} // namespace
int main(int argc,char** argv) {
    try {
        if(argc==5&&std::string(argv[1])=="compare")return Compare(argv[2],argv[3],argv[4]);
        if((argc==5||argc==6)&&std::string(argv[1])=="run")return Run(argv[2],argv[3],argv[4],argc==6?argv[5]:nullptr);
        throw std::runtime_error("Usage: robo-dyna-guided run WALL NEW-STUDY 1|2|4 [NEW-BUNDLE] | compare COARSE FINE NEW-REPORT");
    } catch(const std::exception& e) {std::cerr<<"Guided study incomplete: "<<e.what()<<'\n';return 1;}
}
