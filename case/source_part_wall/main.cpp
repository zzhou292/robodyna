#include "case/source_part_elastic/SourcePartElasticPilot.h"
#include "case/SourcePartWallArtifacts.h"
#include "case/source_part_wall/SourcePartWallSetup.h"
#include "case/source_part_wall/SourcePartWallContact.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
namespace {
std::uint64_t Count(const char* text,std::uint64_t cap) {
    const std::string value(text);std::uint64_t count=0;const auto r=std::from_chars(value.data(),value.data()+value.size(),count);
    if(r.ec!=std::errc{}||r.ptr!=value.data()+value.size()||!count||count>cap)throw std::invalid_argument("Count is outside the bounded wall pilot");return count;
}
std::string Failure(const crash::cases::source_part_elastic::Report& r) {
    std::ostringstream out;out<<std::setprecision(17)<<r.message<<" parent="<<r.source_parent_id<<" measured="<<r.measured<<" limit="<<r.limit;return out.str();
}
}
int main(int argc,char** argv) {
    namespace part=crash::cases::source_part_elastic;namespace wall=crash::cases::source_part_wall;
    namespace data=crash::case_data;namespace source=crash::qualification::source_contact;
    std::unique_ptr<wall::SourcePartWallArtifacts> artifacts;
    try {
        if(argc!=7)throw std::invalid_argument("usage: robo_dyna_source_part_wall READINESS WALL REFINEMENT BASE_STEPS FRAME_EVERY_BASE NEW_DIR");
        const auto refinement=unsigned(Count(argv[3],4));const auto base_steps=Count(argv[4],part::PilotHorizonSteps);
        const auto every=Count(argv[5],part::PilotHorizonSteps);
        if(every>base_steps||base_steps%every||base_steps/every+3>1000)throw std::invalid_argument("Frame stride must divide the bounded horizon");
        source::SourcePartContactFixture input;const auto loaded=source::LoadPinnedSourcePartContact(argv[1],&input);
        if(loaded.status!=source::FixtureStatus::Ok)throw std::runtime_error(loaded.diagnostic);
        const auto bytes=data::ReadPinnedWallManifest(argv[2]);std::istringstream stream(bytes);data::CanonicalWall canonical;
        const auto loaded_wall=canonical.Load(stream);if(loaded_wall.status!=data::WallStatus::Ok)throw std::runtime_error(loaded_wall.message);
        part::SourcePartElasticCase run;const auto config=part::MeshWallConfig(refinement);
        const auto initialized=run.Initialize(input,config,canonical,bytes,part::MeshWallSettings(config));
        if(!initialized)throw std::runtime_error(Failure(initialized));const auto steps=base_steps*refinement;
        artifacts=std::make_unique<wall::SourcePartWallArtifacts>(argv[6],run,steps,unsigned(every*refinement),
            0x53505752554e3031ULL+refinement,0x535057544f503031ULL);artifacts->WriteFrame(run);
        std::uint64_t last_saved=0;const auto started=std::chrono::steady_clock::now();
        const auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
        for(std::uint64_t step=1;step<=steps;++step) {
            const auto base=run.owner().accepted();const auto result=run.Step();
            if(!result) {
                const auto reason=Failure(result);if(!base.epoch)throw std::runtime_error(reason);
                if(base.epoch!=last_saved)artifacts->WriteFrame(run);
                artifacts->FinishPrefix(run,elapsed(),reason);
                std::cerr<<"Accepted wall prefix archived through epoch "<<base.epoch<<": "<<reason<<'\n';return 2;
            }
            artifacts->RecordInterval(base,run);
            if(step==1||step%(every*refinement)==0||step==steps){artifacts->WriteFrame(run);last_saved=step;}
            if(step%(512*refinement)==0||step==steps) {
                const auto& c=run.accepted_contact()->diagnostics;
                std::cout<<std::setprecision(8)<<"accepted "<<step<<'/'<<steps<<" t="<<run.owner().accepted().time
                    <<" s wall_force="<<c.resultant.value<<" N penetration="<<c.maximum_penetration
                    <<" m energy_residual="<<run.diagnostics().energy_residual<<" J\n"<<std::flush;
            }
        }
        artifacts->Finish(run,elapsed());std::cout<<"Completed bounded source mesh-wall horizon: "<<argv[6]<<'\n';return 0;
    } catch(const std::exception& error) {
        if(artifacts)artifacts->Fail(error.what());std::cerr<<"Source wall output incomplete: "<<error.what()<<'\n';return 1;
    }
}
