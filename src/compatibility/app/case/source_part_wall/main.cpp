#include "case/source_part_elastic/SourcePartElasticPilot.h"
#include "SourcePartWallExecution.h"
#include "SourcePartWallSetup.h"
#include "case/CanonicalWallArtifacts.h"
#include <iostream>
#include <sstream>

int main(int argc,char** argv) {
    namespace part=crash::cases::source_part_elastic;
    namespace wall=crash::cases::source_part_wall;
    namespace data=crash::case_data;
    namespace source=crash::qualification::source_contact;
    try {
        if(argc!=7) throw std::invalid_argument(
            "usage: robo_dyna_source_part_wall READINESS WALL REFINEMENT BASE_STEPS FRAME_EVERY_BASE NEW_DIR");
        const auto refinement=unsigned(wall::ParseWallCount(argv[3],4));
        const auto base_steps=wall::ParseWallCount(argv[4],part::PilotHorizonSteps);
        const auto every=wall::ParseWallCount(argv[5],part::PilotHorizonSteps);
        if(every>base_steps||base_steps%every||base_steps/every+3>1000)
            throw std::invalid_argument("Frame stride must divide the bounded horizon");
        source::SourcePartContactFixture input;
        const auto loaded=source::LoadPinnedSourcePartContact(argv[1],&input);
        if(loaded.status!=source::FixtureStatus::Ok) throw std::runtime_error(loaded.diagnostic);
        const auto bytes=data::ReadPinnedWallManifest(argv[2]);
        std::istringstream stream(bytes); data::CanonicalWall canonical;
        const auto loaded_wall=canonical.Load(stream);
        if(loaded_wall.status!=data::WallStatus::Ok) throw std::runtime_error(loaded_wall.message);
        part::SourcePartElasticCase run;
        const auto config=part::MeshWallConfig(refinement);
        const auto initialized=run.Initialize(input,config,canonical,bytes,part::MeshWallSettings(config));
        if(!initialized) throw std::runtime_error(wall::FormatWallFailure(initialized));
        return wall::ExecuteWallCase(run,base_steps*refinement,unsigned(every*refinement),argv[6],
            0x53505752554e3031ULL+refinement,0x535057544f503031ULL);
    } catch(const std::exception& error) {
        std::cerr<<"Source wall output incomplete: "<<error.what()<<'\n';return 1;
    }
}
