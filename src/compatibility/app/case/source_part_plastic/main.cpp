#include "SourcePartPlasticPilot.h"
#include "case/source_part_wall/SourcePartWallExecution.h"
#include "case/CanonicalWallArtifacts.h"
#include <iostream>
#include <sstream>

int main(int argc,char** argv) {
    namespace plastic=crash::cases::source_part_plastic;
    namespace wall=crash::cases::source_part_wall;
    namespace part=crash::cases::source_part_elastic;
    namespace source=crash::qualification::source_contact;
    namespace data=crash::case_data;
    try {
        if(argc!=7&&argc!=8) throw std::invalid_argument(
            "usage: robo_dyna_source_part_plastic_wall READINESS WALL REFINEMENT BASE_STEPS FRAME_EVERY_BASE NEW_DIR [SPEED_M_PER_S]");
        const auto refinement=unsigned(wall::ParseWallCount(argv[3],4));
        const auto base_steps=wall::ParseWallCount(argv[4],plastic::PlasticPilotMaximumBaseSteps);
        const auto every=wall::ParseWallCount(argv[5],plastic::PlasticPilotMaximumBaseSteps);
        if(every>base_steps||base_steps%every||base_steps/every+3>1000)
            throw std::invalid_argument("Frame stride must divide the declared horizon and fit the saved-frame cap");
        double speed=8;
        if(argc==8) {
            std::string value(argv[7]); std::size_t used=0;
            speed=std::stod(value,&used);
            if(used!=value.size()) throw std::invalid_argument("Invalid impact speed");
        }
        source::SourcePartContactFixture input;
        const auto loaded=source::LoadPinnedSourcePartContact(argv[1],&input);
        if(loaded.status!=source::FixtureStatus::Ok) throw std::runtime_error(loaded.diagnostic);
        plastic::SourcePartMaterial material;
        const auto loaded_material=plastic::LoadPinnedSourcePartMaterial(argv[1],&material);
        if(!loaded_material) throw std::runtime_error(loaded_material.message);
        const auto bytes=data::ReadPinnedWallManifest(argv[2]);
        std::istringstream stream(bytes); data::CanonicalWall canonical;
        const auto loaded_wall=canonical.Load(stream);
        if(loaded_wall.status!=data::WallStatus::Ok) throw std::runtime_error(loaded_wall.message);
        const auto config=plastic::PlasticWallConfig(material,refinement,speed);
        part::SourcePartElasticCase run;
        const auto initialized=run.Initialize(input,config,canonical,bytes,plastic::PlasticWallSettings(config));
        if(!initialized) throw std::runtime_error(wall::FormatWallFailure(initialized));
        return wall::ExecuteWallCase(run,base_steps*refinement,unsigned(every*refinement),argv[6],
            0x5350504c52554e31ULL+refinement,0x5350504c544f5031ULL);
    } catch(const std::exception& error) {
        std::cerr<<"Source plastic wall output incomplete: "<<error.what()<<'\n';return 1;
    }
}
