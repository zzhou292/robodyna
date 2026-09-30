#include "GuidedPlateCli.h"
#include "GuidedPlateStudyIO.h"
#include "GuidedPlateExperimentProtocol.h"
#include "output/ArtifactIO.h"
#include <iostream>
#include <stdexcept>

namespace crash::case_data {
namespace io=output;
namespace {
constexpr const char* usage="Usage: robo-dyna-guided run WALL NEW-STUDY 1|2|4 [NEW-BUNDLE] [--wall=original|flip|subdivide] [--wall-provenance=NEW-SIDECAR] [--contact-integration=scalar|rectangular] [--experiment=original|penalty-margin-v1] | compare COARSE FINE NEW-REPORT | compare-wall WALL DERIVED-STUDY DERIVED-SIDECAR CANONICAL-STUDY CANONICAL-SIDECAR NEW-REPORT";
}
GuidedPlateCommand ParseGuidedPlateCommand(int argc,const char* const* argv) {
    io::Require(argv&&argc>=2&&argc<=10,usage);
    for(int i=0;i<argc;++i)io::Require(argv[i]&&*argv[i],"Command arguments must be nonempty");
    GuidedPlateCommand out;const std::string command=argv[1];
    if(command=="compare") {
        io::Require(argc==5,usage);out.kind=GuidedPlateCommandKind::CompareRefinement;
        out.coarse_study=argv[2];out.fine_study=argv[3];out.refinement_report=argv[4];return out;
    }
    if(command=="compare-wall") {
        io::Require(argc==8,usage);out.kind=GuidedPlateCommandKind::CompareWall;
        out.wall_comparison={argv[2],argv[3],argv[4],argv[5],argv[6],argv[7]};return out;
    }
    io::Require(command=="run"&&argc>=5,usage);out.run.wall=argv[2];out.run.study=argv[3];
    const std::string refinement=argv[4];io::Require(refinement=="1"||refinement=="2"||refinement=="4","Refinement must be 1, 2 or 4");
    out.run.config.refinement=static_cast<unsigned>(refinement[0]-'0');
    bool wall_seen=false,backend_seen=false,experiment_seen=false;
    for(int i=5;i<argc;++i) {
        const std::string arg=argv[i];
        if(arg.rfind("--wall=",0)==0) {
            io::Require(!wall_seen,"Duplicate wall transform option");wall_seen=true;const auto kind=arg.substr(7);
            if(kind=="original")out.run.wall_kind=WallTessellationKind::Original;
            else if(kind=="flip")out.run.wall_kind=WallTessellationKind::FlipConvexPairs;
            else if(kind=="subdivide")out.run.wall_kind=WallTessellationKind::UniformFour;
            else throw std::runtime_error("Unknown wall transform option");
        } else if(arg.rfind("--wall-provenance=",0)==0) {
            io::Require(!out.run.wall_provenance,"Duplicate wall provenance option");
            const auto path=arg.substr(18);io::Require(!path.empty(),"Wall provenance path must be nonempty");out.run.wall_provenance=path;
        } else if(arg.rfind("--contact-integration=",0)==0) {
            io::Require(!backend_seen,"Duplicate contact integration option");backend_seen=true;const auto name=arg.substr(22);
            if(name=="scalar")out.run.config.integration_backend=tlfea::contact::Q4PlanarIntegrationBackend::ScalarDyadicSquares;
            else if(name=="rectangular")out.run.config.integration_backend=tlfea::contact::Q4PlanarIntegrationBackend::RectangularDyadic;
            else throw std::runtime_error("Unknown contact integration option");
        } else if(arg.rfind("--experiment=",0)==0) {
            io::Require(!experiment_seen,"Duplicate guided experiment option");experiment_seen=true;
            io::Require(ParseGuidedExperiment(arg.substr(13),out.run.config.experiment),"Unknown guided experiment option");
        } else {
            io::Require(arg.rfind("--",0)!=0,"Unknown guided run option");
            io::Require(!out.run.bundle,"Only one optional bundle path is supported");out.run.bundle=arg;
        }
    }
    io::Require(out.run.wall_kind==WallTessellationKind::Original||out.run.wall_provenance.has_value(),"Derived wall runs require an explicit new provenance sidecar");
    io::Require(out.run.wall_kind==WallTessellationKind::Original||!out.run.bundle,"Derived walls cannot create a canonical guided replay bundle");
    return out;
}
int ExecuteGuidedPlateCommand(const GuidedPlateCommand& command) {
    if(command.kind==GuidedPlateCommandKind::Run)return RunGuidedPlate(command.run);
    if(command.kind==GuidedPlateCommandKind::CompareWall) {
        const auto result=CompareAndWriteGuidedPlateWallStudies(command.wall_comparison);
        std::cout<<result.diagnostic<<'\n';return result.passed?0:2;
    }
    io::Require(command.kind==GuidedPlateCommandKind::CompareRefinement,"Unknown guided command kind");
    namespace fs=std::filesystem;
    io::Require(!command.refinement_report.empty()&&!fs::exists(fs::symlink_status(command.refinement_report)),"Comparison output must be new");
    const auto parent=command.refinement_report.has_parent_path()?command.refinement_report.parent_path():fs::path(".");
    io::Require(fs::is_directory(parent),"Comparison output parent must exist");
    const auto coarse_bytes=io::ReadBounded(command.coarse_study,kGuidedStudyByteCap);
    const auto fine_bytes=io::ReadBounded(command.fine_study,kGuidedStudyByteCap);
    const auto coarse=ParseGuidedPlateStudy(coarse_bytes),fine=ParseGuidedPlateStudy(fine_bytes);
    GuidedStudyComparison result;std::string error;
    if(!CompareGuidedPlateStudies(coarse,fine,result,error))throw std::runtime_error(error);
    WriteGuidedPlateComparison(command.refinement_report,result,io::Sha256(coarse_bytes),io::Sha256(fine_bytes));
    std::cout<<result.diagnostic<<'\n';return result.passed?0:2;
}
} // namespace crash::case_data
