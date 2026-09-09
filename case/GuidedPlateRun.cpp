#include "GuidedPlateRun.h"
#include "GuidedPlateContactProtocol.h"
#include "GuidedPlateExperimentProtocol.h"
#include "GuidedPlateArtifacts.h"
#include "GuidedPlateStudyIO.h"
#include "WallStudyProvenance.h"
#include "CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>

namespace crash::case_data {
namespace io=output;
namespace fs=std::filesystem;
namespace {
void Check(const GuidedPlateReport& report) {
    if(report.status!=GuidedPlateStatus::Ok)throw std::runtime_error(report.diagnostic);
}
fs::path NewPath(const fs::path& path) {
    io::Require(!path.empty()&&!fs::exists(fs::symlink_status(path)),"Output path must be new");
    const auto parent=path.has_parent_path()?path.parent_path():fs::path(".");
    io::Require(fs::is_directory(parent),"Output parent directory must exist");
    return fs::weakly_canonical(path);
}
void CheckRuntimeWall(const GuidedPlateCase& run,const WallTessellation& staged) {
    const auto* actual=run.wall_tessellation();const auto* expected=staged.metadata();
    const auto mesh=run.wall_mesh(),reference=staged.view();
    io::Require(actual&&expected&&actual->kind==expected->kind&&actual->transform_version==expected->transform_version&&
        actual->source_manifest_sha256==expected->source_manifest_sha256&&actual->mesh_sha256==expected->mesh_sha256&&
        mesh.vertex_count==reference.vertex_count&&mesh.triangle_count==reference.triangle_count&&run.guided_data()&&
        run.guided_data()->wall_binding_id==WallTessellationBindingId(expected->kind)&&run.metrics()&&
        run.metrics()->contact.wall_binding_id==run.guided_data()->wall_binding_id,
        "Runtime wall differs from the staged authenticated provenance transform");
}
} // namespace
void CheckGuidedPlateRunOptions(const GuidedPlateRunOptions& options) {
    io::Require(!options.wall.empty()&&WallTessellationBindingId(options.wall_kind),"Invalid wall path or transform kind");
    io::Require(options.config.refinement==1||options.config.refinement==2||options.config.refinement==4,"Refinement must be 1, 2 or 4");
    io::Require(options.config.diagnostic_intervals>0&&options.config.diagnostic_intervals<=64,"Invalid guided audit cadence");
    io::Require(ValidGuidedContactBackend(options.config.integration_backend),"Invalid contact integration backend");
    io::Require(reference::FindGuidedPlateExperiment(options.config.experiment),"Invalid guided experiment");
    const bool derived=options.wall_kind!=WallTessellationKind::Original;
    io::Require(!derived||options.wall_provenance.has_value(),"Derived wall runs require an explicit new provenance sidecar");
    io::Require(!derived||!options.bundle.has_value(),"Derived walls cannot create a canonical guided replay bundle");
    const auto study=NewPath(options.study);
    std::optional<fs::path> provenance,bundle;
    if(options.wall_provenance) {provenance=NewPath(*options.wall_provenance);io::Require(*provenance!=study,"Study and provenance outputs alias");}
    if(options.bundle) {
        bundle=NewPath(*options.bundle);io::Require(*bundle!=study&&(!provenance||*bundle!=*provenance),"Bundle and evidence outputs alias");
    }
}
int RunGuidedPlate(const GuidedPlateRunOptions& options) {
    CheckGuidedPlateRunOptions(options);
    const auto bytes=ReadPinnedWallManifest(options.wall.string());std::istringstream input(bytes);CanonicalWall wall;
    const auto loaded=wall.Load(input);io::Require(loaded.status==WallStatus::Ok,loaded.message);
    // Pure host source object for the existing sidecar API. Case owns its own
    // immutable runtime transform; compare identities after initialization.
    // This is repeated startup geometry preparation, not a second FE state.
    std::unique_ptr<WallTessellation> provenance_wall;
    if(options.wall_provenance) {
        provenance_wall=std::make_unique<WallTessellation>();
        const auto report=provenance_wall->Initialize(wall,bytes,options.wall_kind);
        io::Require(report.status==WallTessellationStatus::Ok,report.diagnostic.c_str());
    }
    GuidedPlateCase run;GuidedPlateStudy observer;std::unique_ptr<GuidedPlateArtifacts> artifacts;
    const auto begin=std::chrono::steady_clock::now();
    try {
        // Single case-initialization boundary. Case owns backend admission;
        // this driver adds no contact algorithm or alternate state/clock.
        if(provenance_wall) {
            Check(run.InitializeTessellated(wall,bytes,options.wall_kind,options.config));CheckRuntimeWall(run,*provenance_wall);
        } else Check(run.Initialize(wall,options.config));
        io::Require(run.integration_backend()==options.config.integration_backend,"Initialized contact backend differs from request");
        io::Require(run.experiment()==options.config.experiment,"Initialized guided experiment differs from request");
        GuidedPlateFrame frame;Check(run.Capture(frame));GuidedStudyConfig study_config;std::string error;
        if(!PrepareGuidedStudyConfig(*run.metrics(),*run.model_data(),*run.guided_data(),run.contact_reference(),
                                    options.config.refinement,study_config,error)||
           !observer.Initialize(study_config,*run.metrics(),frame,error))throw std::runtime_error(error);
        constexpr unsigned frame_every=100;
        if(options.bundle) {
            artifacts=std::make_unique<GuidedPlateArtifacts>(options.bundle->string(),bytes,wall,run,frame_every);
            artifacts->WriteFrame(run);
        }
        const auto steps=run.metrics()->required_steps;
        for(std::uint64_t epoch=1;epoch<=steps;++epoch) {
            const auto base=run.metrics()->stamp;Check(run.Step());
            if(artifacts)artifacts->RecordInterval(base,*run.metrics());
            GuidedPlateFrame* sample=nullptr;
            if(observer.NeedsSample(epoch)) {Check(run.Capture(frame));sample=&frame;}
            if(!observer.Record(*run.metrics(),sample,error))throw std::runtime_error(error);
            if(artifacts&&(epoch%frame_every==0||epoch==steps))artifacts->WriteFrame(run);
            if(epoch%((steps+9)/10)==0||epoch==steps) {
                const auto& s=observer.data()->summary;
                std::cout<<"Accepted "<<epoch<<'/'<<steps<<", time="<<s.accepted_time<<" s, peak depth="<<s.maximum_penetration
                         <<" m, max certified energy error="<<s.maximum_certified_energy_relative_error<<'\n'<<std::flush;
            }
        }
        GuidedStudyData completed;if(!observer.Finish(completed,error))throw std::runtime_error(error);
        WriteGuidedPlateStudy(options.study,completed);
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        if(artifacts)artifacts->Finish(run,elapsed);
        if(provenance_wall) {
            CheckRuntimeWall(run,*provenance_wall);
            const auto exact=io::ReadBounded(options.study,kGuidedStudyByteCap);const auto saved=ParseGuidedPlateStudy(exact);
            io::Require(saved.config.owner_id==completed.config.owner_id&&saved.config.wall_binding_id==completed.config.wall_binding_id&&
                saved.config.experiment_sha256==completed.config.experiment_sha256&&saved.summary.accepted_epoch==completed.summary.accepted_epoch&&
                saved.config.integration_backend==completed.config.integration_backend&&
                saved.config.experiment==completed.config.experiment&&saved.config.qualification_id==completed.config.qualification_id&&
                io::Bits(saved.summary.accepted_time)==io::Bits(completed.summary.accepted_time),"Persisted Study identity differs from completed run");
            // Last output: no valid sidecar can precede completed numerical
            // evidence (and optional canonical bundle). Partial writes/failure
            // remain rejected by bounded full-field sidecar parsing.
            WriteWallStudyProvenance(*options.wall_provenance,wall,bytes,*provenance_wall,exact);
        }
        std::cout<<"Guided study completed in "<<elapsed<<" s; report "<<options.study
                 <<"; wall="<<WallTessellationName(options.wall_kind)
                 <<"; contact="<<GuidedContactBackendName(run.integration_backend())
                 <<"; experiment="<<GuidedExperimentName(run.experiment())
                 <<"; separated rebound observed="<<completed.summary.separated_rebounding<<'\n';return 0;
    } catch(const std::exception& e) {if(artifacts)artifacts->Fail(e.what());throw;}
}
} // namespace crash::case_data
