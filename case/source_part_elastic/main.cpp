#include "SourcePartElasticPilot.h"
#include "case/SourcePartElasticArtifacts.h"
#include "output/ArtifactIO.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <limits>

namespace {
std::uint64_t Count(const char* input,std::uint64_t cap) {
    std::string text(input);
    if(text.empty() || text.find_first_not_of("0123456789")!=std::string::npos)
        throw std::invalid_argument("Expected a positive decimal count");
    std::size_t used=0;
    const auto value=std::stoull(text,&used);
    if(used!=text.size() || !value || value>cap)
        throw std::invalid_argument("Count exceeds this bounded experiment");
    return value;
}
void Require(const crash::cases::source_part_elastic::Report& r) {
    if(!r) {
        std::cerr << std::setprecision(17) << r.message << " parent=" << r.source_parent_id
                  << " measured=" << r.measured << " limit=" << r.limit << '\n';
        throw std::runtime_error(r.message);
    }
}
}
int main(int argc,char** argv) {
    namespace part=crash::cases::source_part_elastic;
    namespace source=crash::qualification::source_contact;
    std::unique_ptr<part::SourcePartElasticArtifacts> artifacts;
    try {
        if(argc!=6) throw std::invalid_argument(
            "usage: robo_dyna_source_part_elastic READINESS REFINEMENT BASE_STEPS FRAME_EVERY_BASE NEW_DIR");
        const auto refinement=static_cast<unsigned>(Count(argv[2],4));
        const auto base_steps=Count(argv[3],part::PilotHorizonSteps);
        const auto every=Count(argv[4],part::PilotHorizonSteps);
        if(every>base_steps || base_steps%every || base_steps/every+2>1000)
            throw std::invalid_argument("Frame stride must divide horizon and fit bounded replay");
        source::SourcePartContactFixture input;
        const auto loaded=source::LoadPinnedSourcePartContact(argv[1],&input);
        if(loaded.status!=source::FixtureStatus::Ok) throw std::runtime_error(loaded.diagnostic);
        part::SourcePartElasticCase run;
        Require(run.Initialize(input,part::PilotConfig(refinement)));
        const auto steps=base_steps*refinement;
        artifacts=std::make_unique<part::SourcePartElasticArtifacts>(argv[5],run,steps,
            static_cast<unsigned>(every*refinement),0x53504552554e3031ULL+refinement,
            0x535045544f503031ULL);
        artifacts->WriteFrame(run);
        std::uint64_t last_saved=0;
        const auto started=std::chrono::steady_clock::now();
        for(std::uint64_t step=1;step<=steps;++step) {
            const auto base=run.owner().accepted();
            const auto result=run.Step();
            if(!result) {
                // Retain the exact last accepted endpoint while the case is
                // alive, including a failure between scheduled video frames.
                if(base.epoch!=last_saved) artifacts->WriteFrame(run);
                Require(result);
            }
            const auto accepted=run.owner().accepted();
            artifacts->RecordInterval(base,accepted,run.diagnostics());
            if(step==1 || step%(every*refinement)==0 || step==steps) {
                artifacts->WriteFrame(run);
                last_saved=step;
            }
            if(step%(1024*refinement)==0 || step==steps) {
                const auto& d=run.diagnostics();
                std::cout << std::setprecision(8) << "accepted " << step << '/' << steps
                    << " t=" << accepted.time << " s displacement=" << d.max_relative_displacement
                    << " m energy_residual=" << d.energy_residual << " J\n" << std::flush;
            }
        }
        const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        artifacts->Finish(run,elapsed);
        std::cout << "Completed original-source elastic experiment: " << argv[5] << '\n';
        return 0;
    } catch(const std::exception& e) {
        if(artifacts) { try { artifacts->Fail(e.what()); } catch(...) {} }
        std::cerr << "Source-part run incomplete: " << e.what() << '\n';
        return 1;
    }
}
