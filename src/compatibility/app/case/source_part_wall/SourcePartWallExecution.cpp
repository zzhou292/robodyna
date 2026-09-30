#include "SourcePartWallExecution.h"
#include "SourcePartWallContact.h"
#include "case/SourcePartWallArtifacts.h"
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace crash::cases::source_part_wall {
std::uint64_t ParseWallCount(const char* text,std::uint64_t maximum) {
    const std::string value(text);
    std::uint64_t count=0;
    const auto result=std::from_chars(value.data(),value.data()+value.size(),count);
    if(result.ec!=std::errc{}||result.ptr!=value.data()+value.size()||!count||count>maximum)
        throw std::invalid_argument("Count exceeds the declared source-wall experiment");
    return count;
}
std::string FormatWallFailure(const source_part_elastic::Report& r) {
    std::ostringstream out;
    out<<std::setprecision(17)<<r.message<<" parent="<<r.source_parent_id
       <<" measured="<<r.measured<<" limit="<<r.limit;
    return out.str();
}
int ExecuteWallCase(source_part_elastic::SourcePartElasticCase& run,std::uint64_t steps,
    unsigned every,const std::string& directory,std::uint64_t run_id,std::uint64_t topology_id) {
    SourcePartWallArtifacts artifacts(directory,run,steps,every,run_id,topology_id);
    try {
        artifacts.WriteFrame(run);
        std::uint64_t last_saved=0;
        const auto started=std::chrono::steady_clock::now();
        const auto elapsed=[&] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        };
        for(std::uint64_t step=1;step<=steps;++step) {
            const auto base=run.owner().accepted();
            const auto result=run.Step();
            if(!result) {
                const auto reason=FormatWallFailure(result);
                if(!base.epoch) throw std::runtime_error(reason);
                if(base.epoch!=last_saved) artifacts.WriteFrame(run);
                artifacts.FinishPrefix(run,elapsed(),reason);
                std::cerr<<"Accepted wall prefix archived through epoch "<<base.epoch<<": "<<reason<<'\n';
                return 2;
            }
            artifacts.RecordInterval(base,run);
            if(step==1||step%every==0||step==steps) {artifacts.WriteFrame(run);last_saved=step;}
            if(step%2048==0||step==steps) {
                const auto& c=run.accepted_contact()->diagnostics;
                std::cout<<std::setprecision(8)<<"accepted "<<step<<'/'<<steps<<" t="<<run.owner().accepted().time
                    <<" s wall_force="<<c.resultant.value<<" N penetration="<<c.maximum_penetration
                    <<" m energy_residual="<<run.diagnostics().energy_residual<<" J";
                if(run.config().material_model!=source_part_elastic::MaterialModel::ElasticLaw1) {
                    source_part_elastic::Snapshot snapshot;
                    const auto captured=run.Capture(&snapshot);
                    if(!captured) throw std::runtime_error(FormatWallFailure(captured));
                    std::cout<<" max_plastic_strain="<<snapshot.plastic.maximum_plastic_strain
                        <<" yielded_points="<<snapshot.plastic.yielded_points
                        <<" chord_change="<<snapshot.diagnostics.maximum_chord_change<<" m";
                }
                std::cout<<'\n'<<std::flush;
            }
        }
        artifacts.Finish(run,elapsed());
        std::cout<<"Completed declared source mesh-wall horizon: "<<directory<<'\n';
        return 0;
    } catch(const std::exception& error) {artifacts.Fail(error.what());throw;}
}
}
