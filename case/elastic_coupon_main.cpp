#include "ElasticCouponArtifacts.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc,char** argv) {
    namespace cd=crash::case_data;
    cd::ElasticCouponCase run; std::unique_ptr<cd::ElasticCouponArtifacts> output;
    try {
        if(argc<2||argc>3)throw std::runtime_error("Usage: robo-dyna-coupon NEW-output-directory [1|2|4 refinement]");
        cd::ElasticCouponConfig config;
        if(argc==3) {
            const std::string value=argv[2];
            if(value!="1"&&value!="2"&&value!="4")throw std::runtime_error("Refinement must be 1, 2 or 4");
            config.refinement=static_cast<unsigned>(value[0]-'0');
        }
        const auto begin=std::chrono::steady_clock::now(); auto report=run.Initialize(config);
        if(report.status!=cd::CouponStatus::Ok)throw std::runtime_error(report.diagnostic);
        // Approximately 80 intervals, matched base-step indices for refinement.
        const unsigned cadence=static_cast<unsigned>((run.modal()->step_count+79)/80)*config.refinement;
        output=std::make_unique<cd::ElasticCouponArtifacts>(argv[1],run,cadence); output->WriteFrame(run);
        const auto steps=run.metrics()->required_steps;
        for(std::uint64_t step=1;step<=steps;++step) {
            const auto base=run.metrics()->stamp; report=run.Step();
            if(report.status!=cd::CouponStatus::Ok)throw std::runtime_error(report.diagnostic);
            output->RecordInterval(base,*run.metrics());
            if(step%cadence==0||step==steps)output->WriteFrame(run);
            if(step%((steps+9)/10)==0)std::cout<<"Accepted "<<step<<'/'<<steps<<" steps, "<<run.metrics()->stamp.time<<" s\n"<<std::flush;
        }
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        output->Finish(run,elapsed);
        std::cout<<"Completed synthetic elastic coupon. Results: "<<argv[1]<<'\n'; return 0;
    } catch(const std::exception& error) {
        if(output)output->Fail(error.what()); std::cerr<<"Elastic coupon incomplete: "<<error.what()<<'\n'; return 1;
    }
}
