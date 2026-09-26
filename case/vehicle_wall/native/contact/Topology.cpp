#include "Internal.h"
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
namespace {
void Check(s::Report value,NumericalStage stage,const char* reason) {
    if(value.status==s::Status::Ok)return;
    Report report;
    report.status=value.status==s::Status::ResourceLimit?Status::ResourceLimit:Status::InvalidInput;
    report.reason=reason;report.numerical_stage=stage;report.startup=value;
    throw Failure{std::move(report)};
}
}
void BuildTopology(Fields& out,const WallSource&,Declaration declaration,const Plan& plan) {
    if(!out.topology_arena.Initialize(plan.topology.output_bytes) ||
        !out.ready_arena.Initialize(plan.topology.ready_output_bytes))
        Reject(Status::ResourceLimit,"Wall topology output allocation failed");
    tl::util::HostArena scratch;
    if(!scratch.Initialize(plan.forecast.topology_scratch))
        Reject(Status::ResourceLimit,"Wall topology scratch allocation failed");
    const auto input=out.Mesh(declaration,plan.shape.units);
    Check(s::BuildStarter(input,plan.topology_limits,out.topology_arena,scratch,&out.starter),
        NumericalStage::Starter,"Native finite-wall Starter topology rejected");
    if(out.starter.primary_count!=1 || out.starter.main_count!=2)
        Reject(Status::SourceMismatch,"One declared wall Q4 did not produce its true two SH2 sides");
    Check(s::BuildFixedMain(input,out.starter,{out.main_k.data(),out.main_k.size()},plan.topology_limits,
        out.ready_arena,scratch,&out.ready),NumericalStage::FixedReady,"Native fixed-ready wall cache rejected");
}
}
