#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
std::vector<n::source_gaps::PhysicalShell> GapShells(tl::util::ConstView<n::source_gaps::PhysicalShell> original,
    std::size_t quads,const n::source_gaps::PhysicalShell& wall,std::size_t cap) {
    if(quads>original.size() || wall.layout!=n::ShellLayout::Quad4 || original.size()>524288 ||
        (original.size() && !original.data()) || original.size()+1>cap/sizeof(n::source_gaps::PhysicalShell))
        Reject(Status::ResourceLimit,"Invalid complete gap shell extent/cap");
    for(std::size_t i=0;i<original.size();++i)
        if(original[i].layout!=(i<quads?n::ShellLayout::Quad4:n::ShellLayout::Triangle3))
            Reject(Status::SourceMismatch,"Original gap shell family order differs");
    std::vector<n::source_gaps::PhysicalShell> rows;
    rows.reserve(original.size()+1);
    if(rows.capacity()*sizeof(n::source_gaps::PhysicalShell)>cap)
        Reject(Status::ResourceLimit,"Actual combined gap shell capacity exceeds forecast");
    if(quads)rows.insert(rows.end(),original.begin(),original.begin()+quads);
    rows.push_back(wall);
    if(quads<original.size())rows.insert(rows.end(),original.begin()+quads,original.end());
    return rows;
}
void BuildGaps(Fields& out,const VehicleSource& vehicle,const Component& component,const Controls& controls,const Plan& plan) {
    const auto& operands=vehicle.gap_operands();const auto original=operands.shells();
    const auto shells=GapShells(original,plan.shape.quads,component.shell,plan.forecast.gap_shell_copy);
    n::source_gaps::Input input;
    input.profile=controls.gaps;input.node_count=out.nodes.size();
    input.shells=shells.data();input.shell_count=shells.size();
    input.beams=operands.beams().data();input.beam_count=operands.beams().size();
    input.springs=operands.springs().data();input.spring_count=operands.springs().size();
    input.mains=out.starter.mains;input.main_count=out.starter.main_count;input.primary_count=1;
    input.secondary_nodes=out.nsv.data();input.secondary_count=out.nsv.size();
    input.main_nodes=out.msr.data();input.main_node_count=out.msr.size();
    n::source_gaps::Forecast actual;
    auto report=n::source_gaps::Preflight(input,plan.gap_limits,actual);
    if(report.status!=n::source_gaps::Status::Ok || actual.scratch_bytes>plan.forecast.gap_scratch)
        Reject(Status::ResourceLimit,"Complete wall-specific gap source forecast rejected");
    tl::util::HostArena scratch;
    if(!scratch.Initialize(actual.scratch_bytes))Reject(Status::ResourceLimit,"Wall gap source scratch allocation failed");
    report=n::source_gaps::Build(input,plan.gap_limits,scratch.data(),scratch.bytes(),
        {out.secondary_gap.data(),out.secondary_gap.size(),out.main_node_gaps.data(),out.main_node_gaps.size(),
         out.main_gaps.data(),out.main_gaps.size()});
    if(report.status!=n::source_gaps::Status::Ok || !report.completed) {
        Report failure;failure.status=Status::InvalidInput;failure.reason="Complete wall-specific native gap calculation rejected";
        failure.numerical_stage=NumericalStage::Gaps;failure.gaps=report;throw Failure{std::move(failure)};
    }
    out.gaps=report;
}
}
