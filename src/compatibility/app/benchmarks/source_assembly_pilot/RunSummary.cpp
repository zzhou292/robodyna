#include "ComparisonInput.h"
#include "output/AcceptedReplayCsv.h"
#include "output/source_assembly/SourceAssemblyWallSchema.h"

namespace crash::benchmarks::assembly_pilot {
Value RunSummary(Document& d,const Run& run) {
    auto& a=d.GetAllocator();Value out(rapidjson::kObjectType);const auto& b=run.bundle;
    out.AddMember("directory",Value(b.directory.string().c_str(),a),a);
    out.AddMember("manifest_sha256",Value(run.manifest_hash.c_str(),a),a);
    out.AddMember("source_inventory_sha256",Value(b.info.source_assembly->inventory_sha256.c_str(),a),a);
    out.AddMember("fixed_dt_s",b.fixed_dt,a);out.AddMember("reference_step_multiple",run.multiple,a);
    out.AddMember("requested_steps",b.assembly->requested_steps,a);out.AddMember("requested_horizon_s",rd::Real(run.configuration,"requested_horizon_s"),a);
    out.AddMember("accepted_epoch",b.info.final_epoch,a);out.AddMember("accepted_time_s",b.info.final_time,a);
    out.AddMember("saved_frames",b.entries.size(),a);out.AddMember("horizon_complete",b.info.horizon_complete,a);
    out.AddMember("stop_reason",Value(b.info.stop_reason.c_str(),a),a);
    std::uint64_t first_contact=0,first_yield=0;double peak_force=0,peak_penetration=0;
    std::array<double,2> contact_time{},yield_time{};
    for(const auto& file:b.assembly->interval_files) {
        std::istringstream input(rd::VerifiedBytes(b,file));std::string line;std::getline(input,line);
        Require(line+"\n"==output::assembly::WallIntervalHeader,"Pilot ledger header changed");
        while(std::getline(input,line)) {
            const auto row=rd::ReplayCsvRow<output::assembly::WallIntervalColumns>(line);
            const auto epoch=rd::ReplayCsvNumber<std::uint64_t>(row[4]);
            if(!first_contact&&rd::ReplayCsvNumber<double>(row[32])>0) {first_contact=epoch;
                contact_time={rd::ReplayCsvNumber<double>(row[3]),rd::ReplayCsvNumber<double>(row[5])};}
            if(!first_yield&&rd::ReplayCsvNumber<double>(row[33])>0) {first_yield=epoch;
                yield_time={rd::ReplayCsvNumber<double>(row[3]),rd::ReplayCsvNumber<double>(row[5])};}
            peak_force=std::max(peak_force,rd::ReplayCsvNumber<double>(row[20]));
            peak_penetration=std::max(peak_penetration,rd::ReplayCsvNumber<double>(row[31]));
        }
    }
    for(const auto& event:{std::pair<const char*,std::uint64_t>{"first_contact_record",first_contact},{"first_positive_PLA_record",first_yield}}) {
        Value v(rapidjson::kObjectType);v.AddMember("observed",event.second!=0,a);
        if(event.second){const auto& time=event.first==std::string("first_contact_record")?contact_time:yield_time;
            v.AddMember("epoch",event.second,a);v.AddMember("previous_endpoint_s",time[0],a);v.AddMember("endpoint_s",time[1],a);}
        out.AddMember(Value(event.first,a),v,a);
    }
    out.AddMember("all_interval_peak_wall_resultant_N",peak_force,a);out.AddMember("all_interval_maximum_penetration_m",peak_penetration,a);
    const auto final=run.Frame(b.entries.size()-1);const auto& diag=rd::Member(final,"diagnostics");
    for(const char* key:{"native_internal_work_J","cumulative_plastic_work_J","maximum_plastic_strain"})
        out.AddMember(Value(key,a),Value(rd::Real(diag,key)),a);
    for(const char* key:{"yielded_points","yielded_parents","active_contact_nodes"})out.AddMember(Value(key,a),Value(rd::Unsigned(diag,key)),a);
    return out;
}
} // namespace crash::benchmarks::assembly_pilot
