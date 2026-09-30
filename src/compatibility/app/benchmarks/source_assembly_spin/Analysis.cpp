#include "AnalysisInternal.h"
#include <cmath>
#include <limits>
namespace crash::benchmarks::assembly_spin {
Document Analyze(const std::filesystem::path& inventory,const std::filesystem::path& trace) {
    Context context(inventory);const auto bytes=output::ReadBounded(trace,TraceCap);Require(!bytes.empty()&&bytes.back()=='\n',"Spin trace has a truncated terminal row");
    Document result;result.SetObject();output::String(result,"schema","robo-dyna.source-qeph-spin-analysis.v1");
    output::String(result,"trace_sha256",output::Sha256(bytes));output::Integer(result,"trace_bytes",bytes.size());
    output::String(result,"source_inventory_sha256",context.inventory.data().identity.sha256);
    output::String(result,"scope","Independent native one-step recurrence from source geometry/material and paired recorded history/motion. "
        "Diagnostic metrics only; no guard authorization, convergence verdict or claim that normal spin is harmless. "
        "Own-parent-normal removal is a frozen-state native projection sensitivity, not a modified simulation or a common-node null-mode proof. "
        "Carried powers and stored native J energy are not collocated work or aggregate rigid-group energy. "
        "Work increments below concern individual sampled intervals and are not summed across cadence gaps.");
    Metrics metrics;Value observations(rapidjson::kArrayType);auto& allocator=result.GetAllocator();
    std::size_t offset=0,row_count=0;std::uint64_t saved_epoch=0,attempt=0;double saved_time=0,force_time=0;
    bool header=false,complete=false;std::array<double,4> native_metric{};
    while(offset<bytes.size()) {
        const auto newline=bytes.find('\n',offset);Require(newline!=std::string::npos,"Spin trace row is unterminated");
        auto row=Parse(bytes.data()+offset,newline-offset);const auto kind=json::Text(row,"record");
        if(!header) {context.Header(row);Require(bytes.size()<=json::Unsigned(row,"forecast_bytes"),"Spin bytes exceed their admitted forecast");header=true;output::Integer(result,"source_node_id",context.source_node);
            output::Integer(result,"source_instance_id",context.source_instance);output::Number(result,"fixed_dt_s",context.dt);}
        else if(kind=="accepted_force_stage") {
            Require(!complete&&row_count<MaxRows,"Spin observation count exceeds bounded complete trace");
            const auto& base=json::Member(row,"base_stamp");const auto& next=json::Member(row,"enclosing_accepted_stamp");
            const auto epoch=json::Unsigned(next,"epoch",context.requested);Require(epoch&&epoch>saved_epoch,"Spin trace epochs are duplicated or unordered");
            if(!row_count)Require(epoch==1,"Spin trace does not start at its first completed interval");
            // A non-cadence last row is allowed only immediately before footer;
            // the next-iteration check below closes that exception.
            if(row_count&&saved_epoch!=1&&saved_epoch%context.cadence)Require(false,"Spin unscheduled row was not terminal");
            const auto expected_next=((saved_epoch/context.cadence)+1)*context.cadence;
            if(row_count)Require(epoch<=expected_next,"Spin trace skipped a scheduled saved frame");
            context.Stamp(base,epoch-1);context.Stamp(next,epoch);
            const double bt=json::Real(base,"time"),nt=json::Real(next,"time");
            Require(nt==bt+context.dt&&json::Real(next,"reaction_time")==bt,"Spin paired force/enclosing phases differ");
            const auto a=json::Unsigned(row,"attempt");Require(a>attempt,"Spin attempt did not advance");attempt=a;
            Require(json::Unsigned(row,"source_instance_id")==context.source_instance&&json::Unsigned(row,"source_node_id")==context.source_node&&
                json::Unsigned(row,"global_node")==context.global_node&&json::Unsigned(row,"complete_incident_qeph_parent_count")==context.parents.size()&&
                json::Unsigned(row,"incident_t3_parent_count")==0,"Spin row source/count scope differs");
            json::TextIs(row,"phase","retained_accepted_force_at_base_time_with_carried_midpoint_motion");
            CheckSharedMotion(context,row);
            const auto& parents=json::Array(row,"parents",context.parents.size(),context.parents.size());
            const auto& candidates=json::Array(row,"enclosing_candidate_parents",context.parents.size(),context.parents.size());
            std::array<double,4> now{};Array(row,"native",now);
            Require(now[0]>0&&now[1]>0&&now[2]>0&&now[3]>=0,"Spin native metric is invalid");
            if(!row_count)native_metric=now;else for(unsigned i=0;i<4;++i)json::Same(now[i],native_metric[i]);
            std::array<double,4> q{};Array(row,"orientation_endpoint_wxyz",q);
            long double norm=0;for(double x:q)norm+=static_cast<long double>(x)*x;Require(std::abs(norm-1)<1e-10L,"Spin endpoint quaternion is not unit");
            std::array<double,3> assembled{};Array(row,"actual_assembled_couple_N_m",assembled);
            std::array<double,3> sum{};const auto& omega=Numbers(row,"omega_previous_midpoint_rad_s",3);
            for(std::size_t i=0;i<context.parents.size();++i) {
                const auto& selected=context.parents[i];const auto& old=parents[static_cast<unsigned>(i)];const auto& candidate=candidates[static_cast<unsigned>(i)];
                CheckParent(context,selected,old,epoch-1,bt,json::Real(base,"reaction_time"));CheckParent(context,selected,candidate,epoch,nt,bt);
                const auto previous=History(selected.reference,old);const auto input=Interval(candidate,context.dt);layered::QephTrial expected;
                const auto status=layered::Evaluate(selected.reference,previous,input,selected.law,expected);
                Require(status==native::Status::kSuccess,"Actual source-pair native recurrence failed");Compare(metrics,selected,candidate,expected);
                auto& work=metrics["replayed_cumulative_plastic_work"];work.units="J";
                work.Add(json::Real(old,"cumulative_plastic_work_J")+expected.section.plastic_work_increment_j,
                    json::Real(candidate,"cumulative_plastic_work_J"),row_count,selected.parent->source_id);
                auto diagnostic=LocalDiagnostic(context,selected,row,old,candidate,expected);
                output::Number(diagnostic,"base_total_orientation_rad",assembly_pilot::RotationDistance({1,0,0,0},q));
                output::Number(diagnostic,"base_time_s",bt);output::Number(diagnostic,"base_carried_velocity_time_s",json::Real(base,"velocity_time"));
                Value value;value.CopyFrom(diagnostic,allocator);observations.PushBack(value,allocator);
                const auto& force=Numbers(old,"positive_internal_couple_xyz_N_m",12);const auto& w=Numbers(old,"omega_previous_midpoint_xyz_rad_s",12);
                for(unsigned j=0;j<3;++j){sum[j]+=-json::Real(force[3*selected.local+j]);json::Same(json::Real(w[3*selected.local+j]),json::Real(omega[j]));}
            }
            const auto& recorded=Numbers(row,"negative_parent_couple_sum_N_m",3);const auto& residual=Numbers(row,"assembly_couple_residual_N_m",3);
            for(unsigned j=0;j<3;++j){json::Same(sum[j],json::Real(recorded[j]));json::Same(assembled[j]-sum[j],json::Real(residual[j]));}
            ++row_count;saved_epoch=epoch;saved_time=nt;force_time=bt;
        } else {
            json::TextIs(row,"record","completion");Require(!complete&&newline+1==bytes.size(),"Spin completion is not the final trace row");
            complete=true;const auto epoch=json::Unsigned(row,"accepted_epoch",context.requested);
            Require(epoch==saved_epoch&&json::Unsigned(row,"saved_force_stages")==row_count&&
                json::Unsigned(row,"trace_bytes_before_footer")==offset,"Spin completion does not describe its exact saved prefix");
            json::Same(json::Real(row,"accepted_time_s"),saved_time);json::Same(json::Real(row,"last_observed_force_time_s"),force_time);
            json::Flag(row,"has_force_stage",row_count!=0);json::Flag(row,"horizon_complete",epoch==context.requested);
            const auto reason=json::Text(row,"stop_reason");Require(epoch==context.requested?reason.empty():!reason.empty(),"Spin prefix reason is invalid");
            output::Integer(result,"accepted_epoch",epoch);output::Number(result,"accepted_time_s",saved_time);
            output::Boolean(result,"horizon_complete",epoch==context.requested);output::String(result,"stop_reason",reason);
        }
        offset=newline+1;
    }
    Require(header&&complete,"Spin trace is incomplete; no diagnostic result published");
    output::Integer(result,"sampled_force_stages",row_count);output::Integer(result,"incident_parents",context.parents.size());
    if(row_count){output::FiniteArray(result,"native_mass_totalJ_physicalJ_addedJ",native_metric.data(),4);
        output::Number(result,"added_inertia_fraction",native_metric[3]/native_metric[1]);}
    result.AddMember("native_recurrence_differences",assembly_pilot::MetricDocument(result,metrics),allocator);
    result.AddMember("local_observations",observations,allocator);return result;
}
}
