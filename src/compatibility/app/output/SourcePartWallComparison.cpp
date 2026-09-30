#include "SourcePartWallComparison.h"
#include "SourcePartWallComparisonInput.h"

namespace crash::output {
namespace {
namespace wc=wall_comparison;
Value Bracket(Document& report,const wc::EventBracket& e) {
    Value out(rapidjson::kObjectType);auto& a=report.GetAllocator();
    out.AddMember("observed",e.observed,a);out.AddMember("lower_s",e.lower,a);out.AddMember("upper_s",e.upper,a);
    out.AddMember("certificate_uncertainty_s",e.uncertainty,a);return out;
}
Value RunSummary(Document& report,const wc::Run& run) {
    Value out(rapidjson::kObjectType);auto& a=report.GetAllocator();const auto& e=run.events;
    out.AddMember("refinement",run.refinement,a);
    out.AddMember("manifest_sha256",Value(run.manifest_sha256.c_str(),a),a);
    out.AddMember("horizon_complete",true,a);out.AddMember("completed_rebound",e.rebound.observed,a);
    out.AddMember("contact_intervals",e.contact_intervals,a);
    out.AddMember("terminal_separation_start_epoch",e.terminal_separation_start,a);
    out.AddMember("first_contact",Bracket(report,e.onset),a);out.AddMember("terminal_rebound",Bracket(report,e.rebound),a);
    out.AddMember("peak_reaction_N",e.peak_reaction,a);out.AddMember("peak_reaction_certificate_error_N",e.peak_reaction_error,a);
    out.AddMember("maximum_penetration_m",e.peak_penetration,a);
    out.AddMember("maximum_physical_energy_uncertainty_J",e.maximum_energy_uncertainty,a);
    out.AddMember("maximum_chord_change_m",run.maximum_chord,a);
    out.AddMember("maximum_absolute_strain",run.maximum_strain,a);
    out.AddMember("maximum_thickness_curvature",run.maximum_thickness_curvature,a);return out;
}
}
SourcePartWallComparison CompareSourcePartWall(const std::array<std::filesystem::path,3>& directories) {
    std::array<wc::Run,3> runs;const unsigned refinements[]{1,2,4};
    for(unsigned r=0;r<3;++r) {
        runs[r].Open(directories[r],refinements[r]);
        Require(runs[r].physical_configuration==runs[0].physical_configuration&&runs[r].placement==runs[0].placement&&
            runs[r].placed_mesh==runs[0].placed_mesh&&runs[r].original_wall_manifest==runs[0].original_wall_manifest,
            "Refinements changed source/native M/J, physical setup or actual placed wall");
    }
    wc::Differences coarse{},fine{};
    for(std::uint64_t epoch=0;epoch<=wc::BaseHorizon;epoch+=wc::CommonStride) {
        std::array<Document,3> fields;
        for(unsigned r=0;r<3;++r)fields[r]=runs[r].CommonFrame(epoch);
        const auto a=wc::Difference(fields[0],wc::Contact(fields[0]),fields[1],wc::Contact(fields[1]),runs[0].scales);
        const auto b=wc::Difference(fields[1],wc::Contact(fields[1]),fields[2],wc::Contact(fields[2]),runs[0].scales);
        for(unsigned c=0;c<wc::Channels;++c) {coarse[c]=std::max(coarse[c],a[c]);fine[c]=std::max(fine[c],b[c]);}
    }
    SourcePartWallComparison result;auto& d=result.report;d.SetObject();auto& allocator=d.GetAllocator();
    bool refinement_passed=true,deformation_passed=true,events_passed=true,completed_rebound=true;
    Value channels(rapidjson::kArrayType),summaries(rapidjson::kArrayType);
    for(unsigned c=0;c<wc::Channels;++c) {
        const bool valid=wc::Converged(coarse[c],fine[c]);refinement_passed&=valid;
        Value item(rapidjson::kObjectType);item.AddMember("field",Value(wc::ChannelNames[c],allocator),allocator);
        item.AddMember("coarse_medium",coarse[c],allocator);item.AddMember("medium_fine",fine[c],allocator);
        item.AddMember("passed",valid,allocator);channels.PushBack(item,allocator);
    }
    for(const auto& run:runs) {
        deformation_passed&=run.maximum_chord>=1e-5&&std::isfinite(run.maximum_strain)&&
            std::isfinite(run.maximum_thickness_curvature)&&run.maximum_strain<=.005&&run.maximum_thickness_curvature<=.01;
        events_passed&=run.events.onset.observed;completed_rebound&=run.events.rebound.observed;
        summaries.PushBack(RunSummary(d,run),allocator);
    }
    for(unsigned r=0;r<2;++r) {
        events_passed&=wc::EventsAgree(runs[r].events.onset,runs[r+1].events.onset,wc::BaseStep/refinements[r]);
        events_passed&=wc::EventsAgree(runs[r].events.rebound,runs[r+1].events.rebound,wc::BaseStep/refinements[r]);
    }
    const bool qualified=refinement_passed&&deformation_passed&&events_passed;
    String(d,"schema","robo_dyna.source_part_wall_comparison.v1");
    String(d,"status",qualified?(completed_rebound?"qualified_completed_rebound":"qualified_partial_impact"):"failed");
    String(d,"scope","Experimental elastic original source part against its placed mesh wall; not a full vehicle or original material response");
    String(d,"physical_configuration_sha256",Sha256(runs[0].physical_configuration));
    String(d,"momentum_timing","Native mass weighted raw carried midpoint velocity; every interval reconstructed from wall kick impulse and residual with conservative uncertainty");
    String(d,"rebound_policy","Terminal strictly-separated run spanning at least 128H, zero certified nodal force/potential, negative carried COM upper bound; later recontact resets it");
    Boolean(d,"refinement_passed",refinement_passed);Boolean(d,"deformation_passed",deformation_passed);
    Boolean(d,"events_passed",events_passed);Boolean(d,"completed_rebound",completed_rebound&&qualified);
    Integer(d,"common_samples",wc::CommonSamples);Number(d,"physical_horizon_s",wc::BaseStep*wc::BaseHorizon);
    Number(d,"required_terminal_separation_duration_s",wc::CommonStride*wc::BaseStep);
    Number(d,"total_energy_scale_J",.1);Number(d,"contact_potential_scale_J",runs[0].scales.initial_kinetic);
    Number(d,"wall_impulse_scale_N_s",runs[0].scales.impulse);Number(d,"wall_reaction_scale_N",runs[0].scales.reaction);
    Number(d,"native_total_mass_kg",runs[0].scales.mass);d.AddMember("channels",channels,allocator);d.AddMember("runs",summaries,allocator);
    result.exit_code=qualified?(completed_rebound?0:2):1;return result;
}
} // namespace crash::output
