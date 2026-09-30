#include "SourcePartElasticInternal.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_part_elastic {
Report SourcePartElasticCase::Impl::InitializePlasticObservation() {
    q::BatchDiagnostics qd; t::BatchDiagnostics td;
    const auto qr=qeph.CopyAcceptedSectionHistory(accepted.stamp,
        accepted_plastic.qeph.data(),accepted_plastic.qeph.size(),&qd);
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message);
    const auto tr=t3.CopyAcceptedSectionHistory(accepted.stamp,
        accepted_plastic.t3.data(),accepted_plastic.t3.size(),&td);
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message);
    const auto zero=[](const auto& states) {
        for(const auto& state:states) for(const auto& point:state.history.point) {
            if(point.plastic_strain!=0||point.filtered_rate_per_s!=0) return false;
            for(double stress:point.stress) if(stress!=0) return false;
        }
        return true;
    };
    if(!zero(accepted_plastic.qeph)||!zero(accepted_plastic.t3))
        return Failure(Status::ComponentFailure,"Native plastic startup contains nonzero material history");
    for(unsigned e=0;e<source::Q4Count;++e)
        accepted_plastic.qeph_reported_thickness[e]=binding.qeph_reference(e).input.thickness;
    for(unsigned e=0;e<source::T3Count;++e)
        accepted_plastic.t3_reported_thickness[e]=binding.t3_reference(e).input.thickness;
    return Success();
}
Report SourcePartElasticCase::Impl::ObservePlasticSections() {
    auto qr=qeph.CopyPreparedSectionHistory(trial.diagnostics.shells.qeph,
        trial_plastic.qeph.data(),trial_plastic.qeph.size());
    if(qr.status!=q::BatchStatus::Success) return Failure(Status::ComponentFailure,qr.message);
    auto tr=t3.CopyPreparedSectionHistory(trial.diagnostics.shells.t3,
        trial_plastic.t3.data(),trial_plastic.t3.size());
    if(tr.status!=t::BatchStatus::Success) return Failure(Status::ComponentFailure,tr.message);
    for(unsigned e=0;e<source::Q4Count;++e)
        trial_plastic.qeph_reported_thickness[e]=qresult[e].proposed_history.data().thickness;
    for(unsigned e=0;e<source::T3Count;++e)
        trial_plastic.t3_reported_thickness[e]=tresult[e].proposed_history.data().thickness;
    auto& summary=trial.plastic;
    summary={}; summary.enabled=true;
    long double weighted_plastic=0,volume=0,work=0;
    const double maximum=config.material.plastic_strain().back();
    const auto inspect=[&](const auto& states,const auto& previous,const auto& reference,
                           const auto& parent) -> Report {
        for(unsigned e=0;e<states.size();++e) {
            const auto& state=states[e]; const auto& old=previous[e];
            const auto& r=reference(e);
            const double dv=r.area*r.input.thickness;
            if(!std::isfinite(state.cumulative_plastic_work_J)||
               state.cumulative_plastic_work_J<old.cumulative_plastic_work_J||
               !std::isfinite(dv)||dv<=0)
                return Failure(Status::EnvelopeFailure,"Plastic work or reference volume is invalid",0,0,parent(e));
            bool yielded=false;
            for(unsigned p=0;p<3;++p) {
                const auto& point=state.history.point[p];
                const double value=point.plastic_strain;
                if(!std::isfinite(value)||value<old.history.point[p].plastic_strain||value>maximum)
                    return Failure(Status::EnvelopeFailure,"Plastic strain decreased or exceeded source curve",value,maximum,parent(e));
                for(double stress:point.stress) if(!std::isfinite(stress))
                    return Failure(Status::EnvelopeFailure,"Nonfinite accepted-layer stress",0,0,parent(e));
                if(!std::isfinite(point.filtered_rate_per_s)||point.filtered_rate_per_s<0)
                    return Failure(Status::EnvelopeFailure,"Nonfinite or negative accepted strain-rate filter history",0,0,parent(e));
                if(value>0) {++summary.yielded_points; yielded=true;}
                summary.maximum_plastic_strain=std::max(summary.maximum_plastic_strain,value);
                weighted_plastic+=dv*fe::sections::LayerForceWeight(p)*value;
            }
            summary.yielded_parents+=yielded;
            volume+=dv; work+=state.cumulative_plastic_work_J;
        }
        return Success();
    };
    auto report=inspect(trial_plastic.qeph,accepted_plastic.qeph,
        [&](unsigned e)->const auto& {return binding.qeph_reference(e);},
        [&](unsigned e){return binding.qeph_source_id(e);});
    if(!report) return report;
    report=inspect(trial_plastic.t3,accepted_plastic.t3,
        [&](unsigned e)->const auto& {return binding.t3_reference(e);},
        [&](unsigned e){return binding.t3_source_id(e);});
    if(!report) return report;
    summary.mean_plastic_strain=static_cast<double>(weighted_plastic/volume);
    summary.cumulative_plastic_work_J=static_cast<double>(work);
    if(!std::isfinite(summary.mean_plastic_strain)||!std::isfinite(summary.cumulative_plastic_work_J))
        return Failure(Status::EnvelopeFailure,"Nonfinite complete plastic section summary");
    return Success();
}

Report SourcePartElasticCase::CapturePlasticSectionHistory(source_part_plastic::SourcePartPlasticState* out) const {
    if(!out) return Failure(Status::InvalidInput,"Missing plastic section output");
    if(!impl_->initialized) return Failure(Status::NotInitialized,"Source-part case is not initialized");
    if(impl_->config.material_model==MaterialModel::ElasticLaw1)
        return Failure(Status::InvalidInput,"Elastic material has no plastic section sidecar");
    const auto stamp=impl_->owner.accepted();
    if(stamp.owner_id!=impl_->accepted.stamp.owner_id||stamp.epoch!=impl_->accepted.stamp.epoch||
       stamp.time!=impl_->accepted.stamp.time)
        return Failure(Status::ComponentFailure,"Owner advanced outside its source-part case");
    *out=impl_->accepted_plastic;
    return Success();
}
}
