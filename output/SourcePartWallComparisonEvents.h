#pragma once
#include "SourcePartWallComparisonMath.h"
#include <cstdint>
#include <limits>

namespace crash::output::wall_comparison {
struct IntervalPoint {
    std::uint64_t epoch=0;
    double time=0,reaction=0,reaction_error=0,potential=0,potential_error=0;
    double impulse=0,impulse_error=0,momentum_residual=0,momentum_allowance=0;
    double penetration=0,physical_energy_uncertainty=0;
    unsigned active_nodes=0,strictly_separated_nodes=0;
};
inline long double MomentumArithmetic(long double initial,double impulse,double residual) {
    return 64*std::numeric_limits<double>::epsilon()*(std::abs(initial)+std::abs(impulse)+std::abs(residual))+1e-18L;
}
inline long double MomentumUpper(const IntervalPoint& p,long double initial) {
    return initial-p.impulse+p.momentum_residual+p.impulse_error+p.momentum_allowance+
        MomentumArithmetic(initial,p.impulse,p.momentum_residual);
}
struct Events {
    EventBracket onset,rebound;
    std::uint64_t final_epoch=0,contact_intervals=0,terminal_separation_start=0;
    double peak_reaction=0,peak_reaction_error=0,peak_penetration=0,maximum_energy_uncertainty=0;
};
// Consumes every accepted interval. A later touch, force or nonnegative COM
// breaks the terminal separated run, even if an earlier rebound persisted.
class EventTracker {
 public:
    EventTracker(unsigned refinement,long double initial_momentum,double onset_uncertainty)
        :refinement_(refinement),initial_(initial_momentum),uncertainty_(onset_uncertainty) {
        Require((refinement==1||refinement==2||refinement==4)&&std::isfinite(initial_)&&initial_>0&&
            std::isfinite(uncertainty_)&&uncertainty_>=0,"Invalid source wall event inputs");
    }
    void Observe(const IntervalPoint& p) {
        const double h=BaseStep/refinement_;
        Require(p.epoch==events_.final_epoch+1&&Bits(p.time)==Bits(p.epoch*h),"Nonsequential source wall event samples");
        for(double x:{p.reaction,p.reaction_error,p.potential,p.potential_error,p.impulse,p.impulse_error,
            p.momentum_allowance,p.penetration,p.physical_energy_uncertainty})
            Require(std::isfinite(x)&&x>=0,"Invalid source wall event certificate");
        Require(std::isfinite(p.momentum_residual)&&p.active_nodes<=117&&p.strictly_separated_nodes<=117&&
            p.active_nodes+p.strictly_separated_nodes<=117&&std::abs(p.momentum_residual)<=p.momentum_allowance,
            "Invalid carried momentum or contact-node event counts");
        Require(!p.active_nodes||p.reaction>0,"Active wall contact has no positive reaction");
        events_.final_epoch=p.epoch;
        if(p.reaction>events_.peak_reaction) { events_.peak_reaction=p.reaction; events_.peak_reaction_error=p.reaction_error; }
        events_.peak_penetration=std::max(events_.peak_penetration,p.penetration);
        events_.maximum_energy_uncertainty=std::max(events_.maximum_energy_uncertainty,p.physical_energy_uncertainty);
        if(p.active_nodes) {
            if(!events_.onset.observed) events_.onset={true,(p.epoch-1)*h,p.time,uncertainty_};
            ++events_.contact_intervals;
        }
        const bool separated=events_.onset.observed&&p.strictly_separated_nodes==117&&p.active_nodes==0&&
            p.reaction<=p.reaction_error&&p.potential<=p.potential_error&&MomentumUpper(p,initial_)<0;
        if(!separated) { events_.terminal_separation_start=0; events_.rebound={}; return; }
        if(!events_.terminal_separation_start) events_.terminal_separation_start=p.epoch;
        // Endpoints e..e+128*r prove exactly 128H of continuous qualification.
        if(p.epoch-events_.terminal_separation_start>=CommonStride*refinement_) {
            const auto start=events_.terminal_separation_start;
            events_.rebound={true,(start-1)*h,start*h,0};
        }
    }
    const Events& events() const noexcept { return events_; }
 private:
    unsigned refinement_;
    long double initial_;
    double uncertainty_;
    Events events_{};
};
} // namespace crash::output::wall_comparison
