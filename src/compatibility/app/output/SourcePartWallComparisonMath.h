#pragma once
#include "SourcePartComparison.h"
#include <array>
#include <cmath>

namespace crash::output::wall_comparison {
inline constexpr double BaseStep=0x1p-24;
inline constexpr unsigned BaseHorizon=32768,CommonStride=128,CommonSamples=257;
inline constexpr std::size_t Channels=8;
using Differences=std::array<double,Channels>;
inline constexpr const char* ChannelNames[]{"position","rotation","synchronized_velocity","synchronized_omega",
    "total_native_contact_energy","contact_potential","carried_wall_impulse","wall_reaction"};

struct Scales {
    double initial_kinetic=0,mass=0,speed=0,design_depth=0,reaction=0,impulse=0;
};
inline Scales MakeScales(double initial_kinetic,double mass,double speed,double design_depth) {
    for(double x:{initial_kinetic,mass,speed,design_depth})
        Require(std::isfinite(x)&&x>0,"Invalid source wall comparison scale");
    Scales result{initial_kinetic,mass,speed,design_depth,2*initial_kinetic/design_depth,mass*speed};
    Require(std::isfinite(result.reaction)&&result.reaction>0&&std::isfinite(result.impulse)&&result.impulse>0,
        "Source wall comparison scale overflow");
    return result;
}
// Parsed from the validated actual wall schema. Keep these units independent
// of its JSON nesting and of any solver/native implementation dependencies.
struct ContactSample {
    double synchronized_kinetic=0,native_internal_work=0,potential=0,wall_impulse=0,wall_reaction=0;
};
inline Differences Difference(const Value& a,const ContactSample& ca,const Value& b,const ContactSample& cb,const Scales& scales) {
    for(double x:{scales.initial_kinetic,scales.impulse,scales.reaction})
        Require(std::isfinite(x)&&x>0,"Invalid source wall comparison denominator");
    for(const auto& c:{ca,cb}) {
        for(double x:{c.synchronized_kinetic,c.native_internal_work,c.potential,c.wall_impulse,c.wall_reaction})
            Require(std::isfinite(x),"Nonfinite source wall comparison sample");
        Require(c.synchronized_kinetic>=0&&c.potential>=0&&c.wall_impulse>=0&&c.wall_reaction>=0,
            "Negative source wall kinetic/contact quantity");
    }
    const auto kinematics=source_comparison::Kinematics(a,b);
    const long double ea=static_cast<long double>(ca.synchronized_kinetic)+ca.native_internal_work+ca.potential;
    const long double eb=static_cast<long double>(cb.synchronized_kinetic)+cb.native_internal_work+cb.potential;
    return {kinematics[0],kinematics[1],kinematics[2],kinematics[3],static_cast<double>(std::abs(ea-eb)/.1L),
        std::abs(ca.potential-cb.potential)/scales.initial_kinetic,
        std::abs(ca.wall_impulse-cb.wall_impulse)/scales.impulse,
        std::abs(ca.wall_reaction-cb.wall_reaction)/scales.reaction};
}
inline bool Converged(double coarse,double fine) noexcept {
    return std::isfinite(coarse)&&coarse>=0&&std::isfinite(fine)&&fine>=0&&
        coarse<=.10&&fine<=.05&&fine<=.8*coarse+1e-6;
}
struct EventBracket {
    bool observed=false;
    double lower=0,upper=0,uncertainty=0;
};
inline bool Valid(const EventBracket& b) noexcept {
    return std::isfinite(b.lower)&&std::isfinite(b.upper)&&std::isfinite(b.uncertainty)&&
        b.lower>=0&&b.upper>=b.lower&&b.uncertainty>=0;
}
inline bool EventsAgree(const EventBracket& a,const EventBracket& b,double coarser_step) {
    Require(Valid(a)&&Valid(b)&&std::isfinite(coarser_step)&&coarser_step>0,"Invalid source wall event bracket");
    if(a.observed!=b.observed) return false;
    if(!a.observed) return true;
    // One combined coarser-step expansion, plus the independently certified
    // timing uncertainties, avoids counting the same sampling slack twice.
    const long double separation=std::max(static_cast<long double>(a.lower)-b.upper,
        static_cast<long double>(b.lower)-a.upper);
    return separation<=static_cast<long double>(coarser_step)+a.uncertainty+b.uncertainty;
}
} // namespace crash::output::wall_comparison
