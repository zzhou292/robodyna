#include "SourcePartPlasticComparisonInput.h"
#include <iomanip>
#include <sstream>

namespace crash::output::plastic_comparison {
Schedule MatchConfiguration(const Value& a,const Value& b) {
    Schedule s{wc::Real(a,"fixed_dt_s"),wc::Real(b,"fixed_dt_s"),0,
        wc::Real(a,"requested_horizon_s"),wc::Unsigned(a,"frame_every"),wc::Unsigned(b,"frame_every")};
    Require(s.coarse_step>0&&s.fine_step>0&&Bits(s.coarse_step)==Bits(2*s.fine_step)&&
        s.coarse_stride&&s.fine_stride&&s.requested_horizon>0,"Plastic comparison requires positive h/h2 steps and cadence");
    s.cadence=s.coarse_stride*s.coarse_step;
    Require(std::isfinite(s.cadence)&&Bits(s.cadence)==Bits(s.fine_stride*s.fine_step)&&
        Bits(s.requested_horizon)==Bits(wc::Real(b,"requested_horizon_s")),
        "Plastic runs changed physical horizon or saved sample cadence");
    auto ca=wc::PhysicalConfiguration(a),cb=wc::PhysicalConfiguration(b);
    // Alpha changes because h changes. Its input C/P/cutoff and angular
    // coefficient remain compared; AcceptedReplay independently checks alpha.
    ca.RemoveMember("rate_filter_alpha");cb.RemoveMember("rate_filter_alpha");
    Require(wc::Encode(ca)==wc::Encode(cb),"Plastic runs changed physical material, geometry, source or limits");
    return s;
}
std::uint64_t SharedStrideCount(const Schedule& s,std::uint64_t coarse_final,std::uint64_t fine_final) {
    Require(s.coarse_stride&&s.fine_stride,"Missing shared plastic sample cadence");
    const auto count=std::min(coarse_final/s.coarse_stride,fine_final/s.fine_stride);
    Require(count>0,"Plastic prefixes contain no common sample after startup");
    return count;
}
void Run::Open(const std::filesystem::path& path) {
    directory=path;
    const auto opened=replay.Open(directory);
    if(opened.status!=ReplayStatus::Ok)throw std::runtime_error(opened.diagnostic);
    Require(replay.info()->kind==ReplayKind::SourcePartWall&&replay.info()->source_plasticity,
        "Plastic comparison requires two validated source-wall v2 bundles");
    configuration=wc::ReadDocument(directory/"configuration.json");
    manifest=wc::ReadDocument(directory/"manifest.json");final=wc::ReadDocument(directory/"final-metrics.json");
    manifest_hash=Sha256(ReadBounded(directory/"manifest.json",1024*1024));
    const double dt=wc::Real(configuration,"fixed_dt_s");
    for(unsigned r:{1u,2u,4u})if(Bits(dt)==Bits(wc::BaseStep/r))refinement=r;
    Require(refinement!=0,"Plastic comparison does not recognize this source-part timestep family");
    auto physical=wc::PhysicalConfiguration(configuration);physical.RemoveMember("rate_filter_alpha");
    physical_configuration=wc::Encode(physical);
    const auto& nodes=wc::Field(configuration,"reference_nodes");
    Require(nodes.IsArray()&&nodes.Size()==117,"Incomplete native source mass binding");
    long double sum=0;
    for(unsigned n=0;n<117;++n) {
        mass[n]=wc::Real(nodes[n],"mass_kg");Require(mass[n]>0,"Nonpositive source native mass");sum+=mass[n];
    }
    total_mass=static_cast<double>(sum);initial_kinetic=wc::Real(configuration,"initial_kinetic_J");
    const auto& velocity=wc::Field(configuration,"initial_velocity_xyz_m_per_s");
    Require(velocity.IsArray()&&velocity.Size()==3,"Missing source impact velocity");
    speed=Scalar(velocity[0]);initial_momentum=sum*speed;
    Require(std::isfinite(total_mass)&&total_mass>0&&initial_kinetic>0&&speed>0,"Invalid source impact scales");
    observed=ReadIntervals(*this);
    observed.final_maximum_plastic_strain=wc::Real(final,"maximum_plastic_strain");
    observed.final_mean_plastic_strain=wc::Real(final,"mean_plastic_strain");
    observed.final_plastic_work=wc::Real(final,"cumulative_plastic_work_J");
    observed.final_yielded_points=wc::Unsigned(final,"yielded_points");
    observed.final_yielded_parents=wc::Unsigned(final,"yielded_parents");
    observed.final_chord_change=wc::Real(final,"maximum_chord_change_m");
}
Document Run::FrameAt(std::uint64_t epoch) {
    while(replay.frame()->epoch<epoch) {
        Require(replay.frame()->index+1<replay.info()->frame_count,"Missing shared plastic sample");
        const auto loaded=replay.Load(replay.frame()->index+1);
        if(loaded.status!=ReplayStatus::Ok)throw std::runtime_error(loaded.diagnostic);
    }
    Require(replay.frame()->epoch==epoch,"Plastic runs do not contain the declared shared sample");
    std::ostringstream name;name<<"accepted-"<<std::setw(6)<<std::setfill('0')<<epoch<<".fields.json";
    auto fields=wc::ReadDocument(directory/name.str());
    Require(wc::Unsigned(fields,"accepted_epoch")==epoch&&
        Bits(wc::Real(fields,"accepted_time_s"))==Bits(replay.frame()->time),"Plastic sample identity changed");
    CheckFrameMomentum(*this,fields);
    return fields;
}
void CheckFrameMomentum(const Run& run,const Value& f) {
    const auto& velocity=wc::Field(f,"velocity_xyz_m_per_s");
    Require(velocity.IsArray()&&velocity.Size()==351,"Missing native carried velocity");
    long double carried=0,magnitude=0;
    for(unsigned n=0;n<117;++n) {
        const long double term=static_cast<long double>(run.mass[n])*Scalar(velocity[3*n]);
        carried+=term;magnitude+=std::abs(term);
    }
    const double impulse=wc::Real(f,"cumulative_wall_kick_impulse_N_s");
    const double error=wc::Real(f,"cumulative_wall_kick_impulse_error_N_s");
    const double residual=Scalar(wc::Field(f,"carried_momentum_residual_xyz_kg_m_per_s")[0]);
    const double allowance=Scalar(wc::Field(f,"carried_momentum_allowance_xyz_kg_m_per_s")[0]);
    const auto rounding=wc::MomentumArithmetic(run.initial_momentum,impulse,residual)+
        64*std::numeric_limits<double>::epsilon()*magnitude;
    Require(std::abs(carried-(run.initial_momentum-impulse+residual))<=rounding+error+allowance,
        "Plastic saved velocity disagrees with native mass and carried impulse");
    if(run.observed.events.rebound.observed&&wc::Unsigned(f,"accepted_epoch")>=run.observed.events.terminal_separation_start)
        Require(carried+rounding<0,"Saved native COM does not support terminal separated rebound");
}
} // namespace crash::output::plastic_comparison
