#pragma once
#include "SourcePartPlasticComparisonMath.h"

namespace crash::output::plastic_comparison {
struct Schedule {
    double coarse_step=0,fine_step=0,cadence=0,requested_horizon=0;
    std::uint64_t coarse_stride=0,fine_stride=0;
};
// Stages normalization without modifying either caller-owned declaration.
Schedule MatchConfiguration(const Value& coarse,const Value& fine);
std::uint64_t SharedStrideCount(const Schedule&,std::uint64_t coarse_final,std::uint64_t fine_final);
struct Observations {
    wc::Events events;
    double maximum_absolute_energy_residual=0,maximum_energy_bound_fraction=0;
    double final_raw_com_velocity=0,final_chord_change=0;
    double final_maximum_plastic_strain=0,final_mean_plastic_strain=0,final_plastic_work=0;
    unsigned final_yielded_points=0,final_yielded_parents=0;
};
struct Run {
    std::filesystem::path directory;
    AcceptedReplay replay;
    Document configuration,manifest,final;
    unsigned refinement=0;
    std::array<double,117> mass{};
    double total_mass=0,initial_kinetic=0,speed=0;
    long double initial_momentum=0;
    std::string manifest_hash,physical_configuration;
    Observations observed;
    void Open(const std::filesystem::path&);
    Document FrameAt(std::uint64_t epoch);
};
Observations ReadIntervals(const Run&);
void CheckFrameMomentum(const Run&,const Value&);
}
