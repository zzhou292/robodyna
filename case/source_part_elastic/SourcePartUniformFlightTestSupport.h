#pragma once
#include "SourcePartElasticTestSupport.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::source_part_elastic::test {
// Qualification arithmetic bounds frozen before the first moving-source run.
// These check rigid free flight, not an accuracy claim for deforming/contact runs.
struct FlightOracle {
    std::array<long double,NodeCount> mass{},inertia{};
    long double total_mass=0,total_inertia=0,coordinate_scale=0,minimum_edge=0,speed=0,initial_kinetic=0;
    long double maximum_position_error=0,maximum_raw_velocity_error=0,maximum_synchronized_velocity_error=0;
    long double maximum_angular_velocity=0,maximum_chord_change=0,maximum_energy_residual=0;
    void Initialize(const SourcePartElasticCase& run,const NativeSequence& native) {
        const auto& b=run.binding();
        minimum_edge=std::numeric_limits<long double>::infinity();
        auto gather=[&](const auto& nodes,const auto& reference) {
            for(unsigned i=0;i<nodes.size();++i) {
                mass[nodes[i]]+=reference.nodal_mass[i];
                inertia[nodes[i]]+=reference.isotropic_inertia[i];
                const auto a=nodes[i],next=nodes[(i+1)%nodes.size()];
                long double square=0;
                for(unsigned axis=0;axis<3;++axis) {
                    const long double dx=static_cast<long double>(run.source().coordinates()[3*a+axis])-
                        run.source().coordinates()[3*next+axis];
                    square+=dx*dx;
                }
                minimum_edge=std::min(minimum_edge,std::sqrt(square));
            }
        };
        for(std::size_t e=0;e<source::Q4Count;++e) gather(b.qeph_nodes(e),native.qr[e].data());
        for(std::size_t e=0;e<source::T3Count;++e) gather(b.t3_nodes(e),native.tr[e].data());
        for(std::size_t n=0;n<NodeCount;++n) {
            total_mass+=mass[n]; total_inertia+=inertia[n];
            EXPECT_LE(std::abs(mass[n]-b.nodes()[n].native.mass),2e-12L*mass[n]);
            EXPECT_LE(std::abs(inertia[n]-b.nodes()[n].native.isotropic_inertia),2e-12L*inertia[n]);
        }
        for(double x:run.source().coordinates()) coordinate_scale=std::max(coordinate_scale,std::abs(static_cast<long double>(x)));
        for(double v:run.config().initial_velocity) speed+=static_cast<long double>(v)*v;
        speed=std::sqrt(speed);
        initial_kinetic=.5L*total_mass*speed*speed;
        ASSERT_GT(minimum_edge,0); ASSERT_TRUE(std::isfinite(minimum_edge));
        EXPECT_LE(std::abs(run.initial_kinetic_energy()-initial_kinetic),2e-12L*initial_kinetic);
    }
    void Check(const SourcePartElasticCase& run,const Snapshot& actual,unsigned steps) {
        // Retain the existing native kick/drift oracle's per-step coefficient;
        // accumulated bounds explicitly carry coordinate, velocity and length scales.
        const long double r=2e-13L*(steps+1),time=static_cast<long double>(steps)*run.config().dt;
        const long double position_bound=r*(1+coordinate_scale+speed*time);
        const long double velocity_bound=r*(1+speed),spin_bound=velocity_bound/minimum_edge;
        const long double orientation_bound=r*(1+speed*time/minimum_edge);
        const long double strain_bound=position_bound/minimum_edge;
        std::array<long double,3> raw_momentum{},sync_momentum{};
        for(std::size_t n=0;n<NodeCount;++n) {
            for(unsigned a=0;a<3;++a) {
                const auto j=3*n+a;
                const long double v0=run.config().initial_velocity[a];
                const long double expected=run.source().coordinates()[j]+time*v0;
                maximum_position_error=std::max(maximum_position_error,std::abs(actual.position[j]-expected));
                maximum_raw_velocity_error=std::max(maximum_raw_velocity_error,std::abs(actual.velocity[j]-v0));
                maximum_synchronized_velocity_error=std::max(maximum_synchronized_velocity_error,std::abs(actual.synchronized_velocity[j]-v0));
                maximum_angular_velocity=std::max(maximum_angular_velocity,
                    static_cast<long double>(std::max(std::abs(actual.omega[j]),std::abs(actual.synchronized_omega[j]))));
                EXPECT_LE(std::abs(actual.position[j]-expected),position_bound);
                EXPECT_LE(std::abs(actual.velocity[j]-v0),velocity_bound);
                EXPECT_LE(std::abs(actual.synchronized_velocity[j]-v0),velocity_bound);
                EXPECT_LE(std::abs(actual.omega[j]),spin_bound);
                EXPECT_LE(std::abs(actual.synchronized_omega[j]),spin_bound);
                raw_momentum[a]+=mass[n]*actual.velocity[j];
                sync_momentum[a]+=mass[n]*actual.synchronized_velocity[j];
            }
            for(unsigned a=0;a<4;++a)
                EXPECT_LE(std::abs(actual.orientation[4*n+a]-(a==0?1.L:0.L)),orientation_bound);
        }
        for(unsigned a=0;a<3;++a) {
            const long double expected=total_mass*run.config().initial_velocity[a];
            const long double allowance=total_mass*velocity_bound+2e-12L*std::abs(expected);
            EXPECT_LE(std::abs(raw_momentum[a]-expected),allowance);
            EXPECT_LE(std::abs(sync_momentum[a]-expected),allowance);
        }
        const long double dv=std::sqrt(3.L)*velocity_bound;
        const long double kinetic_allowance=total_mass*(speed*dv+.5L*dv*dv)+
            1.5L*total_inertia*spin_bound*spin_bound+2e-12L*initial_kinetic;
        const auto& d=actual.diagnostics;
        maximum_chord_change=std::max(maximum_chord_change,static_cast<long double>(d.maximum_chord_change));
        maximum_energy_residual=std::max(maximum_energy_residual,static_cast<long double>(std::abs(d.energy_residual)));
        EXPECT_LE(std::abs(d.synchronized_kinetic-initial_kinetic),kinetic_allowance);
        EXPECT_LE(std::abs(d.shells.kinetic.translation+d.shells.kinetic.rotation-initial_kinetic),kinetic_allowance);
        EXPECT_LE(d.maximum_chord_change,2*std::sqrt(3.L)*position_bound);
        EXPECT_LE(d.max_relative_displacement,2*std::sqrt(3.L)*position_bound);
        EXPECT_LE(d.maximum_rotation,4*orientation_bound);
        EXPECT_LE(d.shells.qeph.maximum_absolute_strain,strain_bound);
        EXPECT_LE(d.shells.t3.maximum_absolute_strain,strain_bound);
        EXPECT_LE(d.shells.qeph.maximum_thickness_curvature,strain_bound);
        EXPECT_LE(d.shells.t3.maximum_thickness_curvature,strain_bound);
        if(steps) {
            EXPECT_LE(std::abs(d.maximum_area_ratio-1),strain_bound);
            EXPECT_LE(std::abs(d.maximum_thickness_ratio-1),strain_bound);
            EXPECT_LE(std::abs(d.shells.qeph.minimum_area_ratio-1),strain_bound);
            EXPECT_LE(std::abs(d.shells.t3.minimum_area_ratio-1),strain_bound);
            EXPECT_LE(std::abs(d.shells.qeph.minimum_thickness_ratio-1),strain_bound);
            EXPECT_LE(std::abs(d.shells.t3.minimum_thickness_ratio-1),strain_bound);
        }
        EXPECT_EQ(d.external_kick_work,0); EXPECT_EQ(d.external_drift_work,0);
        EXPECT_EQ(d.absolute_external_drift_work,0);
        EXPECT_LE(std::abs(d.energy_residual),run.config().maximum_energy_residual);
        EXPECT_LE(std::abs(d.kinetic_work_residual),d.kinetic_work_allowance);
    }
};
} // namespace crash::cases::source_part_elastic::test
