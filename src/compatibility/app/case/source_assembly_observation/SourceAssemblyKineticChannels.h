#pragma once
#include "SourceAssemblyObservationInternal.h"

namespace crash::cases::source_assembly_observation::detail {
inline constexpr auto MemberFields=std::array{&rigid::MemberKineticChannels::translation,&rigid::MemberKineticChannels::native_rotation,
            &rigid::MemberKineticChannels::physical_rotation,&rigid::MemberKineticChannels::added_rotation,
            &rigid::MemberKineticChannels::total,&rigid::MemberKineticChannels::inertia_partition_residual};
inline constexpr auto AggregateFields=std::array{&rigid::AggregateKineticChannels::translation,&rigid::AggregateKineticChannels::rotation,
            &rigid::AggregateKineticChannels::total,&rigid::AggregateKineticChannels::structural_translation,
            &rigid::AggregateKineticChannels::primary_translation,&rigid::AggregateKineticChannels::member_orbital_rotation,
            &rigid::AggregateKineticChannels::native_member_rotation,&rigid::AggregateKineticChannels::physical_member_rotation,
            &rigid::AggregateKineticChannels::added_member_rotation,&rigid::AggregateKineticChannels::primary_parallel_axis_rotation,
            &rigid::AggregateKineticChannels::primary_isotropic_rotation,&rigid::AggregateKineticChannels::principal_correction_rotation,
            &rigid::AggregateKineticChannels::decomposition_residual,&rigid::AggregateKineticChannels::decomposition_roundoff_budget};
inline void Add(rigid::MemberKineticChannels& a,const rigid::MemberKineticChannels& b) {
    for(auto field:MemberFields) a.*field+=b.*field;
}
inline void Add(rigid::AggregateKineticChannels& a,const rigid::AggregateKineticChannels& b) {
    for(auto field:AggregateFields) a.*field+=b.*field;
}
inline bool Finite(const rigid::MemberKineticChannels& value) noexcept {
    for(auto field:MemberFields) if(!std::isfinite(value.*field)) return false;
    return true;
}
inline bool Finite(const rigid::AggregateKineticChannels& value) noexcept {
    for(auto field:AggregateFields) if(!std::isfinite(value.*field)) return false;
    return true;
}
using NativeKineticSums=std::array<long double,6>;
inline Report AddNativeMotion(const fe::NodalMassPartitions& m,rigid::MemberMotion motion,std::size_t node,
                              NativeKineticSums& all,NativeKineticSums* ordinary=nullptr) noexcept {
    for(unsigned a=0;a<3;++a) {
        const long double speed=a==0?motion.velocity.x:a==1?motion.velocity.y:motion.velocity.z;
        const long double omega=a==0?motion.omega.x:a==1?motion.omega.y:motion.omega.z;
        if(!std::isfinite(speed) || !std::isfinite(omega))
            return {Status::NonfiniteResult,"Nonfinite nodal motion",SIZE_MAX,node,a};
        const long double values[]{.5L*m.mass*speed*speed,.5L*m.isotropic_inertia*omega*omega,
                                  .5L*m.shell.physical_inertia*omega*omega,.5L*m.shell.added_inertia*omega*omega,
                                  .5L*m.connector_mass*speed*speed,.5L*m.connector_inertia*omega*omega};
        for(unsigned c=0;c<6;++c) {all[c]+=values[c]; if(ordinary) (*ordinary)[c]+=values[c];}
    }
    return Success();
}
inline void AssignNative(const NativeKineticSums& sum,rigid::MemberKineticChannels& out) noexcept {
    out.translation=sum[0]; out.native_rotation=sum[1];
    out.physical_rotation=sum[2]; out.added_rotation=sum[3];
    out.total=sum[0]+sum[1]; out.inertia_partition_residual=(sum[1]-sum[2]-sum[3])-sum[5];
}
} // namespace crash::cases::source_assembly_observation::detail
