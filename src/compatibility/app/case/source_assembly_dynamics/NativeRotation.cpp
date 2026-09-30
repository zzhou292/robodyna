#include "NativeRotation.h"
#include "lib_src/constraints/NodalRigidGroupMath.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::source_assembly_dynamics {
namespace value=tl::fea::rigid::detail; // Reuse fixed vector/frame checks only; no constraint mechanics.
Report MeasureNativeRotation(const NativeRotationReference& source,const tl::math::Matrix3& frame,
    const tl::math::Vec3* local,std::size_t arity,double limit,NativeRotationMeasure& output) noexcept {
    auto fail=[&](const char* message,double measured=0) {return Report{Status::EnvelopeFailure,message,source.source_parent,SIZE_MAX,measured,limit};};
    if(!source.source_parent||!local||(arity!=3&&arity!=4)||arity!=source.arity||
       !std::isfinite(limit)||limit<=0||limit>NativeRotationQualifiedBound||
       !tl::fea::trial_identity::Disjoint(&output,sizeof(output),&source,sizeof(source))||
       !tl::fea::trial_identity::Disjoint(&output,sizeof(output),&frame,sizeof(frame))||
       !tl::fea::trial_identity::Disjoint(&output,sizeof(output),local,arity*sizeof(*local))||
       !value::Orthonormal(source.frame)||!value::Orthonormal(frame))return fail("Native rotation frame/source/limit is invalid");
    // Complementary Frobenius chords avoid acos(1) for small rotations.
    long double minus=0,plus=0;
    for(unsigned i=0;i<9;++i){const long double delta=static_cast<long double>(frame.v[i])-source.frame.v[i];minus+=delta*delta;}
    // ||R-R0||F^2 = 8 sin^2(theta/2); the complementary chord avoids acos(1).
    plus=std::max(0.L,8.L-minus);
    NativeRotationMeasure next;next.frame=static_cast<double>(2*std::atan2(std::sqrt(minus),std::sqrt(plus)));
    if(!std::isfinite(next.frame)||next.frame>limit)return fail("Source-native shell frame rotation envelope exceeded",next.frame);
    for(std::size_t i=0;i<arity;++i) {
        const auto old=source.world_normals[i],now=value::ToWorld(frame,local[i]);
        if(!value::Finite(old)||!value::Finite(now)||std::abs(value::Dot(old,old)-1)>1e-12||std::abs(value::Dot(now,now)-1)>1e-12)
            return fail("Source-native nodal normal is not finite and unit");
        const auto cross=value::Cross(old,now);const double sine=std::hypot(std::hypot(cross.x,cross.y),cross.z);
        const double angle=std::atan2(sine,value::Dot(old,now));
        if(!std::isfinite(angle)||angle>limit)return fail("Source-native nodal normal rotation envelope exceeded",angle);
        next.nodal_normal=std::max(next.nodal_normal,angle);
    }
    output=next;return {Status::Ok,"Source-native rotation checked"};
}
}
