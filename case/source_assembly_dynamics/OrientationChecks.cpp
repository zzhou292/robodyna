#include "State.h"
#include "lib_src/math/Quaternion.h"
#include <algorithm>
#include <cmath>
namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyWallCase::Impl::CheckRotations() {
    auto& next=candidate();auto& d=next.diagnostics;d.rotation_domain=config.rotation_domain;
    if(config.rotation_domain==RotationDomain::NativeShellGeometryV1&&(!rotation_reference||
       rotation_reference->source_instance()!=bindings.source_instance_id()||rotation_reference->node_count()!=nodes()||
       rotation_reference->parent_count()!=quads()+triangles()))return Failure(Status::SourceMismatch,"Missing immutable source-native rotation references");
    for(std::size_t n=0;n<nodes();++n) {
        const auto* a=next.fields.orientation.data()+4*n;
        if(!tl::math::UnitQuaternion({a[0],a[1],a[2],a[3]}))
            return Failure(Status::EnvelopeFailure,"Native candidate orientation is not finite and unit",0,n);
        const double rotation=2*std::atan2(std::hypot(std::hypot(a[1],a[2]),a[3]),std::abs(a[0]));
        d.maximum_rotation=std::max(d.maximum_rotation,rotation);
        if(rotation_reference&&rotation_reference->grouped(n))d.maximum_rigid_member_rotation=std::max(d.maximum_rigid_member_rotation,rotation);
        for(unsigned axis=0;axis<3;++axis) {
            const auto j=3*n+axis;
            for(double value:{next.fields.x[j],next.fields.v[j],next.fields.w[j],next.fields.reaction[j],next.fields.couple[j]})
                if(!std::isfinite(value))return Failure(Status::EnvelopeFailure,"Candidate motion/reaction field is nonfinite",0,n);
        }
    }
    if(config.rotation_domain==RotationDomain::NodalQuaternion&&d.maximum_rotation>config.deformation.maximum_rotation)
        return Failure(Status::EnvelopeFailure,"Source total orientation envelope exceeded",0,SIZE_MAX,d.maximum_rotation,config.deformation.maximum_rotation);
    if(config.rotation_domain==RotationDomain::NodalQuaternion)return Success();
    if(d.maximum_rigid_member_rotation>config.deformation.maximum_rotation)
        return Failure(Status::EnvelopeFailure,"Source rigid-member total orientation envelope exceeded",0,SIZE_MAX,
                       d.maximum_rigid_member_rotation,config.deformation.maximum_rotation);
    auto check=[&](std::size_t index,const tl::math::Matrix3& frame,const tl::math::Vec3* normals,std::size_t arity,std::uint64_t source) {
        const auto& reference=rotation_reference->parent(index);
        if(reference.source_parent!=source)return Failure(Status::SourceMismatch,"Source-native rotation parent order differs",source);
        NativeRotationMeasure measure;const auto r=MeasureNativeRotation(reference,frame,normals,arity,config.deformation.maximum_rotation,measure);
        if(!r)return r;
        d.maximum_native_frame_rotation=std::max(d.maximum_native_frame_rotation,measure.frame);
        d.maximum_native_normal_rotation=std::max(d.maximum_native_normal_rotation,measure.nodal_normal);return Success();
    };
    for(std::size_t e=0;e<quads();++e) {
        const auto r=check(e,next.parents.qeph[e].kinematics.frame,next.parents.qeph[e].kinematics.local_normals,4,bindings.shells().qeph_source_id(e));if(!r)return r;
    }
    for(std::size_t e=0;e<triangles();++e) {
        const auto r=check(quads()+e,next.parents.t3[e].kinematics.frame,NativeTriangleNormals,3,bindings.shells().t3_source_id(e));if(!r)return r;
    }
    return Success();
}
}
