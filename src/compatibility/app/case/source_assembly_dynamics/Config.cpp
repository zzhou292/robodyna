#include "Config.h"
#include "NativeRotation.h"
#include <cmath>

namespace crash::cases::source_assembly_dynamics {
bool ValidConfig(const Config& c) noexcept {
    if(c.rotation_domain!=RotationDomain::NodalQuaternion&&c.rotation_domain!=RotationDomain::NativeShellGeometryV1)return false;
    if(c.rotation_domain==RotationDomain::NativeShellGeometryV1&&c.deformation.maximum_rotation>NativeRotationQualifiedBound)return false;
    const auto& d=c.deformation;const auto& s=c.storage;
    for(double x:{c.fixed_dt,d.maximum_displacement,d.maximum_rotation,d.maximum_rotation_increment,d.maximum_strain,
        d.maximum_thickness_curvature,d.minimum_area_ratio,d.maximum_area_ratio,
        d.minimum_thickness_ratio,d.maximum_thickness_ratio,d.maximum_native_dt_fraction})
        if(!std::isfinite(x)||!(x>0))return false;
    if(c.fixed_dt<1e-12||d.minimum_area_ratio>1||d.maximum_area_ratio<1||
       d.minimum_thickness_ratio>1||d.maximum_thickness_ratio<1||d.maximum_native_dt_fraction>1||
       d.maximum_rotation>2*std::acos(-1.)||d.maximum_rotation_increment>=std::acos(-1.))return false;
    if(!s.max_nodes||s.max_nodes>2048||!s.max_parents||s.max_parents>1024||
       !s.max_host_bytes||s.max_host_bytes>128*1024*1024||!s.max_device_bytes||s.max_device_bytes>64*1024*1024||
       !s.owner_device_bytes||s.owner_device_bytes>tl::fea::MaxTranslationDeviceBytes||
       !s.qeph_device_bytes||!s.t3_device_bytes||!s.publication.max_device_bytes||
       !s.contact.max_device_bytes||!s.contact.max_host_bytes)return false;
    std::size_t remaining=s.max_device_bytes;
    for(std::size_t bytes:{s.owner_device_bytes,s.qeph_device_bytes,s.t3_device_bytes,
                           s.publication.max_device_bytes,s.contact.max_device_bytes}) {
        if(bytes>remaining)return false;remaining-=bytes;
    }
    return true;
}
} // namespace crash::cases::source_assembly_dynamics
