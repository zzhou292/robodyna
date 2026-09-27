#pragma once
#include "modelio/physical_domain/Policy.h"
#include "modelio/type45/SourcePolicy.h"
#include "lib_src/elements/solids/Model.h"
namespace crash::cases::vehicle_self_contact::native::source_policy {
using Domain=modelio::physical_domain::Policy;
using Solid=modelio::solid_source::Policy;
using Joint=modelio::type45::Policy;
inline bool NativeV6(Domain p)noexcept{return p==Domain::RetainedShellAssembliesNativeSupportsV6;}
inline bool Geometry(Domain d,Solid s)noexcept {
    return (d==Domain::RetainedShellAssembliesVehicleSupportsV5&&s==Solid::OriginalVehicleSupportsV5)||
        (NativeV6(d)&&s==Solid::NativeConvertedSupportsV6);
}
inline bool Joints(Domain d,Joint j)noexcept {
    return (d==Domain::RetainedShellAssembliesVehicleSupportsV5&&j==Joint::OriginalDirectSdiType45VehicleSupportsV5)||
        (NativeV6(d)&&j==Joint::OriginalDirectSdiType45NativeSupportsV6);
}
inline bool Controls(Domain d,const tl::fea::solids::Model& solids)noexcept {
    if(!NativeV6(d))return true;
    const auto* selected=solids.control_selection();
    return selected&&selected->profile()==tl::fea::solids::control::Profile::SourceDeclared&&
        selected->controlled_count()==3686;
}
} // namespace crash::cases::vehicle_self_contact::native::source_policy
