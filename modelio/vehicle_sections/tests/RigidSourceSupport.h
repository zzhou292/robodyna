#pragma once
#include "GlassSourceSupport.h"
#include "modelio/rigid_part/RigidPartSource.h"
namespace crash::modelio::vehicle::test {
inline const VehicleSectionResolution& MidlayerResolution() {
    static const auto value=VehicleSectionResolution::ResolveOriginalMidlayer(GlassResolution(),ResolutionProfile::OriginalMidlayerV1);
    return value;
}
inline const std::string& OriginalMember() {
    static const auto bytes=[] {
        const auto& in=Canonical().data().inputs;
        return output::ReadBounded(in.member_root/in.source_member.file,in.source_member.bytes);
    }();
    return bytes;
}
inline const VehicleSectionResolution& RigidResolution() {
    static const auto value=VehicleSectionResolution::ResolveOriginalRigidParts(MidlayerResolution(),OriginalMember(),
        ResolutionProfile::OriginalRigidPartsV1, ResolutionLimits::CompleteRigidOverlay());
    return value;
}
}
