#include "../OriginalSources.h"
#include "case/vehicle_self_contact/native/SourcePolicies.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::source {
TEST(NativeV6SourceAdmission, ExplicitProfilesNeverCrossBindLegacyAndNativeSources) {
    namespace p=vehicle_self_contact::native::source_policy;
    EXPECT_TRUE(p::Geometry(p::Domain::RetainedShellAssembliesVehicleSupportsV5,p::Solid::OriginalVehicleSupportsV5));
    EXPECT_TRUE(p::Geometry(p::Domain::RetainedShellAssembliesNativeSupportsV6,p::Solid::NativeConvertedSupportsV6));
    EXPECT_FALSE(p::Geometry(p::Domain::RetainedShellAssembliesVehicleSupportsV5,p::Solid::NativeConvertedSupportsV6));
    EXPECT_FALSE(p::Geometry(p::Domain::RetainedShellAssembliesNativeSupportsV6,p::Solid::OriginalVehicleSupportsV5));
    EXPECT_TRUE(p::Joints(p::Domain::RetainedShellAssembliesNativeSupportsV6,p::Joint::OriginalDirectSdiType45NativeSupportsV6));
    EXPECT_FALSE(p::Joints(p::Domain::RetainedShellAssembliesNativeSupportsV6,p::Joint::OriginalDirectSdiType45VehicleSupportsV5));
    tl::fea::solids::Model empty;EXPECT_FALSE(p::Controls(p::Domain::RetainedShellAssembliesNativeSupportsV6,empty));
}
TEST(NativeV6SourceAdmission, MissingPathsAndForeignCaseRejectBeforeReadersOrOwner) {
    vehicle_run::OriginalPaths paths;modelio::solid_control_packets::Artifact artifact;
    EXPECT_THROW(OriginalSources::Prepare(paths,artifact),std::runtime_error);
    for(auto* p:{&paths.canonical,&paths.scope,&paths.member,&paths.declarations,&paths.glass_resolution,
        &paths.type13,&paths.auxiliary_member,&paths.original_wall_member,&paths.wall_manifest,&paths.self_contact_combine_member})*p="/not-read";
    artifact.case_profile="legacy_v5";artifact.bytes=1;
    EXPECT_THROW(OriginalSources::Prepare(paths,artifact),std::runtime_error);
    artifact.case_profile="native_v6_raw8_heph_explicit_cin28";
    Limits limits;limits.member_bytes=42846753;
    EXPECT_THROW(OriginalSources::Prepare(paths,artifact,limits),std::runtime_error);
    limits={};limits.host_bytes=1;
    EXPECT_THROW(OriginalSources::Prepare(paths,artifact,limits),std::runtime_error);
}
}
