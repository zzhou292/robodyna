#include "../OriginalSources.h"
#include "../MemberStorage.h"
#include "output/full_shell/tests/TestSupport.h"
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
TEST(NativeV6SourceAdmission, CompactAuthenticatedStoragePreservesBytesAndChargesReaderCopies) {
    output::full_shell::test::Directory directory;
    const auto path=directory.path/"source.key";
    std::string bytes(8193,'x');bytes[17]='\0';bytes[4096]='\r';
    output::WriteBytes(path,bytes);
    auto grown=output::ReadBounded(path,bytes.size());
    ASSERT_GT(grown.capacity(),grown.size());
    const auto prior=grown.capacity()+1;
    const auto compact=detail::CompactMember(std::move(grown),bytes.size());
    EXPECT_EQ(compact,bytes);EXPECT_EQ(output::Sha256(compact),output::Sha256(bytes));
    EXPECT_LE(compact.capacity()+1,bytes.size()+64);
    EXPECT_GE(detail::ReadAndCompactPeak(64u<<20,bytes.size()),(64u<<20)+prior+compact.capacity()+1);
}

}
