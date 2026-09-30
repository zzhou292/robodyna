#include "SourceAssemblyWallFieldTestSupport.h"
#include "case/source_assembly_dynamics/NativeRotation.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::output::assembly::test {
namespace {
std::string Bytes(const Document& d) {rapidjson::StringBuffer b;rapidjson::Writer<rapidjson::StringBuffer> w(b);d.Accept(w);return {b.GetString(),b.GetSize()};}
}
TEST(SourceAssemblyRotationFields,ExplicitQualifiedDomainPreservesEveryLegacyFieldByte) {
    WallFields f;auto config=Configuration();const auto old_config=wall_fields::ConfigurationDocument(f.bindings,f.setup,config,f.surface,Request());
    EXPECT_FALSE(old_config.HasMember("rotation_domain"));config.rotation_domain=dynamics::RotationDomain::NativeShellGeometryV1;
    auto next_config=wall_fields::ConfigurationDocument(f.bindings,f.setup,config,f.surface,Request());
    const auto& domain=next_config["rotation_domain"];EXPECT_STREQ(domain["policy"].GetString(),dynamics::NativeRotationPolicy);
    EXPECT_STREQ(domain["native_qualification_commit"].GetString(),dynamics::NativeRotationQualification);
    EXPECT_EQ(domain["maximum_supported_rotation_rad"].GetDouble(),1.5);
    next_config.EraseMember("rotation_domain");EXPECT_EQ(Bytes(next_config),Bytes(old_config));
    const auto old_frame=wall_fields::FrameDocument(f.View());f.diagnostics.rotation_domain=config.rotation_domain;
    auto next_frame=wall_fields::FrameDocument(f.View());const auto& measured=next_frame["diagnostics"]["native_rotation_domain"];
    EXPECT_EQ(measured["maximum_frame_rotation_rad"].GetDouble(),0);
    EXPECT_EQ(measured["maximum_nodal_normal_rotation_rad"].GetDouble(),0);
    EXPECT_EQ(measured["maximum_rigid_member_rotation_rad"].GetDouble(),0);
    next_frame["diagnostics"].EraseMember("native_rotation_domain");EXPECT_EQ(Bytes(next_frame),Bytes(old_frame));
}
TEST(SourceAssemblyRotationFields,ActualQuaternionChannelRemainsSeparateAndMalformedDeclarationRejects) {
    WallFields f;f.Interval();f.diagnostics.rotation_domain=dynamics::RotationDomain::NativeShellGeometryV1;
    f.diagnostics.maximum_rotation=1.2;f.diagnostics.maximum_native_frame_rotation=.2;
    f.diagnostics.maximum_native_normal_rotation=.3;f.diagnostics.maximum_rigid_member_rotation=.1;
    const auto d=wall_fields::DiagnosticsDocument(f.diagnostics);
    EXPECT_EQ(d["maximum_rotation_rad"].GetDouble(),1.2);
    EXPECT_EQ(d["native_rotation_domain"]["maximum_frame_rotation_rad"].GetDouble(),.2);
    EXPECT_EQ(d["native_rotation_domain"]["maximum_nodal_normal_rotation_rad"].GetDouble(),.3);
    EXPECT_EQ(d["native_rotation_domain"]["maximum_rigid_member_rotation_rad"].GetDouble(),.1);
    for(double v:{-1.,1.5001,std::numeric_limits<double>::quiet_NaN()}) {
        auto malformed=f.diagnostics;malformed.maximum_native_normal_rotation=v;
        EXPECT_THROW(wall_fields::DiagnosticsDocument(malformed),std::runtime_error);
    }
    auto config=Configuration();config.rotation_domain=static_cast<dynamics::RotationDomain>(255);
    EXPECT_THROW(wall_fields::ConfigurationDocument(f.bindings,f.setup,config,f.surface,Request()),std::runtime_error);
    config.rotation_domain=dynamics::RotationDomain::NativeShellGeometryV1;config.deformation.maximum_rotation=1.5001;
    EXPECT_THROW(wall_fields::ConfigurationDocument(f.bindings,f.setup,config,f.surface,Request()),std::runtime_error);
}
}
