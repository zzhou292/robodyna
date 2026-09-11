#include "../GlassDeclarations.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "modelio/source_assembly/SourceShellReferenceInput.h"
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <gtest/gtest.h>

namespace crash::modelio::vehicle::test {
namespace {
output::Document Fixture() {
    const auto* path = std::getenv("ROBO_GLASS_FIELDS");
    output::Require(path && *path,"Missing explicit glass field fixture");
    const auto bytes = output::ReadBounded(path,128*1024);
    output::Document doc;
    doc.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    output::Require(!doc.HasParseError(),"Invalid glass field fixture");
    return doc;
}
const output::Value& Declaration(const output::Value& value) {
    return value["glass_declarations"]["parts"][0];
}
void Same(double a,double b) { EXPECT_EQ(output::Bits(a),output::Bits(b)); }
}
TEST(VehicleGlassFields, LiteralCardsAndResolvedRateFailureAreSeparate) {
    const auto doc = Fixture();
    const auto& value = doc["centered"];
    resolution::CheckGlassPolicy(value["glass_declarations"]);
    const auto glass = resolution::ReadGlassDeclaration(Declaration(value));
    const auto native = resolution::NativeGlassMaterial(glass.material);
    EXPECT_EQ(native.material_id,38);
    EXPECT_EQ(native.curve_id,0);
    EXPECT_EQ(native.hardening,tl::material::ShellPlasticityHardeningKind::LinearLaw44);
    EXPECT_EQ(native.rate.policy,tl::material::ShellPlasticityRatePolicy::FilteredZeroC);
    EXPECT_TRUE(native.rate.enabled);
    Same(native.rate.cowper_symonds_c_per_s,0);
    Same(native.rate.cowper_symonds_p,1);
    Same(native.rate.cutoff_hz,10000);
    Same(native.young_pa,70000*1e6);
    Same(native.density_kg_m3,2.5e-9*(1000/(.001*.001*.001)));
    Same(glass.failure_strain,.015);
    for (unsigned field : {0u,1u,4u}) EXPECT_FALSE(glass.material.cards[1].values[field]);
    assembly::Data legacy;
    legacy.schema = assembly::SectionInventorySchema;
    EXPECT_THROW(assembly::reader::ReadLaw44Material(Declaration(value)["material"],legacy),std::runtime_error);
    // A supported generic rate type cannot substitute for the explicit zero-C adapter.
    tl::material::TabulatedShellPlasticityParameters parameters;
    ASSERT_EQ(tl::material::PrepareLinearLaw44ShellPlasticity(native.young_pa,
        native.poisson_ratio,native.density_kg_m3,native.linear,native.rate,parameters),
        tl::material::TabulatedShellPlasticityStatus::Ok);
}
TEST(VehicleGlassFields, PlacementSignsKeepSourceCoordinatesAndNativeFamilyInertia) {
    const auto doc = Fixture();
    const assembly::SourceReferenceNode qnodes[4]{{1,{0,0,0}},{2,{.01,0,0}},
        {3,{.01,.008,0}},{4,{0,.008,0}}};
    const assembly::SourceReferenceNode tnodes[3]{qnodes[0],qnodes[1],qnodes[3]};
    double q_inertia[3]{},t_inertia[3]{};
    unsigned index = 0;
    for (const char* name : {"centered","bottom","top"}) {
        SCOPED_TRACE(name);
        const auto glass = resolution::ReadGlassDeclaration(Declaration(doc[name]));
        auto qinput = assembly::PackShellReference<tl::fea::qeph::ReferenceInput>(qnodes,glass.material,glass.section);
        auto tinput = assembly::PackShellReference<tl::fea::t3::ReferenceInput>(tnodes,glass.material,glass.section);
        qinput.placement = glass.placement;
        tinput.placement = glass.placement;
        tl::fea::qeph::ReferenceData q;
        tl::fea::t3::ReferenceData t;
        ASSERT_EQ(tl::fea::qeph::InitializeReference(qinput,q),tl::fea::qeph::Status::kSuccess);
        ASSERT_EQ(tl::fea::t3::InitializeReference(tinput,t),tl::fea::t3::Status::kSuccess);
        EXPECT_EQ(q.input.placement,glass.placement);
        EXPECT_EQ(t.input.placement,glass.placement);
        const auto expected = index == 0 ? tl::fea::ShellReferencePlacement::Centered :
            (index == 1 ? tl::fea::ShellReferencePlacement::BottomReferencePlane :
                          tl::fea::ShellReferencePlacement::TopReferencePlane);
        EXPECT_EQ(glass.placement,expected);
        for (unsigned n = 0; n < 4; ++n) Same(q.input.position[n].x,qnodes[n].position_m.x);
        for (unsigned n = 0; n < 3; ++n) Same(t.input.position[n].y,tnodes[n].position_m.y);
        q_inertia[index] = q.isotropic_inertia[0];
        t_inertia[index++] = t.element_isotropic_inertia;
    }
    EXPECT_GT(q_inertia[1],q_inertia[0]);
    Same(q_inertia[1],q_inertia[2]);
    Same(t_inertia[0],t_inertia[1]);
    Same(t_inertia[0],t_inertia[2]);
}
TEST(VehicleGlassFields, LateCardPolicyAndSignBitCorruptionRejectThenRetry) {
    for (unsigned fault = 0; fault < 12; ++fault) {
        SCOPED_TRACE(fault);
        auto doc = Fixture();
        auto& value = doc["top"]["glass_declarations"];
        auto& part = value["parts"][0];
        auto& m = part["material"];
        auto& s = part["section"];
        switch (fault) {
        case 0: m["source_numint"].SetDouble(2); break;
        case 1: m["failure_strain"].SetDouble(.03); break;
        case 2:
            m["young_pa"].SetDouble(35e9);
            m["cards"][0]["values"][2].SetDouble(35000);
            break;
        case 3: m["cards"][1]["values"][3].SetDouble(-0.0); break;
        case 4: s["source_nloc"].SetDouble(-1); break;
        case 5: s["placement"].SetString("bottom_reference_plane",doc.GetAllocator()); break;
        case 6: s["thickness_m"][3].SetDouble(0); break;
        case 7: part["part"]["material_id"].SetUint64(37); break;
        case 8: value["policy"]["resolved_IFAIL_SH"].SetUint(2); break;
        case 9: value["policy"]["resolved_VP"].SetUint(0); break;
        case 10: value["policy"]["contact_projection"].SetBool(true); break;
        case 11: m["cards"][3]["source_line"].SetUint64(m["cards"][2]["source_line"].GetUint64()); break;
        }
        EXPECT_THROW({
            resolution::CheckGlassPolicy(value);
            (void)resolution::ReadGlassDeclaration(part);
        },std::runtime_error);
    }
    const auto doc = Fixture();
    EXPECT_EQ(resolution::ReadGlassDeclaration(Declaration(doc["top"])).placement,
              tl::fea::ShellReferencePlacement::TopReferencePlane);
}
} // namespace crash::modelio::vehicle::test
