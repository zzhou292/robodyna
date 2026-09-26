#include "Fixture.h"
#include "modelio/source_assembly/JsonReader.h"
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
TEST(MainShellContext, IndependentModifiersDisableOnlyTheGroupingCertificate) {
    for (const char* keyword : {"*MAT_ADD_EROSION", "*MAT_ADD_THERMAL_EXPANSION", "*CONTROL_ADAPTIVE", "*PART_SENSOR"}) {
        output::Document doc;
        const std::string json=std::string("{\"member\":{\"keyword_counts\":{\"")+keyword+"\":1}}}";
        doc.Parse(json.c_str());
        ASSERT_FALSE(doc.HasParseError());
        EXPECT_FALSE(detail::GroupingKeywords(doc));
    }
    output::Document allowed;
    allowed.Parse(R"({"member":{"keyword_counts":{"*MAT_ELASTIC":2,"*PART":2,"*CONTROL_TIMESTEP":1}}})");
    EXPECT_TRUE(detail::GroupingKeywords(allowed));
}
TEST(MainShellContext, OptionalImsclCardCannotBorrowTheGlobalNoAmsProof) {
    modelio::tied_shell::SourceEvidence source;
    source.cards = {{2,"         0       .67         0         0    -1e-06         0         0         0"}};
    EXPECT_TRUE(detail::NoOptionalAmsCard({source}));
    source.cards.push_back({3,"         0         0        12"});
    EXPECT_FALSE(detail::NoOptionalAmsCard({source}));
    source.cards.back().second = "         0         0         0";
    EXPECT_FALSE(detail::NoOptionalAmsCard({source})); // Optional0 is deliberately not inferred.
    source.cards.back().second.clear();
    EXPECT_TRUE(detail::NoOptionalAmsCard({source}));
}
TEST(MainShellContext, OrdinaryPartTailIsExplicitSourceEvidence) {
    output::Document part;
    part.Parse(R"({"raw_fields":[1,2,3,0,0,0,0,0],"blank_field_mask":248})");
    EXPECT_TRUE(detail::OrdinaryPartControls(part));
    part["raw_fields"][4].SetInt(12);
    EXPECT_FALSE(detail::OrdinaryPartControls(part));
    part["raw_fields"][4].SetInt(0);
    part["blank_field_mask"].SetInt(232); // Supplied HGID is outside the blank-tail proof even if zero.
    EXPECT_FALSE(detail::OrdinaryPartControls(part));
}
TEST(MainShellContext, DigestsBindContactIdentityMechanicalOwnerAndPhase) {
    Provenance p;
    p.input_digest=p.topology_digest=p.property_digest=p.material_digest=p.import_digest=std::string(64,'a');
    Certificate c;
    std::vector<PrimaryBinding> bindings(1);
    bindings[0].contact_element=1; bindings[0].support_element=2; bindings[0].partner=2;
    std::vector<CandidateOwner> owners{{2,20,1},{3,30,2}};
    std::vector<double> values{10,20};
    const auto hash=detail::Digest(p,c,bindings,owners,values,1u<<20);
    auto copied=bindings;
    EXPECT_EQ(hash,detail::Digest(p,c,copied,owners,values,1u<<20));
    copied[0].support_element=3;
    EXPECT_NE(hash,detail::Digest(p,c,copied,owners,values,1u<<20));
    copied=bindings; copied[0].role=coated::RoleState::CoatingReversed;
    EXPECT_NE(hash,detail::Digest(p,c,copied,owners,values,1u<<20));
    values[1]=std::nextafter(20.,21.);
    EXPECT_NE(hash,detail::Digest(p,c,bindings,owners,values,1u<<20));
    EXPECT_THROW(detail::Digest(p,c,bindings,owners,values,1), std::exception);
}
}
