#include "FamilySupport.h"
#include "tests/archive_support/StreamArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include <iomanip>

namespace robodyna::body_family_test {
TEST(BodyFamilyConstructors, AuxiliaryDefaultCopyCloneAndFrameUpdates) {
    auto body = std::make_shared<chrono::ChBodyAuxRef>();
    body->SetMass(7.25);
    body->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(.25,-.5,.75)));
    body->SetFrameRefToAbs(chrono::ChFramed(chrono::ChVector3d(4,5,6)));
    body->SetPosDt({.125,-.25,.5});
    ExpectIdentity(body,"ChBodyAuxRef");
    EXPECT_LT((body->GetPos()-chrono::ChVector3d(4.25,4.5,6.75)).Length(),1e-14);
    auto copy = std::make_shared<chrono::ChBodyAuxRef>(*body);
    std::unique_ptr<chrono::ChBodyAuxRef> clone(body->Clone());
    for (auto* value : {copy.get(),clone.get()}) {
        EXPECT_NE(value,body.get());
        EXPECT_DOUBLE_EQ(value->GetMass(),7.25);
        EXPECT_EQ(value->GetPos(),body->GetPos());
        EXPECT_EQ(value->GetPosDt(),body->GetPosDt());
        EXPECT_EQ(value->GetFrameCOMToRef().GetPos(),body->GetFrameCOMToRef().GetPos());
        EXPECT_EQ(value->GetFrameRefToAbs().GetPos(),body->GetFrameRefToAbs().GetPos());
    }
}

template <class Writer,class Reader>
void RoundTrip(std::shared_ptr<chrono::ChBody> source) {
    const auto expected_tag = chrono::ChClassFactory::GetClassTagName(typeid(*source));
    const auto bytes = archive_test::Write<Writer>(source);
    ASSERT_LT(bytes.size(),1024u*1024u);
    std::shared_ptr<chrono::ChBody> restored;
    ASSERT_NO_THROW(archive_test::Read<Reader>(bytes,restored));
    ASSERT_NE(restored,nullptr);
    EXPECT_EQ(chrono::ChClassFactory::GetClassTagName(typeid(*restored)),expected_tag);
    EXPECT_DOUBLE_EQ(restored->GetMass(),source->GetMass());
    EXPECT_LT((restored->GetInertia()-source->GetInertia()).norm(),1e-12);
    EXPECT_EQ(restored->GetPos(),source->GetPos());
    if (auto* expected = dynamic_cast<chrono::ChBodyAuxRef*>(source.get())) {
        auto* actual = dynamic_cast<chrono::ChBodyAuxRef*>(restored.get());
        ASSERT_NE(actual,nullptr);
        // The archive stores reference-to-COM coordinates/quaternion, while the
        // opposite transform below is recomputed from its cached rotation matrix.
        EXPECT_EQ(actual->GetFrameRefToCOM().GetPos(),expected->GetFrameRefToCOM().GetPos());
        EXPECT_EQ(actual->GetFrameRefToCOM().GetRot(),expected->GetFrameRefToCOM().GetRot());
        const auto actual_pos=actual->GetFrameCOMToRef().GetPos();
        const auto expected_pos=expected->GetFrameCOMToRef().GetPos();
        // Pre-rename JSON/XML/binary all differed by 4 ULP in x and 1 ULP in z
        // when rebuilding the cached matrix from the exactly retained quaternion.
        for (unsigned axis=0;axis<3;++axis)
            EXPECT_DOUBLE_EQ(actual_pos[axis],expected_pos[axis]) << std::setprecision(17)
                << "reader=" << typeid(Reader).name() << " actual=" << actual_pos
                << " expected=" << expected_pos << " delta=" << (actual_pos-expected_pos);
    }
}

TEST(BodyFamilyArchives, SupportedRepresentedStateSurvivesThreeFormats) {
    const std::vector<chrono::ChVector3d> positions{{-2,0,0},{2,0,0}};
    const std::vector<double> radii{1,1};
    std::vector<std::shared_ptr<chrono::ChBody>> bodies{
        std::make_shared<chrono::ChBodyEasySphere>(1,3,false,false),
        std::make_shared<chrono::ChBodyEasyEllipsoid>(chrono::ChVector3d(2,4,6),3,false,false),
        std::make_shared<chrono::ChBodyEasyCylinder>(chrono::ChAxis::Z,1,2,3,false,false),
        std::make_shared<chrono::ChBodyEasyBox>(2,4,6,3,false,false),
        std::make_shared<chrono::ChBodyEasyClusterOfSpheres>(positions,radii,3,false,false),
        std::make_shared<chrono::ChBodyEasyMesh>(BoxMesh(),3,true,false,false),
    };
    // No hull restart claim: the original hull constructor archives write
    // m_mesh but read mesh in text formats. Preserve that separate limitation.
    for (const auto& body : bodies) {
        SCOPED_TRACE(chrono::ChClassFactory::GetClassTagName(typeid(*body)));
        RoundTrip<chrono::ChArchiveOutJSON,chrono::ChArchiveInJSON>(body);
        RoundTrip<chrono::ChArchiveOutXML,chrono::ChArchiveInXML>(body);
        RoundTrip<chrono::ChArchiveOutBinary,chrono::ChArchiveInBinary>(body);
    }
}
}  // namespace robodyna::body_family_test
