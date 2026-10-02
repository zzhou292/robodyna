#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono_cascade/ChCascadeDoc.h"

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Version.hxx>
#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace {
TEST(NativeCad, RetainedBodyAdapterUsesActualCadMassAndInertia) {
    EXPECT_STREQ(OCC_VERSION_COMPLETE, "7.9.3");
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape();
    ASSERT_FALSE(shape.IsNull());
    chrono::cascade::ChBodyEasyCascade body(shape, 7.0, false, false);
    EXPECT_NEAR(body.GetMass(), 42.0, 1e-10);
    const auto inertia = body.GetInertiaXX();
    EXPECT_NEAR(inertia.x(), 45.5, 1e-9);
    EXPECT_NEAR(inertia.y(), 35.0, 1e-9);
    EXPECT_NEAR(inertia.z(), 17.5, 1e-9);
    const auto center = body.GetFrameCOMToRef().GetPos();
    EXPECT_NEAR(center.x(), .5, 1e-12);
    EXPECT_NEAR(center.y(), 1.0, 1e-12);
    EXPECT_NEAR(center.z(), 1.5, 1e-12);
}

TEST(NativeCad, RealStepWriterAndRetainedDocumentReaderPreserveVolume) {
    const char* temporary = std::getenv("TEST_TMPDIR");
    ASSERT_NE(temporary, nullptr);
    const auto path = std::filesystem::path(temporary) / "native-box.stp";
    ASSERT_FALSE(std::filesystem::exists(path));
    // The retained STEP reader explicitly selects meters. Write a metric CAD
    // box expressed in millimeters, and verify the real unit conversion.
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(1000.0, 2000.0, 3000.0).Shape();
    STEPControl_Writer writer;
    ASSERT_TRUE(Interface_Static::SetCVal("xstep.cascade.unit", "MM"));
    ASSERT_TRUE(Interface_Static::SetCVal("write.step.unit", "MM"));
    ASSERT_EQ(writer.Transfer(shape, STEPControl_AsIs), IFSelect_RetDone);
    ASSERT_EQ(writer.Write(path.c_str()), IFSelect_RetDone);
    chrono::cascade::ChCascadeDoc document;
    ASSERT_TRUE(document.LoadSTEP(path.c_str()));
    TopoDS_Shape restored;
    ASSERT_TRUE(document.GetRootShape(restored));
    ASSERT_FALSE(restored.IsNull());
    GProp_GProps properties;
    BRepGProp::VolumeProperties(restored, properties);
    EXPECT_NEAR(properties.Mass(), 6.0, 1e-9);
    EXPECT_NEAR(properties.CentreOfMass().X(), .5, 1e-12);
    EXPECT_NEAR(properties.CentreOfMass().Y(), 1.0, 1e-12);
    EXPECT_NEAR(properties.CentreOfMass().Z(), 1.5, 1e-12);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    const auto resources = std::filesystem::absolute(argv[1]).parent_path().parent_path();
    // Match OCCT's declared resource directories without changing global host
    // installation or relying on an unrelated system CAD package.
    for (const char* key : {"CSF_PluginDefaults", "CSF_StandardDefaults", "CSF_XCAFDefaults"})
        setenv(key, (resources / "StdResource").c_str(), 1);
    for (const char* key : {"CSF_STEPDefaults", "CSF_IGESDefaults"})
        setenv(key, (resources / "XSTEPResource").c_str(), 1);
    setenv("CSF_XSMessage", (resources / "XSMessage").c_str(), 1);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
