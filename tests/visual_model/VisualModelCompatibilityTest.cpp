// Baseline behavior for the generic visual-model / FEA adapter separation.
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <sstream>
#include <vector>

#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/serialization/ChArchiveJSON.h"

namespace {
class TransformedMesh : public chrono::fea::ChMesh {
  public:
    chrono::ChFrame<> GetVisualModelFrame(unsigned int = 0) const override {
        return chrono::ChFrame<>(chrono::ChVector3d(10, 20, 30));
    }
};

struct MeshVisual {
    std::shared_ptr<TransformedMesh> mesh = std::make_shared<TransformedMesh>();
    std::shared_ptr<chrono::fea::ChNodeFEAxyz> node;
    std::shared_ptr<chrono::ChVisualShapeFEA> visual = std::make_shared<chrono::ChVisualShapeFEA>();

    explicit MeshVisual(const chrono::ChVector3d& position) {
        node = std::make_shared<chrono::fea::ChNodeFEAxyz>(position);
        mesh->AddNode(node);
        visual->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::NONE);
        visual->SetFEMglyphType(chrono::ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        visual->SetDefaultSymbolsColor({0.25f, 0.5f, 0.75f});
        mesh->AddVisualShapeFEA(visual);
    }

    std::shared_ptr<chrono::ChGlyphs> Glyphs() const {
        return std::dynamic_pointer_cast<chrono::ChGlyphs>(mesh->GetVisualModel()->GetShape(1));
    }
};

class OrdinaryObserver : public chrono::ChVisualShape {
  public:
    explicit OrdinaryObserver(std::shared_ptr<chrono::ChGlyphs> glyphs) : glyphs_(std::move(glyphs)) {}
    void Update(chrono::ChObj* owner, const chrono::ChFrame<>& frame) override {
        observed_points = glyphs_->points;
        observed_owner = owner;
        observed_frame = frame;
        ++updates;
    }
    int updates = 0;
    chrono::ChObj* observed_owner = nullptr;
    chrono::ChFrame<> observed_frame;
    std::vector<chrono::ChVector3d> observed_points;
  private:
    std::shared_ptr<chrono::ChGlyphs> glyphs_;
};

void ExpectPoint(const chrono::ChVector3d& value, const chrono::ChVector3d& expected) {
    EXPECT_DOUBLE_EQ(value.x(), expected.x());
    EXPECT_DOUBLE_EQ(value.y(), expected.y());
    EXPECT_DOUBLE_EQ(value.z(), expected.z());
}

TEST(VisualModelCompatibility, UpdatesOrdinaryShapesBeforeFeaAndPreservesWorldCoordinates) {
    MeshVisual fixture({1, 2, 3});
    auto model = fixture.mesh->GetVisualModel();
    ASSERT_EQ(model->GetNumShapes(), 2u);
    ASSERT_EQ(model->GetNumShapesFEA(), 1u);
    EXPECT_EQ(fixture.mesh->GetVisualShapeFEA(0), fixture.visual);
    auto glyphs = fixture.Glyphs();
    ASSERT_NE(glyphs, nullptr);
    auto observer = std::make_shared<OrdinaryObserver>(glyphs);
    fixture.mesh->AddVisualShape(observer);
    fixture.mesh->UpdateVisualModel();
    EXPECT_EQ(observer->updates, 1);
    EXPECT_TRUE(observer->observed_points.empty());
    EXPECT_EQ(observer->observed_owner, fixture.mesh.get());
    ExpectPoint(observer->observed_frame.GetPos(), {10, 20, 30});
    ASSERT_EQ(glyphs->points.size(), 1u);
    ExpectPoint(glyphs->points[0], {1, 2, 3});
    ASSERT_NE(glyphs->colors, nullptr);
    ASSERT_EQ(glyphs->colors->size(), 1u);
    EXPECT_FLOAT_EQ((*glyphs->colors)[0].R, 0.25f);
    EXPECT_FLOAT_EQ((*glyphs->colors)[0].G, 0.5f);
    EXPECT_FLOAT_EQ((*glyphs->colors)[0].B, 0.75f);

    fixture.node->SetPos({4, 5, 6});
    fixture.mesh->UpdateVisualModel();
    ASSERT_EQ(observer->observed_points.size(), 1u);
    ExpectPoint(observer->observed_points[0], {1, 2, 3});
    ExpectPoint(glyphs->points[0], {4, 5, 6});
}

TEST(VisualModelCompatibility, CopyAndAssignmentRetainSharedFeaObjectsAndStoredOwner) {
    MeshVisual fixture({1, 2, 3});
    auto model = fixture.mesh->GetVisualModel();
    auto copied = std::make_shared<chrono::ChVisualModel>(*model);
    auto assigned = std::make_shared<chrono::ChVisualModel>();
    *assigned = *model;
    chrono::fea::ChMesh other;
    other.AddNode(std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(40, 50, 60)));
    for (const auto& copy : {copied, assigned}) {
        ASSERT_EQ(copy->GetNumShapesFEA(), 1u);
        EXPECT_EQ(copy->GetShapeFEA(0), fixture.visual);
        EXPECT_EQ(copy->GetShape(1), model->GetShape(1));
        other.AddVisualModel(copy);
        other.UpdateVisualModel();
        // An FE visual retains its original mesh owner when only the model is shared.
        ASSERT_EQ(fixture.Glyphs()->points.size(), 1u);
        ExpectPoint(fixture.Glyphs()->points[0], {1, 2, 3});
    }
    copied->Clear();
    EXPECT_EQ(copied->GetNumShapes(), 0u);
    EXPECT_EQ(copied->GetNumShapesFEA(), 0u);
    EXPECT_EQ(model->GetNumShapes(), 2u);
    EXPECT_EQ(assigned->GetNumShapesFEA(), 1u);
}

TEST(VisualModelCompatibility, ClearAndReattachUseTheNewExplicitFeaOwner) {
    MeshVisual fixture({1, 2, 3});
    auto model = fixture.mesh->GetVisualModel();
    auto glyphs = fixture.Glyphs();
    fixture.mesh->UpdateVisualModel();
    model->Clear();
    fixture.node->SetPos({7, 8, 9});
    fixture.mesh->UpdateVisualModel();
    ASSERT_EQ(glyphs->points.size(), 1u);
    ExpectPoint(glyphs->points[0], {1, 2, 3});
    fixture.mesh->AddVisualShapeFEA(fixture.visual);
    fixture.mesh->UpdateVisualModel();
    ASSERT_EQ(model->GetNumShapes(), 2u);
    ExpectPoint(glyphs->points[0], {7, 8, 9});

    chrono::fea::ChMesh other;
    other.AddNode(std::make_shared<chrono::fea::ChNodeFEAxyz>(chrono::ChVector3d(-1, -2, -3)));
    other.AddVisualShapeFEA(fixture.visual);
    fixture.mesh->UpdateVisualModel();
    // Explicit reattachment changes the FE visual's owner even in a shared old model.
    ExpectPoint(glyphs->points[0], {-1, -2, -3});
}

TEST(VisualModelCompatibility, ArchiveRetainsOrdinaryShapesWithoutInventingFeaRestart) {
    MeshVisual fixture({1, 2, 3});
    auto model = fixture.mesh->GetVisualModel();
    // Freeze supported attachment/archive behavior before glyph properties are
    // populated. Their existing clone/registry limitation is documented separately.
    std::stringstream bytes;
    {
        chrono::ChArchiveOutJSON archive(bytes);
        archive << CHNVP(model);
    }
    EXPECT_EQ(bytes.str().find("m_shapesFEA"), std::string::npos);
    std::shared_ptr<chrono::ChVisualModel> restored;
    {
        chrono::ChArchiveInJSON archive(bytes);
        archive >> CHNVP(restored, "model");
    }
    ASSERT_NE(restored, nullptr);
    EXPECT_EQ(restored->GetNumShapes(), model->GetNumShapes());
    EXPECT_EQ(restored->GetNumShapesFEA(), 0u);
    EXPECT_NE(std::dynamic_pointer_cast<chrono::ChGlyphs>(restored->GetShape(1)), nullptr);
}

TEST(VisualModelCompatibility, PopulatedVisualArchiveStillOmitsFeaAttachments) {
    MeshVisual fixture({1, 2, 3});
    fixture.mesh->UpdateVisualModel();
    auto model = fixture.mesh->GetVisualModel();
    std::stringstream bytes;
    {
        chrono::ChArchiveOutJSON archive(bytes);
        archive << CHNVP(model);
    }
    EXPECT_NE(bytes.str().find("m_shapes"), std::string::npos);
    EXPECT_EQ(bytes.str().find("m_shapesFEA"), std::string::npos);
    // No read/restart claim: populated glyph properties hit the inherited issue
    // in docs/migration/PREEXISTING_ISSUES.md. Rendering itself is tested above.
}
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2 || !std::filesystem::is_regular_file(argv[1]))
        return 2;
    chrono::SetChronoDataPath(std::filesystem::path(argv[1]).parent_path().parent_path().string() + "/");
    return RUN_ALL_TESTS();
}
