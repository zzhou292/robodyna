#include "chrono/ChConfig.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/physics/ChObject.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <sstream>
#include <string>

#ifndef CHRONO_FEA
#error "The standalone visual boundary must be tested with FEA enabled"
#endif

namespace {
using chrono::make_ChNameValue;
std::string texture_path;

class SceneObject : public chrono::ChObj {
  public:
    SceneObject* Clone() const override { return new SceneObject(*this); }
    chrono::ChFrame<> GetVisualModelFrame(unsigned int = 0) const override {
        return chrono::ChFrame<>(chrono::ChVector3d(10, 20, 30));
    }
};

class ObservedShape : public chrono::ChVisualShape {
  public:
    void Update(chrono::ChObj* owner, const chrono::ChFrame<>& frame) override {
        last_owner = owner;
        last_frame = frame;
        ++updates;
    }
    chrono::ChAABB GetBoundingBox() const override {
        return chrono::ChAABB({-1, -2, -3}, {1, 2, 3});
    }
    int updates = 0;
    chrono::ChObj* last_owner = nullptr;
    chrono::ChFrame<> last_frame;
};

class ObservedCamera : public chrono::ChCamera {
  public:
    void Update() override { ++updates; }
    int updates = 0;
};

void ExpectPoint(const chrono::ChVector3d& actual, const chrono::ChVector3d& expected) {
    EXPECT_DOUBLE_EQ(actual.x(), expected.x());
    EXPECT_DOUBLE_EQ(actual.y(), expected.y());
    EXPECT_DOUBLE_EQ(actual.z(), expected.z());
}

TEST(StandaloneVisualObject, UpdateFlagsPreserveOwnerFrameAndCameraDispatch) {
    SceneObject object;
    EXPECT_EQ(object.GetVisualShapeFEA(0), nullptr);  // no visual model yet
    auto shape = std::make_shared<ObservedShape>();
    auto camera = std::make_shared<ObservedCamera>();
    object.AddVisualShape(shape, chrono::ChFrame<>(chrono::ChVector3d(1, 2, 3)));
    object.AddCamera(camera);
    object.Update(.25, chrono::UpdateFlags::DYNAMICS);
    EXPECT_DOUBLE_EQ(object.GetChTime(), .25);
    EXPECT_EQ(shape->updates, 0);
    EXPECT_EQ(camera->updates, 0);
    object.Update(.5, chrono::UpdateFlags::VISUAL_ASSETS);
    EXPECT_EQ(shape->updates, 1);
    EXPECT_EQ(camera->updates, 1);
    EXPECT_EQ(shape->last_owner, &object);
    ExpectPoint(shape->last_frame.GetPos(), {11, 22, 33});
    EXPECT_EQ(object.GetVisualModel()->GetNumShapesFEA(), 0u);
}

TEST(StandaloneVisualObject, ModelCopyClearEraseAndBoundsNeedNoFeImplementation) {
    SceneObject first;
    auto shape = std::make_shared<ObservedShape>();
    first.AddVisualShape(shape, chrono::ChFrame<>(chrono::ChVector3d(2, 0, 0)));
    auto original = first.GetVisualModel();
    auto copy = std::make_shared<chrono::ChVisualModel>(*original);
    auto assigned = std::make_shared<chrono::ChVisualModel>();
    *assigned = *copy;
    SceneObject second;
    second.AddVisualModel(copy);
    second.UpdateVisualModel();
    EXPECT_EQ(shape->last_owner, &second);
    EXPECT_EQ(copy->GetShape(0), original->GetShape(0));
    const auto bounds = copy->GetBoundingBox();
    ExpectPoint(bounds.min, {1, -2, -3});
    ExpectPoint(bounds.max, {3, 2, 3});
    copy->Clear();
    EXPECT_EQ(copy->GetNumShapes(), 0u);
    EXPECT_EQ(copy->GetNumShapesFEA(), 0u);
    second.UpdateVisualModel();
    EXPECT_EQ(shape->updates, 1);
    EXPECT_EQ(original->GetNumShapes(), 1u);
    assigned->Erase(shape);
    EXPECT_EQ(assigned->GetNumShapes(), 0u);
    copy->AddShape(shape);
    second.UpdateVisualModel();
    EXPECT_EQ(shape->updates, 2);
}

TEST(StandaloneVisualObject, ObjectCopyPreservesHistoricalIdentityAndAttachmentRules) {
    SceneObject object;
    object.SetName("named-object");
    object.SetTag(17);
    object.SetChTime(.75);
    object.AddVisualShape(std::make_shared<ObservedShape>());
    object.AddCamera(std::make_shared<chrono::ChCamera>());
    std::unique_ptr<SceneObject> copy(object.Clone());
    EXPECT_NE(copy->GetIdentifier(), object.GetIdentifier());
    EXPECT_EQ(copy->GetName(), object.GetName());
    EXPECT_EQ(copy->GetTag(), -1);
    EXPECT_DOUBLE_EQ(copy->GetChTime(), .75);
    EXPECT_EQ(copy->GetVisualModel(), nullptr);
    EXPECT_TRUE(copy->GetCameras().empty());
}

TEST(StandaloneVisualObject, MaterialsKeepIndependentDefaultAndTextureValues) {
    auto first = std::make_shared<ObservedShape>();
    auto second = std::make_shared<ObservedShape>();
    const auto original_color = second->GetColor();
    first->SetColor({.25f, .5f, .75f});
    first->SetOpacity(.5f);
    first->SetTexture(texture_path, 2, 3);
    EXPECT_FLOAT_EQ(first->GetColor().R, .25f);
    EXPECT_FLOAT_EQ(first->GetOpacity(), .5f);
    EXPECT_EQ(first->GetTexture(), texture_path);
    EXPECT_FLOAT_EQ(first->GetMaterial(0)->GetTextureScale().x(), 2);
    EXPECT_FLOAT_EQ(first->GetMaterial(0)->GetTextureScale().y(), 3);
    EXPECT_FLOAT_EQ(second->GetColor().R, original_color.R);
    EXPECT_FLOAT_EQ(second->GetColor().G, original_color.G);
    EXPECT_FLOAT_EQ(second->GetColor().B, original_color.B);
    EXPECT_TRUE(second->GetMaterials().empty());
}

TEST(StandaloneVisualObject, FactoryOnlyVisualTypesRoundTripWithoutTheAggregate) {
    chrono::ChVisualModel* raw_model = nullptr;
    chrono::ChClassFactory::create("ChVisualModel", &raw_model);
    std::shared_ptr<chrono::ChVisualModel> model(raw_model);
    auto camera = std::make_shared<chrono::ChCamera>();
    camera->SetPosition({1, 2, 3});
    auto material = std::make_shared<chrono::ChVisualMaterial>();
    material->SetDiffuseColor({.125f, .25f, .5f});
    std::stringstream bytes;
    {
        chrono::ChArchiveOutJSON archive(bytes);
        archive << CHNVP(model) << CHNVP(camera) << CHNVP(material);
    }
    model.reset();
    camera.reset();
    material.reset();
    {
        chrono::ChArchiveInJSON archive(bytes);
        archive >> CHNVP(model) >> CHNVP(camera) >> CHNVP(material);
    }
    ASSERT_NE(model, nullptr);
    ASSERT_NE(camera, nullptr);
    ASSERT_NE(material, nullptr);
    EXPECT_EQ(model->GetNumShapes(), 0u);
    EXPECT_EQ(model->GetNumShapesFEA(), 0u);
    ExpectPoint(camera->GetPosition(), {1, 2, 3});
    EXPECT_FLOAT_EQ(material->GetDiffuseColor().G, .25f);
}
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2 || !std::filesystem::is_regular_file(argv[1]))
        return 2;
    texture_path = std::filesystem::absolute(argv[1]).string();
    return RUN_ALL_TESTS();
}
