#include "ReissnerReferenceFixture.h"
#include "ReissnerShellSetup.h"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace crash::qualification {
namespace {
namespace tlr = tl::fea::reissner;
using reference::CopyReissnerShellSetup;

class ReissnerShellSetup : public ReissnerReference {};

class ConfiguredReference final : public ReissnerReference {
  public:
    explicit ConfiguredReference(const Frames& frames) : ReissnerReference(frames) {}
    void Initialize() { ReissnerReference::SetUp(); }
    void Copy(tlr::ShellReference& reference, tlr::ElasticSection& section) {
        CopyReissnerShellSetup(*element, reference, section);
    }
  private:
    void TestBody() override {}
};

// A deliberately unusable material history implementation. Admission must
// reject its presence, including when a null history would select elasticity.
class UnusedPlasticity final : public chrono::fea::ChPlasticityReissner {
  public:
    bool ComputeStressWithReturnMapping(Vec&, Vec&, Vec&, Vec&,
                                        chrono::fea::ChShellReissnerInternalData&,
                                        const Vec&, const Vec&, const Vec&, const Vec&,
                                        const chrono::fea::ChShellReissnerInternalData&,
                                        double, double, double) override {
        throw std::logic_error("Plasticity must not be evaluated by the elastic setup adapter");
    }
};

template <class Value>
std::array<unsigned char, sizeof(Value)> Bytes(const Value& value) {
    static_assert(std::is_trivially_copyable_v<Value>);
    std::array<unsigned char, sizeof(Value)> result;
    std::memcpy(result.data(), &value, sizeof(value));
    return result;
}

void ExpectVector(tlr::Vec3 actual, const Vec& expected) {
    EXPECT_DOUBLE_EQ(actual.x, expected.x());
    EXPECT_DOUBLE_EQ(actual.y, expected.y());
    EXPECT_DOUBLE_EQ(actual.z, expected.z());
}

void ExpectFrame(const tlr::Matrix3& actual, const chrono::ChMatrix33<>& expected) {
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            EXPECT_DOUBLE_EQ(actual.v[3 * row + col], expected(row, col));
}

TEST_F(ReissnerShellSetup, CopiesActualReferenceFieldsAndIndependentSectionEntries) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    ASSERT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
    ASSERT_TRUE(rest.prepared);
    ASSERT_TRUE(section.prepared);
    EXPECT_DOUBLE_EQ(section.thickness, kThickness);
    EXPECT_DOUBLE_EQ(section.density, 1000);
    EXPECT_DOUBLE_EQ(section.stiffness[0], kC);
    EXPECT_DOUBLE_EQ(section.stiffness[4], kNu * kC);
    EXPECT_NEAR(section.stiffness[12 * 2 + 2], (5.0 / 6.0) * kE * kThickness / (2 * (1 + kNu)), 1e-12);
    EXPECT_NEAR(section.stiffness[12 * 7 + 7], kD, 1e-16);
    EXPECT_NEAR(section.stiffness[12 * 7 + 9], -kNu * kD, 1e-16);
    for (unsigned p = 0; p < 4; ++p) {
        SCOPED_TRACE(p);
        ExpectVector(rest.initial_position[p], initial.x[p]);
        ExpectFrame(tlr::detail::Rotation(rest.node_frame_offset[p]), element->iTa[p]);
        ExpectFrame(rest.gauss[p].frame_offset, element->iTa_i[p]);
        ExpectFrame(rest.ans[p].frame_offset, element->iTa_A[p]);
        EXPECT_DOUBLE_EQ(rest.gauss[p].area_weight, element->alpha_i[p] * Element::w_i[p]);
        EXPECT_DOUBLE_EQ(rest.gauss[p].area_weight, kArea / 4);
        EXPECT_DOUBLE_EQ(rest.ans[p].area_weight, 0);
        ExpectVector(rest.gauss[p].strain0[0], element->eps_tilde_1_0_i[p]);
        ExpectVector(rest.gauss[p].strain0[1], element->eps_tilde_2_0_i[p]);
        ExpectVector(rest.gauss[p].curvature0[0], element->k_tilde_1_0_i[p]);
        ExpectVector(rest.gauss[p].curvature0[1], element->k_tilde_2_0_i[p]);
        ExpectVector(rest.ans[p].strain0[0], element->eps_tilde_1_0_A[p]);
        ExpectVector(rest.ans[p].strain0[1], element->eps_tilde_2_0_A[p]);
        Element::ShapeVector gauss_shape, ans_shape;
        element->ShapeFunctions(gauss_shape, Element::xi_i[p][0], Element::xi_i[p][1]);
        element->ShapeFunctions(ans_shape, Element::xi_A[p][0], Element::xi_A[p][1]);
        for (unsigned n = 0; n < 4; ++n) {
            EXPECT_DOUBLE_EQ(rest.gauss[p].shape[n], gauss_shape(n));
            EXPECT_DOUBLE_EQ(rest.ans[p].shape[n], ans_shape(n));
            for (unsigned axis = 0; axis < 2; ++axis) {
                EXPECT_DOUBLE_EQ(rest.gauss[p].gradient[n][axis], element->L_alpha_beta_i[p](n, axis));
                EXPECT_DOUBLE_EQ(rest.ans[p].gradient[n][axis], element->L_alpha_beta_A[p](n, axis));
            }
        }
    }
    ReferenceUnchanged();
}

TEST_F(ReissnerShellSetup, SectionMatrixMatchesActualStressAcrossMixedPhysicalScales) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    ASSERT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
    const auto material = element->GetLayer(0).GetMaterial();
    for (const double scale : {1e-5, .01, 1.0}) {
        const Vec strain[4] = {scale * Vec(.2, -.3, .7), scale * Vec(.4, -.8, -.2),
                               scale * Vec(-.1, .6, -.9), scale * Vec(.8, -.4, .3)};
        Vec stress[4];
        material->ComputeStress(stress[0], stress[1], stress[2], stress[3],
                                strain[0], strain[1], strain[2], strain[3],
                                -kThickness / 2, kThickness / 2, 0);
        for (unsigned row = 0; row < 12; ++row) {
            double predicted = 0;
            for (unsigned col = 0; col < 12; ++col)
                predicted += section.stiffness[12 * row + col] * strain[col / 3][col % 3];
            const double actual = stress[row / 3][row % 3];
            EXPECT_NEAR(predicted, actual, 1e-12 * std::max(1.0, std::abs(actual)));
        }
    }
}

TEST(ReissnerShellSetupAdmission, CommonInitialRotationAndDistinctNodalFramesAreAdmitted) {
    auto frames = Neutral();
    frames.q = {{Rotation(1.4, Vec(1, 0, 0)), Rotation(-1.6, Vec(1, 2, -1)),
                 Rotation(2.3, Vec(-2, .5, 1)), Rotation(-2.2, Vec(.4, -1, 2))}};
    const auto common = Rotation(2.8, Vec(-2, 1, .3));
    frames = Transform(frames, common, Vec(2.1, -1.3, .7));
    ConfiguredReference source(frames);
    source.Initialize();
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    ASSERT_NO_THROW(source.Copy(rest, section));
    const chrono::ChMatrix33<> expected(common);
    for (unsigned n = 0; n < 4; ++n) {
        ExpectVector(rest.initial_position[n], frames.x[n]);
        const auto physical = tlr::detail::Rotation(tlr::detail::Product(rest.initial_rotation[n], rest.node_frame_offset[n]));
        for (unsigned row = 0; row < 3; ++row)
            for (unsigned col = 0; col < 3; ++col)
                EXPECT_NEAR(physical.v[3 * row + col], expected(row, col), 3e-14);
    }
}

TEST(ReissnerShellSetupAdmission, MissingSetupIsRejectedBeforeReadingRestCaches) {
    Element element;
    const auto initial = Neutral();
    std::array<std::shared_ptr<Node>, 4> nodes;
    for (unsigned n = 0; n < 4; ++n)
        nodes[n] = chrono_types::make_shared<Node>(chrono::ChFrame<>(initial.x[n], initial.q[n]));
    element.SetNodes(nodes[0], nodes[1], nodes[2], nodes[3]);
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    const auto rest_bytes = Bytes(rest);
    const auto section_bytes = Bytes(section);
    EXPECT_THROW(CopyReissnerShellSetup(element, rest, section), std::invalid_argument);
    EXPECT_EQ(Bytes(rest), rest_bytes);
    EXPECT_EQ(Bytes(section), section_bytes);
}

TEST(ReissnerShellSetupAdmission, DistortedAndWarpedInitialGeometryAreRejected) {
    auto distorted = Neutral();
    distorted.x[0].x() += .1;
    auto warped = Neutral();
    warped.x[0].z() += .01;
    auto slender = Neutral();
    for (auto& position : slender.x)
        position.y() *= .01;
    for (const auto& frames : {distorted, warped, slender}) {
        ConfiguredReference source(frames);
        source.Initialize();
        ASSERT_FALSE(::testing::Test::HasFatalFailure());
        tlr::ShellReference rest;
        tlr::ElasticSection section;
        EXPECT_THROW(source.Copy(rest, section), std::invalid_argument);
        EXPECT_FALSE(rest.prepared);
        EXPECT_FALSE(section.prepared);
    }
}

TEST_F(ReissnerShellSetup, RejectsChangedCurrentFramesWithoutRecapturingReference) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    const auto rest_position = element->xa_0[0];
    nodes[0]->SetPos(nodes[0]->GetPos() + Vec(0, 0, .01));
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    EXPECT_EQ(element->xa_0[0], rest_position);
    nodes[0]->SetPos(initial.x[0]);
    nodes[0]->SetRot(Rotation(.2, Vec(1, 0, 0)));
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    nodes[0]->SetRot(initial.q[0]);
    EXPECT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
    ReferenceUnchanged();
}

TEST_F(ReissnerShellSetup, RejectsMultipleLayersSectionOffsetsAndNonfiniteAngle) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    element->SetLayerZreference(.25 * kThickness);
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    element->SetLayerZreferenceCentered();
    EXPECT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
    const auto material = element->GetLayer(0).GetMaterial();
    element->AddLayer(kThickness, 0, material);
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    element->m_layers.clear();
    element->AddLayer(kThickness, std::numeric_limits<double>::quiet_NaN(), material);
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    // A finite fiber angle has no effect for the exact admitted isotropic law.
    element->m_layers.clear();
    element->AddLayer(kThickness, .7, material);
    EXPECT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
}

TEST_F(ReissnerShellSetup, RejectsPlasticityAndDampingEvenWithNullHistory) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    const auto material = element->GetLayer(0).GetMaterial();
    material->SetPlasticity(chrono_types::make_shared<UnusedPlasticity>());
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    // Chrono's component setters require non-null objects. Replace the layer
    // material without repeating Setup; null setters dereference their input.
    const auto fresh_material = [this] {
        auto elasticity = chrono_types::make_shared<chrono::fea::ChElasticityReissnerIsothropic>(kE, kNu, 5.0 / 6.0, .01);
        auto result = chrono_types::make_shared<chrono::fea::ChMaterialShellReissner>(elasticity);
        result->SetDensity(1000);
        element->m_layers.clear();
        element->AddLayer(kThickness, 0, result);
        return result;
    };
    const auto damped = fresh_material();
    damped->SetDamping(chrono_types::make_shared<chrono::fea::ChDampingReissnerRayleigh>(damped->GetElasticity(), .01));
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    fresh_material();
    EXPECT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
}

TEST_F(ReissnerShellSetup, RejectsUnsupportedElasticityAndNonpositiveStiffness) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    const auto material = element->GetLayer(0).GetMaterial();
    const auto original = material->GetElasticity();
    material->SetElasticity(chrono_types::make_shared<chrono::fea::ChElasticityReissnerOrthotropic>(kE, kNu));
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    for (const double modulus : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
        material->SetElasticity(chrono_types::make_shared<chrono::fea::ChElasticityReissnerIsothropic>(modulus, kNu));
        EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    }
    material->SetElasticity(chrono_types::make_shared<chrono::fea::ChElasticityReissnerIsothropic>(kE, kNu, 1.0, 0.0));
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    material->SetElasticity(original);
    EXPECT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
}

TEST_F(ReissnerShellSetup, LateMaterialFailurePreservesBothOutputsAndAllowsCleanRetry) {
    tlr::ShellReference rest;
    tlr::ElasticSection section;
    ASSERT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
    const auto rest_bytes = Bytes(rest);
    const auto section_bytes = Bytes(section);
    const auto material = element->GetLayer(0).GetMaterial();
    material->SetDensity(std::numeric_limits<double>::quiet_NaN());
    EXPECT_THROW(CopyReissnerShellSetup(*element, rest, section), std::invalid_argument);
    EXPECT_EQ(Bytes(rest), rest_bytes);
    EXPECT_EQ(Bytes(section), section_bytes);
    material->SetDensity(1000);
    EXPECT_NO_THROW(CopyReissnerShellSetup(*element, rest, section));
    EXPECT_TRUE(rest.prepared);
    EXPECT_TRUE(section.prepared);
    ReferenceUnchanged();
}

}  // namespace
}  // namespace crash::qualification
