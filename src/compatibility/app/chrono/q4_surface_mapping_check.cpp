#include "lib_src/collision/Q4SurfaceMapping.h"
#include "chrono/core/ChFrame.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>

namespace crash::qualification {
namespace {
namespace sc = tlfea::contact;
using Element = chrono::fea::ChElementShellReissner4;
using Node = chrono::fea::ChNodeFEAxyzrot;
using Vec = chrono::ChVector3d;
constexpr double kTolerance = 2e-13;

// C1 prescribed midsurface mapping only. These actual Chrono public mapping
// methods read shape values/nodal kinematics and need no shell Setup/material.
// No wall law, area Jacobian, quadrature, contact activation or dynamics is
// qualified here. In particular, ComputeNF's approximate detJ is not an oracle
// for C2/C3 surface integration.
struct MappingFixture {
    Element element;
    std::array<std::shared_ptr<Node>,4> nodes;
    std::array<double,18> positions{}, velocities{};
    sc::SurfaceQ4 parent{{4,1,5,2},73,42,2,0};

    MappingFixture() {
        const std::array<Vec,4> x{{{.12,.08,.015},{-.09,.11,-.025},{-.13,-.07,.02},{.1,-.09,-.01}}};
        const std::array<Vec,4> v{{{.3,-.4,.9},{-.7,.2,.5},{.1,.8,-.6},{-.5,-.3,.4}}};
        for (unsigned n = 0; n < 4; ++n) {
            nodes[n] = std::make_shared<Node>(chrono::ChFrame<>(x[n]));
            nodes[n]->SetPosDt(v[n]);
            const double half_angle = .13*(n+1), s = std::sin(half_angle)/std::sqrt(6.);
            nodes[n]->SetRot({std::cos(half_angle),s,-s,2*s});
            nodes[n]->SetAngVelParent(Vec(.7,-.2*(n+1),.4));
        }
        element.SetNodes(nodes[0],nodes[1],nodes[2],nodes[3]);
        CopyFields();
    }
    void CopyFields() {
        for (unsigned local = 0; local < 4; ++local) {
            const auto global = parent.nodes[local];
            for (unsigned axis = 0; axis < 3; ++axis) {
                positions[global+6*axis] = nodes[local]->GetPos()[axis];
                velocities[3*global+axis] = nodes[local]->GetPosDt()[axis];
            }
        }
    }
    sc::Q4SurfaceView view() const { return {{positions.data(),6,1,6},{velocities.data(),6,3,1},&parent,1}; }
};

Vec Chrono(sc::Vec3 value) { return {value.x,value.y,value.z}; }
void Near(sc::Vec3 actual, const Vec& expected) {
    for (unsigned axis = 0; axis < 3; ++axis)
        EXPECT_NEAR(Chrono(actual)[axis],expected[axis],kTolerance*(1+std::abs(expected[axis])));
}

TEST(Q4SurfaceMappingChrono, NaturalOrderAndAsymmetricShapesMatchActualElement) {
    Element element;
    // Read actual donor corner coordinates, then independent non-corner points.
    for (unsigned corner = 0; corner < 4; ++corner) {
        const auto* uv = Element::xi_n[corner];
        Element::ShapeVector expected; element.ShapeFunctions(expected,uv[0],uv[1]);
        double actual[4]; ASSERT_EQ(sc::EvaluateQ4Shape(uv[0],uv[1],actual),sc::Status::kOk);
        for (unsigned n = 0; n < 4; ++n) {
            EXPECT_DOUBLE_EQ(actual[n],expected(n)); EXPECT_DOUBLE_EQ(actual[n],n == corner ? 1 : 0);
        }
    }
    for (double u : {-1.,-.73,0.,.2,.81,1.}) for (double v : {-1.,-.41,0.,.17,.6,1.}) {
        SCOPED_TRACE(::testing::Message() << "u=" << u << ", v=" << v);
        Element::ShapeVector expected; element.ShapeFunctions(expected,u,v);
        double actual[4]; ASSERT_EQ(sc::EvaluateQ4Shape(u,v,actual),sc::Status::kOk);
        for (unsigned n = 0; n < 4; ++n) EXPECT_DOUBLE_EQ(actual[n],expected(n));
    }
}

TEST(Q4SurfaceMappingChrono, PositionsAndVelocitiesUseActualChronoMidsurfaceInterpolation) {
    MappingFixture fixture;
    for (double u : {-1.,-.73,0.,.2,.81,1.}) for (double v : {-1.,-.41,.17,.6,1.}) {
        SCOPED_TRACE(::testing::Message() << "u=" << u << ", v=" << v);
        sc::Q4PointKinematics actual;
        ASSERT_EQ(sc::EvaluateQ4Point(fixture.view(),{0,u,v},&actual),sc::Status::kOk);
        Vec position, velocity; // Chrono EvaluateSectionVelNorm ADDS into its zero input.
        fixture.element.EvaluateSectionPoint(u,v,position);
        fixture.element.EvaluateSectionVelNorm(u,v,velocity);
        Near(actual.position,position); Near(actual.velocity,velocity);
        Element::ShapeVector shape; fixture.element.ShapeFunctions(shape,u,v);
        for (unsigned n = 0; n < 4; ++n) EXPECT_DOUBLE_EQ(actual.shape[n],shape(n));
    }
    EXPECT_EQ(fixture.parent.feature_id,73u); EXPECT_EQ(fixture.parent.parent_element_id,42u);
    EXPECT_EQ(fixture.parent.parent_face_id,2u);
}

TEST(Q4SurfaceMappingChrono, TransposeForcesPreserveChronoPointWorkResultantAndMoment) {
    const sc::Vec3 force{3,-4,9};
    const std::array<Vec,4> displacement{{{.003,-.001,.005},{0,.006,-.002},{-.004,.002,.001},{.001,-.007,.004}}};
    for (const auto& uv : std::array<std::array<double,2>,4>{{{1,1},{-.73,.17},{.2,-.4},{0,0}}}) {
        MappingFixture fixture;
        Element::ShapeVector shape; fixture.element.ShapeFunctions(shape,uv[0],uv[1]);
        sc::Q4NodalForces projected;
        ASSERT_EQ(sc::ProjectQ4PointForce(fixture.view(),{0,uv[0],uv[1]},force,&projected),sc::Status::kOk);
        Vec resultant, moment, point, velocity;
        fixture.element.EvaluateSectionPoint(uv[0],uv[1],point);
        fixture.element.EvaluateSectionVelNorm(uv[0],uv[1],velocity);
        double power = 0, work = 0;
        for (unsigned n = 0; n < 4; ++n) {
            EXPECT_EQ(projected.nodes[n],fixture.parent.nodes[n]);
            Near(projected.forces[n],shape(n)*Chrono(force)); Near(projected.couples[n],Vec(0,0,0));
            const auto nodal = Chrono(projected.forces[n]);
            resultant += nodal; moment += fixture.nodes[n]->GetPos().Cross(nodal);
            power += fixture.nodes[n]->GetPosDt().Dot(nodal);
            work += displacement[n].Dot(nodal);
            fixture.nodes[n]->SetPos(fixture.nodes[n]->GetPos()+displacement[n]);
        }
        EXPECT_NEAR((resultant-Chrono(force)).Length(),0,kTolerance);
        EXPECT_NEAR((moment-point.Cross(Chrono(force))).Length(),0,kTolerance);
        EXPECT_NEAR(power,velocity.Dot(Chrono(force)),kTolerance*(1+std::abs(power)));
        Vec moved; fixture.element.EvaluateSectionPoint(uv[0],uv[1],moved);
        EXPECT_NEAR(work,Chrono(force).Dot(moved-point),kTolerance);
    }
}
}  // namespace
}  // namespace crash::qualification
