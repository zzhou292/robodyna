#include "tests/serialization_compat/FixtureModel.h"

#include <cmath>
#include <stdexcept>

namespace robodyna::serialization_compat {
namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void CheckVector(const chrono::ChVector3d& actual, const chrono::ChVector3d& expected, const char* message) {
    Require(std::isfinite(actual.x()) && std::isfinite(actual.y()) && std::isfinite(actual.z()) &&
                (actual - expected).Length() < 1e-14,
            message);
}
}  // namespace

void FixtureModel::ArchiveOut(chrono::ChArchiveOut& archive) {
    archive << CHNVP(vector) << CHNVP(frame) << CHNVP(plain_body) << CHNVP(body) << CHNVP(alias)
            << CHNVP(frame_alias) << CHNVP(item_alias) << CHNVP(null_body);
}

void FixtureModel::ArchiveIn(chrono::ChArchiveIn& archive) {
    archive >> CHNVP(vector) >> CHNVP(frame) >> CHNVP(plain_body) >> CHNVP(body) >> CHNVP(alias)
            >> CHNVP(frame_alias) >> CHNVP(item_alias) >> CHNVP(null_body);
}

void PopulateFixture(FixtureModel& fixture) {
    fixture.vector = {1.25, -2.5, 0.125};
    fixture.frame = chrono::ChFramed({-1, 4, 0.5}, chrono::ChQuaterniond(0.5, 0.5, 0.5, 0.5));
    fixture.plain_body.SetName("plain legacy body");
    fixture.plain_body.SetTag(17);
    fixture.plain_body.SetMass(7.25);
    fixture.plain_body.SetInertiaXX({2, 3, 4});
    fixture.plain_body.SetPos({2.5, -1, 4});
    fixture.plain_body.SetFixed(true);

    auto body = std::make_shared<chrono::ChBodyAuxRef>();
    body->SetName("shared auxiliary body");
    body->SetTag(23);
    body->SetMass(12.5);
    body->SetInertiaXX({3, 4, 5});
    body->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.25, -0.5, 0.75)));
    body->SetFrameRefToAbs(chrono::ChFramed(chrono::ChVector3d(4, 5, 6)));
    body->SetPosDt({0.125, -0.25, 0.5});
    body->SetAngVelLocal({0.25, -0.125, 0.5});
    body->SetChTime(0.125);
    fixture.body = body;
    fixture.alias = body;
    fixture.frame_alias = body;
    fixture.item_alias = body;
    fixture.null_body.reset();
}

void CheckFixture(FixtureModel& fixture) {
    CheckVector(fixture.vector, {1.25, -2.5, 0.125}, "vector value changed");
    CheckVector(fixture.frame.GetPos(), {-1, 4, 0.5}, "frame position changed");
    const auto& rotation = fixture.frame.GetRot();
    Require(rotation.e0() == 0.5 && rotation.e1() == 0.5 && rotation.e2() == 0.5 && rotation.e3() == 0.5,
            "frame quaternion changed");
    Require(fixture.plain_body.GetName() == "plain legacy body" && fixture.plain_body.GetTag() == 17,
            "plain body metadata changed");
    Require(fixture.plain_body.GetMass() == 7.25 && fixture.plain_body.IsFixed(), "plain body properties changed");
    CheckVector(fixture.plain_body.GetInertiaXX(), {2, 3, 4}, "plain body inertia changed");
    CheckVector(fixture.plain_body.GetPos(), {2.5, -1, 4}, "plain body position changed");
    const auto body = std::dynamic_pointer_cast<chrono::ChBodyAuxRef>(fixture.body);
    Require(bool(body), "factory did not preserve ChBodyAuxRef dynamic type");
    Require(body->GetName() == "shared auxiliary body" && body->GetTag() == 23, "derived body metadata changed");
    Require(body->GetMass() == 12.5 && !body->IsFixed() && body->GetChTime() == 0.125, "derived body values changed");
    CheckVector(body->GetInertiaXX(), {3, 4, 5}, "derived body inertia changed");
    CheckVector(body->GetFrameCOMToRef().GetPos(), {0.25, -0.5, 0.75}, "auxiliary COM frame changed");
    CheckVector(body->GetFrameRefToAbs().GetPos(), {4, 5, 6}, "auxiliary reference frame changed");
    CheckVector(body->GetPosDt(), {0.125, -0.25, 0.5}, "linear velocity changed");
    CheckVector(body->GetAngVelLocal(), {0.25, -0.125, 0.5}, "angular velocity changed");
    Require(fixture.alias.get() == fixture.body.get(), "repeated pointer identity changed");
    Require(fixture.frame_alias.get() == static_cast<chrono::ChBodyFrame*>(body.get()), "frame base pointer not adjusted");
    Require(fixture.item_alias.get() == static_cast<chrono::ChPhysicsItem*>(body.get()), "item base pointer changed");
    Require(!fixture.body.owner_before(fixture.frame_alias) && !fixture.frame_alias.owner_before(fixture.body) &&
                !fixture.body.owner_before(fixture.alias) && !fixture.alias.owner_before(fixture.body) &&
                !fixture.body.owner_before(fixture.item_alias) && !fixture.item_alias.owner_before(fixture.body),
            "shared pointers do not retain one ownership control block");
    Require(!fixture.null_body, "null pointer changed");
}
}  // namespace robodyna::serialization_compat
