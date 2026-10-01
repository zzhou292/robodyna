#include "tests/body_compat/BodyFixture.h"
#include "chrono/physics/ChBodyEasy.h"
#include <stdexcept>

namespace robodyna::body_compat {
using chrono::make_ChNameValue;

void BodyGraph::ArchiveOut(chrono::ChArchiveOut& archive) {
    archive << CHNVP(body) << CHNVP(marker_alias) << CHNVP(force_alias);
}
void BodyGraph::ArchiveIn(chrono::ChArchiveIn& archive) {
    archive >> CHNVP(body) >> CHNVP(marker_alias) >> CHNVP(force_alias);
}
void Fixture::ArchiveOut(chrono::ChArchiveOut& archive) {
    archive << CHNVP(plain) << CHNVP(easy);
}
void Fixture::ArchiveIn(chrono::ChArchiveIn& archive) {
    archive >> CHNVP(plain) >> CHNVP(easy);
}

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void PopulateGraph(BodyGraph& graph, bool easy) {
    if (easy)
        graph.body = std::make_shared<chrono::ChBodyEasyBox>(2, 4, 6, 3, false, false);
    else
        graph.body = std::make_shared<chrono::ChBody>();
    graph.body->SetPos({3, -2, 1});
    graph.body->SetTag(easy ? 29 : 23);
    graph.body->SetName(easy ? "legacy easy box" : "legacy parent body");
    graph.marker_alias = std::make_shared<chrono::ChMarker>();
    graph.force_alias = std::make_shared<chrono::ChForce>();
    graph.marker_alias->SetName("retained marker");
    graph.force_alias->SetName("retained force");
    graph.body->AddMarker(graph.marker_alias);
    graph.body->AddForce(graph.force_alias);
    graph.marker_alias->ImposeRelativeTransform(chrono::ChFramed(chrono::ChVector3d(.25, .5, -.75)));
}

void CheckGraph(BodyGraph& graph, bool easy) {
    Require(bool(graph.body), "body archive lost its dynamic object");
    if (easy) {
        Require(dynamic_cast<chrono::ChBodyEasyBox*>(graph.body.get()) != nullptr,
                "BodyEasyBox archive constructor did not restore its type");
        Require(graph.body->GetMass() == 144 &&
                    (graph.body->GetInertiaXX() - chrono::ChVector3d(624, 480, 240)).Length() < 1e-12,
                "BodyEasyBox base mass/inertia changed");
    } else {
        Require(typeid(*graph.body) == typeid(chrono::ChBody), "plain body dynamic type changed");
    }
    Require(chrono::ChClassFactory::GetClassTagName(typeid(*graph.body)) == (easy ? "ChBodyEasyBox" : "ChBody"),
            "body factory wire tag changed");
    Require(graph.body->GetTag() == (easy ? 29 : 23) &&
                (graph.body->GetPos() - chrono::ChVector3d(3, -2, 1)).Length() < 1e-14,
            "body metadata or position changed");
    Require(graph.body->GetMarkers().size() == 1 && graph.body->GetForces().size() == 1,
            "body children were not restored");
    Require(graph.marker_alias && graph.force_alias &&
                graph.marker_alias.get() == graph.body->GetMarkers()[0].get() &&
                graph.force_alias.get() == graph.body->GetForces()[0].get(),
            "child aliases lost object identity");
    Require(graph.marker_alias->GetBody() == graph.body.get() && graph.force_alias->GetBody() == graph.body.get(),
            "marker/force parent rebinding changed");
    Require((graph.marker_alias->GetPos() - chrono::ChVector3d(.25, .5, -.75)).Length() < 1e-14,
            "marker relative position changed during archive rebinding");
    Require(!graph.marker_alias.owner_before(graph.body->GetMarkers()[0]) &&
                !graph.body->GetMarkers()[0].owner_before(graph.marker_alias) &&
                !graph.force_alias.owner_before(graph.body->GetForces()[0]) &&
                !graph.body->GetForces()[0].owner_before(graph.force_alias),
            "child aliases lost shared ownership");
    Require(graph.marker_alias->GetName() == "retained marker" && graph.force_alias->GetName() == "retained force",
            "child metadata changed");
}
}  // namespace

void Populate(Fixture& fixture) {
    PopulateGraph(fixture.plain, false);
    PopulateGraph(fixture.easy, true);
}
void Check(Fixture& fixture) {
    CheckGraph(fixture.plain, false);
    CheckGraph(fixture.easy, true);
}
}  // namespace robodyna::body_compat
