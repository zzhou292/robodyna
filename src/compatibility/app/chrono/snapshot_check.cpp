// Headless output-adapter checks. No solver, CUDA launch or dynamics step.
#include "AcceptedSurfaceMesh.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace crash::visual;
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
void Expect(Report actual, Status expected) {
    if (actual.status != expected)
        throw std::runtime_error(std::string("Unexpected adapter status: ") + actual.message);
}
using Positions = std::array<double, 18>;
Positions InitialPositions() {
    return {.05, 0, 0, .05, 1, 0, .05, 2, 0, .05, 0, 1, .05, 1, 1, .05, 2, 1};
}
tl::fea::HostNodalKinematicsView View(const Positions& p) { return {p.data(), nullptr, nullptr, 6}; }
Binding MakeBinding() {
    Binding b;
    b.identity = {17, 21, 31};
    b.tl_node_count = 6;
    const std::uint64_t ids[] = {9007199254741007ULL, 9007199254741003ULL, 9007199254741099ULL,
                                 9007199254741031ULL, 9007199254741021ULL, 9007199254741081ULL};
    // Two adjacent Q4s, with shared physical nodes duplicated only for display.
    for (std::uint32_t node : {4u, 1u, 0u, 3u, 5u, 2u, 1u, 4u})
        b.vertices.push_back({node, {1001, 0, ids[node]}});
    b.triangles = {{{2, 1, 0}, 1001, 0, 600001, 73, 0, 0},
                   {{2, 0, 3}, 1001, 0, 600001, 73, 0, 1},
                   {{6, 5, 4}, 1001, 0, 600007, 79, 0, 0},
                   {{6, 4, 7}, 1001, 0, 600007, 79, 0, 1}};
    return b;
}
FrameStamp Stamp(std::uint64_t epoch = 0, double time = 0) { return {{17, 21, 31}, epoch, time}; }
void Start(AcceptedSurfaceMesh& s, const Positions& p) {
    Expect(s.Initialize(MakeBinding()), Status::Ok);
    Expect(s.Publish(View(p), Stamp()), Status::Ok);
}
void CheckCoordinates(const AcceptedSurfaceMesh& s, const Positions& p) {
    Require(s.mesh() && s.frame(), "Published surface is missing");
    const auto& b = *s.binding();
    const auto& vertices = s.mesh()->GetCoordsVertices();
    Require(vertices.size() == b.vertices.size(), "Display vertex count changed");
    for (std::size_t i = 0; i < vertices.size(); ++i)
        for (int j = 0; j < 3; ++j)
            Require(vertices[i][j] == p[3 * b.vertices[i].tl_node + j], "Mapped double coordinate differs");
    const auto& faces = s.mesh()->GetIndicesVertices();
    Require(faces.size() == b.triangles.size(), "Display triangle count changed");
    for (std::size_t i = 0; i < faces.size(); ++i)
        for (int j = 0; j < 3; ++j)
            Require(faces[i][j] == static_cast<int>(b.triangles[i].vertices[j]), "Source winding/index changed");
}
struct Capture {
    FrameStamp stamp;
    std::vector<chrono::ChVector3d> vertices;
    std::vector<chrono::ChVector3i> indices;
};
Capture Save(const AcceptedSurfaceMesh& s) {
    return {*s.frame(), s.mesh()->GetCoordsVertices(), s.mesh()->GetIndicesVertices()};
}
void Preserved(const AcceptedSurfaceMesh& s, const Capture& old) {
    const auto& now = *s.frame();
    Require(now.identity.owner == old.stamp.identity.owner && now.identity.run == old.stamp.identity.run &&
                now.identity.topology == old.stamp.identity.topology && now.epoch == old.stamp.epoch &&
                now.time == old.stamp.time, "Rejected frame changed published metadata");
    Require(s.mesh()->GetCoordsVertices() == old.vertices, "Rejected frame changed published vertices");
    Require(s.mesh()->GetIndicesVertices() == old.indices, "Rejected frame changed published topology");
}

void InitialAndUpdatedFrames() {
    auto b = MakeBinding();
    auto p = InitialPositions();
    AcceptedSurfaceMesh s;
    Expect(s.Publish(View(p), Stamp()), Status::NotInitialized);
    Expect(s.Initialize(b), Status::Ok);
    Require(!s.frame() && !s.mesh() && !s.visual_shape(), "Unpublished mesh escaped initialization");
    // Binding and source metadata are owned copies, independent of input lifetime.
    b.vertices[0].source.node = 1;
    b.triangles[0].part = 1;
    b.triangles[0].vertices = {0, 0, 0};
    b.vertices.clear();
    Expect(s.Publish(View(p), Stamp()), Status::Ok);
    CheckCoordinates(s, p);
    Require(s.binding()->vertices[0].source.node == 9007199254741021ULL, "64-bit source ID lost precision");
    Require(s.binding()->triangles[0].part == 73, "Parent source metadata is borrowed");
    Require(s.binding()->vertices.size() == 8 && s.binding()->triangles[0].vertices[0] == 2,
            "Caller mutation changed the owned source topology");
    Require(s.mesh()->GetCoordsVertices()[0].x() != static_cast<double>(static_cast<float>(.05)),
            "Direct-double path unexpectedly rounded through float");
    auto shape = s.visual_shape();
    for (std::size_t node = 0; node < 6; ++node) p[3 * node] += 1e-9 * (node + 1);
    Expect(s.Publish(View(p), Stamp(2, .125)), Status::Ok); // output cadence may skip epochs.
    Require(s.visual_shape() == shape, "Frame update replaced the attached shape handle");
    CheckCoordinates(s, p);
    const auto saved = Save(s);
    p.fill(0); // No borrowed position pointer is retained after publication.
    Preserved(s, saved);
}

void SharedNodesAndSourceInstances() {
    auto b = MakeBinding();
    const auto p = InitialPositions();
    // Same original numeric ID in different includes is a distinct identity.
    b.vertices[3].source.node = b.vertices[2].source.node;
    b.vertices[3].source.instance = 9;
    AcceptedSurfaceMesh s;
    Expect(s.Initialize(b), Status::Ok);
    Expect(s.Publish(View(p), Stamp()), Status::Ok);
    CheckCoordinates(s, p);
    const auto& vertices = s.mesh()->GetCoordsVertices();
    Require(vertices[1] == vertices[6] && vertices[0] == vertices[7], "Shared TL node was duplicated physically");
    Require(s.binding()->triangles[1].element == 600001 && s.binding()->triangles[3].element == 600007,
            "Display triangles lost their parent elements");
    Require(s.binding()->triangles[0].local_face == s.binding()->triangles[1].local_face &&
                s.binding()->triangles[0].subtriangle == 0 && s.binding()->triangles[1].subtriangle == 1,
            "Display triangulation changed physical-face identity");
    for (unsigned i = 0; i < s.mesh()->GetNumTriangles(); ++i) {
        const auto triangle = s.mesh()->GetTriangle(i);
        chrono::ChVector3d normal;
        Require(triangle.Normal(normal) && normal.x() == 1, "Source triangle orientation changed");
    }
}

void InvalidBindingsAndCleanRetry() {
    const auto good = MakeBinding();
    const auto p = InitialPositions();
    auto rejects = [&](const Binding& b, Status expected = Status::InvalidBinding) {
        AcceptedSurfaceMesh s;
        Expect(s.Initialize(b), expected);
        Require(!s.binding() && !s.frame() && !s.mesh(), "Failed setup published partial state");
        Expect(s.Initialize(good), Status::Ok);
        Expect(s.Publish(View(p), Stamp()), Status::Ok);
        CheckCoordinates(s, p);
    };
    auto b = good; b.vertices[0].tl_node = 6; rejects(b);
    b = good; b.triangles[0].vertices[1] = 8; rejects(b);
    b = good; b.triangles[0].vertices[0] = 6; rejects(b); // aliases node of vertex 1.
    b = good; b.vertices[7].source.node += 1; rejects(b);
    b = good; b.vertices[2].source = b.vertices[3].source; rejects(b);
    b = good; b.triangles[1].subtriangle = 0; rejects(b);
    b = good; b.triangles[1].part = 74; rejects(b);
    b = good; b.identity.run = 0; rejects(b);
    b = good; b.tl_node_count = std::numeric_limits<std::size_t>::max(); rejects(b);
    b = good; b.max_vertices = 7; rejects(b, Status::ResourceLimit);
    b = good; b.max_triangles = 3; rejects(b, Status::ResourceLimit);
    b = good; b.max_vertices = 4097; rejects(b, Status::ResourceLimit);
}

void InvalidFramesPreservePublishedState() {
    const auto original = InitialPositions();
    AcceptedSurfaceMesh s;
    Start(s, original);
    const auto saved = Save(s);
    auto rejects = [&](tl::fea::HostNodalKinematicsView view, FrameStamp stamp = Stamp(1, .01)) {
        Expect(s.Publish(view, stamp), Status::InvalidFrame);
        Preserved(s, saved);
    };
    auto p = original; p[3 * 3] = std::numeric_limits<double>::quiet_NaN(); rejects(View(p));
    p = original; p[3 * 5 + 2] = std::numeric_limits<double>::infinity(); rejects(View(p));
    p = original; p.fill(0); rejects(View(p)); // finite degenerate geometry.
    p = original; p[0] = -std::numeric_limits<double>::max(); p[3] = std::numeric_limits<double>::max();
    rejects(View(p)); // finite positions whose subtraction overflows.
    auto missing = View(original); missing.position_xyz = nullptr; rejects(missing);
    auto wrong_count = View(original); wrong_count.node_count = 5; rejects(wrong_count);
    auto bad_time = Stamp(1, .01); bad_time.time = std::numeric_limits<double>::quiet_NaN(); rejects(View(original), bad_time);
    bad_time.time = -1; rejects(View(original), bad_time);
    Expect(s.Publish(View(original), Stamp(1, .01)), Status::Ok);
    CheckCoordinates(s, original);
    Require(s.frame()->epoch == 1 && s.frame()->time == .01, "Clean retry did not publish metadata");
}

void ProvenanceAndStaleFramesPreservePublishedState() {
    const auto p = InitialPositions();
    AcceptedSurfaceMesh s;
    Start(s, p);
    Expect(s.Publish(View(p), Stamp(3, .2)), Status::Ok);
    const auto saved = Save(s);
    auto rejects = [&](FrameStamp stamp, Status expected) {
        Expect(s.Publish(View(p), stamp), expected);
        Preserved(s, saved);
    };
    auto stamp = Stamp(4, .3); stamp.identity.owner++; rejects(stamp, Status::WrongOwner);
    stamp = Stamp(4, .3); stamp.identity.run++; rejects(stamp, Status::WrongRun);
    stamp = Stamp(4, .3); stamp.identity.topology++; rejects(stamp, Status::WrongTopology);
    rejects(Stamp(3, .3), Status::StaleFrame);
    rejects(Stamp(2, .3), Status::StaleFrame);
    rejects(Stamp(4, .2), Status::StaleFrame);
    rejects(Stamp(4, .1), Status::StaleFrame);
    Expect(s.Initialize(MakeBinding()), Status::InvalidBinding);
    Preserved(s, saved);
    Expect(s.Publish(View(p), Stamp(4, .3)), Status::Ok);
}

void FixedVisualCarrierUsesTheSameMesh() {
    const auto p = InitialPositions();
    AcceptedSurfaceMesh s;
    Start(s, p);
    auto body = std::make_shared<chrono::ChBody>();
    body->SetFixed(true);
    auto shape = s.visual_shape();
    body->AddVisualShape(shape);
    Require(body->IsFixed() && shape->IsMutable() && shape->IsFixedConnectivity(), "Invalid visual carrier properties");
    Require(shape->GetMesh().get() == s.mesh().get(), "Attached shape uses a different mesh");
    Require(body->GetPos() == chrono::ChVector3d(0, 0, 0), "Visual carrier applies an extra translation");
    auto updated = p;
    for (std::size_t node = 0; node < 6; ++node) updated[3 * node + 2] += .25;
    Expect(s.Publish(View(updated), Stamp(1, .1)), Status::Ok);
    CheckCoordinates(s, updated);
    Require(shape->GetMesh().get() == s.mesh().get(), "Update detached the visual shape");
}
}  // namespace

int main() {
    try {
        InitialAndUpdatedFrames();
        SharedNodesAndSourceInstances();
        InvalidBindingsAndCleanRetry();
        InvalidFramesPreservePublishedState();
        ProvenanceAndStaleFramesPreservePublishedState();
        FixedVisualCarrierUsesTheSameMesh();
        std::cout << "{\"status\":\"passed\",\"checks\":6,\"scope\":\"TL host view to Chrono core accepted-surface adapter\","
                     "\"gpu_used\":false,\"dynamics_stepped\":false,\"actual_TL_commit_bridge_qualified\":false}\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
