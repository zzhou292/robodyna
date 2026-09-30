#include "CoverageFixtureCapture.h"

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/FixedContactFacetValues.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace contact = tlfea::contact;
namespace sct = tlfea::contact::self_contact_transaction;
EndpointGeometry DiscoverDirect(
    const std::array<contact::CurrentFixedTriangle, 2>& triangles,
    contact::FixedTriangleFeatureTaskMask mask) {
    std::array<contact::FixedTriangleFeatureCandidate, 15> features;
    contact::fixed_triangle_features::PairFeatureResult feature_result;
    if (contact::fixed_triangle_features::
            EvaluatePairFeaturesMaskedOnce(
                triangles[0], triangles[1], mask,
                features.data(), features.size(),
                &feature_result) !=
        contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(
            "Direct coupon feature evaluation failed");
    EndpointGeometry result;
    result.features.assign(
        features.begin(),
        features.begin() + feature_result.feature_count);
    contact::FixedTriangleIntersection intersection;
    bool intersects = false;
    if (contact::fixed_triangle_features::
            ClassifyPairIntersection(
                triangles[0], triangles[1],
                &intersection, &intersects) !=
        contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(
            "Direct coupon intersection evaluation failed");
    if (intersects) result.intersections.push_back(intersection);
    return result;
}

double MinimumDistance(const EndpointGeometry& geometry) {
    double result = std::numeric_limits<double>::infinity();
    for (const auto& feature : geometry.features) {
        result = std::min(result, feature.distance_m);
    }
    return result;
}

std::size_t ExpectedFeatureCount(
    contact::FixedTriangleFeatureTaskMask mask) {
    std::size_t result = 15;
    for (unsigned task = 0; task < 15; ++task)
        result -= bool(
            mask.local_tasks &
            contact::FixedTriangleFeatureTaskBit(task));
    return result;
}

bool StrictlySeparated(
    const EndpointGeometry& geometry,
    contact::FixedTriangleFeatureTaskMask mask,
    double thickness) {
    if (geometry.features.size() != ExpectedFeatureCount(mask))
        return false;
    for (const auto& feature : geometry.features)
        if (!((feature.distance_m - thickness) >
              feature.representation_error_m))
            return false;
    return true;
}

bool StrictlyWithinThickness(
    const EndpointGeometry& geometry, double thickness) {
    for (const auto& feature : geometry.features)
        if (feature.distance_m + feature.representation_error_m <
            thickness)
            return true;
    return false;
}

bool HasLocalIntersection(const EndpointGeometry& geometry) {
    for (const auto& intersection : geometry.intersections)
        if (!contact::RequiresIntersectionAdmission(intersection))
            return true;
    return false;
}

bool HasAdmittedIntersection(const EndpointGeometry& geometry) {
    for (const auto& intersection : geometry.intersections)
        if (contact::RequiresIntersectionAdmission(intersection))
            return true;
    return false;
}

bool HasVertex(
    const contact::CurrentFixedTriangle& triangle,
    const contact::FacetVertexKey& key) {
    for (const auto& candidate : triangle.vertex_keys)
        if (contact::fixed_triangle_features::Compare(
                candidate, key) == 0)
            return true;
    return false;
}

bool HasEdge(
    const contact::CurrentFixedTriangle& triangle,
    const contact::FacetEdgeKey& key) {
    for (const auto& candidate : triangle.edge_keys)
        if (contact::fixed_triangle_features::Compare(
                candidate, key) == 0)
            return true;
    return false;
}

bool CouldOwnQuadraticPair(
    const std::array<contact::CurrentFixedTriangle, 2>& triangles,
    const sct::AcceptedEventCertificate& certificate) {
    const auto& feature = certificate.discovery.key;
    if (feature.kind ==
        contact::FixedTriangleCandidateKind::VertexFace) {
        if (feature.vertex_face.target.kind !=
            contact::FixedTriangleStratumKind::Face)
            return false;
        unsigned target = 2;
        for (unsigned side = 0; side < 2; ++side)
            if (contact::fixed_triangle_features::Compare(
                    triangles[side].key,
                    feature.vertex_face.target.face) == 0)
                target = target == 2 ? side : 3;
        return target < 2 &&
            HasVertex(
                triangles[1 - target],
                feature.vertex_face.vertex) &&
            !HasVertex(
                triangles[target],
                feature.vertex_face.vertex);
    }
    unsigned sides[2]{2, 2};
    for (unsigned edge = 0; edge < 2; ++edge) {
        for (unsigned side = 0; side < 2; ++side)
            if (HasEdge(
                    triangles[side],
                    feature.edge_edge.edges[edge]))
                sides[edge] = sides[edge] == 2 ? side : 3;
        if (sides[edge] >= 2) return false;
    }
    return sides[0] != sides[1];
}

bool CouldOwnFacetPairConservatively(
    const std::array<contact::CurrentFixedTriangle, 2>& triangles,
    const sct::AcceptedEventCertificate& certificate) {
    const auto& feature = certificate.discovery.key;
    for (unsigned target = 0; target < 2; ++target) {
        if (feature.kind == contact::FixedTriangleCandidateKind::EdgeEdge) {
            if (HasEdge(triangles[target], feature.edge_edge.edges[0]) &&
                HasEdge(triangles[1 - target], feature.edge_edge.edges[1]))
                return true;
            continue;
        }
        const auto& vf = feature.vertex_face;
        if (!HasVertex(triangles[1 - target], vf.vertex)) continue;
        switch (vf.target.kind) {
            case contact::FixedTriangleStratumKind::Face:
                if (contact::fixed_triangle_features::Compare(
                        triangles[target].key, vf.target.face) == 0) return true;
                break;
            case contact::FixedTriangleStratumKind::Edge:
                if (HasEdge(triangles[target], vf.target.edge)) return true;
                break;
            case contact::FixedTriangleStratumKind::Vertex:
                if (HasVertex(triangles[target], vf.target.vertex)) return true;
                break;
        }
    }
    return false;
}


}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
