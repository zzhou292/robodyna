#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/SelfContactActiveUseTypes.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include "output/full_shell/static_bundle/MappingArrays.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test {
namespace {

namespace contact = tlfea::contact;
namespace vehicle = modelio::vehicle;
namespace source = output::full_shell::source;

constexpr std::uint64_t AnchorEid = 2100084;
constexpr std::uint64_t StrictEid = 2279821;
constexpr std::uint64_t BoundaryEid = 2288735;
constexpr std::uint64_t IntersectionFirstEid = 2125365;
constexpr std::uint64_t IntersectionSecondEid = 2348922;
constexpr std::uint64_t IntersectionSharedNode = 2352112;
constexpr std::uint16_t IntersectionLocalTaskMask = 0x6c30;
constexpr std::array<std::uint64_t, 4> IntersectionFirstNodes{
    2120447, 2352113, 2352112, 2352118};
constexpr std::array<std::uint64_t, 4> IntersectionSecondNodes{
    2352127, 2352111, 2352112, 2352113};

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <class T>
std::vector<T> Decode(const source::CanonicalData& data, const char* name) {
    const auto& array = source::FindArray(data, name);
    return output::arrays::Decode<T>(array.descriptor, array.bytes);
}

std::uint64_t DigestPrefix(const std::string& digest) {
    Require(digest.size() == 64, "Canonical archive digest is malformed");
    std::uint64_t value = 0;
    for (unsigned i = 0; i < 16; ++i) {
        const char digit = digest[i];
        const unsigned nibble =
            digit >= '0' && digit <= '9' ? unsigned(digit - '0') :
            digit >= 'a' && digit <= 'f' ? unsigned(digit - 'a' + 10) :
            16;
        Require(nibble < 16, "Canonical archive digest is not hexadecimal");
        value = (value << 4) | nibble;
    }
    Require(value != 0, "Canonical archive identity cannot be zero");
    return value;
}

bool Same(const contact::FacetVertexKey& a,
          const contact::FacetVertexKey& b) {
    return std::tie(a.source_instance_id, a.kind, a.first, a.second,
                    a.numerator, a.denominator, a.level, a.grid_i,
                    a.grid_j) ==
           std::tie(b.source_instance_id, b.kind, b.first, b.second,
                    b.numerator, b.denominator, b.level, b.grid_i,
                    b.grid_j);
}

bool Same(const contact::FacetEdgeKey& a,
          const contact::FacetEdgeKey& b) {
    return a.parent_eid == b.parent_eid &&
           a.parent_boundary == b.parent_boundary &&
           Same(a.endpoints[0], b.endpoints[0]) &&
           Same(a.endpoints[1], b.endpoints[1]);
}

bool SameEdgePair(const contact::FixedTriangleFeatureCandidate& value,
                  const contact::FacetEdgeKey& a,
                  const contact::FacetEdgeKey& b) {
    if (value.key.kind != contact::FixedTriangleCandidateKind::EdgeEdge)
        return false;
    const auto& edges = value.key.edge_edge.edges;
    return (Same(edges[0], a) && Same(edges[1], b)) ||
           (Same(edges[0], b) && Same(edges[1], a));
}

struct Parent {
    std::size_t canonical = SIZE_MAX;
    std::uint64_t eid = 0, pid = 0, mid = 0, sid = 0;
    unsigned arity = 0, material_points = 0;
    double thickness_m = 0;
    std::array<std::uint64_t, 4> node_ids{};
    std::array<contact::Vec3, 4> positions{};
};

class AuthenticatedGeometry {
  public:
    AuthenticatedGeometry()
        : plan_(vehicle::test::Plan()),
          data_(plan_.canonical().data()),
          records_(Decode<std::uint64_t>(data_, "shells_records")),
          connections_(Decode<std::uint32_t>(
              data_, "shells_node_indices")),
          node_ids_(Decode<std::uint64_t>(data_, "node_ids")),
          positions_(Decode<double>(data_, "node_positions")),
          plan_parent_(data_.canonical_shells, SIZE_MAX),
          source_instance_(DigestPrefix(data_.archive_sha256)) {
        Require(records_.size() == 6 * data_.canonical_shells &&
                    connections_.size() == 4 * data_.canonical_shells &&
                    node_ids_.size() == data_.canonical_nodes &&
                    positions_.size() == 3 * data_.canonical_nodes,
                "Authenticated canonical geometry extents differ");
        for (std::size_t i = 0; i < plan_.parents().size(); ++i) {
            const auto canonical = plan_.parents()[i].canonical_parent;
            Require(canonical < plan_parent_.size() &&
                        plan_parent_[canonical] == SIZE_MAX,
                    "Vehicle source plan repeats a canonical parent");
            plan_parent_[canonical] = i;
        }
        for (const auto eid :
             {AnchorEid, StrictEid, BoundaryEid, IntersectionFirstEid,
              IntersectionSecondEid})
            Require(FindCanonical(eid) != SIZE_MAX,
                    "Required real-Yaris parent is absent");
    }

    Parent Read(std::uint64_t eid) const {
        const auto canonical = FindCanonical(eid);
        Require(canonical != SIZE_MAX,
                "Requested real-Yaris parent is absent");
        return ReadCanonical(canonical);
    }

    std::vector<Parent> Neighborhood(
        const std::vector<contact::FacetEdgeKey>& edges) const {
        std::vector<Parent> result;
        for (std::size_t p = 0; p < plan_.parents().size(); ++p) {
            const auto canonical = plan_.parents()[p].canonical_parent;
            const auto eid = records_[6 * canonical];
            const bool triangle =
                records_[6 * canonical + 4] ==
                records_[6 * canonical + 5];
            bool selected = false;
            for (const auto& edge : edges) {
                if (!edge.parent_boundary) {
                    selected = selected || edge.parent_eid == eid;
                    continue;
                }
                const auto first = edge.endpoints[0].first;
                const auto second = edge.endpoints[1].first;
                const unsigned arity = triangle ? 3 : 4;
                for (unsigned local = 0; local < arity; ++local) {
                    const auto a = records_[6 * canonical + 2 + local];
                    const auto b =
                        records_[6 * canonical + 2 +
                                 ((local + 1) % arity)];
                    selected = selected ||
                        ((a == first && b == second) ||
                         (a == second && b == first));
                }
            }
            if (selected) result.push_back(ReadCanonical(canonical));
        }
        Require(!result.empty(),
                "Real-Yaris incident neighborhood is empty");
        Require(result.size() < 128,
                "Real-Yaris coupon expanded beyond adjacent parents");
        return result;
    }

    const source::CanonicalData& data() const noexcept { return data_; }
    const vehicle::VehicleSourcePlan& plan() const noexcept { return plan_; }
    std::uint64_t source_instance() const noexcept {
        return source_instance_;
    }

  private:
    std::size_t FindCanonical(std::uint64_t eid) const {
        std::size_t found = SIZE_MAX;
        for (std::size_t canonical = 0;
             canonical < data_.canonical_shells; ++canonical) {
            if (records_[6 * canonical] != eid) continue;
            Require(found == SIZE_MAX,
                    "Canonical source repeats a requested EID");
            found = canonical;
        }
        return found;
    }

    Parent ReadCanonical(std::size_t canonical) const {
        Require(canonical < plan_parent_.size() &&
                    plan_parent_[canonical] != SIZE_MAX,
                "Coupon parent is outside the authenticated retained plan");
        const auto& mapped =
            plan_.parents().at(plan_parent_[canonical]);
        const auto& part = plan_.parts().at(mapped.part_index);
        const auto* material = plan_.material(mapped.part_index);
        const auto* section = plan_.section(mapped.part_index);
        Require(material && section,
                "Coupon parent lacks an authenticated typed declaration");
        Require(part.part_id == records_[6 * canonical + 1] &&
                    part.material_id == material->id &&
                    part.section_id == section->id,
                "Coupon parent declaration identity differs");
        Parent result;
        result.canonical = canonical;
        result.eid = records_[6 * canonical];
        result.pid = part.part_id;
        result.mid = material->id;
        result.sid = section->id;
        result.arity = records_[6 * canonical + 4] ==
                               records_[6 * canonical + 5]
                           ? 3
                           : 4;
        result.material_points = section->through_thickness_points;
        result.thickness_m = section->thickness_m[0];
        Require(result.thickness_m > 0,
                "Coupon parent thickness is not positive");
        for (unsigned local = 0; local < result.arity; ++local) {
            const auto node = connections_[4 * canonical + local];
            Require(node < node_ids_.size() &&
                        records_[6 * canonical + 2 + local] ==
                            node_ids_[node],
                    "Coupon node association differs");
            result.node_ids[local] = node_ids_[node];
            result.positions[local] = {
                positions_[3 * node], positions_[3 * node + 1],
                positions_[3 * node + 2]};
        }
        return result;
    }

    const vehicle::VehicleSourcePlan& plan_;
    const source::CanonicalData& data_;
    std::vector<std::uint64_t> records_;
    std::vector<std::uint32_t> connections_;
    std::vector<std::uint64_t> node_ids_;
    std::vector<double> positions_;
    std::vector<std::size_t> plan_parent_;
    std::uint64_t source_instance_ = 0;
};

const AuthenticatedGeometry& Authority() {
    static const AuthenticatedGeometry value;
    return value;
}

struct GeometrySet {
    std::vector<Parent> parents;
    std::map<std::uint64_t, std::uint32_t> node_index;
    std::vector<double> positions;
    std::vector<contact::FixedContactFacet> facets;

    explicit GeometrySet(std::vector<Parent> input)
        : parents(std::move(input)) {
        for (const auto& parent : parents)
            for (unsigned local = 0; local < parent.arity; ++local)
                node_index.emplace(parent.node_ids[local], 0);
        std::uint32_t next = 0;
        for (auto& entry : node_index) entry.second = next++;
        positions.assign(3 * node_index.size(), 0);
        std::vector<unsigned char> written(node_index.size(), 0);
        for (const auto& parent : parents) {
            for (unsigned local = 0; local < parent.arity; ++local) {
                const auto node = node_index.at(parent.node_ids[local]);
                const auto point = parent.positions[local];
                if (written[node]) {
                    Require(positions[3 * node] == point.x &&
                                positions[3 * node + 1] == point.y &&
                                positions[3 * node + 2] == point.z,
                            "Shared canonical node position differs");
                } else {
                    positions[3 * node] = point.x;
                    positions[3 * node + 1] = point.y;
                    positions[3 * node + 2] = point.z;
                    written[node] = 1;
                }
            }
        }
        for (std::size_t parent = 0; parent < parents.size(); ++parent) {
            const unsigned count = parents[parent].arity == 4 ? 2 : 1;
            for (unsigned local = 0; local < count; ++local)
                facets.push_back(MakeFacet(parent, local));
        }
    }

    contact::VectorView view() const noexcept {
        return {positions.data(),
                static_cast<std::uint32_t>(node_index.size()), 3, 1};
    }

    contact::FixedContactFacet Facet(std::uint64_t eid,
                                     unsigned local) const {
        for (const auto& facet : facets)
            if (facet.source.source_parent_id == eid &&
                facet.local_facet == local)
                return facet;
        throw std::runtime_error("Requested coupon facet is absent");
    }

    contact::CurrentFixedTriangle Evaluate(
        const contact::FixedContactFacet& facet) const {
        contact::CurrentFixedTriangle result;
        Require(contact::EvaluateCurrentFixedTriangle(
                    facet, view(), &result) == contact::Status::kOk,
                "FixedContactFacet current evaluation failed");
        return result;
    }

    double ParentArea(std::size_t index) const {
        const auto& parent = parents.at(index);
        const auto triangle_area = [&](unsigned a, unsigned b, unsigned c) {
            const auto u = contact::Subtract(
                parent.positions[b], parent.positions[a]);
            const auto v = contact::Subtract(
                parent.positions[c], parent.positions[a]);
            const contact::Vec3 cross{
                u.y * v.z - u.z * v.y,
                u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x};
            return .5 * std::sqrt(contact::Dot(cross, cross));
        };
        const double result = parent.arity == 4
            ? triangle_area(0, 1, 2) + triangle_area(0, 2, 3)
            : triangle_area(0, 1, 2);
        Require(std::isfinite(result) && result > 0,
                "Coupon parent area is not positive finite");
        return result;
    }

  private:
    static bool VertexLess(const contact::FacetVertexKey& a,
                           const contact::FacetVertexKey& b) {
        return std::tie(a.source_instance_id, a.kind, a.first, a.second,
                        a.numerator, a.denominator, a.level, a.grid_i,
                        a.grid_j) <
               std::tie(b.source_instance_id, b.kind, b.first, b.second,
                        b.numerator, b.denominator, b.level, b.grid_i,
                        b.grid_j);
    }

    contact::FixedContactFacet MakeFacet(std::size_t parent_index,
                                         unsigned local_facet) const {
        const auto& parent = parents.at(parent_index);
        const unsigned local[2][3]{{0, 1, 2}, {0, 2, 3}};
        Require(local_facet < (parent.arity == 4 ? 2u : 1u),
                "Coupon local facet is out of range");
        contact::FixedContactFacet result;
        result.source.family = parent.arity == 4
            ? tl::fea::ShellBindingFamily::Qeph
            : tl::fea::ShellBindingFamily::T3;
        result.source.family_index = parent.canonical;
        result.source.source_parent_id = parent.eid;
        result.source.source_part_id = parent.pid;
        result.source.material_id = parent.mid;
        result.source.section_id = parent.sid;
        result.source_instance_id = Authority().source_instance();
        result.parent_index = parent_index;
        result.local_facet = local_facet;
        result.material_points = parent.material_points;
        result.reference_half_thickness_m = .5 * parent.thickness_m;
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            const unsigned corner =
                parent.arity == 4 ? local[local_facet][vertex] : vertex;
            auto& point = result.vertices[vertex];
            point.count = parent.arity;
            for (unsigned slot = 0; slot < parent.arity; ++slot) {
                point.nodes[slot] =
                    node_index.at(parent.node_ids[slot]);
                point.weights[slot] = slot == corner ? 1 : 0;
            }
            auto& key = result.vertex_keys[vertex];
            key.source_instance_id = result.source_instance_id;
            key.first = parent.node_ids[corner];
        }
        for (unsigned edge = 0; edge < 3; ++edge) {
            const unsigned next = (edge + 1) % 3;
            const auto& a = result.vertex_keys[edge];
            const auto& b = result.vertex_keys[next];
            auto& key = result.edge_keys[edge];
            key.endpoints[0] = VertexLess(a, b) ? a : b;
            key.endpoints[1] = VertexLess(a, b) ? b : a;
            unsigned corner_a =
                parent.arity == 4 ? local[local_facet][edge] : edge;
            unsigned corner_b = parent.arity == 4
                ? local[local_facet][next]
                : next;
            const bool boundary =
                (corner_a + 1) % parent.arity == corner_b ||
                (corner_b + 1) % parent.arity == corner_a;
            key.parent_boundary = boundary;
            key.parent_eid = boundary ? 0 : parent.eid;
        }
        return result;
    }
};

struct DiscoveryResult {
    contact::FixedTriangleDiscoveryReport report;
    std::vector<contact::FixedTriangleFeatureCandidate> features;
    std::vector<contact::FixedTriangleIntersection> intersections;
};

DiscoveryResult DiscoverImpl(
    const std::vector<contact::CurrentFixedTriangle>& triangles,
    const std::vector<contact::FixedTrianglePair>& pairs,
    const contact::FixedTriangleFeatureTaskMask* masks) {
    Require(!triangles.empty() && !pairs.empty(),
            "Coupon discovery input is empty");
    contact::FixedTriangleFeatureLimits limits;
    limits.max_input_pairs = pairs.size();
    limits.max_triangle_references = 2 * pairs.size();
    limits.max_vertex_references = 6 * pairs.size();
    limits.max_edge_references = 6 * pairs.size();
    limits.max_raw_feature_candidates = 15 * pairs.size();
    limits.max_feature_candidates = 15 * pairs.size();
    limits.max_raw_intersections = pairs.size();
    limits.max_intersections = pairs.size();
    limits.max_host_bytes = 16u << 20;
    contact::FixedTriangleFeatureDiscovery discovery;
    Require(discovery.Initialize(limits).status ==
                contact::FixedTriangleDiscoveryStatus::Ok,
            "Coupon discovery initialization failed");
    DiscoveryResult result;
    result.report = masks
        ? discovery.DiscoverMasked(
              triangles.data(), triangles.size(), pairs.data(),
              pairs.size(), masks)
        : discovery.Discover(
              triangles.data(), triangles.size(), pairs.data(),
              pairs.size());
    Require(result.report.status ==
                contact::FixedTriangleDiscoveryStatus::Ok,
            result.report.message);
    const auto features = discovery.features();
    const auto intersections = discovery.intersections();
    Require(features.complete,
            "Coupon feature publication is incomplete");
    Require(intersections.complete,
            "Coupon intersection publication is incomplete");
    if (features.count)
        result.features.assign(
            features.data, features.data + features.count);
    if (intersections.count)
        result.intersections.assign(
            intersections.data,
            intersections.data + intersections.count);
    return result;
}

DiscoveryResult Discover(
    const std::vector<contact::CurrentFixedTriangle>& triangles,
    const std::vector<contact::FixedTrianglePair>& pairs) {
    return DiscoverImpl(triangles, pairs, nullptr);
}

DiscoveryResult DiscoverMasked(
    const std::vector<contact::CurrentFixedTriangle>& triangles,
    const std::vector<contact::FixedTrianglePair>& pairs,
    const std::vector<contact::FixedTriangleFeatureTaskMask>& masks) {
    Require(masks.size() == pairs.size(),
            "Coupon mask count differs from its pair count");
    return DiscoverImpl(triangles, pairs, masks.data());
}

bool HasTriangle(const contact::FixedTriangleFeatureCandidate& feature,
                 std::uint64_t eid, unsigned local) {
    for (const auto& triangle : feature.triangles)
        if (triangle.parent_eid == eid &&
            triangle.local_facet == local)
            return true;
    return false;
}

const contact::FixedTriangleFeatureCandidate& MinimumEe(
    const DiscoveryResult& discovery, std::uint64_t first_eid,
    unsigned first_local, std::uint64_t second_eid,
    unsigned second_local,
    contact::SelfContactEdgeEdgeCase expected_case) {
    const contact::FixedTriangleFeatureCandidate* result = nullptr;
    for (const auto& feature : discovery.features) {
        if (feature.key.kind !=
                contact::FixedTriangleCandidateKind::EdgeEdge ||
            !HasTriangle(feature, first_eid, first_local) ||
            !HasTriangle(feature, second_eid, second_local))
            continue;
        unsigned boundary = 0;
        for (const auto parameter : feature.edge_parameters)
            boundary += parameter == 0 || parameter == 1;
        const auto edge_case = boundary == 0
            ? contact::SelfContactEdgeEdgeCase::
                  StrictInteriorInteriorMinimum
            : boundary == 1
                ? contact::SelfContactEdgeEdgeCase::
                      BoundaryVertexEdgeMinimum
                : contact::SelfContactEdgeEdgeCase::
                      EdgeEdgeOnlyPenetrationOrCrossing;
        if (edge_case != expected_case) continue;
        if (!result || feature.distance_m < result->distance_m)
            result = &feature;
    }
    Require(result, "Required real-Yaris EE feature was not discovered");
    return *result;
}

struct SegmentOracle {
    long double parameters[2]{};
    long double distance = 0;
    long double coordinate_scale = 1;
};

contact::Vec3 Lookup(const GeometrySet& geometry,
                     const contact::FacetVertexKey& key) {
    Require(key.kind == contact::FacetVertexKind::SourceVertex &&
                key.first != 0,
            "Level-zero coupon edge is not a canonical source edge");
    const auto found = geometry.node_index.find(key.first);
    Require(found != geometry.node_index.end(),
            "Coupon edge endpoint is absent from canonical nodes");
    const auto node = found->second;
    return {geometry.positions[3 * node],
            geometry.positions[3 * node + 1],
            geometry.positions[3 * node + 2]};
}

SegmentOracle Oracle(
    const GeometrySet& geometry,
    const contact::FixedTriangleFeatureCandidate& feature) {
    Require(feature.key.kind ==
                contact::FixedTriangleCandidateKind::EdgeEdge,
            "Segment oracle requires an EE feature");
    const auto edge = [&](unsigned side, unsigned endpoint) {
        return Lookup(
            geometry,
            feature.key.edge_edge.edges[side].endpoints[endpoint]);
    };
    const auto a0 = edge(0, 0), a1 = edge(0, 1);
    const auto b0 = edge(1, 0), b1 = edge(1, 1);
    const auto coordinate = [](contact::Vec3 value, unsigned axis) {
        return static_cast<long double>(
            axis == 0 ? value.x : axis == 1 ? value.y : value.z);
    };
    long double u[3], v[3], w[3];
    SegmentOracle result;
    for (unsigned axis = 0; axis < 3; ++axis) {
        u[axis] = coordinate(a1, axis) - coordinate(a0, axis);
        v[axis] = coordinate(b1, axis) - coordinate(b0, axis);
        w[axis] = coordinate(a0, axis) - coordinate(b0, axis);
        result.coordinate_scale = std::max(
            result.coordinate_scale,
            std::max({std::fabs(coordinate(a0, axis)),
                      std::fabs(coordinate(a1, axis)),
                      std::fabs(coordinate(b0, axis)),
                      std::fabs(coordinate(b1, axis))}));
    }
    const auto dot = [](const long double* a, const long double* b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    const long double aa = dot(u, u), bb = dot(u, v);
    const long double cc = dot(v, v), dd = dot(u, w);
    const long double ee = dot(v, w);
    const long double denominator = aa * cc - bb * bb;
    Require(aa > 0 && cc > 0 && denominator > 0,
            "Coupon edge oracle is degenerate or parallel");
    long double s = (bb * ee - cc * dd) / denominator;
    long double t = (aa * ee - bb * dd) / denominator;
    const auto clamp = [](long double value) {
        return std::max(0.L, std::min(1.L, value));
    };
    s = clamp(s);
    t = clamp((bb * s + ee) / cc);
    s = clamp((bb * t - dd) / aa);
    long double delta[3];
    for (unsigned axis = 0; axis < 3; ++axis)
        delta[axis] = w[axis] + s * u[axis] - t * v[axis];
    result.parameters[0] = s;
    result.parameters[1] = t;
    result.distance = std::sqrt(dot(delta, delta));
    return result;
}

SegmentOracle ParallelOracle(
    const GeometrySet& geometry,
    const contact::FixedTriangleFeatureCandidate& feature) {
    Require(feature.key.kind ==
                contact::FixedTriangleCandidateKind::EdgeEdge,
            "Parallel segment oracle requires an EE feature");
    long double endpoints[2][2][3]{};
    SegmentOracle result;
    for (unsigned edge = 0; edge < 2; ++edge) {
        const auto& key = feature.key.edge_edge.edges[edge];
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
            const auto point =
                Lookup(geometry, key.endpoints[endpoint]);
            endpoints[edge][endpoint][0] = point.x;
            endpoints[edge][endpoint][1] = point.y;
            endpoints[edge][endpoint][2] = point.z;
            for (unsigned axis = 0; axis < 3; ++axis)
                result.coordinate_scale = std::max(
                    result.coordinate_scale,
                    std::fabs(endpoints[edge][endpoint][axis]));
        }
    }
    result.distance = std::numeric_limits<long double>::infinity();
    const auto consider = [&](long double first_parameter,
                              long double second_parameter) {
        const long double parameters[2]{
            first_parameter, second_parameter};
        long double squared = 0;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const long double first =
                endpoints[0][0][axis] +
                parameters[0] *
                    (endpoints[0][1][axis] -
                     endpoints[0][0][axis]);
            const long double second =
                endpoints[1][0][axis] +
                parameters[1] *
                    (endpoints[1][1][axis] -
                     endpoints[1][0][axis]);
            const long double delta = first - second;
            squared += delta * delta;
        }
        const long double distance = std::sqrt(squared);
        if (distance < result.distance) {
            result.parameters[0] = first_parameter;
            result.parameters[1] = second_parameter;
            result.distance = distance;
        }
    };
    const auto project =
        [&](unsigned point_edge, unsigned point_endpoint,
            unsigned target_edge) {
        long double numerator = 0, squared = 0;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const long double direction =
                endpoints[target_edge][1][axis] -
                endpoints[target_edge][0][axis];
            numerator +=
                (endpoints[point_edge][point_endpoint][axis] -
                 endpoints[target_edge][0][axis]) *
                direction;
            squared += direction * direction;
        }
        Require(squared > 0,
                "Parallel segment oracle edge is degenerate");
        return std::max(
            0.L, std::min(1.L, numerator / squared));
    };
    for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
        consider(endpoint,
                 project(0, endpoint, 1));
        consider(project(1, endpoint, 0),
                 endpoint);
    }
    return result;
}

contact::SelfContactEdgeEdgeCase EdgeCase(
    const contact::FixedTriangleFeatureCandidate& feature) {
    unsigned boundary = 0;
    for (const auto parameter : feature.edge_parameters)
        boundary += parameter == 0 || parameter == 1;
    Require(boundary <= 1,
            "Real-Yaris blocker is not strict or vertex-edge EE");
    return boundary
        ? contact::SelfContactEdgeEdgeCase::BoundaryVertexEdgeMinimum
        : contact::SelfContactEdgeEdgeCase::
              StrictInteriorInteriorMinimum;
}

void ExpectContained(
    const GeometrySet& geometry,
    const contact::FixedTriangleFeatureCandidate& feature) {
    const auto oracle = Oracle(geometry, feature);
    const long double arithmetic =
        256 * std::numeric_limits<double>::epsilon() *
        oracle.coordinate_scale;
    const long double tolerance =
        feature.representation_error_m + arithmetic;
    EXPECT_LE(static_cast<long double>(feature.distance_m) - tolerance,
              oracle.distance);
    EXPECT_GE(static_cast<long double>(feature.distance_m) + tolerance,
              oracle.distance);
    for (unsigned side = 0; side < 2; ++side)
        EXPECT_LE(std::fabs(
                      static_cast<long double>(
                          feature.edge_parameters[side]) -
                      oracle.parameters[side]),
                  arithmetic);
}

std::size_t SharedNodes(const Parent& a, const Parent& b) {
    std::size_t result = 0;
    for (unsigned i = 0; i < a.arity; ++i)
        for (unsigned j = 0; j < b.arity; ++j)
            result += a.node_ids[i] == b.node_ids[j];
    return result;
}

constexpr std::uint64_t HashBasis = 14695981039346656037ULL;
constexpr std::uint64_t HashPrime = 1099511628211ULL;

template <class T,
          std::enable_if_t<std::is_integral_v<T>, int> = 0>
void Hash(T input, std::uint64_t& hash) {
    const auto value = static_cast<std::uint64_t>(input);
    for (unsigned byte = 0; byte < 8; ++byte) {
        hash ^= (value >> (8 * byte)) & 0xffu;
        hash *= HashPrime;
    }
}

void Hash(double value, std::uint64_t& hash) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    Hash(bits, hash);
}

void Hash(const contact::FixedTriangleKey& key, std::uint64_t& hash) {
    Hash(key.source_instance_id, hash);
    Hash(key.parent_eid, hash);
    Hash(key.level, hash);
    Hash(key.local_facet, hash);
}

void Hash(const contact::FacetVertexKey& key, std::uint64_t& hash) {
    Hash(key.source_instance_id, hash);
    Hash(static_cast<std::uint8_t>(key.kind), hash);
    Hash(key.first, hash);
    Hash(key.second, hash);
    Hash(key.numerator, hash);
    Hash(key.denominator, hash);
    Hash(key.level, hash);
    Hash(key.grid_i, hash);
    Hash(key.grid_j, hash);
}

void Hash(const contact::FacetEdgeKey& key, std::uint64_t& hash) {
    Hash(key.parent_boundary, hash);
    Hash(key.parent_eid, hash);
    Hash(key.endpoints[0], hash);
    Hash(key.endpoints[1], hash);
}

void Hash(const contact::FixedTriangleStratumKey& key,
          std::uint64_t& hash) {
    Hash(static_cast<std::uint8_t>(key.kind), hash);
    if (key.kind == contact::FixedTriangleStratumKind::Vertex)
        Hash(key.vertex, hash);
    else if (key.kind == contact::FixedTriangleStratumKind::Edge)
        Hash(key.edge, hash);
    else
        Hash(key.face, hash);
}

void Hash(contact::Vec3 value, std::uint64_t& hash) {
    Hash(value.x, hash);
    Hash(value.y, hash);
    Hash(value.z, hash);
}

void Hash(const contact::FixedTriangleFeatureCandidate& value,
          std::uint64_t& hash) {
    Hash(static_cast<std::uint8_t>(value.key.kind), hash);
    if (value.key.kind ==
        contact::FixedTriangleCandidateKind::VertexFace) {
        Hash(value.key.vertex_face.vertex, hash);
        Hash(value.key.vertex_face.target, hash);
    } else {
        Hash(value.key.edge_edge.edges[0], hash);
        Hash(value.key.edge_edge.edges[1], hash);
    }
    for (unsigned side = 0; side < 2; ++side) {
        Hash(value.triangles[side], hash);
        Hash(value.local_features[side], hash);
        Hash(value.points[side], hash);
        Hash(value.edge_parameters[side], hash);
    }
    for (double weight : value.face_weights) Hash(weight, hash);
    Hash(value.distance_m, hash);
    Hash(value.representation_error_m, hash);
}

void Hash(const contact::FixedTriangleIntersection& value,
          std::uint64_t& hash) {
    Hash(value.triangles[0], hash);
    Hash(value.triangles[1], hash);
    Hash(static_cast<std::uint8_t>(value.kind), hash);
    Hash(static_cast<std::uint8_t>(value.local_exclusion), hash);
}

std::uint64_t DiscoveryHash(const DiscoveryResult& value) {
    std::uint64_t result = HashBasis;
    Hash(value.report.potential_tasks, result);
    Hash(value.report.local_masked_tasks, result);
    Hash(value.report.exact_executed_tasks, result);
    Hash(value.report.raw_feature_candidates, result);
    Hash(value.report.feature_candidates, result);
    Hash(value.report.raw_intersections, result);
    Hash(value.report.intersections, result);
    Hash(value.features.size(), result);
    for (const auto& feature : value.features) Hash(feature, result);
    Hash(value.intersections.size(), result);
    for (const auto& intersection : value.intersections)
        Hash(intersection, result);
    return result;
}

unsigned FeatureTask(
    const contact::FixedTriangleFeatureCandidate& feature) {
    if (feature.key.kind ==
        contact::FixedTriangleCandidateKind::VertexFace) {
        const bool first_vertex = feature.local_features[0] < 3;
        Require(first_vertex != (feature.local_features[1] < 3),
                "Coupon VF provenance does not select one vertex");
        const unsigned side = first_vertex ? 0 : 1;
        return contact::FixedTriangleVertexFaceTaskSlot(
            side, feature.local_features[side]);
    }
    Require(feature.local_features[0] < 3 &&
                feature.local_features[1] < 3,
            "Coupon EE provenance is out of range");
    return contact::FixedTriangleEdgeEdgeTaskSlot(
        feature.local_features[0], feature.local_features[1]);
}

const char* FeatureStratum(
    const contact::FixedTriangleFeatureCandidate& feature) {
    if (feature.key.kind ==
        contact::FixedTriangleCandidateKind::EdgeEdge)
        return "EE(edge-edge)";
    switch (feature.key.vertex_face.target.kind) {
      case contact::FixedTriangleStratumKind::Vertex:
        return "VF(vertex)";
      case contact::FixedTriangleStratumKind::Edge:
        return "VF(edge)";
      case contact::FixedTriangleStratumKind::Face:
        return "VF(face)";
    }
    return "invalid";
}

const char* GeometryPolicyClass(
    const contact::FixedTriangleFeatureCandidate& feature) {
    if (feature.key.kind ==
        contact::FixedTriangleCandidateKind::VertexFace)
        return "VertexFace";
    if (feature.distance_m == 0) return "ZeroDistance";
    const bool strict =
        feature.edge_parameters[0] > 0 &&
        feature.edge_parameters[0] < 1 &&
        feature.edge_parameters[1] > 0 &&
        feature.edge_parameters[1] < 1;
    return strict ? "StrictInteriorInteriorMinimum"
                  : "BoundaryVertexEdgeMinimum";
}

struct LongVec3 {
    long double x = 0, y = 0, z = 0;
};

LongVec3 ToLong(contact::Vec3 value) {
    return {value.x, value.y, value.z};
}

LongVec3 Add(LongVec3 a, LongVec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

LongVec3 Subtract(LongVec3 a, LongVec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

LongVec3 Scale(LongVec3 value, long double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

long double Dot(LongVec3 a, LongVec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

LongVec3 Cross(LongVec3 a, LongVec3 b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

long double Length(LongVec3 value) {
    return std::sqrt(Dot(value, value));
}

long double CoordinateScale(
    const contact::CurrentFixedTriangle& first,
    const contact::CurrentFixedTriangle& second) {
    long double result = 1;
    for (const auto* triangle : {&first, &second})
        for (const auto point : triangle->vertices)
            result = std::max(
                result,
                std::max({std::fabs(static_cast<long double>(point.x)),
                          std::fabs(static_cast<long double>(point.y)),
                          std::fabs(static_cast<long double>(point.z))}));
    return result;
}

struct PlaneCutResult {
    LongVec3 points[2];
    std::size_t count = 0;
};

PlaneCutResult PlaneCut(
    const contact::CurrentFixedTriangle& triangle,
    const contact::CurrentFixedTriangle& plane,
    long double tolerance) {
    const std::array<LongVec3, 3> points{
        ToLong(triangle.vertices[0]), ToLong(triangle.vertices[1]),
        ToLong(triangle.vertices[2])};
    const LongVec3 origin = ToLong(plane.vertices[0]);
    const LongVec3 normal = Cross(
        Subtract(ToLong(plane.vertices[1]), origin),
        Subtract(ToLong(plane.vertices[2]), origin));
    Require(Length(normal) > 0,
            "Intersection oracle plane is degenerate");
    long double side[3];
    for (unsigned i = 0; i < 3; ++i) {
        side[i] = Dot(normal, Subtract(points[i], origin));
        const long double arithmetic =
            4096 * std::numeric_limits<long double>::epsilon() *
            Length(normal) *
            std::max(1.L, Length(Subtract(points[i], origin)));
        if (std::fabs(side[i]) <= arithmetic) side[i] = 0;
    }

    PlaneCutResult cut;
    const auto append = [&](LongVec3 value) {
        for (std::size_t i = 0; i < cut.count; ++i)
            if (Length(Subtract(cut.points[i], value)) <= tolerance)
                return;
        Require(cut.count < 2,
                "Intersection oracle plane cut has excess points");
        cut.points[cut.count++] = value;
    };
    for (unsigned i = 0; i < 3; ++i)
        if (side[i] == 0) append(points[i]);
    for (unsigned i = 0; i < 3; ++i) {
        const unsigned next = (i + 1) % 3;
        if (!((side[i] < 0 && side[next] > 0) ||
              (side[i] > 0 && side[next] < 0)))
            continue;
        const long double parameter =
            side[i] / (side[i] - side[next]);
        append(Add(points[i], Scale(
            Subtract(points[next], points[i]), parameter)));
    }
    Require(cut.count == 1 || cut.count == 2,
            "Intersection oracle plane cut is empty");
    return cut;
}

LongVec3 PointAtProjection(
    const PlaneCutResult& segment,
    LongVec3 direction, long double projection) {
    Require(segment.count == 2,
            "Intersection oracle projection requires a segment");
    const long double first = Dot(segment.points[0], direction);
    const long double second = Dot(segment.points[1], direction);
    Require(first != second,
            "Intersection oracle segment projection is degenerate");
    return Add(segment.points[0], Scale(
        Subtract(segment.points[1], segment.points[0]),
        (projection - first) / (second - first)));
}

struct TriangleIntersectionOracle {
    LongVec3 endpoints[2];
    long double segment_length = 0;
    long double cut_mismatch = 0;
    long double plane_cross_sine = 0;
    long double shared_parameter = 0;
    long double shared_distance = 0;
    long double tolerance = 0;
};

TriangleIntersectionOracle TriangleIntersection(
    const contact::CurrentFixedTriangle& first,
    const contact::CurrentFixedTriangle& second,
    contact::Vec3 shared_vertex) {
    TriangleIntersectionOracle result;
    const long double scale = CoordinateScale(first, second);
    result.tolerance =
        4096 * std::numeric_limits<long double>::epsilon() * scale;
    const auto first_cut = PlaneCut(first, second, result.tolerance);
    const auto second_cut = PlaneCut(second, first, result.tolerance);
    const auto normal = [](const contact::CurrentFixedTriangle& value) {
        return Cross(
            Subtract(ToLong(value.vertices[1]),
                     ToLong(value.vertices[0])),
            Subtract(ToLong(value.vertices[2]),
                     ToLong(value.vertices[0])));
    };
    const LongVec3 first_normal = normal(first);
    const LongVec3 second_normal = normal(second);
    const LongVec3 cross = Cross(first_normal, second_normal);
    const long double first_norm = Length(first_normal);
    const long double second_norm = Length(second_normal);
    const long double cross_norm = Length(cross);
    Require(first_norm > 0 && second_norm > 0 && cross_norm > 0,
            "Intersection oracle planes are degenerate or parallel");
    result.plane_cross_sine =
        cross_norm / (first_norm * second_norm);
    const LongVec3 direction = Scale(cross, 1 / cross_norm);
    const auto interval = [&](const PlaneCutResult& cut) {
        Require(cut.count == 2,
                "Intersection oracle interval requires a segment");
        const long double a = Dot(cut.points[0], direction);
        const long double b = Dot(cut.points[1], direction);
        return std::array<long double, 2>{
            std::min(a, b), std::max(a, b)};
    };
    long double lower = 0, upper = 0;
    if (first_cut.count == 2 && second_cut.count == 2) {
        const auto first_interval = interval(first_cut);
        const auto second_interval = interval(second_cut);
        lower = std::max(first_interval[0], second_interval[0]);
        upper = std::min(first_interval[1], second_interval[1]);
        Require(upper >= lower,
                "Intersection oracle plane cuts are disjoint");
        for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
            const long double projection = endpoint ? upper : lower;
            const auto from_first =
                PointAtProjection(first_cut, direction, projection);
            const auto from_second =
                PointAtProjection(second_cut, direction, projection);
            result.cut_mismatch = std::max(
                result.cut_mismatch,
                Length(Subtract(from_first, from_second)));
            result.endpoints[endpoint] =
                Scale(Add(from_first, from_second), .5L);
        }
    } else if (first_cut.count == 2 || second_cut.count == 2) {
        const auto& segment =
            first_cut.count == 2 ? first_cut : second_cut;
        const auto& point =
            first_cut.count == 1 ? first_cut : second_cut;
        lower = upper = Dot(point.points[0], direction);
        const auto represented =
            PointAtProjection(segment, direction, lower);
        result.cut_mismatch =
            Length(Subtract(represented, point.points[0]));
        result.endpoints[0] = result.endpoints[1] =
            Scale(Add(represented, point.points[0]), .5L);
    } else {
        lower = upper = Dot(first_cut.points[0], direction);
        result.cut_mismatch =
            Length(Subtract(first_cut.points[0],
                            second_cut.points[0]));
        result.endpoints[0] = result.endpoints[1] =
            Scale(Add(first_cut.points[0], second_cut.points[0]),
                  .5L);
    }
    result.segment_length =
        Length(Subtract(result.endpoints[1], result.endpoints[0]));
    const LongVec3 shared = ToLong(shared_vertex);
    const long double shared_projection = Dot(shared, direction);
    result.shared_parameter = upper == lower
        ? 0
        : (shared_projection - lower) / (upper - lower);
    result.shared_distance = std::max(
        Length(Subtract(shared, result.endpoints[0])),
        Length(Subtract(shared, result.endpoints[1])));
    return result;
}

long double EdgeCrossSine(
    const GeometrySet& geometry,
    const contact::FixedTriangleFeatureCandidate& feature) {
    Require(feature.key.kind ==
                contact::FixedTriangleCandidateKind::EdgeEdge,
            "Edge cross oracle requires EE geometry");
    const auto vector = [&](unsigned edge) {
        const auto& key = feature.key.edge_edge.edges[edge];
        return Subtract(ToLong(Lookup(geometry, key.endpoints[1])),
                        ToLong(Lookup(geometry, key.endpoints[0])));
    };
    const auto first = vector(0);
    const auto second = vector(1);
    const long double denominator =
        Length(first) * Length(second);
    Require(denominator > 0,
            "Edge cross oracle has a degenerate edge");
    return Length(Cross(first, second)) / denominator;
}

struct SmallCoupon {
    GeometrySet geometry;
    DiscoveryResult discovery;
    contact::FixedTriangleFeatureCandidate strict;
    contact::FixedTriangleFeatureCandidate boundary;

    SmallCoupon()
        : geometry({Authority().Read(AnchorEid),
                    Authority().Read(StrictEid),
                    Authority().Read(BoundaryEid)}),
          discovery([&] {
              const std::vector<contact::CurrentFixedTriangle> triangles{
                  geometry.Evaluate(geometry.Facet(AnchorEid, 0)),
                  geometry.Evaluate(geometry.Facet(StrictEid, 1)),
                  geometry.Evaluate(geometry.Facet(BoundaryEid, 0))};
              return Discover(triangles, {{0, 1}, {0, 2}});
          }()),
          strict(MinimumEe(
              discovery, AnchorEid, 0, StrictEid, 1,
              contact::SelfContactEdgeEdgeCase::
                  StrictInteriorInteriorMinimum)),
          boundary(MinimumEe(
              discovery, AnchorEid, 0, BoundaryEid, 0,
              contact::SelfContactEdgeEdgeCase::
                  BoundaryVertexEdgeMinimum)) {}
};

const SmallCoupon& Small() {
    static const SmallCoupon value;
    return value;
}

bool FacetContains(const contact::FixedContactFacet& facet,
                   const contact::FacetEdgeKey& edge) {
    return Same(facet.edge_keys[0], edge) ||
           Same(facet.edge_keys[1], edge) ||
           Same(facet.edge_keys[2], edge);
}

std::vector<std::uint32_t> IncidentTriangles(
    const GeometrySet& geometry,
    const std::vector<contact::CurrentFixedTriangle>& triangles,
    const contact::FacetEdgeKey& edge) {
    std::vector<std::uint32_t> result;
    for (std::size_t i = 0; i < geometry.facets.size(); ++i)
        if (FacetContains(geometry.facets[i], edge))
            result.push_back(static_cast<std::uint32_t>(i));
    Require(result.size() <= triangles.size(),
            "Incident facet inventory is invalid");
    return result;
}

}  // namespace

TEST(RealYarisSelfContactGeometry,
     ExactStrictAndBoundaryEeRemainCanonicalAndUnexcluded) {
    const auto& authority = Authority();
    EXPECT_EQ(authority.data().inputs.canonical_manifest.sha256,
              "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8");
    EXPECT_EQ(authority.data().archive_sha256,
              "aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451");
    EXPECT_EQ(authority.plan().identity().sha256,
              "a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d");

    const auto& coupon = Small();
    ASSERT_EQ(coupon.geometry.parents.size(), 3u);
    const auto& anchor = coupon.geometry.parents[0];
    const auto& strict_parent = coupon.geometry.parents[1];
    const auto& boundary_parent = coupon.geometry.parents[2];
    EXPECT_EQ(anchor.eid, AnchorEid);
    EXPECT_EQ(strict_parent.eid, StrictEid);
    EXPECT_EQ(boundary_parent.eid, BoundaryEid);
    EXPECT_EQ(anchor.arity, 4u);
    EXPECT_EQ(strict_parent.arity, 4u);
    EXPECT_EQ(boundary_parent.arity, 4u);
    EXPECT_EQ(anchor.pid, strict_parent.pid);
    EXPECT_EQ(SharedNodes(anchor, strict_parent), 0u);
    EXPECT_EQ(SharedNodes(anchor, boundary_parent), 0u);
    const std::array<std::pair<const Parent*, unsigned>, 3> owners{{
        {&anchor, 0}, {&strict_parent, 1}, {&boundary_parent, 0}}};
    for (const auto owner : owners) {
        const auto facet =
            coupon.geometry.Facet(owner.first->eid, owner.second);
        EXPECT_EQ(facet.source.source_parent_id, owner.first->eid);
        EXPECT_EQ(facet.source.source_part_id, owner.first->pid);
        EXPECT_EQ(facet.source.material_id, owner.first->mid);
        EXPECT_EQ(facet.source.section_id, owner.first->sid);
        EXPECT_EQ(facet.local_facet, owner.second);
        EXPECT_EQ(facet.reference_half_thickness_m,
                  .5 * owner.first->thickness_m);
    }

    EXPECT_EQ(coupon.discovery.report.feature_tasks, 30u);
    EXPECT_GT(coupon.discovery.report.feature_candidates, 0u);
    for (const auto* feature : {&coupon.strict, &coupon.boundary}) {
        EXPECT_EQ(feature->key.kind,
                  contact::FixedTriangleCandidateKind::EdgeEdge);
        EXPECT_GT(feature->distance_m, 0);
        EXPECT_LT(feature->distance_m, .5 * (
            coupon.geometry.parents[0].thickness_m +
            (feature == &coupon.strict
                 ? strict_parent.thickness_m
                 : boundary_parent.thickness_m)));
        EXPECT_GE(feature->representation_error_m, 0);
        ExpectContained(coupon.geometry, *feature);
    }
    EXPECT_EQ(EdgeCase(coupon.strict),
              contact::SelfContactEdgeEdgeCase::
                  StrictInteriorInteriorMinimum);
    EXPECT_GT(coupon.strict.edge_parameters[0], 0);
    EXPECT_LT(coupon.strict.edge_parameters[0], 1);
    EXPECT_GT(coupon.strict.edge_parameters[1], 0);
    EXPECT_LT(coupon.strict.edge_parameters[1], 1);
    EXPECT_EQ(EdgeCase(coupon.boundary),
              contact::SelfContactEdgeEdgeCase::
                  BoundaryVertexEdgeMinimum);
    const unsigned boundary_parameters =
        unsigned(coupon.boundary.edge_parameters[0] == 0 ||
                 coupon.boundary.edge_parameters[0] == 1) +
        unsigned(coupon.boundary.edge_parameters[1] == 0 ||
                 coupon.boundary.edge_parameters[1] == 1);
    EXPECT_EQ(boundary_parameters, 1u);
    std::cout << "Real Yaris EE strict_distance_m="
              << coupon.strict.distance_m
              << " strict_parameters="
              << coupon.strict.edge_parameters[0] << ","
              << coupon.strict.edge_parameters[1]
              << " boundary_distance_m="
              << coupon.boundary.distance_m
              << " boundary_parameters="
              << coupon.boundary.edge_parameters[0] << ","
              << coupon.boundary.edge_parameters[1] << '\n';
}

TEST(RealYarisSelfContactGeometry,
     FirstAcceptedNonlocalIntersectionIsTransverseAndPolicyUnresolved) {
    const auto& authority = Authority();
    EXPECT_EQ(authority.data().inputs.canonical_manifest.sha256,
              "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8");
    EXPECT_EQ(authority.data().archive_sha256,
              "aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451");
    EXPECT_EQ(authority.plan().identity().sha256,
              "a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d");

    GeometrySet geometry({
        authority.Read(IntersectionFirstEid),
        authority.Read(IntersectionSecondEid)});
    ASSERT_EQ(geometry.parents.size(), 2u);
    const auto& first_parent = geometry.parents[0];
    const auto& second_parent = geometry.parents[1];
    EXPECT_EQ(first_parent.eid, IntersectionFirstEid);
    EXPECT_EQ(second_parent.eid, IntersectionSecondEid);
    EXPECT_EQ(first_parent.pid, 2000546u);
    EXPECT_EQ(second_parent.pid, 2000546u);
    EXPECT_EQ(first_parent.arity, 4u);
    EXPECT_EQ(second_parent.arity, 4u);
    EXPECT_EQ(first_parent.node_ids, IntersectionFirstNodes);
    EXPECT_EQ(second_parent.node_ids, IntersectionSecondNodes);
    EXPECT_EQ(SharedNodes(first_parent, second_parent), 2u);

    const auto first_facet =
        geometry.Facet(IntersectionFirstEid, 0);
    const auto second_facet =
        geometry.Facet(IntersectionSecondEid, 0);
    const std::vector<contact::CurrentFixedTriangle> triangles{
        geometry.Evaluate(first_facet),
        geometry.Evaluate(second_facet)};
    ASSERT_EQ(triangles[0].vertex_keys[2].first,
              IntersectionSharedNode);
    ASSERT_EQ(triangles[1].vertex_keys[2].first,
              IntersectionSharedNode);
    unsigned shared_facet_vertices = 0;
    for (const auto& first : triangles[0].vertex_keys)
        for (const auto& second : triangles[1].vertex_keys)
            shared_facet_vertices += Same(first, second);
    EXPECT_EQ(shared_facet_vertices, 1u);
    EXPECT_EQ(first_parent.positions[2].x,
              second_parent.positions[2].x);
    EXPECT_EQ(first_parent.positions[2].y,
              second_parent.positions[2].y);
    EXPECT_EQ(first_parent.positions[2].z,
              second_parent.positions[2].z);

    contact::FixedTriangleFeatureTaskMask mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  triangles[0], triangles[1], &mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    EXPECT_EQ(mask.local_tasks, IntersectionLocalTaskMask);
    unsigned masked_tasks = 0;
    for (unsigned slot = 0; slot < 15; ++slot)
        masked_tasks += bool(
            mask.local_tasks &
            contact::FixedTriangleFeatureTaskBit(slot));
    EXPECT_EQ(masked_tasks, 6u);
    EXPECT_EQ(
        mask.local_tasks &
            (contact::FixedTriangleFeatureTaskBit(4) |
             contact::FixedTriangleFeatureTaskBit(5)),
        contact::FixedTriangleFeatureTaskBit(4) |
            contact::FixedTriangleFeatureTaskBit(5));
    for (const unsigned slot : {10u, 11u, 13u, 14u})
        EXPECT_NE(mask.local_tasks &
                      contact::FixedTriangleFeatureTaskBit(slot),
                  0u);
    const auto edge_contains_shared =
        [&](unsigned side, unsigned edge) {
            const auto& key = triangles[side].edge_keys[edge];
            return Same(key.endpoints[0],
                        triangles[0].vertex_keys[2]) ||
                Same(key.endpoints[1],
                     triangles[0].vertex_keys[2]);
        };
    for (unsigned slot = 0; slot < 15; ++slot) {
        if (!(mask.local_tasks &
              contact::FixedTriangleFeatureTaskBit(slot)))
            continue;
        if (slot < 6) {
            const unsigned side = slot % 2;
            EXPECT_TRUE(Same(
                triangles[side].vertex_keys[slot / 2],
                triangles[0].vertex_keys[2]));
        } else {
            const unsigned pair_slot = slot - 6;
            EXPECT_TRUE(edge_contains_shared(0, pair_slot / 3));
            EXPECT_TRUE(edge_contains_shared(1, pair_slot % 3));
        }
    }

    const std::vector<contact::FixedTrianglePair> pair{{0, 1}};
    const auto discovered =
        DiscoverMasked(triangles, pair, {mask});
    EXPECT_EQ(discovered.report.potential_tasks, 15u);
    EXPECT_EQ(discovered.report.local_masked_tasks, 6u);
    EXPECT_EQ(discovered.report.exact_executed_tasks, 9u);
    EXPECT_EQ(discovered.report.feature_tasks, 9u);
    EXPECT_EQ(discovered.report.raw_feature_candidates, 9u);
    EXPECT_EQ(discovered.report.feature_candidates,
              discovered.features.size());
    EXPECT_EQ(discovered.report.raw_intersections, 1u);
    ASSERT_EQ(discovered.intersections.size(), 1u);
    const auto& intersection = discovered.intersections[0];
    EXPECT_EQ(intersection.triangles[0].parent_eid,
              IntersectionFirstEid);
    EXPECT_EQ(intersection.triangles[0].local_facet, 0u);
    EXPECT_EQ(intersection.triangles[1].parent_eid,
              IntersectionSecondEid);
    EXPECT_EQ(intersection.triangles[1].local_facet, 0u);
    EXPECT_EQ(intersection.kind,
              contact::FixedTriangleIntersectionKind::Transverse);
    EXPECT_EQ(intersection.local_exclusion,
              contact::FixedTriangleLocalExclusion::None);
    EXPECT_TRUE(contact::RequiresIntersectionAdmission(intersection));

    std::set<unsigned> emitted_tasks;
    std::size_t vf_vertex = 0, vf_edge = 0, vf_face = 0, ee = 0;
    std::size_t zero = 0, nonzero = 0;
    std::size_t zero_nonparallel_ee = 0;
    std::size_t positive_thickness_contacts = 0;
    long double minimum_zero_edge_cross_sine = 1;
    const double thickness_distance =
        .5 * (first_parent.thickness_m +
              second_parent.thickness_m);
    std::cout << std::setprecision(17);
    for (const auto& feature : discovered.features) {
        SegmentOracle edge_oracle;
        bool has_edge_oracle = false;
        bool unique_edge_oracle = false;
        long double edge_cross_sine = 0;
        const unsigned task = FeatureTask(feature);
        EXPECT_EQ(mask.local_tasks &
                      contact::FixedTriangleFeatureTaskBit(task),
                  0u);
        EXPECT_TRUE(emitted_tasks.insert(task).second);
        EXPECT_TRUE(std::isfinite(feature.distance_m));
        EXPECT_GE(feature.distance_m, 0);
        EXPECT_TRUE(std::isfinite(feature.representation_error_m));
        EXPECT_GE(feature.representation_error_m, 0);
        EXPECT_EQ(feature.representation_error_m, 0);
        const long double represented_distance =
            Length(Subtract(ToLong(feature.points[0]),
                            ToLong(feature.points[1])));
        const long double distance_tolerance =
            256 * std::numeric_limits<double>::epsilon() *
            std::max(1.L, represented_distance);
        EXPECT_LE(std::fabs(
                      represented_distance -
                      static_cast<long double>(feature.distance_m)),
                  distance_tolerance);

        if (feature.distance_m == 0)
            ++zero;
        else {
            ++nonzero;
            if (feature.distance_m <= thickness_distance)
                ++positive_thickness_contacts;
        }

        if (feature.key.kind ==
            contact::FixedTriangleCandidateKind::EdgeEdge) {
            ++ee;
            edge_cross_sine = EdgeCrossSine(geometry, feature);
            unique_edge_oracle =
                edge_cross_sine >
                1024 * std::numeric_limits<long double>::epsilon();
            edge_oracle = unique_edge_oracle
                ? Oracle(geometry, feature)
                : ParallelOracle(geometry, feature);
            has_edge_oracle = true;
            if (unique_edge_oracle) {
                ExpectContained(geometry, feature);
            } else {
                const long double arithmetic =
                    256 * std::numeric_limits<double>::epsilon() *
                    edge_oracle.coordinate_scale;
                EXPECT_LE(std::fabs(
                              static_cast<long double>(
                                  feature.distance_m) -
                              edge_oracle.distance),
                          arithmetic);
            }
            for (const double parameter : feature.edge_parameters) {
                EXPECT_GE(parameter, 0);
                EXPECT_LE(parameter, 1);
            }
            if (feature.distance_m == 0) {
                const long double cross_sine =
                    EdgeCrossSine(geometry, feature);
                minimum_zero_edge_cross_sine = std::min(
                    minimum_zero_edge_cross_sine, cross_sine);
                if (cross_sine > 0) ++zero_nonparallel_ee;
            }
        } else {
            const auto stratum =
                feature.key.vertex_face.target.kind;
            vf_vertex +=
                stratum == contact::FixedTriangleStratumKind::Vertex;
            vf_edge +=
                stratum == contact::FixedTriangleStratumKind::Edge;
            vf_face +=
                stratum == contact::FixedTriangleStratumKind::Face;
            if (stratum ==
                contact::FixedTriangleStratumKind::Edge) {
                EXPECT_GT(feature.edge_parameters[0], 0);
                EXPECT_LT(feature.edge_parameters[0], 1);
            }
        }
        std::cout
            << "V5_FIRST_NONLOCAL_FEATURE"
            << " task=" << task
            << " stratum=" << FeatureStratum(feature)
            << " zero=" << (feature.distance_m == 0)
            << " distance_m=" << feature.distance_m
            << " edge_parameters="
            << feature.edge_parameters[0] << ","
            << feature.edge_parameters[1]
            << " representation_error_m="
            << feature.representation_error_m
            << " geometry_policy_class="
            << GeometryPolicyClass(feature)
            << " geometry_mappable=1"
            << " active_use_authority=not_constructed";
        if (has_edge_oracle)
            std::cout
                << " long_double_edge_distance_m="
                << edge_oracle.distance
                << " long_double_edge_parameters="
                << edge_oracle.parameters[0] << ","
                << edge_oracle.parameters[1]
                << " edge_cross_sine=" << edge_cross_sine
                << " edge_minimizer="
                << (unique_edge_oracle ? "unique" : "parallel");
        std::cout << '\n';
    }
    if (!zero_nonparallel_ee)
        minimum_zero_edge_cross_sine = 0;
    EXPECT_EQ(emitted_tasks.size(), discovered.features.size());
    for (const auto task : emitted_tasks)
        EXPECT_EQ(mask.local_tasks &
                      contact::FixedTriangleFeatureTaskBit(task),
                  0u);
    EXPECT_EQ(vf_vertex + vf_edge + vf_face + ee,
              discovered.features.size());
    EXPECT_EQ(vf_vertex, 0u);
    EXPECT_EQ(vf_edge, 3u);
    EXPECT_EQ(vf_face, 1u);
    EXPECT_EQ(ee, 5u);
    EXPECT_EQ(zero, 0u);
    EXPECT_EQ(nonzero, 9u);
    EXPECT_EQ(zero_nonparallel_ee, 0u);
    EXPECT_EQ(positive_thickness_contacts, 0u);

    const auto oracle = TriangleIntersection(
        triangles[0], triangles[1], first_parent.positions[2]);
    EXPECT_GT(oracle.plane_cross_sine, 0);
    EXPECT_EQ(oracle.segment_length, 0);
    EXPECT_LE(oracle.cut_mismatch, 32 * oracle.tolerance);
    EXPECT_EQ(oracle.shared_parameter, 0);
    EXPECT_LE(oracle.shared_distance, 32 * oracle.tolerance);

    contact::FixedTriangleFeatureTaskMask swapped_mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  triangles[1], triangles[0], &swapped_mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    EXPECT_EQ(swapped_mask.local_tasks, mask.local_tasks);
    const auto swapped =
        DiscoverMasked(triangles, {{1, 0}}, {swapped_mask});
    const std::vector<contact::CurrentFixedTriangle> permuted_triangles{
        triangles[1], triangles[0]};
    contact::FixedTriangleFeatureTaskMask permuted_mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  permuted_triangles[0], permuted_triangles[1],
                  &permuted_mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    EXPECT_EQ(permuted_mask.local_tasks, mask.local_tasks);
    const auto permuted = DiscoverMasked(
        permuted_triangles, {{0, 1}}, {permuted_mask});
    const auto hash = DiscoveryHash(discovered);
    EXPECT_EQ(DiscoveryHash(swapped), hash);
    EXPECT_EQ(DiscoveryHash(permuted), hash);

    const auto print_point = [](LongVec3 point) {
        std::cout << static_cast<long double>(point.x) << ","
                  << static_cast<long double>(point.y) << ","
                  << static_cast<long double>(point.z);
    };
    std::cout << std::setprecision(
                     std::numeric_limits<long double>::max_digits10)
              << "V5_FIRST_NONLOCAL_INTERSECTION"
              << " parents=" << IntersectionFirstEid << ":0,"
              << IntersectionSecondEid << ":0"
              << " pid=" << first_parent.pid
              << " kind=Transverse"
              << " local_exclusion=None"
              << " requires_admission=1"
              << " local_mask=0x" << std::hex
              << mask.local_tasks << std::dec
              << " masked_shared_vertex_tasks=" << masked_tasks
              << " vf_vertex=" << vf_vertex
              << " vf_edge=" << vf_edge
              << " vf_face=" << vf_face
              << " ee=" << ee
              << " zero_features=" << zero
              << " nonzero_features=" << nonzero
              << " zero_nonparallel_ee="
              << zero_nonparallel_ee
              << " minimum_zero_edge_cross_sine="
              << minimum_zero_edge_cross_sine
              << " thickness_distance_m="
              << thickness_distance
              << " positive_thickness_contacts="
              << positive_thickness_contacts
              << " segment_endpoint0=";
    print_point(oracle.endpoints[0]);
    std::cout << " segment_endpoint1=";
    print_point(oracle.endpoints[1]);
    std::cout << " segment_length_m=" << oracle.segment_length
              << " shared_segment_parameter="
              << oracle.shared_parameter
              << " shared_distance_m=" << oracle.shared_distance
              << " cut_mismatch_m=" << oracle.cut_mismatch
              << " plane_cross_sine="
              << oracle.plane_cross_sine
              << " deterministic_hash=" << hash
              << " swapped_hash=" << DiscoveryHash(swapped)
              << " permuted_hash=" << DiscoveryHash(permuted)
              << " tl_policy=reject_nonlocal_intersection_before_features"
              << '\n';
}

TEST(RealYarisSelfContactGeometry,
     AdjacentFacetSeamsCanonicalizeWithoutDroppingParentLocalOwners) {
    const auto& small = Small();
    const std::vector<contact::FacetEdgeKey> blocker_edges{
        small.strict.key.edge_edge.edges[0],
        small.strict.key.edge_edge.edges[1],
        small.boundary.key.edge_edge.edges[0],
        small.boundary.key.edge_edge.edges[1]};
    GeometrySet geometry(Authority().Neighborhood(blocker_edges));
    std::vector<contact::CurrentFixedTriangle> triangles;
    triangles.reserve(geometry.facets.size());
    for (const auto& facet : geometry.facets)
        triangles.push_back(geometry.Evaluate(facet));

    std::set<std::pair<std::uint32_t, std::uint32_t>> unique_pairs;
    const auto add_repetitions =
        [&](const contact::FixedTriangleFeatureCandidate& blocker) {
            const auto first = IncidentTriangles(
                geometry, triangles, blocker.key.edge_edge.edges[0]);
            const auto second = IncidentTriangles(
                geometry, triangles, blocker.key.edge_edge.edges[1]);
            for (const auto a : first)
                for (const auto b : second)
                    if (a != b)
                        unique_pairs.emplace(
                            std::min(a, b), std::max(a, b));
        };
    add_repetitions(small.strict);
    add_repetitions(small.boundary);
    ASSERT_GT(unique_pairs.size(), 2u);
    ASSERT_LT(unique_pairs.size(), 4096u);
    std::vector<contact::FixedTrianglePair> pairs;
    for (const auto pair : unique_pairs)
        pairs.push_back({pair.first, pair.second});

    const auto discovered = Discover(triangles, pairs);
    EXPECT_GT(discovered.report.raw_feature_candidates,
              discovered.report.feature_candidates);
    const auto count_feature =
        [&](const contact::FixedTriangleFeatureCandidate& blocker) {
            return std::count_if(
                discovered.features.begin(), discovered.features.end(),
                [&](const auto& feature) {
                    return SameEdgePair(
                        feature, blocker.key.edge_edge.edges[0],
                        blocker.key.edge_edge.edges[1]);
                });
        };
    EXPECT_EQ(count_feature(small.strict), 1);
    EXPECT_EQ(count_feature(small.boundary), 1);

    bool repeated_facet_seam = false;
    bool distinct_parent_owners = false;
    std::size_t complete_owner_inventory = 0;
    for (const auto& edge : blocker_edges) {
        std::size_t occurrences = 0;
        std::map<std::uint64_t, double> owners;
        for (const auto& facet : geometry.facets) {
            if (!FacetContains(facet, edge)) continue;
            ++occurrences;
            const auto parent = facet.parent_index;
            owners.emplace(
                facet.source.source_parent_id,
                geometry.ParentArea(parent));
        }
        repeated_facet_seam =
            repeated_facet_seam || occurrences > owners.size();
        distinct_parent_owners =
            distinct_parent_owners || owners.size() > 1;
        for (const auto& owner : owners) {
            EXPECT_GT(owner.first, 0u);
            EXPECT_GT(owner.second, 0);
            ++complete_owner_inventory;
        }
    }
    EXPECT_TRUE(repeated_facet_seam);
    EXPECT_TRUE(distinct_parent_owners);
    EXPECT_GT(complete_owner_inventory, blocker_edges.size());
    std::cout << "Real Yaris incident neighborhood parents="
              << geometry.parents.size()
              << " facets=" << geometry.facets.size()
              << " repeated_pairs=" << pairs.size()
              << " canonical_features="
              << discovered.report.feature_candidates
              << " parent_local_owners="
              << complete_owner_inventory << '\n';

    std::reverse(pairs.begin(), pairs.end());
    const auto reversed = Discover(triangles, pairs);
    EXPECT_EQ(reversed.report.feature_candidates,
              discovered.report.feature_candidates);
    EXPECT_EQ(reversed.report.raw_feature_candidates,
              discovered.report.raw_feature_candidates);
    EXPECT_EQ(std::count_if(
                  reversed.features.begin(), reversed.features.end(),
                  [&](const auto& feature) {
                      return SameEdgePair(
                          feature, small.strict.key.edge_edge.edges[0],
                          small.strict.key.edge_edge.edges[1]);
                  }),
              1);
    EXPECT_EQ(std::count_if(
                  reversed.features.begin(), reversed.features.end(),
                  [&](const auto& feature) {
                      return SameEdgePair(
                          feature, small.boundary.key.edge_edge.edges[0],
                          small.boundary.key.edge_edge.edges[1]);
                  }),
              1);
}

}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
