#include "Source.h"

#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_wall/LoadedWall.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/self_contact_transaction/Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

namespace crash::cases::vehicle_self_contact {

struct CandidateRigidCouponSnapshot {
    const vehicle_dynamics::Fields* accepted = nullptr;
    const vehicle_dynamics::Fields* prepared = nullptr;
    tl::fea::NodalPreparedView prepared_view;
    std::vector<tl::fea::NodalRigidGroupSnapshot> accepted_groups;
    std::vector<tl::fea::NodalRigidGroupSnapshot> prepared_groups;
};

class CandidateRigidCouponAccess {
  public:
    static CandidateRigidCouponSnapshot Prepare(
        vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
        dynamics.PrepareStep();
        auto& storage = *dynamics.storage_;
        auto& owner = storage.state().owner;
        const auto group_count =
            storage.prepared.rigid_groups.group_count;
        CandidateRigidCouponSnapshot result;
        result.accepted = &storage.fields[storage.accepted_slot];
        result.prepared = &storage.candidate_fields();
        result.prepared_view = storage.prepared;
        result.accepted_groups.resize(group_count);
        result.prepared_groups.resize(group_count);
        tl::fea::NodalStamp accepted_stamp;
        auto report = owner.CopyAcceptedRigidGroups(
            {result.accepted_groups.data(), group_count},
            &accepted_stamp);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        tl::fea::NodalPreparedView prepared_view;
        report = owner.CopyPreparedRigidGroups(
            storage.token,
            {result.prepared_groups.data(), group_count},
            &prepared_view);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        if (!tl::fea::trial_identity::SameStamp(
                accepted_stamp, storage.stamp) ||
            !tl::fea::trial_identity::SamePrepared(
                prepared_view, storage.prepared))
            throw std::runtime_error(
                "Coupon rigid snapshots differ from the prepared owner");
        return result;
    }
};

}  // namespace crash::cases::vehicle_self_contact

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test {
namespace {

namespace contact = tlfea::contact;
namespace dynamics = vehicle_dynamics;
namespace fe = tl::fea;
namespace sct = tlfea::contact::self_contact_transaction;

constexpr std::uint64_t LinearEid = 2100005;
constexpr std::uint64_t MixedEid = 2100048;
constexpr unsigned CouponLocalFacet = 0;
constexpr double PhysicalStepS = 2e-7;

double Down(double value) {
    return std::nextafter(
        value, -std::numeric_limits<double>::infinity());
}

double Up(double value) {
    return std::nextafter(
        value, std::numeric_limits<double>::infinity());
}

double Component(contact::Vec3 value, unsigned component) {
    return component == 0 ? value.x :
        component == 1 ? value.y : value.z;
}

void SetComponent(contact::Vec3* value, unsigned component,
                  double next) {
    if (component == 0) value->x = next;
    else if (component == 1) value->y = next;
    else value->z = next;
}

contact::Vec3 Point(const dynamics::Fields& fields,
                    std::size_t node) {
    return {
        fields.position[3 * node],
        fields.position[3 * node + 1],
        fields.position[3 * node + 2]};
}

std::size_t ParentOrdinal(
    const contact::SelfContactActiveUseBinding& active,
    std::uint64_t eid) {
    for (std::size_t parent = 0;
         parent < active.parents().size(); ++parent)
        if (active.parents()[parent].source.source_parent_id == eid)
            return parent;
    throw std::runtime_error("Coupon active parent is absent");
}

std::uint32_t RigidGroup(
    const fe::NodalRigidAssemblyBinding& rigid,
    std::size_t node) {
    const auto* member = rigid.FindMember(node);
    if (!member) return UINT32_MAX;
    const auto members = rigid.members();
    if (member < members.data() ||
        member >= members.data() + members.size())
        throw std::runtime_error(
            "Coupon rigid member is outside the authenticated roster");
    const auto member_index =
        static_cast<std::size_t>(member - members.data());
    for (std::size_t group = 0;
         group < rigid.groups().size(); ++group) {
        const auto& value = rigid.groups()[group];
        if (member_index >= value.member_offset &&
            member_index < value.member_offset + value.member_count)
            return static_cast<std::uint32_t>(group);
    }
    throw std::runtime_error(
        "Coupon rigid member has no authenticated group");
}

contact::SelfContactFacetMotion FacetMotion(
    const contact::FixedContactFacet& facet,
    const fe::NodalRigidAssemblyBinding& rigid) {
    bool all_linear = true;
    bool all_complete = true;
    std::uint32_t common = UINT32_MAX;
    for (const auto& vertex : facet.vertices) {
        std::uint32_t point_group = UINT32_MAX;
        bool point_all_rigid = true;
        bool point_same = true;
        std::size_t nonzero = 0;
        for (unsigned slot = 0; slot < vertex.count; ++slot) {
            if (vertex.weights[slot] == 0) continue;
            ++nonzero;
            const auto group = RigidGroup(rigid, vertex.nodes[slot]);
            if (group == UINT32_MAX) {
                point_all_rigid = false;
                point_same = false;
            } else if (point_group == UINT32_MAX) {
                point_group = group;
            } else if (point_group != group) {
                point_same = false;
            }
        }
        if (!nonzero)
            throw std::runtime_error(
                "Coupon facet vertex has no contributing node");
        const bool point_complete =
            point_all_rigid && point_same &&
            point_group != UINT32_MAX;
        all_linear = all_linear && point_group == UINT32_MAX;
        all_complete = all_complete && point_complete;
        if (point_complete) {
            if (common == UINT32_MAX) common = point_group;
            else if (common != point_group) all_complete = false;
        }
    }
    if (all_linear)
        return contact::SelfContactFacetMotion::LinearNodalV1;
    if (all_complete && common != UINT32_MAX)
        return contact::SelfContactFacetMotion::CompleteRigidGroup;
    return contact::SelfContactFacetMotion::PartialOrMixedRigid;
}

contact::SelfContactSweptParentBounds NodeBounds(
    std::size_t node,
    const dynamics::Fields& accepted,
    const dynamics::Fields& prepared,
    const fe::NodalRigidAssemblyBinding& rigid,
    const vehicle_self_contact::CandidateRigidCouponSnapshot& snapshot) {
    const auto first = Point(accepted, node);
    const auto second = Point(prepared, node);
    const auto group = RigidGroup(rigid, node);
    contact::SelfContactSweptParentBounds result;
    if (group == UINT32_MAX) {
        for (unsigned component = 0; component < 3; ++component) {
            SetComponent(&result.lower, component, std::min(
                Component(first, component),
                Component(second, component)));
            SetComponent(&result.upper, component, std::max(
                Component(first, component),
                Component(second, component)));
        }
        return result;
    }
    if (group >= snapshot.accepted_groups.size() ||
        sct::BuildRigidMemberSweepBounds(
            first, second,
            snapshot.accepted_groups[group],
            snapshot.prepared_groups[group],
            snapshot.prepared_view.rigid_member_trajectory,
            snapshot.prepared_view.proposed_time -
                snapshot.prepared_view.base_time,
            &result) != sct::RigidMemberSweepStatus::Ok)
        throw std::runtime_error(
            "Coupon rigid member sweep certificate failed");
    return result;
}

contact::SelfContactSweptParentBounds FacetBounds(
    const contact::FixedContactFacet& facet,
    const dynamics::Fields& accepted,
    const dynamics::Fields& prepared,
    const fe::NodalRigidAssemblyBinding& rigid,
    const vehicle_self_contact::CandidateRigidCouponSnapshot& snapshot) {
    const double infinity = std::numeric_limits<double>::infinity();
    contact::SelfContactSweptParentBounds result{
        {infinity, infinity, infinity},
        {-infinity, -infinity, -infinity}};
    for (const auto& vertex : facet.vertices) {
        contact::SelfContactSweptParentBounds point;
        for (unsigned component = 0; component < 3; ++component) {
            double lower = 0;
            double upper = 0;
            for (unsigned slot = 0; slot < vertex.count; ++slot) {
                if (vertex.weights[slot] == 0) continue;
                const auto node = NodeBounds(
                    vertex.nodes[slot], accepted, prepared,
                    rigid, snapshot);
                lower = Down(lower + Down(
                    vertex.weights[slot] *
                    Component(node.lower, component)));
                upper = Up(upper + Up(
                    vertex.weights[slot] *
                    Component(node.upper, component)));
            }
            SetComponent(&point.lower, component, lower);
            SetComponent(&point.upper, component, upper);
            SetComponent(&result.lower, component, std::min(
                Component(result.lower, component), lower));
            SetComponent(&result.upper, component, std::max(
                Component(result.upper, component), upper));
        }
    }
    for (unsigned component = 0; component < 3; ++component) {
        SetComponent(&result.lower, component, Down(
            Component(result.lower, component) -
            facet.reference_half_thickness_m));
        SetComponent(&result.upper, component, Up(
            Component(result.upper, component) +
            facet.reference_half_thickness_m));
    }
    return result;
}

bool Overlap(const contact::SelfContactSweptParentBounds& first,
             const contact::SelfContactSweptParentBounds& second) {
    for (unsigned component = 0; component < 3; ++component)
        if (Component(first.upper, component) <
                Component(second.lower, component) ||
            Component(second.upper, component) <
                Component(first.lower, component))
            return false;
    return true;
}

struct EndpointGeometry {
    std::vector<contact::FixedTriangleFeatureCandidate> features;
    std::vector<contact::FixedTriangleIntersection> intersections;
    contact::FixedTriangleDiscoveryReport report;
};

EndpointGeometry Discover(
    const std::array<contact::CurrentFixedTriangle, 2>& triangles,
    const contact::FixedTriangleFeatureTaskMask& mask) {
    contact::FixedTriangleFeatureLimits limits;
    limits.max_input_pairs = 1;
    limits.max_triangle_references = 2;
    limits.max_vertex_references = 6;
    limits.max_edge_references = 6;
    limits.max_raw_feature_candidates = 15;
    limits.max_feature_candidates = 15;
    limits.max_raw_intersections = 1;
    limits.max_intersections = 1;
    limits.max_host_bytes = 16u << 20;
    contact::FixedTriangleFeatureDiscovery discovery;
    auto report = discovery.Initialize(limits);
    if (report.status != contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(report.message);
    const contact::FixedTrianglePair pair{0, 1};
    report = discovery.DiscoverMasked(
        triangles.data(), triangles.size(), &pair, 1, &mask);
    if (report.status != contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(report.message);
    EndpointGeometry result;
    result.report = report;
    const auto features = discovery.features();
    const auto intersections = discovery.intersections();
    if (!features.complete || !intersections.complete)
        throw std::runtime_error(
            "Coupon endpoint geometry publication is incomplete");
    if (features.count)
        result.features.assign(
            features.data, features.data + features.count);
    if (intersections.count)
        result.intersections.assign(
            intersections.data,
            intersections.data + intersections.count);
    return result;
}

double MinimumDistance(const EndpointGeometry& geometry) {
    double result = std::numeric_limits<double>::infinity();
    for (const auto& feature : geometry.features)
        result = std::min(result, feature.distance_m);
    return result;
}

long double QuadraticComponent(
    contact::Vec3 member,
    const fe::NodalRigidGroupSnapshot& accepted_group,
    const fe::NodalRigidGroupSnapshot& prepared_group,
    unsigned component) {
    const long double r[3]{
        static_cast<long double>(member.x) -
            accepted_group.state.center.x,
        static_cast<long double>(member.y) -
            accepted_group.state.center.y,
        static_cast<long double>(member.z) -
            accepted_group.state.center.z};
    const long double w[3]{
        prepared_group.state.omega.x,
        prepared_group.state.omega.y,
        prepared_group.state.omega.z};
    const long double first[3]{
        w[1] * r[2] - w[2] * r[1],
        w[2] * r[0] - w[0] * r[2],
        w[0] * r[1] - w[1] * r[0]};
    const long double second[3]{
        w[1] * first[2] - w[2] * first[1],
        w[2] * first[0] - w[0] * first[2],
        w[0] * first[1] - w[1] * first[0]};
    return second[component];
}

void PrintVertexKey(const contact::FacetVertexKey& key) {
    std::cout << static_cast<unsigned>(key.kind) << ":"
              << key.first << ":" << key.second << ":"
              << key.numerator << "/" << key.denominator << ":"
              << key.level << ":" << key.grid_i << ":" << key.grid_j;
}

void PrintEdgeKey(const contact::FacetEdgeKey& key) {
    std::cout << (key.parent_boundary ? "boundary" : "local")
              << ":" << key.parent_eid << ":";
    PrintVertexKey(key.endpoints[0]);
    std::cout << "-";
    PrintVertexKey(key.endpoints[1]);
}

TEST(VehicleSelfContactCandidateCoupon,
     Candidate142IsAuthenticatedLocalIncidenceBeforeMotionPolicy) {
    const auto& setup = LevelZeroSetup();
    const auto& active = setup.active_uses();
    const auto* rigid = active.rigid();
    ASSERT_NE(rigid, nullptr);
    const std::size_t parent_ordinals[2]{
        ParentOrdinal(active, LinearEid),
        ParentOrdinal(active, MixedEid)};
    std::array<contact::FixedContactFacet, 2> facets;
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        ASSERT_LT(CouponLocalFacet, parent.facet_count);
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, CouponLocalFacet,
                      &facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
        EXPECT_EQ(facets[side].source.source_parent_id,
                  side == 0 ? LinearEid : MixedEid);
        EXPECT_EQ(facets[side].parent_index, parent.surface_parent);
    }

    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = PhysicalStepS;
    auto dynamics = dynamics::VehiclePhysicalDynamics::Prepare(
        Execution(), PhysicalAttachments(), dynamics_config,
        &physical_model::supports_test::Joints());
    const auto snapshot =
        vehicle_self_contact::CandidateRigidCouponAccess::Prepare(
            dynamics);
    ASSERT_NE(snapshot.accepted, nullptr);
    ASSERT_NE(snapshot.prepared, nullptr);
    ASSERT_EQ(
        snapshot.prepared_view.rigid_member_trajectory,
        fe::NodalRigidMemberTrajectory::
            EndpointCorrectedSecondOrderDriftV1);
    EXPECT_EQ(snapshot.prepared_view.proposed_time -
                  snapshot.prepared_view.base_time,
              PhysicalStepS);

    const contact::VectorView accepted_positions{
        snapshot.accepted->position.data(),
        static_cast<std::uint32_t>(dynamics.accepted().node_count),
        3, 1};
    const contact::VectorView prepared_positions{
        snapshot.prepared->position.data(),
        static_cast<std::uint32_t>(dynamics.accepted().node_count),
        3, 1};
    std::array<contact::CurrentFixedTriangle, 2> accepted_triangles;
    std::array<contact::CurrentFixedTriangle, 2> prepared_triangles;
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      facets[side], accepted_positions,
                      &accepted_triangles[side]),
                  contact::Status::kOk);
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      facets[side], prepared_positions,
                      &prepared_triangles[side]),
                  contact::Status::kOk);
    }

    EXPECT_EQ(FacetMotion(facets[0], *rigid),
              contact::SelfContactFacetMotion::LinearNodalV1);
    EXPECT_EQ(FacetMotion(facets[1], *rigid),
              contact::SelfContactFacetMotion::PartialOrMixedRigid);
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  accepted_triangles[0], accepted_triangles[1],
                  &accepted_mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  prepared_triangles[0], prepared_triangles[1],
                  &prepared_mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    EXPECT_EQ(prepared_mask.local_tasks, accepted_mask.local_tasks);
    EXPECT_EQ(accepted_mask.local_tasks, 0x0da4u);
    unsigned shared_vertices = 0;
    for (const auto& first : facets[0].vertex_keys)
        for (const auto& second : facets[1].vertex_keys)
            shared_vertices +=
                contact::SameFacetVertexKey(first, second);
    EXPECT_EQ(shared_vertices, 1u);

    const auto accepted_geometry =
        Discover(accepted_triangles, accepted_mask);
    const auto prepared_geometry =
        Discover(prepared_triangles, prepared_mask);
    EXPECT_EQ(accepted_geometry.report.exact_executed_tasks, 9u);
    EXPECT_EQ(prepared_geometry.report.exact_executed_tasks, 9u);
    EXPECT_EQ(accepted_geometry.features.size(), 9u);
    EXPECT_EQ(prepared_geometry.features.size(), 9u);
    const double contact_thickness =
        facets[0].reference_half_thickness_m +
        facets[1].reference_half_thickness_m;
    const double accepted_minimum =
        MinimumDistance(accepted_geometry);
    const double prepared_minimum =
        MinimumDistance(prepared_geometry);
    EXPECT_GT(accepted_minimum, contact_thickness);
    EXPECT_GT(prepared_minimum, contact_thickness);
    ASSERT_FALSE(accepted_geometry.intersections.empty());
    ASSERT_FALSE(prepared_geometry.intersections.empty());
    for (const auto& intersection : accepted_geometry.intersections)
        EXPECT_FALSE(contact::RequiresIntersectionAdmission(intersection));
    for (const auto& intersection : prepared_geometry.intersections)
        EXPECT_FALSE(contact::RequiresIntersectionAdmission(intersection));

    const contact::SelfContactSweptParentBounds swept[2]{
        FacetBounds(facets[0], *snapshot.accepted,
                    *snapshot.prepared, *rigid, snapshot),
        FacetBounds(facets[1], *snapshot.accepted,
                    *snapshot.prepared, *rigid, snapshot)};
    EXPECT_TRUE(Overlap(swept[0], swept[1]));

    std::cout << std::setprecision(17)
              << "V5_CANDIDATE_142"
              << " transaction_candidate=8"
              << " pair_index=142"
              << " dt_s="
              << snapshot.prepared_view.proposed_time -
                     snapshot.prepared_view.base_time
              << " kick_dt_s=" << snapshot.prepared_view.kick_dt
              << " trajectory="
              << static_cast<unsigned>(
                     snapshot.prepared_view.rigid_member_trajectory)
              << " local_mask=0x" << std::hex
              << accepted_mask.local_tasks << std::dec
              << " shared_vertices=" << shared_vertices
              << " accepted_intersection_kind="
              << static_cast<unsigned>(
                     accepted_geometry.intersections[0].kind)
              << " accepted_local_exclusion="
              << static_cast<unsigned>(
                     accepted_geometry.intersections[0].local_exclusion)
              << " prepared_intersection_kind="
              << static_cast<unsigned>(
                     prepared_geometry.intersections[0].kind)
              << " prepared_local_exclusion="
              << static_cast<unsigned>(
                     prepared_geometry.intersections[0].local_exclusion)
              << " accepted_unmasked_features="
              << accepted_geometry.features.size()
              << " prepared_unmasked_features="
              << prepared_geometry.features.size()
              << " contact_thickness_m=" << contact_thickness
              << " accepted_minimum_nonlocal_m="
              << accepted_minimum
              << " prepared_minimum_nonlocal_m="
              << prepared_minimum
              << " swept_boxes_overlap=1"
              << " overlap_only_local_incidence=1"
              << '\n';
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        const auto facet_use_index =
            parent.facet_offset + CouponLocalFacet;
        ASSERT_LT(facet_use_index, active.facet_uses().size());
        const auto& facet_use = active.facet_uses()[facet_use_index];
        EXPECT_EQ(facet_use.parent, parent_ordinals[side]);
        EXPECT_EQ(facet_use.local_facet, CouponLocalFacet);
        std::cout << "V5_CANDIDATE_142_PARENT"
                  << " side=" << side
                  << " active_parent=" << parent_ordinals[side]
                  << " surface_parent=" << parent.surface_parent
                  << " active_facet_use=" << facet_use_index
                  << " eid=" << parent.source.source_parent_id
                  << " pid=" << parent.source.source_part_id
                  << " mid=" << parent.source.material_id
                  << " sid=" << parent.source.section_id
                  << " arity=" << parent.arity
                  << " facet_offset=" << parent.facet_offset
                  << " facet_count=" << parent.facet_count
                  << " half_thickness_m="
                  << parent.reference_half_thickness_m
                  << " motion="
                  << static_cast<unsigned>(
                         FacetMotion(facets[side], *rigid))
                  << " nodes=";
        for (unsigned slot = 0; slot < parent.arity; ++slot) {
            if (slot) std::cout << ",";
            const auto node = parent.nodes[slot];
            const auto group = RigidGroup(*rigid, node);
            std::cout << NativeNodeId(
                             setup.physical(), parent.source, slot)
                      << "/" << node << "@";
            if (group == UINT32_MAX) {
                std::cout << "linear";
            } else {
                const auto& identity = rigid->groups()[group];
                std::cout << group << "/"
                          << static_cast<unsigned>(identity.source_kind)
                          << "/" << identity.source_id
                          << "/" << identity.source_node_set_id;
            }
        }
        std::cout << '\n';
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            std::cout << "V5_CANDIDATE_142_VERTEX"
                      << " side=" << side
                      << " local=" << vertex
                      << " active_use="
                      << facet_use.vertex_uses[vertex]
                      << " feature="
                      << facet_use.vertex_features[vertex]
                      << " support="
                      << static_cast<unsigned>(
                             active.vertex_uses()[
                                 facet_use.vertex_uses[vertex]].support.status)
                      << " key=";
            PrintVertexKey(facets[side].vertex_keys[vertex]);
            std::cout << " accepted="
                      << accepted_triangles[side].vertices[vertex].x
                      << "," << accepted_triangles[side].vertices[vertex].y
                      << "," << accepted_triangles[side].vertices[vertex].z
                      << " prepared="
                      << prepared_triangles[side].vertices[vertex].x
                      << "," << prepared_triangles[side].vertices[vertex].y
                      << "," << prepared_triangles[side].vertices[vertex].z
                      << " contributors=";
            const auto& point = facets[side].vertices[vertex];
            for (unsigned slot = 0; slot < point.count; ++slot) {
                if (point.weights[slot] == 0) continue;
                const auto node = point.nodes[slot];
                const auto group = RigidGroup(*rigid, node);
                std::cout << node << "*" << point.weights[slot]
                          << "@" << group;
                if (group != UINT32_MAX) {
                    std::cout << "/q=";
                    const auto accepted = Point(*snapshot.accepted, node);
                    for (unsigned component = 0;
                         component < 3; ++component) {
                        if (component) std::cout << ",";
                        const auto q = QuadraticComponent(
                            accepted,
                            snapshot.accepted_groups[group],
                            snapshot.prepared_groups[group],
                            component);
                        EXPECT_EQ(q, 0);
                        std::cout << static_cast<double>(q);
                    }
                }
            }
            std::cout << '\n';
        }
        for (unsigned edge = 0; edge < 3; ++edge) {
            std::cout << "V5_CANDIDATE_142_EDGE"
                      << " side=" << side
                      << " local=" << edge
                      << " active_use="
                      << facet_use.edge_uses[edge]
                      << " feature="
                      << facet_use.edge_features[edge]
                      << " key=";
            PrintEdgeKey(facets[side].edge_keys[edge]);
            std::cout << '\n';
        }
        std::cout << "V5_CANDIDATE_142_SWEPT_BOX"
                  << " side=" << side
                  << " lower=" << swept[side].lower.x << ","
                  << swept[side].lower.y << ","
                  << swept[side].lower.z
                  << " upper=" << swept[side].upper.x << ","
                  << swept[side].upper.y << ","
                  << swept[side].upper.z << '\n';
    }
    std::set<std::uint32_t> contributing_groups;
    for (const auto& facet : facets)
        for (const auto& vertex : facet.vertices)
            for (unsigned slot = 0; slot < vertex.count; ++slot) {
                if (vertex.weights[slot] == 0) continue;
                const auto group =
                    RigidGroup(*rigid, vertex.nodes[slot]);
                if (group != UINT32_MAX)
                    contributing_groups.insert(group);
            }
    ASSERT_EQ(contributing_groups,
              (std::set<std::uint32_t>{612}));
    for (const auto group : contributing_groups) {
        const auto& identity = rigid->groups()[group];
        const auto& accepted = snapshot.accepted_groups[group];
        const auto& prepared = snapshot.prepared_groups[group];
        EXPECT_EQ(prepared.state.omega.x, 0);
        EXPECT_EQ(prepared.state.omega.y, 0);
        EXPECT_EQ(prepared.state.omega.z, 0);
        std::cout << "V5_CANDIDATE_142_RIGID_GROUP"
                  << " binding_group=" << group
                  << " source_kind="
                  << static_cast<unsigned>(identity.source_kind)
                  << " source_id=" << identity.source_id
                  << " source_node_set_id="
                  << identity.source_node_set_id
                  << " accepted_center="
                  << accepted.state.center.x << ","
                  << accepted.state.center.y << ","
                  << accepted.state.center.z
                  << " prepared_center="
                  << prepared.state.center.x << ","
                  << prepared.state.center.y << ","
                  << prepared.state.center.z
                  << " prepared_omega="
                  << prepared.state.omega.x << ","
                  << prepared.state.omega.y << ","
                  << prepared.state.omega.z << '\n';
    }
    dynamics.DiscardStep();
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
