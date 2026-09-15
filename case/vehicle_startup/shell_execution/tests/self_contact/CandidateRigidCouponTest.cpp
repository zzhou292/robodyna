#include "Source.h"

#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_self_contact/RuntimeData.h"
#include "case/vehicle_self_contact/SelfContactFactories.h"
#include "case/vehicle_self_contact/runtime/Stages.h"
#include "case/vehicle_wall/LoadedWall.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"
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

struct AcceptedAssemblyCouponSnapshot {
    std::vector<double> accepted_positions;
    std::vector<double> prepared_positions;
    std::vector<
        tlfea::contact::self_contact_transaction::
            AcceptedEventCertificate> accepted_certificates;
    tl::fea::NodalPreparedView prepared_view;
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

    static AcceptedAssemblyCouponSnapshot PrepareAcceptedAssembly(
        vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
        auto& storage = *dynamics.storage_;
        storage.Prepare();
        auto& owner = storage.state().owner;
        const auto nodes = storage.startup.accepted().node_count;
        AcceptedAssemblyCouponSnapshot result;
        result.accepted_positions.resize(3 * nodes);
        result.prepared_positions.resize(3 * nodes);
        std::vector<double> accepted_velocities(3 * nodes);
        std::vector<double> prepared_velocities(3 * nodes);
        tl::fea::NodalStamp accepted_stamp;
        auto report = owner.CopyAccepted(
            {result.accepted_positions.data(),
             accepted_velocities.data(), nodes},
            &accepted_stamp);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        report = owner.CopyPrepared(
            storage.token,
            {result.prepared_positions.data(),
             prepared_velocities.data(), nodes},
            &result.prepared_view);
        if (report.status != tl::fea::NodalStatus::Ok)
            throw std::runtime_error(report.message);
        if (!tl::fea::trial_identity::SameStamp(
                accepted_stamp, storage.stamp) ||
            !tl::fea::trial_identity::SamePrepared(
                result.prepared_view, storage.prepared))
            throw std::runtime_error(
                "Accepted-assembly coupon differs from owner identity");
        const auto* stages = dynamic_cast<
            const detail::SelfContactStages*>(
                storage.self_contact.get());
        if (!stages)
            throw std::runtime_error(
                "Accepted-assembly coupon has no self-contact stages");
        const auto certificates =
            tlfea::contact::self_contact_transaction::
                QualificationAccess::AcceptedCertificates(
                    stages->contact.data_->transaction);
        if (!certificates.complete ||
            (certificates.count && !certificates.data))
            throw std::runtime_error(
                "Accepted certificate ledger is incomplete");
        if (certificates.count)
            result.accepted_certificates.assign(
                certificates.data,
                certificates.data + certificates.count);
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
constexpr std::uint64_t NonlinearLinearEid = 2100124;
constexpr std::uint64_t NonlinearMixedEid = 2209533;
constexpr std::uint64_t ExhaustedLinearFirstEid = 2100002;
constexpr std::uint64_t ExhaustedLinearSecondEid = 2288690;
constexpr std::uint64_t PersistentLinearFirstEid = 2100082;
constexpr std::uint64_t PersistentLinearSecondEid = 2288743;
constexpr std::uint64_t CanonicalPersistentLinearFirstEid = 2100002;
constexpr std::uint64_t CanonicalPersistentLinearSecondEid = 2288693;
constexpr unsigned CouponLocalFacet = 0;
constexpr unsigned AffineMixedLocalFacet = 1;
constexpr double PhysicalStepS = 2e-7;
constexpr std::size_t FullAcceptedEvents = 32491;
constexpr std::size_t FullParentPairCapacity = 2000000;
constexpr std::size_t FullFacetPairCapacity = 8000000;
constexpr std::size_t FullHostBytes =
    std::size_t{20} * 1000 * 1000 * 1000;
constexpr std::size_t FullDeviceBytes = std::size_t{8} << 30;
constexpr std::uint64_t SelfContactSourceId =
    0x563553454c464354ull;

vehicle_self_contact::RuntimeLimits
FullAcceptedAssemblyLimits() {
    const auto& setup = LevelZeroSetup();
    const contact::SelfContactTransactionLimits::ExactCensus census{
        setup.physical().domain()->node_count(),
        setup.active_uses().parents().size(),
        setup.active_uses().parents().size(),
        setup.counts().q4_parents,
        setup.active_uses().facet_uses().size(),
        FullParentPairCapacity, FullFacetPairCapacity, 0};
    vehicle_self_contact::RuntimeLimits result;
    result.host_bytes = FullHostBytes;
    result.device_bytes = FullDeviceBytes;
    result.transaction =
        contact::SelfContactTransactionLimits::Vehicle(
            census, 4096, FullAcceptedEvents,
            2 * FullAcceptedEvents, 0,
            4095, std::size_t{1} << 20,
            FullFacetPairCapacity * 4095, 20,
            FullHostBytes, FullDeviceBytes, FullHostBytes,
            4, 4, FullAcceptedEvents);
    result.transaction.activity.max_selected_parents = 1000000;
    result.transaction.activity.max_family_parents = 1000000;
    result.transaction.activity.max_host_bytes =
        std::size_t{512} << 20;
    result.transaction.activity.max_startup_host_bytes =
        std::size_t{512} << 20;
    result.transaction.broadphase.max_host_bytes =
        std::size_t{2} << 30;
    result.transaction.broadphase.max_device_bytes =
        std::size_t{2} << 30;
    result.transaction.regularity.max_host_bytes =
        contact::SelfContactCurrentRegularityLimits::Vehicle()
            .max_host_bytes;
    return result;
}

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
    for (const auto& feature : geometry.features) {
        result = std::min(result, feature.distance_m);
    }
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

contact::RepresentedTrianglePath RepresentedPath(
    const contact::CurrentFixedTriangle& accepted,
    const contact::CurrentFixedTriangle& prepared) {
    contact::RepresentedTrianglePath result;
    result.key = {
        prepared.key.source_instance_id, prepared.key.parent_eid,
        prepared.key.level, prepared.key.local_facet};
    result.motion = contact::RepresentedMotion::LinearNodalV1;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
        result.vertices[vertex].key = prepared.vertex_keys[vertex];
        result.vertices[vertex].endpoint[0] =
            accepted.vertices[vertex];
        result.vertices[vertex].endpoint[1] =
            prepared.vertices[vertex];
        result.edge_keys[vertex] = prepared.edge_keys[vertex];
    }
    return result;
}

contact::RepresentedIntervalResult Cross(
    const std::array<contact::RepresentedTrianglePath, 2>& paths,
    std::size_t work, unsigned depth) {
    contact::RepresentedIntervalLimits limits;
    limits.max_paths = 2;
    limits.max_input_pairs = 1;
    limits.max_results = 1;
    limits.max_work_per_pair = work;
    limits.max_total_work = work;
    limits.max_depth = depth;
    limits.max_host_bytes = 16u << 20;
    contact::RepresentedIntervalCrossing crossing;
    const auto initialized = crossing.Initialize(limits);
    if (initialized.status != contact::RepresentedIntervalStatus::Ok)
        throw std::runtime_error(initialized.message);
    const contact::RepresentedTrianglePair pair{0, 1};
    const auto report =
        crossing.Certify(paths.data(), paths.size(), &pair, 1);
    if (report.status != contact::RepresentedIntervalStatus::Ok)
        throw std::runtime_error(report.message);
    const auto results = crossing.results();
    if (!results.complete || results.count != 1)
        throw std::runtime_error(
            "Coupon represented crossing result is incomplete");
    return results.data[0];
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

TEST(VehicleSelfContactCandidateCoupon,
     Candidate143MixedFacetHasExactAffineRepresentedMotion) {
    const auto& setup = LevelZeroSetup();
    const auto& active = setup.active_uses();
    const auto* rigid = active.rigid();
    ASSERT_NE(rigid, nullptr);
    const std::size_t parent_ordinals[2]{
        ParentOrdinal(active, LinearEid),
        ParentOrdinal(active, MixedEid)};
    std::array<contact::FixedContactFacet, 2> facets;
    const unsigned local_facets[2]{
        CouponLocalFacet, AffineMixedLocalFacet};
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        ASSERT_LT(local_facets[side], parent.facet_count);
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, local_facets[side],
                      &facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
    }

    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = PhysicalStepS;
    auto dynamics = dynamics::VehiclePhysicalDynamics::Prepare(
        Execution(), PhysicalAttachments(), dynamics_config,
        &physical_model::supports_test::Joints());
    const auto snapshot =
        vehicle_self_contact::CandidateRigidCouponAccess::Prepare(
            dynamics);
    const auto node_count =
        static_cast<std::uint32_t>(dynamics.accepted().node_count);
    const contact::VectorView accepted_positions{
        snapshot.accepted->position.data(), node_count, 3, 1};
    const contact::VectorView prepared_positions{
        snapshot.prepared->position.data(), node_count, 3, 1};
    std::vector<std::uint32_t> node_groups(node_count, UINT32_MAX);
    for (std::uint32_t node = 0; node < node_count; ++node)
        node_groups[node] = RigidGroup(*rigid, node);

    bool affine[2]{};
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_EQ(sct::CertifyRigidFacetAffineMotion(
                      facets[side], accepted_positions,
                      prepared_positions, node_groups.data(),
                      snapshot.accepted_groups.data(),
                      snapshot.prepared_groups.data(),
                      snapshot.accepted_groups.size(),
                      snapshot.prepared_view.rigid_member_trajectory,
                      PhysicalStepS, affine + side),
                  sct::RigidMemberSweepStatus::Ok);
        EXPECT_TRUE(affine[side]);
    }
    EXPECT_EQ(FacetMotion(facets[0], *rigid),
              contact::SelfContactFacetMotion::LinearNodalV1);
    EXPECT_EQ(FacetMotion(facets[1], *rigid),
              contact::SelfContactFacetMotion::PartialOrMixedRigid);

    const contact::SelfContactSweptParentBounds swept[2]{
        FacetBounds(facets[0], *snapshot.accepted,
                    *snapshot.prepared, *rigid, snapshot),
        FacetBounds(facets[1], *snapshot.accepted,
                    *snapshot.prepared, *rigid, snapshot)};
    EXPECT_TRUE(Overlap(swept[0], swept[1]));
    sct::MotionSupport first;
    first.motion = contact::SelfContactFacetMotion::LinearNodalV1;
    first.certified_affine = affine[0];
    sct::MotionSupport second;
    second.motion =
        contact::SelfContactFacetMotion::PartialOrMixedRigid;
    second.certified_affine = affine[1];
    EXPECT_EQ(sct::ClassifyCandidatePairMotion(
                  first, swept[0], second, swept[1]),
              sct::PairMotionAction::LinearNodalV1);

    std::cout << std::setprecision(17)
              << "V5_CANDIDATE_143"
              << " pair_index=143"
              << " first_eid=" << LinearEid
              << " first_local=" << CouponLocalFacet
              << " first_motion="
              << static_cast<unsigned>(first.motion)
              << " first_affine=" << affine[0]
              << " second_eid=" << MixedEid
              << " second_local=" << AffineMixedLocalFacet
              << " second_motion="
              << static_cast<unsigned>(second.motion)
              << " second_affine=" << affine[1]
              << " represented_action="
              << static_cast<unsigned>(
                     sct::ClassifyCandidatePairMotion(
                         first, swept[0], second, swept[1]))
              << " swept_boxes_overlap=1"
              << " endpoint_chord_substitution=0"
              << '\n';
    dynamics.DiscardStep();
}

TEST(VehicleSelfContactCandidateCoupon,
     Candidate2694SourceTopologyAndPhysicalOnlyBaseline) {
    const auto& setup = LevelZeroSetup();
    const auto& active = setup.active_uses();
    const auto* rigid = active.rigid();
    ASSERT_NE(rigid, nullptr);
    const std::size_t parent_ordinals[2]{
        ParentOrdinal(active, NonlinearLinearEid),
        ParentOrdinal(active, NonlinearMixedEid)};
    std::array<contact::FixedContactFacet, 2> facets;
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        ASSERT_LT(CouponLocalFacet, parent.facet_count);
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, CouponLocalFacet,
                      &facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
    }

    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = PhysicalStepS;
    auto dynamics = dynamics::VehiclePhysicalDynamics::Prepare(
        Execution(), PhysicalAttachments(), dynamics_config,
        &physical_model::supports_test::Joints());
    const auto snapshot =
        vehicle_self_contact::CandidateRigidCouponAccess::Prepare(
            dynamics);
    const auto node_count =
        static_cast<std::uint32_t>(dynamics.accepted().node_count);
    const contact::VectorView accepted_positions{
        snapshot.accepted->position.data(), node_count, 3, 1};
    const contact::VectorView prepared_positions{
        snapshot.prepared->position.data(), node_count, 3, 1};
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
    const auto accepted_geometry =
        Discover(accepted_triangles, accepted_mask);
    const auto prepared_geometry =
        Discover(prepared_triangles, prepared_mask);
    const double contact_thickness =
        facets[0].reference_half_thickness_m +
        facets[1].reference_half_thickness_m;
    const double accepted_minimum =
        MinimumDistance(accepted_geometry);
    const double prepared_minimum =
        MinimumDistance(prepared_geometry);
    unsigned shared_vertices = 0;
    for (const auto& first : facets[0].vertex_keys)
        for (const auto& second : facets[1].vertex_keys)
            shared_vertices +=
                contact::SameFacetVertexKey(first, second);

    std::vector<std::uint32_t> node_groups(node_count, UINT32_MAX);
    for (std::uint32_t node = 0; node < node_count; ++node)
        node_groups[node] = RigidGroup(*rigid, node);
    bool affine[2]{};
    sct::FacetQuadraticCoefficients coefficients[2];
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_EQ(sct::BuildRigidFacetQuadraticCoefficients(
                      facets[side], accepted_positions,
                      prepared_positions, node_groups.data(),
                      snapshot.accepted_groups.data(),
                      snapshot.prepared_groups.data(),
                      snapshot.accepted_groups.size(),
                      snapshot.prepared_view.rigid_member_trajectory,
                      PhysicalStepS, coefficients + side,
                      affine + side),
                  sct::RigidMemberSweepStatus::Ok);
    }
    EXPECT_TRUE(affine[0]);
    // This physical-only snapshot intentionally omits the accepted
    // self-contact assembly that produces the receipt's nonzero group spin.
    EXPECT_TRUE(affine[1]);
    const contact::SelfContactSweptParentBounds swept[2]{
        FacetBounds(facets[0], *snapshot.accepted,
                    *snapshot.prepared, *rigid, snapshot),
        FacetBounds(facets[1], *snapshot.accepted,
                    *snapshot.prepared, *rigid, snapshot)};
    EXPECT_TRUE(Overlap(swept[0], swept[1]));
    const auto nonlinear = sct::CertifyQuadraticFacetSeparation(
        accepted_triangles[0], prepared_triangles[0],
        coefficients[0], facets[0].reference_half_thickness_m,
        accepted_triangles[1], prepared_triangles[1],
        coefficients[1], facets[1].reference_half_thickness_m,
        PhysicalStepS, 4095, 20);
    EXPECT_EQ(nonlinear.status,
              sct::NonlinearSeparationStatus::CertifiedSeparated);

    std::cout << std::setprecision(17)
              << "V5_CANDIDATE_2694"
              << " pair_index=2694"
              << " candidate_facet=222"
              << " first_eid=" << NonlinearLinearEid
              << " first_local=0 first_motion="
              << static_cast<unsigned>(
                     FacetMotion(facets[0], *rigid))
              << " first_affine=" << affine[0]
              << " second_eid=" << NonlinearMixedEid
              << " second_local=0 second_motion="
              << static_cast<unsigned>(
                     FacetMotion(facets[1], *rigid))
              << " second_affine=" << affine[1]
              << " accepted_local_mask=0x" << std::hex
              << accepted_mask.local_tasks
              << " prepared_local_mask=0x"
              << prepared_mask.local_tasks << std::dec
              << " shared_vertices=" << shared_vertices
              << " accepted_intersections="
              << accepted_geometry.intersections.size()
              << " prepared_intersections="
              << prepared_geometry.intersections.size()
              << " accepted_minimum_m=" << accepted_minimum
              << " prepared_minimum_m=" << prepared_minimum
              << " contact_thickness_m=" << contact_thickness
              << " swept_boxes_overlap=1"
              << " physical_only_baseline=1"
              << " accepted_self_contact_assembly_included=0"
              << " nonlinear_status="
              << static_cast<unsigned>(nonlinear.status)
              << " nonlinear_work=" << nonlinear.work
              << " nonlinear_depth=" << nonlinear.deepest
              << '\n';
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        std::cout << "V5_CANDIDATE_2694_PARENT"
                  << " side=" << side
                  << " active_parent=" << parent_ordinals[side]
                  << " surface_parent=" << parent.surface_parent
                  << " active_facet_use="
                  << parent.facet_offset + CouponLocalFacet
                  << " eid=" << parent.source.source_parent_id
                  << " pid=" << parent.source.source_part_id
                  << " half_thickness_m="
                  << parent.reference_half_thickness_m
                  << " nodes=";
        for (unsigned slot = 0; slot < parent.arity; ++slot) {
            if (slot) std::cout << ",";
            const auto node = parent.nodes[slot];
            std::cout << NativeNodeId(
                             setup.physical(), parent.source, slot)
                      << "/" << node << "@"
                      << RigidGroup(*rigid, node);
        }
        std::cout << '\n';
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            std::cout << "V5_CANDIDATE_2694_VERTEX"
                      << " side=" << side
                      << " local=" << vertex
                      << " accepted="
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
                    const auto accepted =
                        Point(*snapshot.accepted, node);
                    std::cout << "/q=";
                    for (unsigned component = 0;
                         component < 3; ++component) {
                        if (component) std::cout << ",";
                        std::cout << static_cast<double>(
                            QuadraticComponent(
                                accepted,
                                snapshot.accepted_groups[group],
                                snapshot.prepared_groups[group],
                                component));
                    }
                }
                std::cout << ";";
            }
            std::cout << '\n';
            std::cout << std::hexfloat
                      << "V5_CANDIDATE_2694_EXACT_Q_INTERVAL"
                      << " side=" << side
                      << " local=" << vertex;
            for (unsigned component = 0;
                 component < 3; ++component)
                std::cout << " q" << component << "=["
                          << coefficients[side].q[vertex][component].lower
                          << ","
                          << coefficients[side].q[vertex][component].upper
                          << "]";
            std::cout << std::defaultfloat << '\n';
        }
        std::cout << "V5_CANDIDATE_2694_SWEPT_BOX"
                  << " side=" << side
                  << " lower=" << swept[side].lower.x << ","
                  << swept[side].lower.y << ","
                  << swept[side].lower.z
                  << " upper=" << swept[side].upper.x << ","
                  << swept[side].upper.y << ","
                  << swept[side].upper.z << '\n';
    }
    dynamics.DiscardStep();
}

TEST(VehicleSelfContactCandidateCoupon,
     ExhaustedLinearPairReproducesFromAuthenticatedYarisState) {
    const auto& setup = LevelZeroSetup();
    const auto& active = setup.active_uses();
    const auto* rigid = active.rigid();
    ASSERT_NE(rigid, nullptr);
    const std::size_t parent_ordinals[2]{
        ParentOrdinal(active, ExhaustedLinearFirstEid),
        ParentOrdinal(active, ExhaustedLinearSecondEid)};
    std::array<contact::FixedContactFacet, 2> facets;
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        ASSERT_LT(CouponLocalFacet, parent.facet_count);
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, CouponLocalFacet,
                      &facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
        EXPECT_EQ(FacetMotion(facets[side], *rigid),
                  contact::SelfContactFacetMotion::LinearNodalV1);
    }

    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = PhysicalStepS;
    auto dynamics = dynamics::VehiclePhysicalDynamics::Prepare(
        Execution(), PhysicalAttachments(), dynamics_config,
        &physical_model::supports_test::Joints());
    const auto snapshot =
        vehicle_self_contact::CandidateRigidCouponAccess::Prepare(
            dynamics);
    EXPECT_EQ(snapshot.prepared_view.proposed_time -
                  snapshot.prepared_view.base_time,
              PhysicalStepS);
    const auto node_count =
        static_cast<std::uint32_t>(dynamics.accepted().node_count);
    const contact::VectorView accepted_positions{
        snapshot.accepted->position.data(), node_count, 3, 1};
    const contact::VectorView prepared_positions{
        snapshot.prepared->position.data(), node_count, 3, 1};
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
    const auto accepted_geometry =
        Discover(accepted_triangles, accepted_mask);
    const auto prepared_geometry =
        Discover(prepared_triangles, prepared_mask);
    const double thickness =
        facets[0].reference_half_thickness_m +
        facets[1].reference_half_thickness_m;
    const double accepted_minimum =
        MinimumDistance(accepted_geometry);
    const double prepared_minimum =
        MinimumDistance(prepared_geometry);
    ASSERT_FALSE(accepted_geometry.features.empty());
    ASSERT_FALSE(prepared_geometry.features.empty());
    const auto closest = [](const EndpointGeometry& geometry)
        -> const contact::FixedTriangleFeatureCandidate& {
        return *std::min_element(
            geometry.features.begin(), geometry.features.end(),
            [](const auto& first, const auto& second) {
                return first.distance_m < second.distance_m;
            });
    };
    const auto& accepted_closest = closest(accepted_geometry);
    const auto& prepared_closest = closest(prepared_geometry);
    unsigned shared_vertices = 0;
    unsigned shared_edges = 0;
    for (const auto& first : facets[0].vertex_keys)
        for (const auto& second : facets[1].vertex_keys)
            shared_vertices +=
                contact::SameFacetVertexKey(first, second);
    for (const auto& first : facets[0].edge_keys)
        for (const auto& second : facets[1].edge_keys)
            shared_edges +=
                contact::SameFacetEdgeKey(first, second);
    EXPECT_EQ(accepted_mask.local_tasks, 0u);
    EXPECT_EQ(prepared_mask.local_tasks, 0u);
    EXPECT_EQ(shared_vertices, 0u);
    EXPECT_EQ(shared_edges, 0u);
    EXPECT_TRUE(accepted_geometry.intersections.empty());
    EXPECT_TRUE(prepared_geometry.intersections.empty());
    EXPECT_GT(accepted_minimum, thickness);
    EXPECT_GT(prepared_minimum, thickness);

    bool prism_valid = false;
    sct::FacetPrismSeparationAxis prism_axis =
        sct::FacetPrismSeparationAxis::None;
    const bool prism_separated =
        sct::CertifiedLinearFacetPrismSeparation(
            accepted_triangles[0], prepared_triangles[0],
            facets[0].reference_half_thickness_m,
            accepted_triangles[1], prepared_triangles[1],
            facets[1].reference_half_thickness_m,
            sct::FacetPrismAxisLimit::VertexVertex,
            &prism_axis, &prism_valid);
    EXPECT_TRUE(prism_valid);
    EXPECT_FALSE(prism_separated);

    const std::array<contact::RepresentedTrianglePath, 2> paths{
        RepresentedPath(
            accepted_triangles[0], prepared_triangles[0]),
        RepresentedPath(
            accepted_triangles[1], prepared_triangles[1])};
    const auto production = Cross(paths, 4095, 20);
    EXPECT_EQ(
        production.classification,
        contact::RepresentedIntervalClassification::CertifiedSeparated);
    EXPECT_EQ(
        production.reason,
        contact::RepresentedIntervalReason::None);
    EXPECT_EQ(production.work, 1u);

    std::cout << std::setprecision(17)
              << "V5_LINEAR_EXHAUSTED_PAIR"
              << " dt_s="
              << snapshot.prepared_view.proposed_time -
                     snapshot.prepared_view.base_time
              << " accepted_local_mask=0x" << std::hex
              << accepted_mask.local_tasks
              << " prepared_local_mask=0x"
              << prepared_mask.local_tasks << std::dec
              << " accepted_features="
              << accepted_geometry.features.size()
              << " prepared_features="
              << prepared_geometry.features.size()
              << " accepted_closest_kind="
              << static_cast<unsigned>(accepted_closest.key.kind)
              << " accepted_closest_local="
              << accepted_closest.local_features[0] << ","
              << accepted_closest.local_features[1]
              << " accepted_closest_error_m="
              << accepted_closest.representation_error_m
              << " prepared_closest_kind="
              << static_cast<unsigned>(prepared_closest.key.kind)
              << " prepared_closest_local="
              << prepared_closest.local_features[0] << ","
              << prepared_closest.local_features[1]
              << " prepared_closest_error_m="
              << prepared_closest.representation_error_m
              << " accepted_intersections="
              << accepted_geometry.intersections.size()
              << " prepared_intersections="
              << prepared_geometry.intersections.size()
              << " thickness_m=" << thickness
              << " accepted_minimum_m=" << accepted_minimum
              << " prepared_minimum_m=" << prepared_minimum
              << " shared_vertices=" << shared_vertices
              << " shared_edges=" << shared_edges
              << " prism_separated=" << prism_separated
              << " prism_axis="
              << static_cast<unsigned>(prism_axis)
              << " production_classification="
              << static_cast<unsigned>(production.classification)
              << " production_reason="
              << static_cast<unsigned>(production.reason)
              << " production_work=" << production.work
              << " legacy_work_exhausted=4095"
              << " classification=separated_common_translation"
              << '\n';
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        std::cout << "V5_LINEAR_EXHAUSTED_PARENT"
                  << " side=" << side
                  << " active_parent=" << parent_ordinals[side]
                  << " surface_parent=" << parent.surface_parent
                  << " active_facet_use="
                  << parent.facet_offset + CouponLocalFacet
                  << " eid=" << parent.source.source_parent_id
                  << " pid=" << parent.source.source_part_id
                  << " mid=" << parent.source.material_id
                  << " sid=" << parent.source.section_id
                  << " arity=" << parent.arity
                  << " half_thickness_m="
                  << parent.reference_half_thickness_m
                  << " nodes=";
        for (unsigned slot = 0; slot < parent.arity; ++slot) {
            if (slot) std::cout << ",";
            const auto node = parent.nodes[slot];
            std::cout << NativeNodeId(
                             setup.physical(), parent.source, slot)
                      << "/" << node << "@"
                      << RigidGroup(*rigid, node);
        }
        std::cout << '\n';
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            std::cout << "V5_LINEAR_EXHAUSTED_VERTEX"
                      << " side=" << side
                      << " local=" << vertex
                      << " key=";
            PrintVertexKey(facets[side].vertex_keys[vertex]);
            std::cout << " accepted="
                      << accepted_triangles[side].vertices[vertex].x
                      << ","
                      << accepted_triangles[side].vertices[vertex].y
                      << ","
                      << accepted_triangles[side].vertices[vertex].z
                      << " prepared="
                      << prepared_triangles[side].vertices[vertex].x
                      << ","
                      << prepared_triangles[side].vertices[vertex].y
                      << ","
                      << prepared_triangles[side].vertices[vertex].z
                      << '\n';
        }
    }
    for (unsigned depth = 0; depth <= 11; ++depth) {
        const auto complete_tree =
            (std::size_t{1} << (depth + 1)) - 1;
        const auto result = Cross(paths, complete_tree, depth);
        EXPECT_EQ(
            result.classification,
            contact::RepresentedIntervalClassification::
                CertifiedSeparated);
        EXPECT_EQ(
            result.reason,
            contact::RepresentedIntervalReason::None);
        EXPECT_EQ(result.work, 1u);
        std::cout << "V5_LINEAR_EXHAUSTED_PROGRESS"
                  << " depth=" << depth
                  << " cap=" << complete_tree
                  << " classification="
                  << static_cast<unsigned>(result.classification)
                  << " reason="
                  << static_cast<unsigned>(result.reason)
                  << " work=" << result.work << '\n';
    }
    dynamics.DiscardStep();
}

TEST(VehicleSelfContactAcceptedAssemblyCoupon,
     ResidualAndPersistentPairsUseAuthenticatedPreparedBits) {
    const auto& setup = LevelZeroSetup();
    const auto& active = setup.active_uses();
    const std::size_t parent_ordinals[2]{
        ParentOrdinal(active, PersistentLinearFirstEid),
        ParentOrdinal(active, PersistentLinearSecondEid)};
    std::array<contact::FixedContactFacet, 2> facets;
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[parent_ordinals[side]];
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, CouponLocalFacet,
                      &facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
    }

    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = PhysicalStepS;
    const vehicle_self_contact::RuntimeConfig runtime_config{
        SelfContactSourceId, FullAcceptedEvents, 0};
    auto dynamics =
        vehicle_self_contact::SelfContactOnly::Prepare(
            setup, dynamics_config, runtime_config,
            FullAcceptedAssemblyLimits(),
            &physical_model::supports_test::Joints());
    const auto snapshot =
        vehicle_self_contact::CandidateRigidCouponAccess::
            PrepareAcceptedAssembly(dynamics);
    ASSERT_EQ(snapshot.accepted_positions.size(),
              snapshot.prepared_positions.size());
    const auto node_count = static_cast<std::uint32_t>(
        snapshot.accepted_positions.size() / 3);
    const contact::VectorView accepted_positions{
        snapshot.accepted_positions.data(), node_count, 3, 1};
    const contact::VectorView prepared_positions{
        snapshot.prepared_positions.data(), node_count, 3, 1};
    std::array<contact::CurrentFixedTriangle, 2> accepted;
    std::array<contact::CurrentFixedTriangle, 2> prepared;
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      facets[side], accepted_positions,
                      &accepted[side]),
                  contact::Status::kOk);
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      facets[side], prepared_positions,
                      &prepared[side]),
                  contact::Status::kOk);
    }

    contact::FixedTriangleFeatureTaskMask mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  prepared[0], prepared[1], &mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(mask.local_tasks, 0u);
    const auto geometry = Discover(prepared, mask);
    ASSERT_EQ(geometry.features.size(), 15u);
    ASSERT_TRUE(geometry.intersections.empty());
    const contact::FixedTriangleFeatureView feature_view{
        geometry.features.data(), geometry.features.size(), true};
    const contact::FixedTriangleIntersectionView intersection_view{
        nullptr, 0, true};
    const auto residual =
        sct::CertifyLinearResidualSeparation(
            accepted[0], prepared[0],
            facets[0].reference_half_thickness_m,
            accepted[1], prepared[1],
            facets[1].reference_half_thickness_m,
            feature_view, intersection_view);
    EXPECT_NE(
        residual.status,
        sct::LinearResidualSeparationStatus::InvalidInput);
    const auto persistent =
        sct::CertifyPersistentLinearContact(
            accepted[0], prepared[0],
            facets[0].reference_half_thickness_m,
            accepted[1], prepared[1],
            facets[1].reference_half_thickness_m,
            feature_view,
            snapshot.accepted_certificates.data(),
            snapshot.accepted_certificates.size());
    EXPECT_EQ(
        persistent.status,
        sct::PersistentLinearContactStatus::CertifiedContact);
    EXPECT_FALSE(persistent.exact_common_translation);
    if (persistent.status ==
        sct::PersistentLinearContactStatus::CertifiedContact) {
        EXPECT_EQ(persistent.feature.kind,
                  contact::RepresentedFeatureKind::VertexFace);
        EXPECT_LT(persistent.accepted_certificate,
                  snapshot.accepted_certificates.size());
    }
    EXPECT_GT(persistent.first_residual_upper_m, 0);
    EXPECT_GT(persistent.second_residual_upper_m, 0);
    EXPECT_GT(persistent.strict_thickness_margin_lower_m, 0);

    const std::array<contact::RepresentedTrianglePath, 2> paths{
        RepresentedPath(accepted[0], prepared[0]),
        RepresentedPath(accepted[1], prepared[1])};
    const auto exact_cap_one = Cross(paths, 1, 20);
    EXPECT_EQ(
        exact_cap_one.classification,
        contact::RepresentedIntervalClassification::Unresolved);
    EXPECT_EQ(exact_cap_one.reason,
              contact::RepresentedIntervalReason::WorkExhausted);
    EXPECT_EQ(exact_cap_one.work, 1u);
    const auto exact_full = Cross(paths, 4095, 20);
    EXPECT_EQ(
        exact_full.classification,
        contact::RepresentedIntervalClassification::Unresolved);
    EXPECT_EQ(exact_full.reason,
              contact::RepresentedIntervalReason::WorkExhausted);
    EXPECT_EQ(exact_full.work, 4095u);

    unsigned shared_vertices = 0;
    unsigned shared_edges = 0;
    for (const auto& first : facets[0].vertex_keys)
        for (const auto& second : facets[1].vertex_keys)
            shared_vertices +=
                contact::SameFacetVertexKey(first, second);
    for (const auto& first : facets[0].edge_keys)
        for (const auto& second : facets[1].edge_keys)
            shared_edges +=
                contact::SameFacetEdgeKey(first, second);
    const auto same_triangle = [](const contact::FixedTriangleKey& first,
                                  const contact::FixedTriangleKey& second) {
        return contact::fixed_triangle_features::Compare(
                   first, second) == 0;
    };
    std::size_t accepted_matches = 0;
    std::size_t accepted_feature_matches = 0;
    for (std::size_t certificate_index = 0;
         certificate_index <
             snapshot.accepted_certificates.size();
         ++certificate_index) {
        const auto& certificate =
            snapshot.accepted_certificates[
                certificate_index];
        const bool same_pair =
            (same_triangle(certificate.discovery.triangles[0],
                           accepted[0].key) &&
             same_triangle(certificate.discovery.triangles[1],
                           accepted[1].key)) ||
            (same_triangle(certificate.discovery.triangles[0],
                           accepted[1].key) &&
             same_triangle(certificate.discovery.triangles[1],
                           accepted[0].key));
        const auto prepared_match = std::find_if(
            geometry.features.begin(), geometry.features.end(),
            [&](const auto& feature) {
                return contact::fixed_triangle_features::Compare(
                           certificate.discovery.key,
                           feature.key) == 0;
            });
        const bool same_feature =
            prepared_match != geometry.features.end();
        if (!same_pair && !same_feature) continue;
        accepted_matches += same_pair;
        accepted_feature_matches += same_feature;
        std::cout << std::setprecision(17)
                  << "V5_LINEAR_PERSISTENT_ACCEPTED"
                  << " same_pair=" << same_pair
                  << " same_feature=" << same_feature
                  << " kind="
                  << static_cast<unsigned>(
                         certificate.discovery.key.kind)
                  << " local="
                  << certificate.discovery.local_features[0]
                  << ","
                  << certificate.discovery.local_features[1]
                  << " distance_m="
                  << certificate.discovery.distance_m
                  << " error_m="
                  << certificate.discovery.representation_error_m
                  << " event_status="
                  << static_cast<unsigned>(
                         certificate.event.classification.status)
                  << " certificate_kind="
                  << static_cast<unsigned>(certificate.kind)
                  << " event_kind="
                  << static_cast<unsigned>(
                         certificate.event.feature.kind)
                  << " pair_kind="
                  << static_cast<unsigned>(
                         certificate.event.classification.kind)
                  << " order=" << certificate.event.source_order
                  << " index=" << certificate_index
                  << " active="
                  << certificate.event.classification.active[0]
                  << ","
                  << certificate.event.classification.active[1]
                  << " excluded="
                  << certificate.event.classification.excluded
                  << " incidence="
                  << certificate.event.classification.local_incidence
                  << " edge_uses="
                  << certificate.event.edge_use[0] << ","
                  << certificate.event.edge_use[1]
                  << " edge_facets="
                  << certificate.edge_facet[0] << ","
                  << certificate.edge_facet[1]
                  << " parameters="
                  << certificate.discovery.edge_parameters[0]
                  << ","
                  << certificate.discovery.edge_parameters[1]
                  << " face_weights=" << std::hexfloat
                  << certificate.discovery.face_weights[0]
                  << ","
                  << certificate.discovery.face_weights[1]
                  << ","
                  << certificate.discovery.face_weights[2]
                  << std::defaultfloat
                  << " endpoints="
                  << certificate.event.endpoints[0].count << ","
                  << certificate.event.endpoints[1].count
                  << " triangles="
                  << certificate.discovery.triangles[0].parent_eid
                  << ":"
                  << certificate.discovery.triangles[0].local_facet
                  << ","
                  << certificate.discovery.triangles[1].parent_eid
                  << ":"
                  << certificate.discovery.triangles[1].local_facet
                  << " key=";
        if (certificate.discovery.key.kind ==
            contact::FixedTriangleCandidateKind::EdgeEdge) {
            PrintEdgeKey(
                certificate.discovery.key.edge_edge.edges[0]);
            std::cout << "/";
            PrintEdgeKey(
                certificate.discovery.key.edge_edge.edges[1]);
        } else {
            PrintVertexKey(
                certificate.discovery.key.vertex_face.vertex);
            std::cout << "/target_kind="
                      << static_cast<unsigned>(
                             certificate.discovery.key.vertex_face.
                                 target.kind);
        }
        std::cout << '\n';
    }
    for (const auto& feature : geometry.features) {
        std::cout << std::setprecision(17)
                  << "V5_LINEAR_PERSISTENT_PREPARED"
                  << " kind="
                  << static_cast<unsigned>(feature.key.kind)
                  << " local=" << feature.local_features[0]
                  << "," << feature.local_features[1]
                  << " distance_m=" << feature.distance_m
                  << " error_m="
                  << feature.representation_error_m
                  << " face_weights=" << std::hexfloat
                  << feature.face_weights[0] << ","
                  << feature.face_weights[1] << ","
                  << feature.face_weights[2]
                  << std::defaultfloat
                  << " key=";
        if (feature.key.kind ==
            contact::FixedTriangleCandidateKind::EdgeEdge) {
            PrintEdgeKey(feature.key.edge_edge.edges[0]);
            std::cout << "/";
            PrintEdgeKey(feature.key.edge_edge.edges[1]);
        } else {
            PrintVertexKey(feature.key.vertex_face.vertex);
            std::cout << "/target_kind="
                      << static_cast<unsigned>(
                             feature.key.vertex_face.target.kind);
        }
        std::cout << '\n';
    }

    std::cout << std::setprecision(17) << std::hexfloat
              << "V5_LINEAR_PERSISTENT_CERTIFICATE"
              << " residual_status="
              << static_cast<unsigned>(residual.status)
              << " persistent_status="
              << static_cast<unsigned>(persistent.status)
              << " persistent_kind="
              << static_cast<unsigned>(persistent.feature.kind)
              << " persistent_accepted="
              << persistent.accepted_certificate
              << " persistent_margin_m="
              << persistent.strict_thickness_margin_lower_m
              << " persistent_weight_normalization_m="
              << persistent.face_weight_normalization_upper_m
              << " persistent_bounded="
              << persistent.bounded_feature_count
              << " persistent_exact_accepted="
              << persistent.exact_accepted_candidate_count
              << " persistent_full_accepted="
              << persistent.full_accepted_candidate_count
              << " exact_common="
              << residual.exact_common_translation
              << " shared_vertices=" << shared_vertices
              << " shared_edges=" << shared_edges
              << " intersections="
              << geometry.intersections.size()
              << " accepted_matches=" << accepted_matches
              << " accepted_feature_matches="
              << accepted_feature_matches
              << " half_thicknesses="
              << facets[0].reference_half_thickness_m << ","
              << facets[1].reference_half_thickness_m
              << " full_classification="
              << static_cast<unsigned>(exact_full.classification)
              << " full_reason="
              << static_cast<unsigned>(exact_full.reason)
              << " full_work=" << exact_full.work
              << " reference="
              << residual.reference_translation.x << ","
              << residual.reference_translation.y << ","
              << residual.reference_translation.z
              << " first_residual_upper_m="
              << residual.first_residual_upper_m
              << " second_residual_upper_m="
              << residual.second_residual_upper_m
              << " prepared_distance_lower_m="
              << residual.prepared_distance_lower_m
              << " strict_gap_lower_m="
              << residual.strict_gap_lower_m
              << std::defaultfloat << '\n';
    for (unsigned side = 0; side < 2; ++side)
        for (unsigned vertex = 0; vertex < 3; ++vertex)
            std::cout << std::hexfloat
                      << "V5_LINEAR_PERSISTENT_BITS"
                      << " side=" << side
                      << " vertex=" << vertex
                      << " accepted="
                      << accepted[side].vertices[vertex].x << ","
                      << accepted[side].vertices[vertex].y << ","
                      << accepted[side].vertices[vertex].z
                      << " prepared="
                      << prepared[side].vertices[vertex].x << ","
                      << prepared[side].vertices[vertex].y << ","
                      << prepared[side].vertices[vertex].z
                      << std::defaultfloat << '\n';

    const std::size_t residual_parents[2]{
        ParentOrdinal(active, ExhaustedLinearFirstEid),
        ParentOrdinal(active, ExhaustedLinearSecondEid)};
    std::array<contact::FixedContactFacet, 2> residual_facets;
    std::array<contact::CurrentFixedTriangle, 2>
        residual_accepted;
    std::array<contact::CurrentFixedTriangle, 2>
        residual_prepared;
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent = active.parents()[
            residual_parents[side]];
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, CouponLocalFacet,
                      &residual_facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      residual_facets[side], accepted_positions,
                      &residual_accepted[side]),
                  contact::Status::kOk);
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      residual_facets[side], prepared_positions,
                      &residual_prepared[side]),
                  contact::Status::kOk);
    }
    contact::FixedTriangleFeatureTaskMask residual_mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  residual_prepared[0], residual_prepared[1],
                  &residual_mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    const auto residual_geometry =
        Discover(residual_prepared, residual_mask);
    const auto separated = sct::CertifyLinearResidualSeparation(
        residual_accepted[0], residual_prepared[0],
        residual_facets[0].reference_half_thickness_m,
        residual_accepted[1], residual_prepared[1],
        residual_facets[1].reference_half_thickness_m,
        {residual_geometry.features.data(),
         residual_geometry.features.size(), true},
        {residual_geometry.intersections.data(),
         residual_geometry.intersections.size(), true});
    EXPECT_EQ(
        separated.status,
        sct::LinearResidualSeparationStatus::
            CertifiedSeparated);
    EXPECT_FALSE(separated.exact_common_translation);
    EXPECT_GT(separated.strict_gap_lower_m, 0);

    const std::size_t canonical_parents[2]{
        ParentOrdinal(active, CanonicalPersistentLinearFirstEid),
        ParentOrdinal(active, CanonicalPersistentLinearSecondEid)};
    std::array<contact::FixedContactFacet, 2> canonical_facets;
    std::array<contact::CurrentFixedTriangle, 2>
        canonical_accepted;
    std::array<contact::CurrentFixedTriangle, 2>
        canonical_prepared;
    for (unsigned side = 0; side < 2; ++side) {
        const auto& parent =
            active.parents()[canonical_parents[side]];
        ASSERT_EQ(setup.facets().Describe(
                      parent.surface_parent, AffineMixedLocalFacet,
                      &canonical_facets[side]).status,
                  contact::FixedContactFacetStatus::Ok);
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      canonical_facets[side], accepted_positions,
                      &canonical_accepted[side]),
                  contact::Status::kOk);
        ASSERT_EQ(contact::EvaluateCurrentFixedTriangle(
                      canonical_facets[side], prepared_positions,
                      &canonical_prepared[side]),
                  contact::Status::kOk);
    }
    contact::FixedTriangleFeatureTaskMask canonical_mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(
                  canonical_prepared[0], canonical_prepared[1],
                  &canonical_mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    const auto canonical_geometry =
        Discover(canonical_prepared, canonical_mask);
    const auto canonical_persistent =
        sct::CertifyPersistentLinearContact(
            canonical_accepted[0], canonical_prepared[0],
            canonical_facets[0].reference_half_thickness_m,
            canonical_accepted[1], canonical_prepared[1],
            canonical_facets[1].reference_half_thickness_m,
            {canonical_geometry.features.data(),
             canonical_geometry.features.size(), true},
            snapshot.accepted_certificates.data(),
            snapshot.accepted_certificates.size());
    EXPECT_EQ(
        canonical_persistent.status,
        sct::PersistentLinearContactStatus::CertifiedContact);
    EXPECT_EQ(canonical_persistent.feature.kind,
              contact::RepresentedFeatureKind::EdgeEdge);

    enum class FamilyOwnerPattern : unsigned {
        Exact,
        LowerOneSide,
        LowerTwoSides,
        UpperOneSide,
        UpperTwoSides,
        Crossed,
        Count,
    };
    constexpr std::uint64_t FirstFamilyBegin = 2100070;
    constexpr std::uint64_t FirstFamilyEnd = 2100090;
    constexpr std::uint64_t SecondFamilyBegin = 2288735;
    constexpr std::uint64_t SecondFamilyEnd = 2288750;
    std::vector<std::size_t> first_family;
    std::vector<std::size_t> second_family;
    for (std::size_t parent = 0;
         parent < active.parents().size(); ++parent) {
        const auto eid =
            active.parents()[parent].source.source_parent_id;
        if (eid >= FirstFamilyBegin && eid <= FirstFamilyEnd)
            first_family.push_back(parent);
        if (eid >= SecondFamilyBegin && eid <= SecondFamilyEnd)
            second_family.push_back(parent);
    }
    std::array<std::size_t, static_cast<unsigned>(
        FamilyOwnerPattern::Count)> pattern_counts{};
    std::array<bool, static_cast<unsigned>(
        FamilyOwnerPattern::Count)> pattern_printed{};
    std::array<std::size_t, 4> persistent_status_counts{};
    std::size_t family_facet_pairs = 0;
    std::size_t family_pairs_with_ledger = 0;
    const auto canonical_pair = [](const auto& feature) {
        std::array<contact::FixedTriangleKey, 2> result{
            feature.triangles[0], feature.triangles[1]};
        if (contact::fixed_triangle_features::Compare(
                result[1], result[0]) < 0)
            std::swap(result[0], result[1]);
        return result;
    };
    for (const auto first_parent : first_family)
        for (const auto second_parent : second_family) {
            const auto& first_use =
                active.parents()[first_parent];
            const auto& second_use =
                active.parents()[second_parent];
            for (unsigned first_local = 0;
                 first_local < first_use.facet_count;
                 ++first_local)
                for (unsigned second_local = 0;
                     second_local < second_use.facet_count;
                     ++second_local) {
                    ++family_facet_pairs;
                    std::array<contact::FixedContactFacet, 2>
                        family_facets;
                    ASSERT_EQ(setup.facets().Describe(
                                  first_use.surface_parent,
                                  first_local,
                                  &family_facets[0]).status,
                              contact::FixedContactFacetStatus::Ok);
                    ASSERT_EQ(setup.facets().Describe(
                                  second_use.surface_parent,
                                  second_local,
                                  &family_facets[1]).status,
                              contact::FixedContactFacetStatus::Ok);
                    std::array<contact::CurrentFixedTriangle, 2>
                        family_accepted;
                    std::array<contact::CurrentFixedTriangle, 2>
                        family_prepared;
                    for (unsigned side = 0; side < 2; ++side) {
                        ASSERT_EQ(
                            contact::EvaluateCurrentFixedTriangle(
                                family_facets[side],
                                accepted_positions,
                                &family_accepted[side]),
                            contact::Status::kOk);
                        ASSERT_EQ(
                            contact::EvaluateCurrentFixedTriangle(
                                family_facets[side],
                                prepared_positions,
                                &family_prepared[side]),
                            contact::Status::kOk);
                    }
                    contact::FixedTriangleFeatureTaskMask
                        family_mask;
                    ASSERT_EQ(
                        contact::BuildFixedTriangleFeatureTaskMask(
                            family_prepared[0],
                            family_prepared[1],
                            &family_mask),
                        contact::FixedTriangleDiscoveryStatus::Ok);
                    const auto family_geometry =
                        Discover(family_prepared, family_mask);
                    bool matched_ledger = false;
                    for (const auto& feature :
                         family_geometry.features) {
                        const auto lower = std::lower_bound(
                            snapshot.accepted_certificates.begin(),
                            snapshot.accepted_certificates.end(),
                            feature.key,
                            [](const auto& certificate,
                               const auto& key) {
                                return contact::
                                           fixed_triangle_features::
                                               Compare(
                                                   certificate.event.
                                                       feature,
                                                   key) < 0;
                            });
                        const auto candidate_pair =
                            canonical_pair(feature);
                        for (auto certificate = lower;
                             certificate !=
                                 snapshot.accepted_certificates.end() &&
                             contact::fixed_triangle_features::Compare(
                                 certificate->event.feature,
                                 feature.key) == 0;
                             ++certificate) {
                            matched_ledger = true;
                            const auto accepted_pair =
                                canonical_pair(
                                    certificate->discovery);
                            const int first_owner =
                                contact::fixed_triangle_features::
                                    Compare(
                                        accepted_pair[0],
                                        candidate_pair[0]);
                            const int second_owner =
                                contact::fixed_triangle_features::
                                    Compare(
                                        accepted_pair[1],
                                        candidate_pair[1]);
                            FamilyOwnerPattern pattern;
                            if (!first_owner && !second_owner)
                                pattern = FamilyOwnerPattern::Exact;
                            else if (first_owner <= 0 &&
                                     second_owner <= 0)
                                pattern =
                                    first_owner < 0 &&
                                            second_owner < 0
                                        ? FamilyOwnerPattern::
                                              LowerTwoSides
                                        : FamilyOwnerPattern::
                                              LowerOneSide;
                            else if (first_owner >= 0 &&
                                     second_owner >= 0)
                                pattern =
                                    first_owner > 0 &&
                                            second_owner > 0
                                        ? FamilyOwnerPattern::
                                              UpperTwoSides
                                        : FamilyOwnerPattern::
                                              UpperOneSide;
                            else
                                pattern =
                                    FamilyOwnerPattern::Crossed;
                            const auto index =
                                static_cast<unsigned>(pattern);
                            ++pattern_counts[index];
                            if (!pattern_printed[index]) {
                                pattern_printed[index] = true;
                                std::cout
                                    << "V5_LINEAR_FAMILY_PATTERN"
                                    << " class=" << index
                                    << " candidate="
                                    << first_use.source.
                                           source_parent_id
                                    << ":" << first_local << ","
                                    << second_use.source.
                                           source_parent_id
                                    << ":" << second_local
                                    << " kind="
                                    << static_cast<unsigned>(
                                           feature.key.kind)
                                    << " distance_m="
                                    << std::setprecision(17)
                                    << feature.distance_m
                                    << " accepted="
                                    << accepted_pair[0].parent_eid
                                    << ":"
                                    << accepted_pair[0].local_facet
                                    << ","
                                    << accepted_pair[1].parent_eid
                                    << ":"
                                    << accepted_pair[1].local_facet
                                    << '\n';
                            }
                        }
                    }
                    if (!matched_ledger) continue;
                    ++family_pairs_with_ledger;
                    const auto family_persistent =
                        sct::CertifyPersistentLinearContact(
                            family_accepted[0], family_prepared[0],
                            family_facets[0].
                                reference_half_thickness_m,
                            family_accepted[1], family_prepared[1],
                            family_facets[1].
                                reference_half_thickness_m,
                            {family_geometry.features.data(),
                             family_geometry.features.size(), true},
                            snapshot.accepted_certificates.data(),
                            snapshot.accepted_certificates.size());
                    const auto family_status =
                        static_cast<unsigned>(
                            family_persistent.status);
                    ++persistent_status_counts[family_status];
                    if (family_persistent.status !=
                        sct::PersistentLinearContactStatus::
                            CertifiedContact) {
                        const auto family_residual =
                            sct::CertifyLinearResidualSeparation(
                                family_accepted[0],
                                family_prepared[0],
                                family_facets[0].
                                    reference_half_thickness_m,
                                family_accepted[1],
                                family_prepared[1],
                                family_facets[1].
                                    reference_half_thickness_m,
                                {family_geometry.features.data(),
                                 family_geometry.features.size(),
                                 true},
                                {family_geometry.intersections.data(),
                                 family_geometry.intersections.size(),
                                 true});
                        std::cout
                            << "V5_LINEAR_FAMILY_UNRESOLVED"
                            << " candidate="
                            << first_use.source.source_parent_id
                            << ":" << first_local << ","
                            << second_use.source.source_parent_id
                            << ":" << second_local
                            << " persistent_status="
                            << family_status
                            << " residual_status="
                            << static_cast<unsigned>(
                                   family_residual.status)
                            << " bounded="
                            << family_persistent.
                                   bounded_feature_count
                            << " exact_accepted="
                            << family_persistent.
                                   exact_accepted_candidate_count
                            << " full_accepted="
                            << family_persistent.
                                   full_accepted_candidate_count
                            << " features="
                            << family_geometry.features.size()
                            << " intersections="
                            << family_geometry.intersections.size()
                            << " local_tasks="
                            << family_mask.local_tasks << '\n';
                    }
                }
        }
    std::cout << "V5_LINEAR_FAMILY_SUMMARY"
              << " facet_pairs=" << family_facet_pairs
              << " ledger_pairs=" << family_pairs_with_ledger;
    for (unsigned pattern = 0;
         pattern < static_cast<unsigned>(
             FamilyOwnerPattern::Count);
         ++pattern)
        std::cout << " pattern" << pattern << "="
                  << pattern_counts[pattern];
    for (unsigned status = 0;
         status < persistent_status_counts.size(); ++status)
        std::cout << " status" << status << "="
                  << persistent_status_counts[status];
    std::cout << '\n';
    EXPECT_EQ(
        persistent_status_counts[static_cast<unsigned>(
            sct::PersistentLinearContactStatus::
                CertifiedContact)],
        family_pairs_with_ledger);
    EXPECT_EQ(
        persistent_status_counts[static_cast<unsigned>(
            sct::PersistentLinearContactStatus::
                PotentialChange)],
        0u);
    dynamics.DiscardStep();
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
