#include "Source.h"
#include "NonlinearCoverageFixture.h"
#include "LinearCoverageFixture.h"

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
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <thread>
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
    std::vector<
        tlfea::contact::self_contact_transaction::
            NonlinearCandidateRosterEntry> nonlinear_roster;
    std::vector<
        tlfea::contact::self_contact_transaction::
            LinearWorkExhaustedRosterEntry> linear_roster;
    tlfea::contact::self_contact_transaction::
        NonlinearCandidateRosterSummary nonlinear_summary;
    tlfea::contact::self_contact_transaction::
        LinearCandidateCensusSummary linear_summary;
    tlfea::contact::self_contact_transaction::
        PreparedMotionCertificateView motion_certificates;
    tl::fea::NodalStamp accepted_stamp;
    tl::fea::NodalPreparedView prepared_view;
    tlfea::contact::SelfContactTransaction* transaction = nullptr;
    const tlfea::contact::SelfContactAcceptedAssemblyReceipt*
        accepted_receipt = nullptr;
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
        const auto started = std::chrono::steady_clock::now();
        auto& storage = *dynamics.storage_;
        storage.Prepare();
        const auto accepted_ready = std::chrono::steady_clock::now();
        std::cerr << "V5_NONLINEAR_COUPON_PHASE accepted_prepare_s="
                  << std::chrono::duration<double>(
                         accepted_ready - started).count()
                  << std::endl;
        auto& owner = storage.state().owner;
        const auto nodes = storage.startup.accepted().node_count;
        AcceptedAssemblyCouponSnapshot result;
        result.accepted_positions.resize(3 * nodes);
        result.prepared_positions.resize(3 * nodes);
        std::vector<double> accepted_velocities(3 * nodes);
        std::vector<double> prepared_velocities(3 * nodes);
        auto report = owner.CopyAccepted(
            {result.accepted_positions.data(),
             accepted_velocities.data(), nodes},
            &result.accepted_stamp);
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
                result.accepted_stamp, storage.stamp) ||
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
        result.transaction = &stages->contact.data_->transaction;
        result.accepted_receipt = &stages->accepted_;
        constexpr std::size_t NonlinearRosterCapacity =
            std::size_t{1} << 20;
        result.nonlinear_roster.resize(NonlinearRosterCapacity);
        constexpr std::size_t LinearRosterCapacity =
            std::size_t{1} << 20;
        result.linear_roster.resize(LinearRosterCapacity);
        std::size_t nonlinear_count = 0;
        std::size_t linear_count = 0;
        const auto roster_report =
            tlfea::contact::self_contact_transaction::
            QualificationAccess::
                ClassifyPreparedCandidateCensus(
                    stages->contact.data_->transaction,
                    owner, storage.token, storage.prepared,
                    stages->accepted_,
                    result.nonlinear_roster.data(),
                    result.nonlinear_roster.size(),
                    &nonlinear_count,
                    &result.nonlinear_summary,
                    result.linear_roster.data(),
                    result.linear_roster.size(),
                    &linear_count,
                    &result.linear_summary,
                    &result.motion_certificates);
        if (roster_report.status !=
            tlfea::contact::SelfContactTransactionStatus::Ok)
            throw std::runtime_error(roster_report.message);
        std::cerr << "V5_NONLINEAR_COUPON_PHASE nonlinear_roster_s="
                  << std::chrono::duration<double>(
                         std::chrono::steady_clock::now() -
                         accepted_ready).count()
                  << std::endl;
        result.nonlinear_roster.resize(nonlinear_count);
        result.linear_roster.resize(linear_count);
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
constexpr std::uint64_t ExpectedNonlinearRosterDigest =
    15183149279991149367ull;
constexpr std::uint64_t ExpectedAmbiguousRosterDigest =
    2928523903779679127ull;
constexpr std::uint64_t ExpectedLinearCensusDigest =
    6473154677596308446ull;
constexpr std::uint64_t ExpectedLinearFixtureRosterDigest =
    10312651066629367413ull;

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
            8, 4, FullAcceptedEvents);
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

void HashUnsigned(std::uint64_t value, std::uint64_t* digest) {
    for (unsigned byte = 0; byte < 8; ++byte) {
        *digest ^= static_cast<unsigned char>(
            value >> (8 * byte));
        *digest *= 1099511628211ull;
    }
}

void HashPath(
    const contact::RepresentedTrianglePathKey& path,
    std::uint64_t* digest) {
    HashUnsigned(path.source_instance_id, digest);
    HashUnsigned(path.parent_eid, digest);
    HashUnsigned(path.level, digest);
    HashUnsigned(path.local_facet, digest);
}

std::uint64_t NonlinearRosterDigest(
    const std::vector<sct::NonlinearCandidateRosterEntry>& roster) {
    std::uint64_t result = 1469598103934665603ull;
    for (const auto& entry : roster) {
        HashUnsigned(entry.facets.first, &result);
        HashUnsigned(entry.facets.second, &result);
        HashPath(entry.key.paths[0], &result);
        HashPath(entry.key.paths[1], &result);
        HashUnsigned(
            static_cast<unsigned>(entry.separation.status), &result);
        HashUnsigned(entry.separation.work, &result);
        HashUnsigned(entry.separation.deepest, &result);
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

nonlinear_fixture::Pair FreezeNonlinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    const sct::NonlinearCandidateRosterEntry& entry) {
    nonlinear_fixture::Pair result;
    result.facet[0] = entry.facets.first;
    result.facet[1] = entry.facets.second;
    result.baseline_status = entry.separation.status;
    result.baseline_work = entry.separation.work;
    result.baseline_depth = entry.separation.deepest;
    const std::uint32_t facets[2]{
        entry.facets.first, entry.facets.second};
    for (unsigned side = 0; side < 2; ++side) {
        if (facets[side] >=
            snapshot.motion_certificates.facet_count)
            throw std::runtime_error(
                "Nonlinear fixture facet is out of range");
        result.accepted[side] =
            snapshot.motion_certificates.accepted_triangles[
                facets[side]];
        result.prepared[side] =
            snapshot.motion_certificates.prepared_triangles[
                facets[side]];
        result.quadratic[side] =
            snapshot.motion_certificates.quadratic[facets[side]];
        result.half_thickness[side] =
            snapshot.motion_certificates.descriptors[
                facets[side]].reference_half_thickness_m;
    }
    const std::array<contact::CurrentFixedTriangle, 2> accepted{
        result.accepted[0], result.accepted[1]};
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        result.prepared[0], result.prepared[1]};
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    if (contact::BuildFixedTriangleFeatureTaskMask(
            accepted[0], accepted[1], &accepted_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok ||
        contact::BuildFixedTriangleFeatureTaskMask(
            prepared[0], prepared[1], &prepared_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(
            "Nonlinear fixture local mask failed");
    result.accepted_mask = accepted_mask.local_tasks;
    result.prepared_mask = prepared_mask.local_tasks;
    auto accepted_geometry =
        DiscoverDirect(accepted, accepted_mask);
    auto prepared_geometry =
        DiscoverDirect(prepared, prepared_mask);
    result.accepted_features =
        std::move(accepted_geometry.features);
    result.prepared_features =
        std::move(prepared_geometry.features);
    result.accepted_intersections =
        std::move(accepted_geometry.intersections);
    result.prepared_intersections =
        std::move(prepared_geometry.intersections);
    for (const auto& certificate :
         snapshot.accepted_certificates)
        if (CouldOwnQuadraticPair(prepared, certificate))
            result.accepted_owners.push_back(certificate);
    if (result.accepted_owners.size() >
        nonlinear_fixture::MaximumOwners)
        throw std::runtime_error(
            "Nonlinear fixture owner roster exceeds hard cap");
    return result;
}

linear_fixture::Pair FreezeLinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    const sct::LinearWorkExhaustedRosterEntry& entry) {
    linear_fixture::Pair result;
    result.facet[0] = entry.facets.first;
    result.facet[1] = entry.facets.second;
    result.baseline_status =
        sct::NonlinearSeparationStatus::WorkExhausted;
    result.baseline_work = entry.crossing.work;
    result.baseline_depth = 20;
    const std::uint32_t facets[2]{
        entry.facets.first, entry.facets.second};
    for (unsigned side = 0; side < 2; ++side) {
        if (facets[side] >=
            snapshot.motion_certificates.facet_count)
            throw std::runtime_error(
                "Linear fixture facet is out of range");
        result.accepted[side] =
            snapshot.motion_certificates.accepted_triangles[
                facets[side]];
        result.prepared[side] =
            snapshot.motion_certificates.prepared_triangles[
                facets[side]];
        result.quadratic[side] =
            snapshot.motion_certificates.quadratic[facets[side]];
        result.half_thickness[side] =
            snapshot.motion_certificates.descriptors[
                facets[side]].reference_half_thickness_m;
    }
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    if (contact::BuildFixedTriangleFeatureTaskMask(
            result.accepted[0], result.accepted[1],
            &accepted_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok ||
        contact::BuildFixedTriangleFeatureTaskMask(
            result.prepared[0], result.prepared[1],
            &prepared_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok)
        throw std::runtime_error(
            "Linear fixture local mask failed");
    result.accepted_mask = accepted_mask.local_tasks;
    result.prepared_mask = prepared_mask.local_tasks;
    const std::array<contact::CurrentFixedTriangle, 2> accepted{
        result.accepted[0], result.accepted[1]};
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        result.prepared[0], result.prepared[1]};
    auto accepted_geometry =
        DiscoverDirect(accepted, accepted_mask);
    auto prepared_geometry =
        DiscoverDirect(prepared, prepared_mask);
    result.accepted_features =
        std::move(accepted_geometry.features);
    result.prepared_features =
        std::move(prepared_geometry.features);
    result.accepted_intersections =
        std::move(accepted_geometry.intersections);
    result.prepared_intersections =
        std::move(prepared_geometry.intersections);
    for (const auto& certificate :
         snapshot.accepted_certificates)
        if (CouldOwnQuadraticPair(prepared, certificate))
            result.accepted_owners.push_back(certificate);
    if (result.accepted_owners.size() >
        nonlinear_fixture::MaximumOwners)
        throw std::runtime_error(
            "Linear fixture owner roster exceeds hard cap");
    return result;
}

std::uint64_t LinearCensusDigest(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto& entry : snapshot.linear_roster) {
        HashPath(entry.key.paths[0], &hash);
        HashPath(entry.key.paths[1], &hash);
        HashUnsigned(entry.task_mask.local_tasks, &hash);
        HashUnsigned(
            static_cast<unsigned>(entry.residual.status), &hash);
        HashUnsigned(
            static_cast<unsigned>(entry.persistent.status), &hash);
        HashUnsigned(
            static_cast<unsigned>(entry.crossing.classification),
            &hash);
        HashUnsigned(
            static_cast<unsigned>(entry.crossing.reason), &hash);
        HashUnsigned(entry.crossing.work, &hash);
    }
    return hash;
}

void FreezeAcceptedPolicies(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    std::vector<nonlinear_fixture::Pair>* pairs) {
    if (!pairs || !snapshot.transaction ||
        !snapshot.accepted_receipt)
        throw std::runtime_error(
            "Nonlinear fixture accepted policy owner is absent");
    std::vector<contact::FixedTriangleFeatureCandidate> features;
    std::vector<std::size_t> offsets;
    offsets.reserve(pairs->size() + 1);
    offsets.push_back(0);
    for (const auto& pair : *pairs) {
        features.insert(
            features.end(), pair.accepted_features.begin(),
            pair.accepted_features.end());
        offsets.push_back(features.size());
    }
    std::vector<sct::AcceptedFeaturePolicyEvidence>
        policy(features.size());
    std::size_t policy_count = 0;
    const auto report =
        sct::QualificationAccess::
            ClassifyAcceptedFeaturePolicies(
                *snapshot.transaction,
                *snapshot.accepted_receipt,
                {features.data(), features.size(), true},
                policy.data(), policy.size(), &policy_count);
    if (report.status !=
            contact::SelfContactTransactionStatus::Ok ||
        policy_count != policy.size())
        throw std::runtime_error(report.message);
    for (std::size_t pair = 0; pair < pairs->size(); ++pair)
        (*pairs)[pair].accepted_policy.assign(
            policy.begin() + offsets[pair],
            policy.begin() + offsets[pair + 1]);
}

std::vector<sct::AcceptedFeatureExclusionCertificate>
FrozenSameRigidExclusions(
    const nonlinear_fixture::Pair& pair) {
    std::vector<sct::AcceptedFeatureExclusionCertificate> result;
    for (std::size_t feature = 0;
         feature < pair.accepted_features.size(); ++feature) {
        const auto& evidence = pair.accepted_policy[feature];
        if (evidence.pair_status !=
                contact::SelfContactPairStatus::
                    ExcludedSameRigidGroup ||
            evidence.endpoint_support[0].status !=
                contact::SelfContactSupportStatus::
                    CompleteRigidGroup ||
            evidence.endpoint_support[1].status !=
                contact::SelfContactSupportStatus::
                    CompleteRigidGroup ||
            evidence.endpoint_support[0].complete_rigid_group !=
                evidence.endpoint_support[1].
                    complete_rigid_group)
            continue;
        result.push_back({
            pair.accepted_features[feature],
            static_cast<std::uint32_t>(
                evidence.endpoint_support[0].
                    complete_rigid_group)});
    }
    return result;
}

std::uint64_t FixtureProfileHash() {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto value : {
             SelfContactSourceId,
             static_cast<std::uint64_t>(FullAcceptedEvents),
             static_cast<std::uint64_t>(FullParentPairCapacity),
             static_cast<std::uint64_t>(FullFacetPairCapacity),
             std::uint64_t{4095}, std::uint64_t{20}})
        HashUnsigned(value, &hash);
    return hash;
}

std::uint64_t FixtureDtHash(
    const fe::NodalPreparedView& prepared) {
    const auto bits = [](double value) {
        std::uint64_t result = 0;
        std::memcpy(&result, &value, sizeof(result));
        return result;
    };
    std::uint64_t hash = 1469598103934665603ull;
    HashUnsigned(
        bits(prepared.proposed_time - prepared.base_time),
        &hash);
    HashUnsigned(bits(prepared.kick_dt), &hash);
    HashUnsigned(
        static_cast<unsigned>(
            prepared.rigid_member_trajectory),
        &hash);
    return hash;
}

nonlinear_fixture::PhaseIdentity FixturePhaseIdentity(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot) {
    const auto bits = [](double value) {
        std::uint64_t result = 0;
        std::memcpy(&result, &value, sizeof(result));
        return result;
    };
    nonlinear_fixture::PhaseIdentity result;
    result.accepted_epoch = snapshot.accepted_stamp.epoch;
    result.prepared_base_epoch =
        snapshot.prepared_view.kinematics.base_epoch;
    result.accepted_time_bits =
        bits(snapshot.accepted_stamp.time);
    result.prepared_base_time_bits =
        bits(snapshot.prepared_view.base_time);
    result.prepared_time_bits =
        bits(snapshot.prepared_view.proposed_time);
    result.accepted_velocity_time_bits =
        bits(snapshot.accepted_stamp.velocity_time);
    result.prepared_velocity_time_bits =
        bits(snapshot.prepared_view.velocity_time);
    result.accepted_temporal_scheme =
        static_cast<unsigned>(
            snapshot.accepted_stamp.temporal_scheme);
    result.prepared_temporal_scheme =
        static_cast<unsigned>(
            snapshot.prepared_view.temporal_scheme);
    result.accepted_velocity_phase =
        static_cast<unsigned>(
            snapshot.accepted_stamp.velocity_phase);
    result.prepared_velocity_phase =
        static_cast<unsigned>(
            snapshot.prepared_view.velocity_phase);
    result.prepared_trajectory =
        static_cast<unsigned>(
            snapshot.prepared_view.rigid_member_trajectory);
    return result;
}

enum class NonlinearEvidenceClass : unsigned {
    ExactLocalIntersection,
    EndpointSeparated,
    PersistentAcceptedLedger,
    EndpointThicknessContact,
    PossibleCurvedCrossing,
    Count,
};

enum class NonlinearAfterClass : unsigned {
    ExactLocalIntersection,
    CertifiedQuadraticResidualSeparation,
    CertifiedPersistentAcceptedContact,
    PossibleCurvedCrossing,
    Count,
};

struct CanonicalFacetPair {
    std::uint64_t first_eid = 0;
    unsigned first_local = 0;
    std::uint64_t second_eid = 0;
    unsigned second_local = 0;
};

constexpr CanonicalFacetPair ExpectedAmbiguousRoster[]{
#include "NonlinearAmbiguousRoster.inc"
};
static_assert(
    sizeof(ExpectedAmbiguousRoster) /
        sizeof(*ExpectedAmbiguousRoster) == 826);

struct NonlinearPairEvidence {
    NonlinearEvidenceClass classification =
        NonlinearEvidenceClass::PossibleCurvedCrossing;
    NonlinearAfterClass after =
        NonlinearAfterClass::PossibleCurvedCrossing;
    sct::LinearResidualSeparationStatus residual_status =
        sct::LinearResidualSeparationStatus::InvalidInput;
    sct::PersistentLinearContactStatus persistent_status =
        sct::PersistentLinearContactStatus::InvalidInput;
    unsigned shared_vertices = 0;
    unsigned shared_edges = 0;
    std::uint16_t accepted_mask = 0;
    std::uint16_t prepared_mask = 0;
    bool local = false;
    bool admitted = false;
    bool endpoint_separated = false;
    bool endpoint_contact = false;
    bool ledger = false;
    bool valid = false;
    double accepted_minimum_m = 0;
    double prepared_minimum_m = 0;
    double thickness_m = 0;
};

NonlinearPairEvidence InspectNonlinearPair(
    const vehicle_self_contact::AcceptedAssemblyCouponSnapshot& snapshot,
    const sct::NonlinearCandidateRosterEntry& entry) {
    NonlinearPairEvidence result;
    if (entry.facets.first >=
            snapshot.motion_certificates.facet_count ||
        entry.facets.second >=
            snapshot.motion_certificates.facet_count)
        return result;
    const std::array<contact::CurrentFixedTriangle, 2> accepted{
        snapshot.motion_certificates.accepted_triangles[
            entry.facets.first],
        snapshot.motion_certificates.accepted_triangles[
            entry.facets.second]};
    const std::array<contact::CurrentFixedTriangle, 2> prepared{
        snapshot.motion_certificates.prepared_triangles[
            entry.facets.first],
        snapshot.motion_certificates.prepared_triangles[
            entry.facets.second]};
    for (unsigned side = 0; side < 2; ++side)
        if (contact::fixed_triangle_features::Compare(
                accepted[side].key, prepared[side].key) != 0)
            return result;
    const auto& first_descriptor =
        snapshot.motion_certificates.descriptors[
            entry.facets.first];
    const auto& second_descriptor =
        snapshot.motion_certificates.descriptors[
            entry.facets.second];
    result.thickness_m =
        first_descriptor.reference_half_thickness_m +
        second_descriptor.reference_half_thickness_m;
    contact::FixedTriangleFeatureTaskMask accepted_mask;
    contact::FixedTriangleFeatureTaskMask prepared_mask;
    if (contact::BuildFixedTriangleFeatureTaskMask(
            accepted[0], accepted[1], &accepted_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok ||
        contact::BuildFixedTriangleFeatureTaskMask(
            prepared[0], prepared[1], &prepared_mask) !=
            contact::FixedTriangleDiscoveryStatus::Ok)
        return result;
    result.accepted_mask = accepted_mask.local_tasks;
    result.prepared_mask = prepared_mask.local_tasks;
    try {
        const auto accepted_geometry =
            DiscoverDirect(accepted, accepted_mask);
        const auto prepared_geometry =
            DiscoverDirect(prepared, prepared_mask);
        for (const auto& first : first_descriptor.vertex_keys)
            for (const auto& second :
                 second_descriptor.vertex_keys)
                result.shared_vertices +=
                    contact::SameFacetVertexKey(first, second);
        for (const auto& first : first_descriptor.edge_keys)
            for (const auto& second :
                 second_descriptor.edge_keys)
                result.shared_edges +=
                    contact::SameFacetEdgeKey(first, second);
        result.local =
            HasLocalIntersection(accepted_geometry) ||
            HasLocalIntersection(prepared_geometry);
        result.admitted =
            HasAdmittedIntersection(accepted_geometry) ||
            HasAdmittedIntersection(prepared_geometry);
        result.endpoint_separated =
            StrictlySeparated(
                accepted_geometry, accepted_mask,
                result.thickness_m) &&
            StrictlySeparated(
                prepared_geometry, prepared_mask,
                result.thickness_m);
        result.endpoint_contact =
            StrictlyWithinThickness(
                accepted_geometry, result.thickness_m) ||
            StrictlyWithinThickness(
                prepared_geometry, result.thickness_m);
        sct::LinearResidualSeparationResult residual;
        if (!result.local && result.endpoint_separated)
            residual = sct::CertifyQuadraticResidualSeparation(
                accepted[0], prepared[0],
                snapshot.motion_certificates.quadratic[
                    entry.facets.first],
                first_descriptor.reference_half_thickness_m,
                accepted[1], prepared[1],
                snapshot.motion_certificates.quadratic[
                    entry.facets.second],
                second_descriptor.reference_half_thickness_m,
                PhysicalStepS,
                {prepared_geometry.features.data(),
                 prepared_geometry.features.size(), true},
                {prepared_geometry.intersections.data(),
                 prepared_geometry.intersections.size(), true});
        sct::PersistentLinearContactResult persistent;
        if (result.endpoint_contact)
            persistent = sct::CertifyPersistentQuadraticContact(
                accepted[0], prepared[0],
                snapshot.motion_certificates.quadratic[
                    entry.facets.first],
                first_descriptor.reference_half_thickness_m,
                accepted[1], prepared[1],
                snapshot.motion_certificates.quadratic[
                    entry.facets.second],
                second_descriptor.reference_half_thickness_m,
                PhysicalStepS,
                {prepared_geometry.features.data(),
                 prepared_geometry.features.size(), true},
                snapshot.accepted_certificates.data(),
                snapshot.accepted_certificates.size());
        result.residual_status = residual.status;
        result.persistent_status = persistent.status;
        result.ledger =
            persistent.status ==
            sct::PersistentLinearContactStatus::CertifiedContact;
        if (result.local && !result.admitted &&
            result.endpoint_separated)
            result.classification =
                NonlinearEvidenceClass::ExactLocalIntersection;
        else if (result.ledger && result.endpoint_contact)
            result.classification =
                NonlinearEvidenceClass::PersistentAcceptedLedger;
        else if (result.endpoint_contact)
            result.classification =
                NonlinearEvidenceClass::EndpointThicknessContact;
        else if (result.endpoint_separated)
            result.classification =
                NonlinearEvidenceClass::EndpointSeparated;
        if (result.local && !result.admitted &&
            result.endpoint_separated)
            result.after =
                NonlinearAfterClass::ExactLocalIntersection;
        else if (residual.status ==
                 sct::LinearResidualSeparationStatus::
                     CertifiedSeparated)
            result.after = NonlinearAfterClass::
                CertifiedQuadraticResidualSeparation;
        else if (persistent.status ==
                 sct::PersistentLinearContactStatus::
                     CertifiedContact)
            result.after = NonlinearAfterClass::
                CertifiedPersistentAcceptedContact;
        result.accepted_minimum_m =
            MinimumDistance(accepted_geometry);
        result.prepared_minimum_m =
            MinimumDistance(prepared_geometry);
        result.valid = true;
        return result;
    } catch (...) {
        return result;
    }
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

TEST(VehicleSelfContactNonlinearRosterCoupon,
     CompleteAcceptedAssemblyRosterSkipsLinearExactTraversal) {
    const auto& setup = LevelZeroSetup();
    const auto& active = setup.active_uses();
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

    ASSERT_TRUE(snapshot.nonlinear_summary.complete);
    ASSERT_TRUE(snapshot.nonlinear_summary.roster_complete);
    ASSERT_TRUE(snapshot.motion_certificates.complete);
    ASSERT_EQ(snapshot.motion_certificates.facet_count,
              active.facet_uses().size());
    ASSERT_EQ(snapshot.nonlinear_roster.size(),
              snapshot.nonlinear_summary.nonlinear_pairs);
    EXPECT_EQ(snapshot.prepared_view.proposed_time -
                  snapshot.prepared_view.base_time,
              PhysicalStepS);
    ASSERT_TRUE(snapshot.linear_summary.complete);
    ASSERT_TRUE(snapshot.linear_summary.roster_complete);
    ASSERT_EQ(
        snapshot.linear_roster.size(),
        snapshot.linear_summary.represented_work_exhausted);

    std::vector<nonlinear_fixture::Pair> linear_fixture_pairs;
    linear_fixture_pairs.reserve(snapshot.linear_roster.size());
    for (const auto& entry : snapshot.linear_roster)
        linear_fixture_pairs.push_back(
            FreezeLinearPair(snapshot, entry));
    FreezeAcceptedPolicies(snapshot, &linear_fixture_pairs);
    std::array<std::size_t, 10> linear_after{};
    std::size_t linear_after_work = 0;
    std::size_t linear_shared_vertex = 0;
    std::size_t linear_shared_edge = 0;
    std::size_t linear_actual_intersection = 0;
    for (std::size_t index = 0;
         index < linear_fixture_pairs.size(); ++index) {
        const auto& pair = linear_fixture_pairs[index];
        for (const auto& first : pair.prepared[0].vertex_keys)
            for (const auto& second :
                 pair.prepared[1].vertex_keys)
                linear_shared_vertex +=
                    contact::SameFacetVertexKey(first, second);
        for (const auto& first : pair.prepared[0].edge_keys)
            for (const auto& second :
                 pair.prepared[1].edge_keys)
                linear_shared_edge +=
                    contact::SameFacetEdgeKey(first, second);
        linear_actual_intersection +=
            !pair.prepared_intersections.empty() &&
            contact::RequiresIntersectionAdmission(
                pair.prepared_intersections[0]);
        const auto exclusions =
            FrozenSameRigidExclusions(pair);
        const auto resolved =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                PhysicalStepS,
                pair.accepted_owners.data(),
                pair.accepted_owners.size(),
                exclusions.data(), exclusions.size(),
                4095, 20);
        const auto status =
            static_cast<unsigned>(resolved.status);
        ASSERT_LT(status, linear_after.size());
        ++linear_after[status];
        linear_after_work += resolved.work;
        if (resolved.status !=
                sct::NonlinearSeparationStatus::
                    CertifiedSeparated &&
            resolved.status !=
                sct::NonlinearSeparationStatus::
                    CertifiedAcceptedCoverage &&
            resolved.status !=
                sct::NonlinearSeparationStatus::
                    CertifiedExactExclusion) {
            std::cout << "V5_LINEAR_ROSTER_UNRESOLVED"
                      << " index=" << index
                      << " first="
                      << pair.prepared[0].key.parent_eid
                      << ":"
                      << pair.prepared[0].key.local_facet
                      << " second="
                      << pair.prepared[1].key.parent_eid
                      << ":"
                      << pair.prepared[1].key.local_facet
                      << " status=" << status
                      << " work=" << resolved.work
                      << " depth=" << resolved.deepest
                      << " owners="
                      << pair.accepted_owners.size()
                      << " exclusions=" << exclusions.size()
                      << " accepted_features="
                      << pair.accepted_features.size()
                      << " prepared_features="
                      << pair.prepared_features.size()
                      << " accepted_mask="
                      << pair.accepted_mask
                      << " prepared_mask="
                      << pair.prepared_mask
                      << " actual_intersection="
                      << (!pair.prepared_intersections.empty() &&
                          contact::RequiresIntersectionAdmission(
                              pair.prepared_intersections[0]))
                      << '\n';
        }
    }
    const auto linear_digest = LinearCensusDigest(snapshot);
    std::cout << "V5_LINEAR_ROSTER"
              << " affine_pairs="
              << snapshot.linear_summary.affine_pairs
              << " swept_separated="
              << snapshot.linear_summary.swept_bounds_separated
              << " prism_separated="
              << snapshot.linear_summary.prism_separated
              << " exact_geometry="
              << snapshot.linear_summary.exact_geometry_pairs
              << " common_translation="
              << snapshot.linear_summary.common_translation
              << " residual_separated="
              << snapshot.linear_summary.residual_separated
              << " persistent_accepted="
              << snapshot.linear_summary.persistent_accepted
              << " represented_pairs="
              << snapshot.linear_summary.represented_pairs
              << " represented_separated="
              << snapshot.linear_summary.represented_separated
              << " represented_crossing="
              << snapshot.linear_summary.represented_crossing
              << " represented_degenerate="
              << snapshot.linear_summary.represented_degenerate
              << " represented_work_exhausted="
              << snapshot.linear_summary.represented_work_exhausted
              << " represented_arithmetic="
              << snapshot.linear_summary.
                     represented_arithmetic_range
              << " represented_work="
              << snapshot.linear_summary.represented_work
              << " roster_digest=" << linear_digest
              << " fixture_roster_digest="
              << nonlinear_fixture::RosterDigest(
                     linear_fixture_pairs)
              << " after_work=" << linear_after_work
              << " shared_vertices=" << linear_shared_vertex
              << " shared_edges=" << linear_shared_edge
              << " actual_intersections="
              << linear_actual_intersection;
    for (std::size_t status = 0;
         status < linear_after.size(); ++status)
        std::cout << " after" << status
                  << "=" << linear_after[status];
    std::cout << '\n';
    EXPECT_EQ(snapshot.linear_summary.affine_pairs, 5809241u);
    EXPECT_EQ(
        snapshot.linear_summary.swept_bounds_separated, 926944u);
    EXPECT_EQ(snapshot.linear_summary.prism_separated, 1362216u);
    EXPECT_EQ(
        snapshot.linear_summary.exact_geometry_pairs, 3520081u);
    EXPECT_EQ(snapshot.linear_summary.common_translation, 28447u);
    EXPECT_EQ(snapshot.linear_summary.residual_separated, 60841u);
    EXPECT_EQ(snapshot.linear_summary.persistent_accepted, 25705u);
    EXPECT_EQ(snapshot.linear_summary.represented_pairs, 3433535u);
    EXPECT_EQ(snapshot.linear_summary.represented_separated, 53u);
    EXPECT_EQ(snapshot.linear_summary.represented_crossing, 3433481u);
    EXPECT_EQ(snapshot.linear_summary.represented_degenerate, 0u);
    EXPECT_EQ(
        snapshot.linear_summary.represented_work_exhausted, 1u);
    EXPECT_EQ(
        snapshot.linear_summary.represented_arithmetic_range, 0u);
    EXPECT_EQ(snapshot.linear_summary.represented_work, 3437629u);
    EXPECT_EQ(linear_digest, ExpectedLinearCensusDigest);
    ASSERT_EQ(linear_fixture_pairs.size(), 1u);
    EXPECT_EQ(
        nonlinear_fixture::RosterDigest(linear_fixture_pairs),
        ExpectedLinearFixtureRosterDigest);
    EXPECT_EQ(
        linear_fixture_pairs[0].prepared[0].key.parent_eid,
        2142381u);
    EXPECT_EQ(
        linear_fixture_pairs[0].prepared[0].key.local_facet, 1u);
    EXPECT_EQ(
        linear_fixture_pairs[0].prepared[1].key.parent_eid,
        2230072u);
    EXPECT_EQ(
        linear_fixture_pairs[0].prepared[1].key.local_facet, 1u);
    EXPECT_EQ(
        linear_after,
        (std::array<std::size_t, 10>{
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0}));
    EXPECT_EQ(linear_after_work, 29u);
    EXPECT_EQ(linear_shared_vertex, 0u);
    EXPECT_EQ(linear_shared_edge, 0u);
    EXPECT_EQ(linear_actual_intersection, 0u);
    if (const char* output =
            std::getenv("ROBO_LINEAR_FIXTURE_OUTPUT");
        output && *output) {
        linear_fixture::Write(
            output, linear_fixture_pairs,
            FixtureProfileHash(),
            FixtureDtHash(snapshot.prepared_view),
            linear_digest, FixturePhaseIdentity(snapshot));
        const auto fixture = nonlinear_fixture::Read(
            output, false);
        std::cout << "V5_LINEAR_FIXTURE"
                  << " path=" << output
                  << " pairs=" << fixture.pairs.size()
                  << " payload_bytes="
                  << fixture.payload_bytes
                  << " payload_hash="
                  << fixture.payload_hash
                  << " roster_digest="
                  << fixture.roster_digest
                  << " source_hash="
                  << fixture.source_hash
                  << " schema_hash="
                  << fixture.schema_hash
                  << " profile_hash="
                  << fixture.profile_hash
                  << " dt_hash="
                  << fixture.dt_hash << '\n';
    }
    if (std::getenv("ROBO_LINEAR_CENSUS_ONLY")) {
        std::cout << "V5_LINEAR_CENSUS_ONLY authenticated=1\n";
        dynamics.DiscardStep();
        return;
    }

    std::size_t rigid_or_mixed = 0;
    std::size_t affine_rigid_or_mixed = 0;
    for (std::size_t facet = 0;
         facet < snapshot.motion_certificates.facet_count;
         ++facet) {
        const auto& motion =
            snapshot.motion_certificates.motion[facet];
        const auto& quadratic =
            snapshot.motion_certificates.quadratic[facet];
        EXPECT_TRUE(quadratic.complete);
        if (motion.motion ==
            contact::SelfContactFacetMotion::LinearNodalV1)
            continue;
        ++rigid_or_mixed;
        affine_rigid_or_mixed += motion.certified_affine;
    }
    EXPECT_EQ(rigid_or_mixed,
              snapshot.nonlinear_summary.rigid_or_mixed_facets);
    EXPECT_EQ(affine_rigid_or_mixed,
              snapshot.nonlinear_summary.
                  affine_rigid_or_mixed_facets);

    // Inspect every unresolved quadratic pair before consulting the frozen
    // roster. The fixture includes both pairs still unclassified after the
    // residual/persistence passes and local-intersection+persistent-ledger
    // pairs whose coverage path must independently exclude a nonlocal
    // crossing. This scope prevents the pinned IDs from defining their own
    // census while avoiding fixture replay for already proved classes.
    std::array<std::size_t, static_cast<unsigned>(
        NonlinearEvidenceClass::Count)> classes{};
    std::array<std::size_t, static_cast<unsigned>(
        NonlinearAfterClass::Count)> after_classes{};
    std::vector<NonlinearPairEvidence> evidence(
        snapshot.nonlinear_roster.size());
    std::atomic<std::size_t> next{0};
    const auto inspect = [&]() {
        for (;;) {
            const auto index =
                next.fetch_add(1, std::memory_order_relaxed);
            if (index >= snapshot.nonlinear_roster.size())
                return;
            const auto& entry = snapshot.nonlinear_roster[index];
            if (entry.separation.status ==
                sct::NonlinearSeparationStatus::CertifiedSeparated)
                continue;
            evidence[index] =
                InspectNonlinearPair(snapshot, entry);
        }
    };
    std::vector<std::thread> workers;
    workers.reserve(24);
    for (unsigned worker = 0; worker < 24; ++worker)
        workers.emplace_back(inspect);
    for (auto& worker : workers) worker.join();

    std::uint64_t ambiguous_digest = 1469598103934665603ull;
    std::vector<CanonicalFacetPair> ambiguous_roster;
    std::vector<nonlinear_fixture::Pair> fixture_pairs;
    std::array<std::size_t, 9> coverage_statuses{};
    std::size_t coverage_work = 0;
    unsigned coverage_depth = 0;
    std::uint64_t coverage_digest = 1469598103934665603ull;
    for (std::size_t index = 0;
         index < snapshot.nonlinear_roster.size(); ++index) {
        const auto& entry = snapshot.nonlinear_roster[index];
        if (entry.separation.status ==
            sct::NonlinearSeparationStatus::CertifiedSeparated)
            continue;
        const auto& first =
            snapshot.motion_certificates.prepared_triangles[
                entry.facets.first];
        const auto& second =
            snapshot.motion_certificates.prepared_triangles[
                entry.facets.second];

        const auto& inspected = evidence[index];
        ASSERT_TRUE(inspected.valid) << "roster index " << index;
        ++classes[static_cast<unsigned>(
            inspected.classification)];
        ++after_classes[static_cast<unsigned>(
            inspected.after)];
        const bool geometry_result_class =
            inspected.after ==
                NonlinearAfterClass::PossibleCurvedCrossing ||
            (inspected.local && inspected.ledger &&
             inspected.endpoint_contact);
        if (!geometry_result_class)
            continue;
        ambiguous_roster.push_back({
            first.key.parent_eid, first.key.local_facet,
            second.key.parent_eid, second.key.local_facet});
        auto frozen = FreezeNonlinearPair(snapshot, entry);
        const auto frozen_coverage =
            sct::CertifyQuadraticFacetCoverage(
                frozen.accepted[0], frozen.prepared[0],
                frozen.quadratic[0],
                frozen.half_thickness[0],
                frozen.accepted[1], frozen.prepared[1],
                frozen.quadratic[1],
                frozen.half_thickness[1],
                PhysicalStepS,
                frozen.accepted_owners.data(),
                frozen.accepted_owners.size(), 4095, 20);
        const auto coverage_status =
            static_cast<unsigned>(frozen_coverage.status);
        ASSERT_LT(coverage_status, coverage_statuses.size());
        ++coverage_statuses[coverage_status];
        coverage_work += frozen_coverage.work;
        coverage_depth =
            std::max(coverage_depth, frozen_coverage.deepest);
        HashPath(entry.key.paths[0], &coverage_digest);
        HashPath(entry.key.paths[1], &coverage_digest);
        HashUnsigned(coverage_status, &coverage_digest);
        HashUnsigned(frozen_coverage.work, &coverage_digest);
        HashUnsigned(
            frozen_coverage.deepest, &coverage_digest);
        HashUnsigned(
            frozen_coverage.accepted_source_order,
            &coverage_digest);
        HashUnsigned(
            frozen_coverage.proof_digest, &coverage_digest);
        if (frozen_coverage.status !=
                sct::NonlinearSeparationStatus::
                    CertifiedSeparated &&
            frozen_coverage.status !=
                sct::NonlinearSeparationStatus::
                    CertifiedAcceptedCoverage)
            std::cout
                << "V5_NONLINEAR_COVERAGE_UNRESOLVED"
                << " first=" << first.key.parent_eid
                << ":" << first.key.local_facet
                << " second=" << second.key.parent_eid
                << ":" << second.key.local_facet
                << " status=" << coverage_status
                << " work=" << frozen_coverage.work
                << " depth=" << frozen_coverage.deepest
                << " owners="
                << frozen.accepted_owners.size()
                << " separated_cells="
                << frozen_coverage.separated_cells
                << " covered_cells="
                << frozen_coverage.covered_cells
                << " work_exhausted="
                << frozen_coverage.work_exhausted
                << " depth_exhausted="
                << frozen_coverage.depth_exhausted
                << '\n';
        fixture_pairs.push_back(std::move(frozen));
        HashPath(entry.key.paths[0], &ambiguous_digest);
        HashPath(entry.key.paths[1], &ambiguous_digest);
        HashUnsigned(
            static_cast<unsigned>(entry.separation.status),
            &ambiguous_digest);
        HashUnsigned(entry.separation.work, &ambiguous_digest);
        HashUnsigned(entry.separation.deepest, &ambiguous_digest);
        std::cout << std::setprecision(17)
                  << "V5_NONLINEAR_AMBIGUOUS"
                  << " first=" << first.key.parent_eid
                  << ":" << first.key.local_facet
                  << " second=" << second.key.parent_eid
                  << ":" << second.key.local_facet
                  << " class="
                  << static_cast<unsigned>(
                         inspected.classification)
                  << " status="
                  << static_cast<unsigned>(
                         entry.separation.status)
                  << " work=" << entry.separation.work
                  << " depth=" << entry.separation.deepest
                  << " motion="
                  << static_cast<unsigned>(
                         snapshot.motion_certificates.motion[
                             entry.facets.first].motion)
                  << ","
                  << static_cast<unsigned>(
                         snapshot.motion_certificates.motion[
                             entry.facets.second].motion)
                  << " affine="
                  << snapshot.motion_certificates.motion[
                         entry.facets.first].certified_affine
                  << ","
                  << snapshot.motion_certificates.motion[
                         entry.facets.second].certified_affine
                  << " shared_vertices="
                  << inspected.shared_vertices
                  << " shared_edges=" << inspected.shared_edges
                  << " accepted_mask=" << inspected.accepted_mask
                  << " prepared_mask=" << inspected.prepared_mask
                  << " local=" << inspected.local
                  << " admitted=" << inspected.admitted
                  << " endpoint_separated="
                  << inspected.endpoint_separated
                  << " endpoint_contact="
                  << inspected.endpoint_contact
                  << " ledger=" << inspected.ledger
                  << " persistent_status="
                  << static_cast<unsigned>(
                         inspected.persistent_status)
                  << " quadratic_residual_status="
                  << static_cast<unsigned>(
                         inspected.residual_status)
                  << " accepted_minimum_m="
                  << inspected.accepted_minimum_m
                  << " prepared_minimum_m="
                  << inspected.prepared_minimum_m
                  << " thickness_m=" << inspected.thickness_m
                  << '\n';
    }

    const auto digest =
        NonlinearRosterDigest(snapshot.nonlinear_roster);
    std::cout << "V5_NONLINEAR_ROSTER"
              << " broadphase_parent_pairs="
              << snapshot.nonlinear_summary.
                     broadphase_parent_pairs
              << " streamed_facet_pairs="
              << snapshot.nonlinear_summary.streamed_facet_pairs
              << " rigid_or_mixed_facets="
              << snapshot.nonlinear_summary.rigid_or_mixed_facets
              << " affine_rigid_or_mixed_facets="
              << snapshot.nonlinear_summary.
                     affine_rigid_or_mixed_facets
              << " nonlinear_pairs="
              << snapshot.nonlinear_summary.nonlinear_pairs
              << " certified_separated="
              << snapshot.nonlinear_summary.certified_separated
              << " unresolved="
              << snapshot.nonlinear_summary.unresolved
              << " potential_contact="
              << snapshot.nonlinear_summary.potential_contact
              << " work_exhausted="
              << snapshot.nonlinear_summary.work_exhausted
              << " depth_exhausted="
              << snapshot.nonlinear_summary.depth_exhausted
              << " work=" << snapshot.nonlinear_summary.work
              << " digest=" << digest
              << " ambiguous_digest=" << ambiguous_digest;
    for (unsigned classification = 0;
         classification <
             static_cast<unsigned>(
                 NonlinearEvidenceClass::Count);
         ++classification)
        std::cout << " class" << classification << "="
                  << classes[classification];
    for (unsigned classification = 0;
         classification <
             static_cast<unsigned>(
                 NonlinearAfterClass::Count);
         ++classification)
        std::cout << " after" << classification << "="
                  << after_classes[classification];
    std::cout << '\n';

    EXPECT_EQ(snapshot.nonlinear_summary.unresolved,
              std::accumulate(
                  classes.begin(), classes.end(),
                  std::size_t{0}));
    EXPECT_EQ(
        snapshot.nonlinear_summary.broadphase_parent_pairs,
        1584555u);
    EXPECT_EQ(
        snapshot.nonlinear_summary.streamed_facet_pairs,
        5989588u);
    EXPECT_EQ(
        snapshot.nonlinear_summary.rigid_or_mixed_facets,
        27542u);
    EXPECT_EQ(
        snapshot.nonlinear_summary.affine_rigid_or_mixed_facets,
        14850u);
    EXPECT_EQ(snapshot.nonlinear_summary.nonlinear_pairs,
              103443u);
    EXPECT_EQ(snapshot.nonlinear_summary.certified_separated,
              37405u);
    EXPECT_EQ(snapshot.nonlinear_summary.unresolved, 66038u);
    EXPECT_EQ(snapshot.nonlinear_summary.potential_contact, 0u);
    EXPECT_EQ(snapshot.nonlinear_summary.work_exhausted, 0u);
    EXPECT_EQ(snapshot.nonlinear_summary.depth_exhausted, 66038u);
    EXPECT_EQ(snapshot.nonlinear_summary.work, 1424361u);
    EXPECT_EQ(
        classes,
        (std::array<std::size_t, 5>{
            52505, 7561, 5655, 317, 0}));
    EXPECT_EQ(
        after_classes,
        (std::array<std::size_t, 4>{
            52505, 7561, 5655, 317}));
    ASSERT_EQ(
        ambiguous_roster.size(),
        sizeof(ExpectedAmbiguousRoster) /
            sizeof(*ExpectedAmbiguousRoster));
    for (std::size_t pair = 0;
         pair < ambiguous_roster.size(); ++pair) {
        EXPECT_EQ(ambiguous_roster[pair].first_eid,
                  ExpectedAmbiguousRoster[pair].first_eid);
        EXPECT_EQ(ambiguous_roster[pair].first_local,
                  ExpectedAmbiguousRoster[pair].first_local);
        EXPECT_EQ(ambiguous_roster[pair].second_eid,
                  ExpectedAmbiguousRoster[pair].second_eid);
        EXPECT_EQ(ambiguous_roster[pair].second_local,
                  ExpectedAmbiguousRoster[pair].second_local);
    }
    EXPECT_EQ(snapshot.nonlinear_summary.unresolved,
              std::accumulate(
                  after_classes.begin(), after_classes.end(),
                  std::size_t{0}));
    EXPECT_EQ(digest, ExpectedNonlinearRosterDigest);
    EXPECT_EQ(
        ambiguous_digest, ExpectedAmbiguousRosterDigest);
    ASSERT_EQ(fixture_pairs.size(), nonlinear_fixture::ExpectedPairs);
    FreezeAcceptedPolicies(snapshot, &fixture_pairs);
    EXPECT_EQ(
        nonlinear_fixture::RosterDigest(fixture_pairs),
        nonlinear_fixture::ExpectedRosterDigest);
    std::cout << "V5_NONLINEAR_COVERAGE"
              << " pairs=" << fixture_pairs.size()
              << " work=" << coverage_work
              << " depth=" << coverage_depth
              << " digest=" << coverage_digest;
    for (std::size_t status = 0;
         status < coverage_statuses.size(); ++status)
        std::cout << " status" << status
                  << "=" << coverage_statuses[status];
    std::cout << '\n';
    if (const char* output =
            std::getenv("ROBO_NONLINEAR_FIXTURE_OUTPUT");
        output && *output) {
        nonlinear_fixture::Write(
            output, fixture_pairs, FixtureProfileHash(),
            FixtureDtHash(snapshot.prepared_view), digest,
            FixturePhaseIdentity(snapshot));
        const auto fixture = nonlinear_fixture::Read(output);
        EXPECT_EQ(fixture.pairs.size(), fixture_pairs.size());
        EXPECT_EQ(
            fixture.source_hash,
            nonlinear_fixture::SourceHash(fixture_pairs));
        std::cout << "V5_NONLINEAR_FIXTURE"
                  << " path=" << output
                  << " payload_bytes="
                  << fixture.payload_bytes
                  << " payload_hash="
                  << fixture.payload_hash
                  << " source_hash="
                  << fixture.source_hash
                  << " schema_hash="
                  << fixture.schema_hash
                  << " profile_hash="
                  << fixture.profile_hash
                  << " dt_hash=" << fixture.dt_hash
                  << '\n';
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
