#pragma once
#include "VehiclePhysicalStartup.h"
#include "ParticipantConfigs.h"
#include "SourceRoles.h"
#include "../vehicle_startup/joints/VehicleJointModel.h"
#include <optional>
namespace crash::cases::vehicle_runtime {
struct VehiclePhysicalStartup::Storage {
    Storage(const Execution& e,const Attachments& a,Config c,Forecast f,const JointModel* joints)
        : execution(e),attachments(a),config(c),forecast(f) {
        if(joints) joint_model.emplace(*joints);
    }
    // Reverse destruction releases the publication claim, participants, then
    // the sole owner. All immutable source handles outlive those consumers.
    Execution execution;
    Attachments attachments;
    Config config;
    Forecast forecast;
    SourceRoles roles;
    std::optional<JointModel> joint_model;
    tl::fea::FENodalState owner;
    tl::fea::qeph::QephBatch qeph;
    tl::fea::t3::T3Batch t3;
    tl::fea::qbat::Batch qbat;
    tl::fea::type25::Batch type25;
    tl::fea::type13::Batch type13;
    tl::fea::solids::Batch solids;
    std::unique_ptr<tl::fea::type45::Batch> type45;
    std::unique_ptr<tl::fea::beam18::Batch> beam18;
    tl::fea::ShellBatchPublication publication;
    void InitializeOwner();
    void InitializeParticipants();
    void BindInitialCaches();
    void InitializePublication();
    tl::fea::NodalAllocationInfo Allocations() const noexcept;
    void InspectOwner(InitialInspection&);
    void InspectShells(InitialInspection&);
    void InspectConnections(InitialInspection&);
    void InspectSolids(InitialInspection&);
    void InitializeStructuralBeams();
    void InspectStructuralBeams(InitialInspection&);
    void InitializeJoints();
    void InspectJoints(InitialInspection&);
};
} // namespace crash::cases::vehicle_runtime
