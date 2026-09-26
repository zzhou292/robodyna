#pragma once
#include "VehiclePhysicalStartup.h"
#include "ParticipantConfigs.h"
#include "SourceRoles.h"
#include "../vehicle_startup/joints/VehicleJointModel.h"
#include <optional>
namespace crash::cases::vehicle_runtime {
struct VehiclePhysicalStartup::Storage {
    Storage(Source input,Config c,Forecast f) : source(std::move(input)),config(c),forecast(f) {}
    // Reverse destruction releases publication, participants, owner, then
    // the complete actual immutable source graph.
    Source source;
    Config config;
    Forecast forecast;
    SourceRoles roles;
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
