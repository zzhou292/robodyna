#pragma once
#include "SourcePartElasticCase.h"

namespace crash::cases::source_part_elastic {
namespace fe = tl::fea;
namespace q = fe::qeph;
namespace t = fe::t3;
struct SourcePartWallState;

struct SourcePartElasticCase::Impl {
    source::SourcePartContactFixture source;
    source::SourceShellCollection collection;
    fe::ShellBatchBinding binding;
    Config config;
    // Destruction order is coordinator, batches, then owner.
    fe::FENodalState owner;
    q::QephBatch qeph;
    t::T3Batch t3;
    fe::ShellBatchPublication publication;
    std::unique_ptr<SourcePartWallState> wall;
    Snapshot accepted, trial;
    std::array<double,NodeCount> inverse_mass{}, inverse_inertia{}, spatial_shape{};
    std::array<double,3*NodeCount> pulse_force{}, endpoint_force{}, endpoint_couple{};
    std::array<q::ForceTrial,source::Q4Count> qresult{};
    std::array<t::ForceTrial,source::T3Count> tresult{};
    std::array<long double,3> accepted_q_work_magnitude{},accepted_t_work_magnitude{};
    std::array<long double,3> trial_q_work_magnitude{},trial_t_work_magnitude{};
    fe::NodalTrialToken token;
    fe::NodalPreparedView prepared;
    double* device_pulse = nullptr;
    double initial_kinetic = 0;
    bool initialized = false;
    ~Impl();
    Report Initialize(const source::SourcePartContactFixture&,const Config&);
    Report InitializeLoading();
    Report InitializeWall(const case_data::CanonicalWall&,const std::string&,const source_part_wall::SourcePartWallSettings&);
    Report AssembleWall(const fe::NodalAssemblyView&);
    Report EvaluateWall();
    Report GatherWallEndpoint();
    Report ObserveWall(long double synchronized_kinetic);
    void CommitWall() noexcept;
    void DiscardWall() noexcept;
    Report Prepare();
    Report EvaluateShells();
    Report Evaluate();
    Report Observe();
    Report Commit();
    void Discard() noexcept {
        owner.Discard(); publication.DiscardTrial(); qeph.DiscardTrial(); t3.DiscardTrial(); DiscardWall();
    }
};

inline Report Success() noexcept { return {Status::Ok,"Success"}; }
inline Report Failure(Status status,const char* message,double value=0,double limit=0,
                      std::uint64_t parent=0) noexcept {
    return {status,message,parent,value,limit};
}
} // namespace crash::cases::source_part_elastic
