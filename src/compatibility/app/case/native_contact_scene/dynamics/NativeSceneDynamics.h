#pragma once
#include "ContactSelection.h"
#include "output/physical_frames/NativeAcceptedFrames.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
namespace crash::cases::native_scene {
namespace qualification {class NativeSceneAccess;}
struct DynamicsLimits {
    std::size_t host_bytes=512u<<20,device_bytes=128u<<20;
    native::TransactionLimits contact=[] {native::TransactionLimits l;
        l.max_device_bytes=32u<<20;l.max_host_bytes=32u<<20;
        l.optimized_candidates=4096;l.sliding_entries=4096;return l;}();
};
struct DynamicsConfig {
    std::uint64_t configuration=0,qualification=0;
    double fixed_dt=3e-7,maximum_rotation_increment=.2;
    DynamicsLimits limits;
};
struct DynamicsForecast {
    std::size_t source_host_bytes=0,packing_bytes=0,retained_host_bytes=0,peak_host_bytes=0,device_bytes=0;
    std::size_t transaction_host_reservation=0,transaction_device_reservation=0;
    tl::fea::NodalAssemblyCinForecast owner;
    tl::fea::ShellMappedFootprint qeph,t3;
    tl::fea::ShellPhysicalPublicationForecast publication;
    tl::fea::ShellPhysicalScratchParticipationForecast roster;
};
struct NativeStepObservation {
    tl::fea::NodalStamp base;
    tl::fea::ShellPhysicalDiagnostics mechanics;
    tl::fea::NodalCinStructuralLimit structural_limit;
    native::TransactionDiagnostics contact;
};
// Stable private physical storage, one TL owner/clock/common publisher. A
// capture created here borrows that storage and must be destroyed first.
class NativeSceneDynamics {
  public:
    static DynamicsForecast Preflight(const ContactSelection&,DynamicsConfig);
    static NativeSceneDynamics Prepare(const ContactSelection&,DynamicsConfig);
    ~NativeSceneDynamics();
    NativeSceneDynamics(NativeSceneDynamics&&) noexcept;
    NativeSceneDynamics& operator=(NativeSceneDynamics&&)=delete;
    NativeSceneDynamics(const NativeSceneDynamics&)=delete;
    NativeSceneDynamics& operator=(const NativeSceneDynamics&)=delete;
    const NativeStepObservation& PrepareStep();
    void CommitStep();
    void DiscardStep() noexcept;
    bool has_prepared_step() const noexcept;
    tl::fea::NodalStamp accepted() const noexcept;
    const NativeStepObservation& last_accepted_step() const;
    const DynamicsForecast& forecast() const noexcept;
    const ContactSelection& source() const noexcept;
    std::unique_ptr<output::physical_frames::NativeAcceptedFrames> MakeCapture(
        const output::full_shell::source::PreparedSourceMapping&,output::full_shell::Identity,
        output::physical_frames::Limits={});
  private:
    friend class qualification::NativeSceneAccess;
    struct Storage;std::unique_ptr<Storage> storage_;
    explicit NativeSceneDynamics(std::unique_ptr<Storage>);
};
}
