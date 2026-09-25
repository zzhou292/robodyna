#pragma once
#include "Buffers.h"
#include "output/full_shell/static_bundle/PreparedSourceMapping.h"
#include "output/physical_run/AcceptedInterval.h"
#include "lib_src/collision/RadiossType25Transaction.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
namespace crash::output::physical_frames {
// Generic live QEPH/T3 + fixed-main native TYPE25 capture. All operations are
// externally serialized with the one owner/common publisher. Those objects and
// participant batches must outlive this capture. No simulation call occurs here.
class NativeAcceptedFrames {
  public:
    NativeAcceptedFrames(const records::source::PreparedSourceMapping&,const tl::fea::ShellPhysicalBinding&,
        tl::fea::FENodalState&,tl::fea::ShellBatchPublication&,tl::fea::qeph::QephBatch&,tl::fea::t3::T3Batch&,
        tlfea::contact::radioss_type25::Transaction&,records::Identity,Limits={});
    ~NativeAcceptedFrames();
    NativeAcceptedFrames(const NativeAcceptedFrames&)=delete;
    NativeAcceptedFrames& operator=(const NativeAcceptedFrames&)=delete;
    void Capture();
    // Reads actual accepted common/native metadata at the current endpoint;
    // it does not require a full frame readback on every interval.
    physical_run::AcceptedInterval Interval() const;
    static physical_run::Profile ObservationProfile() noexcept;
    const records::Context& context() const noexcept;
    const Forecast& forecast() const noexcept;
    const records::FrameRecord* frame() const noexcept;
    const records::activity::ActivityRecord* activity() const noexcept;
    const tlfea::contact::radioss_type25::NativeGeometryHistory* native_history() const noexcept;
    const int* initial_contact_flags() const noexcept;
    std::size_t secondary_count() const noexcept;
  private:
    struct Impl;std::unique_ptr<Impl> impl_;
};
}
