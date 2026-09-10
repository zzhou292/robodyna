#pragma once
#include "ComponentFrameTestSupport.h"
#include "output/source_assembly/binary_frames/SourceAssemblyBinaryFrameState.h"
#include "case/source_assembly_dynamics/tests/Fixture.h"

namespace crash::output::assembly::binary {
struct SourceAssemblyBinaryFrameTestAccess {
    static Document Legacy(const SourceAssemblyBinaryFrames& producer,const dynamics::SourceAssemblyWallCase& run) {
        return SourceAssemblyWallFrameFields(run,producer.impl_->capture);
    }
    static std::array<std::uintptr_t,4> Storage(const SourceAssemblyBinaryFrames& producer) {
        const auto& f=producer.impl_->frames;
        return {reinterpret_cast<std::uintptr_t>(f[0].position_xyz.data()),reinterpret_cast<std::uintptr_t>(f[1].position_xyz.data()),
            reinterpret_cast<std::uintptr_t>(f[0].plastic_points.data()),reinterpret_cast<std::uintptr_t>(f[1].plastic_points.data())};
    }
};
namespace test {
using Access=cases::source_assembly_dynamics::SourceAssemblyDynamicsTestAccess;
inline void CompareAndRoundTrip(SourceAssemblyBinaryFrames& producer,dynamics::SourceAssemblyWallCase& run,
                               const std::filesystem::path& root) {
    producer.Capture(run);ASSERT_NE(producer.frame(),nullptr);
    CompareLegacy(producer.context(),*producer.frame(),SourceAssemblyBinaryFrameTestAccess::Legacy(producer,run));
    const auto file=producer.Write(root,"frame-"+std::to_string(producer.frame()->stamp.epoch));
    Same(*producer.frame(),records::ReadFrame(root,producer.context(),file,producer.frame()->stamp));
    for(std::size_t i=0;i<producer.mapping().binding().triangles.size();++i) {
        const auto& t=producer.mapping().binding().triangles[i];
        EXPECT_EQ(t.element,producer.mapping().triangle_parents()[i]);
        const auto& parents=producer.context().parents();
        const auto found=std::find_if(parents.begin(),parents.end(),[&](const auto& p){return p.source_element==t.element;});
        ASSERT_NE(found,parents.end());EXPECT_EQ(found->source_part,t.part);
    }
}
inline void ActualParity(dynamics::SourceAssemblyWallCase& run,std::size_t nodes,std::size_t parents) {
    fixture::Directory directory;ASSERT_TRUE(std::filesystem::create_directory(directory.path));
    SourceAssemblyBinaryFrames producer(run,{run.owner()->accepted().owner_id,23,31},5);
    ASSERT_EQ(producer.frame(),nullptr);EXPECT_EQ(producer.context().nodes(),nodes);EXPECT_EQ(producer.context().parents().size(),parents);
    const auto allocations=run.allocations();const auto storage=SourceAssemblyBinaryFrameTestAccess::Storage(producer);
    CompareAndRoundTrip(producer,run,directory.path);
    std::uint64_t rejected_attempt=0;
    for(unsigned i=0;i<64;++i) {
        const auto result=run.Step();ASSERT_TRUE(result)<<result.message;
        if(i==31) {
            const auto accepted=run.owner()->accepted();
            rejected_attempt=run.diagnostics()->shells.qeph.attempt+1;
            ASSERT_FALSE(Access::RejectLate(run,Access::Fault::LastWallFace));
            EXPECT_TRUE(tl::fea::trial_identity::SameStamp(accepted,run.owner()->accepted()));
        }
        if((i+1)%32==0) {
            CompareAndRoundTrip(producer,run,directory.path);
            const auto held=*producer.frame();const auto* pointer=producer.frame();
            EXPECT_THROW(producer.Capture(run),std::runtime_error);
            EXPECT_EQ(producer.frame(),pointer);Same(held,*producer.frame());
        }
        EXPECT_EQ(SourceAssemblyBinaryFrameTestAccess::Storage(producer),storage);
        EXPECT_EQ(run.allocations().device_allocations,allocations.device_allocations);
        EXPECT_EQ(run.allocations().device_bytes,allocations.device_bytes);
    }
    EXPECT_GT(producer.frame()->stamp.attempt,rejected_attempt);
    EXPECT_GT(producer.frame()->stamp.attempt,producer.frame()->stamp.epoch);
    EXPECT_GT(run.diagnostics()->contact_intervals,0u);
}
} // namespace test
} // namespace crash::output::assembly::binary
