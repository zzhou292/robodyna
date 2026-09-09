#include "SourceNodalWallProfile.h"
#include "SourceNodalWallTest.h"
#include <iostream>

namespace crash::qualification::source_contact::nodal::test {
namespace {
using Tick=unsigned long long;
struct Span { Tick first=0,last=0,minimum=0,median=0,maximum=0; };
bool Summarize(const Tick* begin,const Tick* end,unsigned count,Span* output) {
    if (!begin || !end || !output || !count || count>Workers) return false;
    std::array<Tick,Workers> durations{};
    Span next; next.first=UINT64_MAX;
    for (unsigned n=0;n<count;++n) {
        if (!begin[n] || end[n]<begin[n]) return false;
        next.first=std::min(next.first,begin[n]); next.last=std::max(next.last,end[n]); durations[n]=end[n]-begin[n];
    }
    std::sort(durations.begin(),durations.begin()+count);
    next.minimum=durations[0]; next.median=durations[count/2]; next.maximum=durations[count-1]; *output=next; return true;
}
std::string Csv(const Tick* values,unsigned count) {
    std::ostringstream text;
    for (unsigned n=0;n<count;++n) { if (n) text<<','; text<<values[n]; }
    return text.str();
}
void Trace(unsigned run,const ProfileClocks& clocks) {
    Span query,law,parent,publish;
    ASSERT_TRUE(Summarize(clocks.query_begin,clocks.query_end,NodeCount,&query));
    ASSERT_TRUE(Summarize(clocks.law_begin,clocks.law_end,NodeCount,&law));
    ASSERT_TRUE(Summarize(clocks.parent_begin,clocks.parent_end,ParentCount,&parent));
    ASSERT_TRUE(Summarize(clocks.publish_begin,clocks.publish_end,Workers,&publish));
    ASSERT_GT(clocks.kernel_begin,0u); ASSERT_GE(clocks.validation_begin,clocks.kernel_begin);
    ASSERT_GE(clocks.validation_end,clocks.validation_begin); ASSERT_GE(query.first,clocks.validation_end);
    for (unsigned n=0;n<NodeCount;++n) ASSERT_GE(clocks.law_begin[n],clocks.query_end[n]);
    ASSERT_GE(clocks.node_scan_begin,law.last); ASSERT_GE(clocks.node_scan_end,clocks.node_scan_begin);
    ASSERT_GE(parent.first,clocks.node_scan_end); ASSERT_GE(clocks.reduction_begin,parent.last);
    ASSERT_GE(clocks.reduction_end,clocks.reduction_begin); ASSERT_GE(publish.first,clocks.reduction_end);
    ASSERT_GT(publish.last,clocks.kernel_begin);
    const Tick total=publish.last-clocks.kernel_begin;
    const std::string prefix="run_"+std::to_string(run)+"_";
    const auto record=[&](const char* name,const std::string& value) { ::testing::Test::RecordProperty(prefix+name,value); };
    const auto cycles=[&](const char* name,Tick value) { record(name,std::to_string(value)); };
    const auto fraction=[&](const char* name,Tick value) { record(name,Number(static_cast<double>(value)/total)); };
    cycles("block_elapsed_cycles",total); cycles("kernel_begin",clocks.kernel_begin);
    cycles("validation_begin",clocks.validation_begin); cycles("validation_end",clocks.validation_end);
    cycles("node_scan_begin",clocks.node_scan_begin); cycles("node_scan_end",clocks.node_scan_end);
    cycles("reduction_begin",clocks.reduction_begin); cycles("reduction_end",clocks.reduction_end);
    cycles("validation_cycles",clocks.validation_end-clocks.validation_begin);
    cycles("node_status_scan_cycles",clocks.node_scan_end-clocks.node_scan_begin);
    cycles("node_stage_cycles",law.last-query.first); cycles("parent_stage_cycles",parent.last-parent.first);
    cycles("global_reduction_cycles",clocks.reduction_end-clocks.reduction_begin);
    cycles("publication_cycles",publish.last-publish.first);
    cycles("query_thread_min_cycles",query.minimum); cycles("query_thread_median_cycles",query.median); cycles("query_thread_max_cycles",query.maximum);
    cycles("law_thread_min_cycles",law.minimum); cycles("law_thread_median_cycles",law.median); cycles("law_thread_max_cycles",law.maximum);
    fraction("validation_fraction",clocks.validation_end-clocks.validation_begin);
    fraction("node_status_scan_fraction",clocks.node_scan_end-clocks.node_scan_begin);
    fraction("node_stage_fraction",law.last-query.first); fraction("query_span_fraction",query.last-query.first);
    fraction("law_span_fraction",law.last-law.first); fraction("parent_stage_fraction",parent.last-parent.first);
    fraction("global_reduction_fraction",clocks.reduction_end-clocks.reduction_begin);
    fraction("publication_fraction",publish.last-publish.first);
    record("query_begin_raw",Csv(clocks.query_begin,NodeCount)); record("query_end_raw",Csv(clocks.query_end,NodeCount));
    record("law_begin_raw",Csv(clocks.law_begin,NodeCount)); record("law_end_raw",Csv(clocks.law_end,NodeCount));
    record("parent_begin_raw",Csv(clocks.parent_begin,ParentCount)); record("parent_end_raw",Csv(clocks.parent_end,ParentCount));
    record("publish_begin_raw",Csv(clocks.publish_begin,Workers)); record("publish_end_raw",Csv(clocks.publish_end,Workers));
}
void Facts(const KernelFacts& facts) {
    const auto record=[](const std::string& key,const std::string& value) { ::testing::Test::RecordProperty(key,value); };
    for (unsigned variant=0;variant<2;++variant) {
        const auto& value=variant?facts.instrumented:facts.original; const std::string prefix=variant?"instrumented_":"original_";
        record(prefix+"registers_per_thread",std::to_string(value.numRegs));
        record(prefix+"local_bytes_per_thread",std::to_string(value.localSizeBytes));
        record(prefix+"static_shared_bytes_per_block",std::to_string(value.sharedSizeBytes));
        record(prefix+"max_threads_per_block",std::to_string(value.maxThreadsPerBlock));
        record(prefix+"binary_version",std::to_string(value.binaryVersion));
        record(prefix+"ptx_version",std::to_string(value.ptxVersion));
        record(prefix+"estimated_max_blocks_per_sm",std::to_string(variant?facts.instrumented_max_blocks_per_sm:facts.original_max_blocks_per_sm));
    }
    record("device_sm_count",std::to_string(facts.multiprocessors)); record("max_threads_per_sm",std::to_string(facts.max_threads_per_sm));
    record("read_only_stack_limit_bytes",std::to_string(facts.stack_limit_bytes));
}
double Median(std::array<double,5> values) { std::sort(values.begin(),values.end()); return values[2]; }
} // namespace

TEST_F(Check, ProfileOriginalCoherentKernelWithoutChangingArithmeticOrLimits) {
    cw::WallTessellation original; ASSERT_NO_FATAL_FAILURE(Geometry(original)); Input input;
    ASSERT_NO_FATAL_FAILURE(Coherent(*original.geometry(),input)); ASSERT_EQ(input.face_count,100u);
    Result expected; ASSERT_EQ(EvaluateHost(prepared,input,&expected).status,Status::Ok);
    std::size_t free_context=0,total=0,free_attributes=0,free_objects=0,free_warm=0,free_final=0;
    ASSERT_EQ(cudaMemGetInfo(&free_context,&total),cudaSuccess);
    KernelFacts facts; ASSERT_EQ(InspectKernels(facts),cudaSuccess); ASSERT_NO_FATAL_FAILURE(Facts(facts));
    ASSERT_GE(facts.original.maxThreadsPerBlock,static_cast<int>(Workers));
    ASSERT_GE(facts.instrumented.maxThreadsPerBlock,static_cast<int>(Workers));
    ASSERT_GT(facts.original_max_blocks_per_sm,0); ASSERT_GT(facts.instrumented_max_blocks_per_sm,0);
    ASSERT_EQ(cudaMemGetInfo(&free_attributes,&total),cudaSuccess);
    Device device; Profiler profiler;
    ASSERT_EQ(device.initialization_status(),cudaSuccess); ASSERT_EQ(profiler.initialization_status(),cudaSuccess);
    ASSERT_EQ(cudaMemGetInfo(&free_objects,&total),cudaSuccess);
    Result control,measured; Report report; Timing control_time,profile_time; ProfileClocks clocks;
    ASSERT_EQ(device.Evaluate(input,control,report,control_time),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(Compare(expected,control));
    ASSERT_EQ(profiler.Evaluate(input,measured,report,profile_time,clocks),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(ExactResult(control,measured));
    ASSERT_EQ(cudaMemGetInfo(&free_warm,&total),cudaSuccess);
    std::array<double,5> original_ms{},instrumented_ms{};
    for (unsigned run=0;run<5;++run) {
        ASSERT_EQ(device.Evaluate(input,control,report,control_time),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
        ASSERT_NO_FATAL_FAILURE(Compare(expected,control));
        ASSERT_EQ(profiler.Evaluate(input,measured,report,profile_time,clocks),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
        ASSERT_NO_FATAL_FAILURE(ExactResult(control,measured));
        ASSERT_NO_FATAL_FAILURE(Trace(run,clocks));
        original_ms[run]=control_time.kernel_ms; instrumented_ms[run]=profile_time.kernel_ms;
        ASSERT_GT(original_ms[run],0); ASSERT_GT(instrumented_ms[run],0);
        RecordProperty("run_"+std::to_string(run)+"_original_event_ms",Number(original_ms[run]));
        RecordProperty("run_"+std::to_string(run)+"_instrumented_event_ms",Number(instrumented_ms[run]));
        RecordProperty("run_"+std::to_string(run)+"_original_end_to_end_ms",Number(control_time.checked_end_to_end_ms));
        RecordProperty("run_"+std::to_string(run)+"_profile_end_to_end_ms",Number(profile_time.checked_end_to_end_ms));
    }
    ASSERT_EQ(cudaMemGetInfo(&free_final,&total),cudaSuccess);
    RecordProperty("original_event_median_ms",Number(Median(original_ms)));
    RecordProperty("instrumented_event_median_ms",Number(Median(instrumented_ms)));
    RecordProperty("instrumentation_event_ratio",Number(Median(instrumented_ms)/Median(original_ms)));
    RecordProperty("original_owned_bytes",static_cast<int>(sizeof(Storage)));
    RecordProperty("instrumented_owned_bytes",static_cast<int>(sizeof(ProfileStorage)));
    RecordProperty("combined_owned_bytes",static_cast<int>(sizeof(Storage)+sizeof(ProfileStorage)));
    RecordProperty("free_after_context_before_introspection_bytes",std::to_string(free_context));
    RecordProperty("free_after_introspection_bytes",std::to_string(free_attributes));
    RecordProperty("free_after_introspection_and_objects_bytes",std::to_string(free_objects));
    RecordProperty("free_after_warm_bytes",std::to_string(free_warm)); RecordProperty("free_after_measured_bytes",std::to_string(free_final));
    RecordProperty("grid_blocks",1); RecordProperty("threads_per_block",128);
    RecordProperty("cycle_scope","clock64 elapsed SM cycles include scheduling; query/law thread spans overlap and are not additive exclusive times");
    RecordProperty("qualification_scope","profiling preserves returned fields; it does not waive the separate original 2 ms/5 ms/20x cost gate"); Metadata();
}
} // namespace crash::qualification::source_contact::nodal::test

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=3) { std::cerr<<"Usage: source_nodal_wall_profile readiness.json wall.manifest.json [gtest options]\n"; return 2; }
    crash::qualification::source_contact::nodal::test::readiness_path=argv[1];
    crash::qualification::source_contact::nodal::test::wall_path=argv[2];
    return RUN_ALL_TESTS();
}
