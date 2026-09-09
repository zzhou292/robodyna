#include "ReissnerShellCudaFixture.h"
#include "lib_utest/qualification/reissner_batch/PrescribedShellBatch.h"

#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace crash::qualification {
namespace {
namespace p1 = tl::qualification::reissner_batch;
struct Selection { unsigned count; unsigned repeats; };
unsigned ReadCount(const char* key, unsigned fallback) {
    const char* value = std::getenv(key);
    if (!value) return fallback;
    const std::string text(value);
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument(std::string(key) + " must be an unsigned decimal integer");
    std::size_t consumed = 0;
    const auto result = std::stoul(text, &consumed);
    if (consumed != text.size() || result > std::numeric_limits<unsigned>::max())
        throw std::invalid_argument(std::string(key) + " is out of range");
    return static_cast<unsigned>(result);
}
const Selection& Selected() {
    // Immutable per-process selection. No size ladder or automatic promotion.
    static const Selection selection = [] {
        Selection result{ReadCount("ROBO_DYNA_P1_ELEMENTS", 2), ReadCount("ROBO_DYNA_P1_REPEATS", 10)};
        if (!p1::PrescribedShellBatch::AdmittedCount(result.count) || !result.repeats || result.repeats > 100)
            throw std::invalid_argument("P1 requires elements=2/8/32/128 and repeats=1..100");
        return result;
    }();
    return selection;
}
void Passed(const p1::Report& report) {
    ASSERT_EQ(report.status, p1::Status::kSuccess)
        << report.operation << ": " << cudaGetErrorString(report.cuda_error);
}
Frames OffsetRest() {
    auto value = Neutral();
    value.q = {{Rotation(1.4, Vec(1, 0, 0)), Rotation(-1.6, Vec(1, 2, -1)),
                Rotation(2.3, Vec(-2, .5, 1)), Rotation(-2.2, Vec(.4, -1, 2))}};
    return value;
}
Frames AspectRest() {
    auto value = Neutral();
    for (auto& x : value.x) { x.x() *= .8; x.y() *= 1.25; }
    return value;
}
class ActualReference final : public ReissnerReference {
  public:
    explicit ActualReference(const Frames& rest) : ReissnerReference(rest) {}
    void Initialize() { ReissnerReference::SetUp(); }
    void Copy(p1::PrescribedInput& value) {
        crash::reference::CopyReissnerShellSetup(*element, value.reference, value.section);
    }
    Evaluation Sample(const Frames& frames) { return Evaluate(frames); }
    void Unchanged() { ReferenceUnchanged(); }
    void CheckCovariance(const Evaluation& before, const Evaluation& after, const Quat& q) {
        Covariant(before, after, q, "p1_common_spin");
    }
  private:
    void TestBody() override {}
};
Frames LocalConfiguration(unsigned index, unsigned family) {
    auto value = GeneralShellDeformation();
    const auto neutral = Neutral();
    for (unsigned n = 0; n < 4; ++n) {
        // All lanes are distinct prescribed finite configurations. Their rest
        // references remain admitted rectangles; no source mesh is flattened.
        value.x[n].x() *= 1 + 2e-6 * index;
        value.x[n].y() += 1e-5 * index * neutral.x[n].x();
        value.x[n].z() += 2e-5 * index * neutral.x[n].x() * neutral.x[n].y();
        value.q[n] = Rotation(2e-4 * index * neutral.x[n].y(), Vec(1, 0, 0)) * value.q[n];
        if (family == 2) { value.x[n].x() *= .8; value.x[n].y() *= 1.25; }
        if (family == 1) value.q[n] = value.q[n] * OffsetRest().q[n];
    }
    return value;
}
Quat CommonRotation(unsigned index) { return Rotation(1.7 + .005 * index, Vec(1, 2, -1)); }
Frames CurrentConfiguration(unsigned index, unsigned family) {
    return Transform(LocalConfiguration(index, family), CommonRotation(index), Vec(.003 * index, -.1, .04));
}
void Upload(p1::PrescribedShellBatch& batch, const p1::PrescribedInput& setup, unsigned index, unsigned family = 0) {
    auto input = setup;
    input.configuration = ToShell(CurrentConfiguration(index, family));
    Passed(batch.UploadElement(index, input));
}
void Property(const char* name, double value) {
    std::ostringstream text;
    text << std::setprecision(17) << value;
    ::testing::Test::RecordProperty(name, text.str());
}
void Bytes(const std::string& name, std::size_t value) {
    ::testing::Test::RecordProperty(name, std::to_string(value));
}
void Memory(const std::string& phase, const p1::MemorySample& memory) {
    Bytes(phase + "_free_bytes", memory.free_bytes);
    Bytes(phase + "_total_bytes", memory.total_bytes);
}
void TimingSeries(const char* name, std::vector<double> samples) {
    std::ostringstream values;
    values << std::setprecision(17);
    for (std::size_t i = 0; i < samples.size(); ++i) { if (i) values << ','; values << samples[i]; }
    ::testing::Test::RecordProperty(std::string(name) + "_samples_ms", values.str());
    std::sort(samples.begin(), samples.end());
    const double median = samples.size() % 2 ? samples[samples.size()/2]
        : .5 * (samples[samples.size()/2-1] + samples[samples.size()/2]);
    Property((std::string(name) + "_median_ms").c_str(), median);
    Property((std::string(name) + "_minimum_ms").c_str(), samples.front());
    Property((std::string(name) + "_maximum_ms").c_str(), samples.back());
}
void SeedOutputs(std::vector<tl_shell::ShellResult>& output, p1::EvaluationTiming& timing) {
    std::memset(output.data(), 0x35, output.size() * sizeof(output[0]));
    std::memset(&timing, 0x36, sizeof(timing));
}
class PublicationSnapshot {
  public:
    PublicationSnapshot(const std::vector<tl_shell::ShellResult>& output, const p1::EvaluationTiming& timing)
        : output_bytes_(output.size() * sizeof(output[0])) {
        std::memcpy(output_bytes_.data(), output.data(), output_bytes_.size());
        std::memcpy(timing_bytes_.data(), &timing, timing_bytes_.size());
    }
    void ExpectUnchanged(const std::vector<tl_shell::ShellResult>& output, const p1::EvaluationTiming& timing) const {
        ASSERT_EQ(output.size() * sizeof(output[0]), output_bytes_.size());
        EXPECT_EQ(std::memcmp(output.data(), output_bytes_.data(), output_bytes_.size()), 0);
        EXPECT_EQ(std::memcmp(&timing, timing_bytes_.data(), timing_bytes_.size()), 0);
    }
  private:
    std::vector<unsigned char> output_bytes_;
    std::array<unsigned char, sizeof(p1::EvaluationTiming)> timing_bytes_;
};
__global__ void InvalidConfigurationInjection() {}
}  // namespace

TEST(PrescribedShellBatch, CapacityAdmissionOccursBeforeAnyDeviceAllocation) {
    p1::PrescribedShellBatch batch;
    for (unsigned count : {0U, 1U, 3U, 127U, 129U, std::numeric_limits<unsigned>::max()}) {
        EXPECT_EQ(batch.Initialize(count).status, p1::Status::kInvalidCapacity);
        EXPECT_EQ(batch.count(), 0U);
        EXPECT_EQ(batch.initialization().owned_device_bytes, 0U);
        EXPECT_FALSE(batch.poisoned());
    }
    for (unsigned count : {2U, 8U, 32U, 128U}) {
        EXPECT_TRUE(p1::PrescribedShellBatch::AdmittedCount(count));
        EXPECT_EQ(p1::PrescribedShellBatch::DeviceBytes(count), count * (4064U + 976U + 4U) + 8U);
    }
}

TEST(PrescribedShellBatch, SelectedResidentBatchMatchesActualChronoAndReportsMeasuredPhases) {
    const auto selected = Selected();
    ActualReference canonical(Neutral()), offset(OffsetRest()), aspect(AspectRest());
    std::array<ActualReference*, 3> references{{&canonical, &offset, &aspect}};
    std::array<p1::PrescribedInput, 3> setup;
    for (unsigned i = 0; i < 3; ++i) { references[i]->Initialize(); references[i]->Copy(setup[i]); }
    ASSERT_FALSE(HasFatalFailure());
    std::vector<tl_shell::ShellResult> output(selected.count);
    p1::MemorySample released;
    {
        p1::PrescribedShellBatch batch;
        Passed(batch.Initialize(selected.count));
        ASSERT_FALSE(HasFatalFailure());
        const auto& initialization = batch.initialization();
        Bytes("selected_elements", selected.count);
        Bytes("selected_repeats", selected.repeats);
        Bytes("threads_per_block", p1::kThreadsPerBlock);
        Bytes("owned_device_bytes", initialization.owned_device_bytes);
        const std::size_t host_buffers = sizeof(batch) + sizeof(setup) + sizeof(p1::PrescribedInput) +
            output.size() * sizeof(output[0]) + 2 * selected.repeats * sizeof(double);
        Bytes("explicit_host_buffer_forecast_bytes", host_buffers);
        ASSERT_LE(host_buffers + initialization.owned_device_bytes, 1024U * 1024U);
        Property("first_memory_query_ms_may_include_context", initialization.first_memory_query_ms);
        Memory("after_context_query", initialization.after_context_query);
        Memory("after_owned_allocation", initialization.after_owned_allocation);
        Memory("after_introspection_may_include_module_loading", initialization.after_kernel_introspection);
        const auto& kernel = initialization.kernel;
        Bytes("kernel_registers_per_thread", kernel.registers_per_thread);
        Bytes("kernel_local_bytes_not_stack", kernel.local_bytes_per_thread);
        Bytes("kernel_static_shared_bytes", kernel.static_shared_bytes);
        Bytes("kernel_max_threads_per_block", kernel.maximum_threads_per_block);
        Bytes("kernel_binary_version", kernel.binary_version);
        Bytes("kernel_ptx_version", kernel.ptx_version);
        Bytes("queried_stack_limit_bytes_unchanged", kernel.current_stack_limit_bytes);
        Bytes("multiprocessors", kernel.multiprocessors);
        Bytes("estimated_active_blocks_per_sm", kernel.estimated_active_blocks_per_multiprocessor);
        Property("estimated_occupancy_not_achieved", kernel.estimated_occupancy);
        ASSERT_GT(kernel.registers_per_thread, 0);
        ASSERT_GE(kernel.maximum_threads_per_block, static_cast<int>(p1::kThreadsPerBlock));
        ASSERT_GT(kernel.estimated_occupancy, 0);
        ASSERT_LE(kernel.estimated_occupancy, 1);
        RecordProperty("scope", "resident_prescribed_rectangles_fixed_centered_elastic_layer_no_state_owner_or_assembly");
        RecordProperty("timing_scope", "force_only_CUDA_events;checked_host_latency_includes_status_reduce_results_and_publication");
        for (unsigned i = 0; i < selected.count; ++i) Upload(batch, setup[i % 3], i, i % 3);
        ASSERT_FALSE(HasFatalFailure());
        p1::MemorySample uploaded;
        ASSERT_EQ(cudaMemGetInfo(&uploaded.free_bytes, &uploaded.total_bytes), cudaSuccess);
        Memory("after_all_resident_uploads_before_force", uploaded);
        std::vector<double> device_ms, checked_ms;
        device_ms.reserve(selected.repeats); checked_ms.reserve(selected.repeats);
        for (unsigned repeat = 0; repeat < selected.repeats; ++repeat) {
            p1::EvaluationTiming timing;
            Passed(batch.EvaluateChecked(output.data(), selected.count, timing));
            ASSERT_FALSE(HasFatalFailure());
            ASSERT_NE(batch.LastElementStatuses(), nullptr);
            for (unsigned i = 0; i < selected.count; ++i) EXPECT_EQ(batch.LastElementStatuses()[i], tl_shell::ShellStatus::kSuccess);
            ASSERT_TRUE(std::isfinite(timing.force_kernel_ms) && timing.force_kernel_ms >= 0);
            ASSERT_TRUE(std::isfinite(timing.checked_end_to_end_ms) && timing.checked_end_to_end_ms > 0);
            device_ms.push_back(timing.force_kernel_ms); checked_ms.push_back(timing.checked_end_to_end_ms);
            if (!repeat) {
                Memory("after_first_evaluation", timing.after_completion);
                Property("first_force_kernel_ms", timing.force_kernel_ms);
                Property("first_checked_ms", timing.checked_end_to_end_ms);
            }
            if (repeat + 1 == selected.repeats) Memory("after_final_evaluation", timing.after_completion);
        }
        EXPECT_EQ(batch.evaluations(), selected.repeats);
        // Actual CPU Chrono and scalar TL host comparisons happen outside both
        // timing intervals. All lanes, not just an aggregate checksum, qualify.
        for (unsigned i = 0; i < selected.count; ++i) {
            SCOPED_TRACE(i);
            const unsigned family = i % 3;
            const auto frames = CurrentConfiguration(i, family);
            ExpectShellAgreement(output[i], references[family]->Sample(frames));
            tl_shell::ShellResult host;
            ASSERT_EQ(tl_shell::ComputeShellForce(setup[family].reference, setup[family].section, ToShell(frames), host),
                      tl_shell::ShellStatus::kSuccess);
            ExpectShellAgreement(output[i], ToEvaluation(host));
            if (i == 0 || i == selected.count/2 || i + 1 == selected.count) {
                const auto base = references[family]->Sample(LocalConfiguration(i, family));
                references[family]->CheckCovariance(base, ToEvaluation(output[i]), CommonRotation(i));
            }
            if (family == 1) {
                // Distinct initial nodal parameterizations represent the same
                // physical shell as the canonical reference, including spin.
                ExpectShellAgreement(output[i], canonical.Sample(CurrentConfiguration(i, 0)));
            }
        }
        for (auto* reference : references) reference->Unchanged();
        TimingSeries("all_force_kernel", device_ms);
        TimingSeries("all_checked", checked_ms);
        if (selected.repeats > 1) {
            TimingSeries("warm_force_kernel", std::vector<double>(device_ms.begin()+1, device_ms.end()));
            TimingSeries("warm_checked", std::vector<double>(checked_ms.begin()+1, checked_ms.end()));
        }
    }
    ASSERT_EQ(cudaMemGetInfo(&released.free_bytes, &released.total_bytes), cudaSuccess);
    Memory("after_owner_release_runtime_may_remain", released);
}

TEST(PrescribedShellBatch, MissingLastSlotAndLateElementFailurePreserveAllCallerOutputsThenRetry) {
    const auto selected = Selected();
    ActualReference reference(Neutral()); reference.Initialize();
    p1::PrescribedInput setup; reference.Copy(setup);
    ASSERT_FALSE(HasFatalFailure());
    p1::PrescribedShellBatch batch;
    Passed(batch.Initialize(selected.count)); ASSERT_FALSE(HasFatalFailure());
    std::vector<tl_shell::ShellResult> output(selected.count);
    p1::EvaluationTiming timing;
    SeedOutputs(output, timing);
    const PublicationSnapshot missing_snapshot(output, timing);
    EXPECT_EQ(batch.EvaluateChecked(nullptr, selected.count, timing).status, p1::Status::kInvalidArgument);
    EXPECT_EQ(batch.EvaluateChecked(output.data(), selected.count - 1, timing).status, p1::Status::kInvalidArgument);
    EXPECT_EQ(batch.EvaluateChecked(output.data(), selected.count, timing).status, p1::Status::kMissingInput);
    for (unsigned i = 0; i + 1 < selected.count; ++i) Upload(batch, setup, i);
    ASSERT_FALSE(HasFatalFailure());
    const auto missing = batch.EvaluateChecked(output.data(), selected.count, timing);
    EXPECT_EQ(missing.status, p1::Status::kMissingInput);
    EXPECT_EQ(missing.element, selected.count - 1);
    EXPECT_EQ(batch.evaluations(), 0U);
    missing_snapshot.ExpectUnchanged(output, timing);
    EXPECT_EQ(batch.LastElementStatuses(), nullptr);
    Upload(batch, setup, selected.count - 1);
    Passed(batch.EvaluateChecked(output.data(), selected.count, timing)); ASSERT_FALSE(HasFatalFailure());
    const auto before = output;
    const PublicationSnapshot valid_snapshot(output, timing);
    auto bad = setup;
    bad.configuration = ToShell(CurrentConfiguration(selected.count - 1, 0));
    bad.configuration.position[2] = {1e308, -1e308, 1e308};
    Passed(batch.UploadElement(selected.count - 1, bad)); ASSERT_FALSE(HasFatalFailure());
    const auto rejected = batch.EvaluateChecked(output.data(), selected.count, timing);
    EXPECT_EQ(rejected.status, p1::Status::kElementFailure);
    EXPECT_EQ(rejected.element, selected.count - 1);
    EXPECT_EQ(rejected.element_status, tl_shell::ShellStatus::kNonfiniteResult);
    ASSERT_NE(batch.LastElementStatuses(), nullptr);
    for (unsigned i = 0; i + 1 < selected.count; ++i) EXPECT_EQ(batch.LastElementStatuses()[i], tl_shell::ShellStatus::kSuccess);
    valid_snapshot.ExpectUnchanged(output, timing);
    Upload(batch, setup, selected.count - 1);
    EXPECT_EQ(batch.LastElementStatuses(), nullptr);
    Passed(batch.EvaluateChecked(output.data(), selected.count, timing)); ASSERT_FALSE(HasFatalFailure());
    EXPECT_EQ(std::memcmp(output.data(), before.data(), output.size()*sizeof(output[0])), 0);
    for (unsigned i = 0; i < selected.count; ++i) ExpectShellAgreement(output[i], reference.Sample(CurrentConfiguration(i, 0)));
    EXPECT_EQ(batch.evaluations(), 3U);
}

TEST(PrescribedShellBatch, LifetimeEvaluationCapCannotBeResetByUploads) {
    const auto selected = Selected();
    p1::PrescribedShellBatch batch;
    Passed(batch.Initialize(selected.count)); ASSERT_FALSE(HasFatalFailure());
    p1::PrescribedInput unprepared;
    for (unsigned i = 0; i < selected.count; ++i) Passed(batch.UploadElement(i, unprepared));
    ASSERT_FALSE(HasFatalFailure());
    std::vector<tl_shell::ShellResult> output(selected.count);
    p1::EvaluationTiming timing;
    SeedOutputs(output, timing);
    const PublicationSnapshot snapshot(output, timing);
    // Fast rejected references exercise the real launch counter without 100
    // expensive physical evaluations. Uploads never replenish the budget.
    for (unsigned i = 0; i < p1::kMaximumEvaluations; ++i) {
        ASSERT_EQ(batch.EvaluateChecked(output.data(), selected.count, timing).status, p1::Status::kElementFailure);
        snapshot.ExpectUnchanged(output, timing);
    }
    EXPECT_EQ(batch.evaluations(), p1::kMaximumEvaluations);
    Passed(batch.UploadElement(selected.count - 1, unprepared));
    EXPECT_EQ(batch.EvaluateChecked(output.data(), selected.count, timing).status, p1::Status::kEvaluationLimit);
    EXPECT_EQ(batch.Initialize(selected.count).status, p1::Status::kAlreadyInitialized);
    EXPECT_EQ(batch.evaluations(), p1::kMaximumEvaluations);
    snapshot.ExpectUnchanged(output, timing);
}

TEST(PrescribedShellBatch, SafeRuntimeErrorPoisonsWithoutPublishingOrAllowingUploadRecovery) {
    const auto selected = Selected();
    p1::PrescribedShellBatch batch;
    Passed(batch.Initialize(selected.count)); ASSERT_FALSE(HasFatalFailure());
    p1::PrescribedInput input;
    for (unsigned i = 0; i < selected.count; ++i) Passed(batch.UploadElement(i, input));
    ASSERT_FALSE(HasFatalFailure());
    std::vector<tl_shell::ShellResult> output(selected.count);
    p1::EvaluationTiming timing;
    SeedOutputs(output, timing);
    const PublicationSnapshot snapshot(output, timing);
    ASSERT_EQ(cudaPeekAtLastError(), cudaSuccess);
    InvalidConfigurationInjection<<<1, 0>>>();  // No kernel or invalid memory executes.
    const auto pending = cudaPeekAtLastError();
    ASSERT_TRUE(pending == cudaErrorInvalidValue || pending == cudaErrorInvalidConfiguration);
    const auto failed = batch.EvaluateChecked(output.data(), selected.count, timing);
    EXPECT_EQ(failed.status, p1::Status::kCudaFailure);
    EXPECT_EQ(failed.cuda_error, pending);
    EXPECT_TRUE(batch.poisoned());
    snapshot.ExpectUnchanged(output, timing);
    EXPECT_EQ(cudaPeekAtLastError(), cudaSuccess);
    EXPECT_EQ(batch.UploadElement(0, input).status, p1::Status::kCudaFailure);
    EXPECT_EQ(batch.EvaluateChecked(output.data(), selected.count, timing).status, p1::Status::kCudaFailure);
    EXPECT_EQ(batch.Initialize(selected.count).status, p1::Status::kCudaFailure);
    snapshot.ExpectUnchanged(output, timing);
}
}  // namespace crash::qualification
