#pragma once

// P1 qualification/measurement only. Resident prescribed configurations are
// not accepted simulation state, and this class owns no mechanics clock,
// nodal assembly, constitutive history, mass or time integration.
#include "lib_src/elements/ReissnerShellForce.h"
#include <cuda_runtime.h>
#include <cstddef>

namespace tl::qualification::reissner_batch {
namespace shell = tl::fea::reissner;

constexpr unsigned kMaximumElements = 128;
constexpr unsigned kMaximumEvaluations = 100;
constexpr unsigned kThreadsPerBlock = 32;

struct PrescribedInput {
  shell::ShellReference reference;
  shell::ElasticSection section;
  shell::ShellConfiguration configuration;
};
static_assert(sizeof(shell::ShellReference) == 2664);
static_assert(sizeof(shell::ElasticSection) == 1176);
static_assert(sizeof(shell::ShellConfiguration) == 224);
static_assert(sizeof(shell::ShellResult) == 976);
static_assert(sizeof(shell::ShellStatus) == 4);
static_assert(sizeof(PrescribedInput) == 4064);

enum class Status {
  kSuccess, kInvalidCapacity, kAlreadyInitialized, kNotInitialized,
  kInvalidArgument, kMissingInput, kEvaluationLimit, kElementFailure, kCudaFailure
};
struct Report {
  Status status = Status::kSuccess;
  const char* operation = "success";
  cudaError_t cuda_error = cudaSuccess;
  unsigned element = kMaximumElements;
  shell::ShellStatus element_status = shell::ShellStatus::kSuccess;
};
struct MemorySample {
  std::size_t free_bytes = 0;
  std::size_t total_bytes = 0;
};
struct KernelResources {
  int registers_per_thread = 0;
  std::size_t local_bytes_per_thread = 0;  // Not a function-stack measurement.
  std::size_t static_shared_bytes = 0;
  int maximum_threads_per_block = 0;
  int binary_version = 0;
  int ptx_version = 0;
  std::size_t current_stack_limit_bytes = 0;  // Queried, never changed.
  int multiprocessors = 0;
  int maximum_threads_per_multiprocessor = 0;
  int estimated_active_blocks_per_multiprocessor = 0;
  double estimated_occupancy = 0;  // Occupancy API bound, NOT achieved occupancy.
};
struct Initialization {
  double first_memory_query_ms = 0;  // Query may initialize the CUDA context.
  MemorySample after_context_query;
  MemorySample after_owned_allocation;
  MemorySample after_kernel_introspection;  // May include CUDA module loading.
  KernelResources kernel;
  std::size_t owned_device_bytes = 0;
  std::size_t owned_host_bytes = 0;
};
struct EvaluationTiming {
  double force_kernel_ms = 0;  // CUDA events exclude reduction/readback.
  double checked_end_to_end_ms = 0;  // Includes completion/status/results/publication.
  MemorySample after_completion;
};

// Calls and destruction must be externally serialized on one host thread.
// One explicit nonblocking stream is owned here; no borrowed state/streams.
// Initialization allocates once. Uploads may replace a prescribed slot, but
// never reset the lifetime <=100 evaluation budget. All slots must be uploaded
// before any launch; uploads cannot turn a poisoned owner healthy.
class PrescribedShellBatch {
 public:
  PrescribedShellBatch() = default;
  ~PrescribedShellBatch();
  PrescribedShellBatch(const PrescribedShellBatch&) = delete;
  PrescribedShellBatch& operator=(const PrescribedShellBatch&) = delete;

  Report Initialize(unsigned count);
  Report UploadElement(unsigned index, const PrescribedInput& input);
  // Caller supplies count valid writable results and an independent timing
  // object. On EVERY rejection both outputs remain byte-for-byte unchanged.
  // Device trial slots may change on failure; they are never published through
  // this API. LastElementStatuses is diagnostic only after completed kernels.
  Report EvaluateChecked(shell::ShellResult* output, unsigned count, EvaluationTiming& timing);
  const Initialization& initialization() const { return initialization_; }
  unsigned count() const { return count_; }
  unsigned evaluations() const { return evaluations_; }
  bool poisoned() const { return poisoned_; }
  const shell::ShellStatus* LastElementStatuses() const { return have_statuses_ ? host_statuses_ : nullptr; }
  static bool AdmittedCount(unsigned count);
  static std::size_t DeviceBytes(unsigned count);

 private:
  struct Aggregate { unsigned failures; unsigned first_failure; };
  static_assert(sizeof(Aggregate) == 8);
  Report CudaFailure(cudaError_t error, const char* operation);
  Report Ready() const;
  void* device_allocation_ = nullptr;
  PrescribedInput* device_inputs_ = nullptr;
  shell::ShellResult* device_results_ = nullptr;
  shell::ShellStatus* device_statuses_ = nullptr;
  Aggregate* device_aggregate_ = nullptr;
  cudaStream_t stream_ = nullptr;
  cudaEvent_t event_start_ = nullptr;
  cudaEvent_t event_stop_ = nullptr;
  shell::ShellResult host_results_[kMaximumElements]{};
  shell::ShellStatus host_statuses_[kMaximumElements]{};
  bool uploaded_[kMaximumElements]{};
  Aggregate host_aggregate_{};
  Initialization initialization_{};
  Report poison_report_{};
  unsigned count_ = 0;
  unsigned evaluations_ = 0;
  bool initialized_ = false;
  bool poisoned_ = false;
  bool have_statuses_ = false;
};

static_assert(sizeof(PrescribedShellBatch) < 128 * 1024);
// Includes this class, its complete device arena, one streamed host input and
// a caller output array. Chrono oracle objects and driver/runtime allocations
// are separate measured resources, not hidden in this explicit buffer ledger.
static_assert(sizeof(PrescribedShellBatch) +
              kMaximumElements * (sizeof(PrescribedInput) + sizeof(shell::ShellResult) + sizeof(shell::ShellStatus)) + 8 +
              sizeof(PrescribedInput) + 2 * kMaximumElements * sizeof(shell::ShellResult) < 1024 * 1024);
}  // namespace tl::qualification::reissner_batch
