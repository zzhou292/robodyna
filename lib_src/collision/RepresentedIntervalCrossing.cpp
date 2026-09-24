// SPDX-License-Identifier: AGPL-3.0-or-later
#include "RepresentedIntervalCrossing.h"
#include "represented_interval_crossing/NormalReuseQualification.h"
#include "represented_interval_crossing/RelativeSeparationQualification.h"
#include "represented_interval_crossing/ExactPathReuseQualification.h"
#include "represented_interval_crossing/CommonPointReuseQualification.h"
#include "represented_interval_crossing/BatchExecution.h"
#include "represented_interval_crossing/native/CellKernel.h"
#include "represented_interval_crossing/NativeStorageQualification.h"
#include "represented_interval_crossing/FixedPolicyQualification.h"
#include "represented_interval_crossing/native/FixedIntegerPolicy.h"
#include "represented_interval_crossing/DeviceExecution.h"
#include "represented_interval_crossing/BusyRelease.h"
#include "represented_interval_crossing/CohortAdmission.h"

#include <algorithm>
#include <atomic>
#include <boost/multiprecision/cpp_int.hpp>
#include <cerrno>
#include <cstring>
#include <limits>
#include <new>
#include <optional>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <tuple>
#include <type_traits>
#include <unistd.h>
#include <utility>
#include <vector>

namespace tlfea::contact {
namespace {

constexpr std::size_t kWorkerStackBytes = 2u << 20;

// Private numerical templates retain the original arithmetic bodies. The owner
// and forecast continue to hold the original wide scratch and DFS frame types.
namespace native = represented_interval_crossing::native;
using namespace native;
using WideKernel = native::CellKernel<16384>;
using NarrowKernel = native::CellKernel<512>;
using ExactScratch = WideKernel::ExactScratch;
using ExactVec3 = WideKernel::ExactVec3;
enum class NativeStorage { Wide, Adaptive };
using StorageCounters = represented_interval_crossing::NativeStorageCounters;


bool AddSize(std::size_t a, std::size_t b, std::size_t* output) noexcept {
  if (a > SIZE_MAX - b)
    return false;
  *output = a + b;
  return true;
}

bool MultiplySize(std::size_t a, std::size_t b,
                  std::size_t* output) noexcept {
  if (a && b > SIZE_MAX / a)
    return false;
  *output = a * b;
  return true;
}

bool RangeDisjoint(const void* a, std::size_t a_bytes, const void* b,
                   std::size_t b_bytes) noexcept {
  if (!a_bytes || !b_bytes)
    return true;
  if (!a || !b)
    return false;
  const auto aa = reinterpret_cast<std::uintptr_t>(a);
  const auto bb = reinterpret_cast<std::uintptr_t>(b);
  if (aa > UINTPTR_MAX - a_bytes || bb > UINTPTR_MAX - b_bytes)
    return false;
  return aa + a_bytes <= bb || bb + b_bytes <= aa;
}

bool Wait(sem_t* semaphore) noexcept {
  if (!semaphore)
    return false;
  while (sem_wait(semaphore) != 0) {
    if (errno != EINTR)
      return false;
  }
  return true;
}

std::size_t PageBytes() noexcept {
  const long value = sysconf(_SC_PAGESIZE);
  return value > 0 ? static_cast<std::size_t>(value) : 0;
}

using represented_interval_crossing::CanonicalPair;

struct VertexLedgerRow {
  FacetVertexKey key;
  Vec3 endpoint[2];
  RepresentedMotion motion = RepresentedMotion::LinearNodalV1;
  std::size_t input_path = SIZE_MAX;
};

bool VertexLedgerLess(const VertexLedgerRow& a,
                      const VertexLedgerRow& b) noexcept {
  return Compare(a.key, b.key) < 0;
}

bool PairLess(const CanonicalPair& a, const CanonicalPair& b) noexcept {
  return Compare(a.key, b.key) < 0;
}

bool SamePair(const CanonicalPair& a, const CanonicalPair& b) noexcept {
  return Compare(a.key, b.key) == 0;
}

void CountStorage(StorageCounters* counters, bool narrow) noexcept {
  if (!counters) return;
  auto& value = narrow ? counters->narrow_pairs : counters->wide_pairs;
  if (value == SIZE_MAX) counters->saturated = true;
  else ++value;
}

template <NormalReuse reuse, SeparationProof separation,
          ExactPathReuse path_reuse, CommonPointReuse point_reuse>
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
RepresentedIntervalResult CertifyNarrow(
    const RepresentedTrianglePath& a, const RepresentedTrianglePath& b,
    RepresentedIntervalLimits limits, RepresentedIntervalPairKey key,
    Cell* dfs, std::size_t dfs_capacity,
    NormalCounters* counters, SeparationCounters* separation_counters,
    ExactPathCounters* path_counters, CommonPointCounters* common_point_counters) noexcept {
  // This is a new stack object only inside the eligible non-inlined callee.
  // Existing retained wide storage and the 2 MiB prefaulted worker stack are
  // unchanged. sizeof is necessary, not a full transitive stack-usage proof.
  static_assert(sizeof(NarrowKernel::ExactScratch) <= 8192);
  static_assert(std::is_nothrow_default_constructible_v<NarrowKernel::ExactScratch>);
  static_assert(std::is_nothrow_destructible_v<NarrowKernel::ExactScratch>);
  NarrowKernel::ExactScratch scratch;
  native::ArithmeticContext context;
  NarrowKernel kernel(context);
  return kernel.template CertifyPair<reuse, separation, path_reuse, point_reuse>(
      a, b, limits, key, dfs, dfs_capacity, &scratch, counters,
      separation_counters, path_counters, common_point_counters);
}

template <NormalReuse reuse = NormalReuse::Memoize,
          SeparationProof separation = SeparationProof::RelativeFaces,
          ExactPathReuse path_reuse = ExactPathReuse::Optimized,
          CommonPointReuse point_reuse = CommonPointReuse::Optimized,
          NativeStorage storage = NativeStorage::Adaptive>
RepresentedIntervalResult CertifyPair(
    const RepresentedTrianglePath& a, const RepresentedTrianglePath& b,
    RepresentedIntervalLimits limits, RepresentedIntervalPairKey key,
    Cell* dfs, std::size_t dfs_capacity, ExactScratch* scratch,
    NormalCounters* counters = nullptr,
    SeparationCounters* separation_counters = nullptr,
    ExactPathCounters* path_counters = nullptr,
    CommonPointCounters* common_point_counters = nullptr,
    StorageCounters* storage_counters = nullptr) noexcept {
  if constexpr (storage == NativeStorage::Adaptive) {
    if (represented_interval_crossing::NativeStorageDomain::FromPaths(a, b, limits.max_depth).eligible()) {
      CountStorage(storage_counters, true);
      return CertifyNarrow<reuse, separation, path_reuse, point_reuse>(
          a, b, limits, key, dfs, dfs_capacity, counters,
          separation_counters, path_counters, common_point_counters);
    }
  }
  CountStorage(storage_counters, false);
  native::ArithmeticContext context;
  WideKernel kernel(context);
  return kernel.template CertifyPair<reuse, separation, path_reuse, point_reuse>(
      a, b, limits, key, dfs, dfs_capacity, scratch, counters,
      separation_counters, path_counters, common_point_counters);
}

bool KnownMotion(RepresentedMotion motion) noexcept {
  return motion == RepresentedMotion::LinearNodalV1 ||
         motion == RepresentedMotion::RigidArc ||
         motion == RepresentedMotion::Nonlinear;
}

bool CanonicalVertexKey(const FacetVertexKey& key,
                        const RepresentedTrianglePathKey& path) noexcept {
  if (key.source_instance_id != path.source_instance_id ||
      key.denominator == 0)
    return false;
  if (key.kind == FacetVertexKind::SourceVertex)
    return key.second == 0 && key.numerator == 0 &&
           key.denominator == 1 && key.level == 0 && key.grid_i == 0 &&
           key.grid_j == 0;
  if (key.kind == FacetVertexKind::SourceEdge)
    return key.first < key.second && key.numerator > 0 &&
           key.numerator < key.denominator &&
           (key.denominator & (key.denominator - 1)) == 0 &&
           key.denominator <= (1u << path.level) &&
           (key.denominator == 1 || (key.numerator & 1u)) &&
           key.level == 0 && key.grid_i == 0 && key.grid_j == 0;
  if (key.kind == FacetVertexKind::ParentInterior) {
    const unsigned n = 1u << path.level;
    return key.first == path.parent_eid && key.second == 0 &&
           key.numerator == 0 && key.denominator == 1 &&
           key.level == path.level && key.grid_i <= n && key.grid_j <= n;
  }
  return false;
}

bool SameTrajectory(const RepresentedVertexPath& a,
                    const RepresentedVertexPath& b) noexcept {
  return Same(a.key, b.key) && SameBits(a.endpoint[0], b.endpoint[0]) &&
         SameBits(a.endpoint[1], b.endpoint[1]);
}

bool CompatiblePath(const RepresentedTrianglePath& a,
                    const RepresentedTrianglePath& b) noexcept {
  if (a.motion != b.motion)
    return false;
  for (const auto& vertex : a.vertices) {
    bool found = false;
    for (const auto& other : b.vertices)
      found = found || SameTrajectory(vertex, other);
    if (!found)
      return false;
  }
  for (const auto& edge : a.edge_keys) {
    bool found = false;
    for (const auto& other : b.edge_keys)
      found = found || Compare(edge, other) == 0;
    if (!found)
      return false;
  }
  return true;
}

RepresentedIntervalStatus ValidatePath(
    const RepresentedTrianglePath& path) noexcept {
  if (!KnownMotion(path.motion) || path.key.level > 2)
    return RepresentedIntervalStatus::InvalidInput;
  for (unsigned i = 0; i < 3; ++i) {
    if (!IsFinite(path.vertices[i].endpoint[0]) ||
        !IsFinite(path.vertices[i].endpoint[1]) ||
        !CanonicalVertexKey(path.vertices[i].key, path.key))
      return RepresentedIntervalStatus::InvalidInput;
    for (unsigned j = 0; j < i; ++j)
      if (Same(path.vertices[i].key, path.vertices[j].key))
        return RepresentedIntervalStatus::InvalidInput;
    const unsigned next = (i + 1) % 3;
    const auto& edge = path.edge_keys[i];
    const bool edge_matches =
        (Same(edge.endpoints[0], path.vertices[i].key) &&
         Same(edge.endpoints[1], path.vertices[next].key)) ||
        (Same(edge.endpoints[1], path.vertices[i].key) &&
         Same(edge.endpoints[0], path.vertices[next].key));
    const bool parent_matches =
        edge.parent_boundary ? edge.parent_eid == 0
                             : edge.parent_eid == path.key.parent_eid;
    if (!edge_matches || Compare(edge.endpoints[0], edge.endpoints[1]) >= 0 ||
        !parent_matches)
      return RepresentedIntervalStatus::InvalidInput;
  }
  return RepresentedIntervalStatus::Ok;
}

RepresentedIntervalStatus QualifyPairInputs(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits, const RepresentedTrianglePath** a,
    const RepresentedTrianglePath** b) noexcept {
  if (!a || !b || !limits.max_work_per_pair || limits.max_depth > 52 ||
      ValidatePath(first) != RepresentedIntervalStatus::Ok ||
      ValidatePath(second) != RepresentedIntervalStatus::Ok || Same(first.key, second.key))
    return RepresentedIntervalStatus::InvalidInput;
  for (const auto& x : first.vertices)
    for (const auto& y : second.vertices)
      if (Same(x.key, y.key) &&
          (first.motion != second.motion || !SameTrajectory(x, y)))
        return RepresentedIntervalStatus::IdentityMismatch;
  const bool reverse = Compare(first.key, second.key) > 0;
  *a = reverse ? &second : &first;
  *b = reverse ? &first : &second;
  return RepresentedIntervalStatus::Ok;
}

const char* Message(RepresentedIntervalStatus status) noexcept {
  switch (status) {
    case RepresentedIntervalStatus::Ok:
      return "OK";
    case RepresentedIntervalStatus::AlreadyInitialized:
      return "already initialized";
    case RepresentedIntervalStatus::NotInitialized:
      return "not initialized";
    case RepresentedIntervalStatus::InvalidInput:
      return "invalid input";
    case RepresentedIntervalStatus::IdentityMismatch:
      return "duplicate path identity";
    case RepresentedIntervalStatus::ResourceLimit:
      return "resource limit";
  }
  return "invalid status";
}

RepresentedIntervalReport FreshReport() noexcept {
  RepresentedIntervalReport result;
  std::fill_n(reinterpret_cast<unsigned char*>(&result), sizeof(result),
              static_cast<unsigned char>(0));
  result.input_path = SIZE_MAX;
  result.input_pair = SIZE_MAX;
  result.message = Message(RepresentedIntervalStatus::Ok);
  return result;
}

RepresentedIntervalReport Failure(RepresentedIntervalStatus status) noexcept {
  RepresentedIntervalReport result = FreshReport();
  result.status = status;
  result.message = Message(status);
  return result;
}

using represented_interval_crossing::BusyRelease;

}  // namespace

struct RepresentedIntervalCrossing::Impl {
  enum class Phase : unsigned {
    Constructing,
    Warm,
    Running,
    Failed,
    Stopping,
  };

  using PairStatus = represented_interval_crossing::PairStatus;

  struct WorkerSlot {
    Impl* owner = nullptr;
    std::size_t index = 0;
    pthread_t thread{};
    void* stack_mapping = nullptr;
    sem_t start{};
    bool start_initialized = false;
    bool started = false;
    bool failed = false;
  };

  // Lexical to one native call/compound operation. No authority is exported
  // or retained in Impl, and the borrowed roster cannot change between slices.
  struct PathRoster {
    const RepresentedTrianglePath* paths;
    std::size_t count;
    bool authenticated = false;
    represented_interval_crossing::PathRosterWork work;
    std::optional<represented_interval_crossing::AuthenticatedScene> device_scene;
    const RepresentedTrianglePair* ordered_pairs = nullptr;
    std::size_t ordered_pair_count = 0, slice_capacity = 0, slice_offset = 0;
    bool prefetch_disjoint = false;
  };
  bool DisjointFromOwned(const void* data, std::size_t bytes) const noexcept;
  RepresentedIntervalReport CertifySlice(
      const RepresentedTrianglePath*, std::size_t,
      const RepresentedTrianglePair*, std::size_t, PathRoster&,
      represented_interval_crossing::DeviceExecution* = nullptr) noexcept;

  explicit Impl(RepresentedIntervalLimits input) : limits(input) {}
  ~Impl() { Shutdown(); }

  RepresentedIntervalLimits limits;
  RepresentedIntervalForecast forecast;
  std::vector<std::uint32_t> path_indices;
  std::vector<CanonicalPair> pairs;
  std::vector<VertexLedgerRow> vertex_ledger;
  std::vector<RepresentedIntervalResult> published;
  std::vector<RepresentedIntervalResult> staging;
  std::unique_ptr<PairStatus[]> pair_status;
  std::unique_ptr<WorkerSlot[]> workers;
  std::unique_ptr<Cell[]> dfs_frames;
  std::unique_ptr<ExactScratch[]> exact_scratch;
  sem_t completed{};
  bool completed_initialized = false;
  std::size_t started_workers = 0;
  std::atomic<std::size_t> next_pair{0};
  std::atomic<bool> stop{false};
  std::atomic<bool> pool_failed{false};
  std::atomic<bool> busy{false};
  std::atomic<Phase> phase{Phase::Constructing};
  const RepresentedTrianglePath* job_paths = nullptr;
  std::size_t job_pair_count = 0;
  bool complete = false;

  bool WorkerStacksDisjoint(const void* input,
                            std::size_t bytes) const noexcept {
    if (!bytes)
      return true;
    if (!input || !workers)
      return false;
    const std::size_t page_bytes = PageBytes();
    if (!page_bytes || kWorkerStackBytes > SIZE_MAX - page_bytes)
      return false;
    const std::size_t mapping_bytes = kWorkerStackBytes + page_bytes;
    for (unsigned i = 0; i < limits.worker_count; ++i)
      if (!workers[i].stack_mapping ||
          !RangeDisjoint(input, bytes, workers[i].stack_mapping,
                         mapping_bytes))
        return false;
    return true;
  }

  static void* WorkerEntry(void* opaque) noexcept {
    auto* worker = static_cast<WorkerSlot*>(opaque);
    if (!worker || !worker->owner)
      return nullptr;
    auto& owner = *worker->owner;
    for (;;) {
      if (!Wait(&worker->start)) {
        worker->failed = true;
        owner.pool_failed.store(true, std::memory_order_release);
        owner.phase.store(Phase::Failed, std::memory_order_release);
        return nullptr;
      }
      if (owner.stop.load(std::memory_order_acquire))
        return nullptr;
      worker->failed = false;
      owner.EvaluateJobs(worker->index);
      if (sem_post(&owner.completed) != 0) {
        worker->failed = true;
        owner.pool_failed.store(true, std::memory_order_release);
        owner.phase.store(Phase::Failed, std::memory_order_release);
        return nullptr;
      }
    }
  }

  void EvaluateJobs(std::size_t worker_index) noexcept {
    Cell* dfs = dfs_frames.get() +
        worker_index * forecast.dfs_frame_capacity;
    ExactScratch* scratch = exact_scratch.get() + worker_index;
    for (;;) {
      const std::size_t pair_index =
          next_pair.fetch_add(1, std::memory_order_relaxed);
      if (pair_index >= job_pair_count)
        return;
      // Device writes finish synchronously before workers start. Every ordinal
      // has exactly one numerical writer; CPU fallback owns only unset rows.
      if (pair_status[pair_index].complete)
        continue;
      // Scheduling affects only worker ownership. Each canonical pair index
      // has one staging/status writer and private DFS/exact scratch; the host
      // folds staging in increasing pair_index order after all workers join.
      const auto& pair = pairs[pair_index];
      const auto result = CertifyPair(
          job_paths[pair.first], job_paths[pair.second], limits, pair.key,
          dfs, forecast.dfs_frame_capacity, scratch);
      StoreResult(result, &staging[pair_index]);
      pair_status[pair_index].complete = true;
    }
  }

  bool RunWorkers(const RepresentedTrianglePath* paths,
                  std::size_t pair_count) noexcept {
    Phase expected = Phase::Warm;
    if (!phase.compare_exchange_strong(
            expected, Phase::Running, std::memory_order_acq_rel))
      return false;
    job_paths = paths;
    job_pair_count = pair_count;
    next_pair.store(0, std::memory_order_relaxed);
    pool_failed.store(false, std::memory_order_release);
    std::size_t posted = 0;
    for (; posted < started_workers; ++posted)
      if (sem_post(&workers[posted].start) != 0)
        break;
    bool failed = posted != started_workers;
    for (std::size_t i = 0; i < posted; ++i)
      if (!Wait(&completed))
        failed = true;
    failed = failed || pool_failed.load(std::memory_order_acquire);
    phase.store(failed ? Phase::Failed : Phase::Warm,
                std::memory_order_release);
    return !failed;
  }

  bool StartWorkers() noexcept {
    const std::size_t page_bytes = PageBytes();
    if (!page_bytes ||
        kWorkerStackBytes < static_cast<std::size_t>(PTHREAD_STACK_MIN) ||
        kWorkerStackBytes % page_bytes ||
        kWorkerStackBytes > SIZE_MAX - page_bytes ||
        sem_init(&completed, 0, 0) != 0)
      return false;
    completed_initialized = true;
    const std::size_t mapping_bytes = kWorkerStackBytes + page_bytes;
    for (unsigned i = 0; i < limits.worker_count; ++i) {
      auto& worker = workers[i];
      worker.owner = this;
      worker.index = i;
      if (sem_init(&worker.start, 0, 0) != 0)
        return false;
      worker.start_initialized = true;
#ifdef MAP_STACK
      constexpr int stack_flag = MAP_STACK;
#else
      constexpr int stack_flag = 0;
#endif
      worker.stack_mapping = mmap(
          nullptr, mapping_bytes, PROT_READ | PROT_WRITE,
          MAP_PRIVATE | MAP_ANONYMOUS | stack_flag, -1, 0);
      if (worker.stack_mapping == MAP_FAILED) {
        worker.stack_mapping = nullptr;
        return false;
      }
      auto* stack = static_cast<unsigned char*>(worker.stack_mapping) +
          page_bytes;
      std::memset(stack, 0, kWorkerStackBytes);
      if (mprotect(worker.stack_mapping, page_bytes, PROT_NONE) != 0)
        return false;
      pthread_attr_t attributes;
      if (pthread_attr_init(&attributes) != 0)
        return false;
      const int guard_status =
          pthread_attr_setguardsize(&attributes, 0);
      const int stack_status = guard_status
          ? guard_status
          : pthread_attr_setstack(
                &attributes, stack, kWorkerStackBytes);
      const int create_status = stack_status
          ? stack_status
          : pthread_create(
                &worker.thread, &attributes, &Impl::WorkerEntry, &worker);
      pthread_attr_destroy(&attributes);
      if (create_status != 0)
        return false;
      worker.started = true;
      ++started_workers;
    }
    phase.store(Phase::Warm, std::memory_order_release);
    // Exercise every thread, stack and semaphore before publication.
    return RunWorkers(nullptr, 0);
  }

  void Shutdown() noexcept {
    phase.store(Phase::Stopping, std::memory_order_release);
    stop.store(true, std::memory_order_release);
    if (workers) {
      for (std::size_t i = 0; i < started_workers; ++i)
        if (workers[i].started)
          sem_post(&workers[i].start);
      for (std::size_t i = 0; i < started_workers; ++i) {
        if (workers[i].started)
          pthread_join(workers[i].thread, nullptr);
        workers[i].started = false;
      }
      const std::size_t page_bytes = PageBytes();
      const std::size_t mapping_bytes =
          page_bytes && kWorkerStackBytes <= SIZE_MAX - page_bytes
              ? kWorkerStackBytes + page_bytes
              : 0;
      for (unsigned i = 0; i < limits.worker_count; ++i) {
        if (workers[i].start_initialized)
          sem_destroy(&workers[i].start);
        workers[i].start_initialized = false;
        if (workers[i].stack_mapping && mapping_bytes)
          munmap(workers[i].stack_mapping, mapping_bytes);
        workers[i].stack_mapping = nullptr;
      }
    }
    started_workers = 0;
    if (completed_initialized)
      sem_destroy(&completed);
    completed_initialized = false;
  }
};

RepresentedIntervalCrossing::RepresentedIntervalCrossing() noexcept = default;
RepresentedIntervalCrossing::~RepresentedIntervalCrossing() = default;
RepresentedIntervalCrossing::RepresentedIntervalCrossing(
    RepresentedIntervalCrossing&&) noexcept = default;
RepresentedIntervalCrossing& RepresentedIntervalCrossing::operator=(
    RepresentedIntervalCrossing&&) noexcept = default;

RepresentedIntervalPreflight RepresentedIntervalCrossing::Preflight(
    RepresentedIntervalLimits limits) noexcept {
  RepresentedIntervalPreflight result{};
  result.report = FreshReport();
  if (!limits.max_paths || !limits.max_input_pairs || !limits.max_results ||
      !limits.max_work_per_pair || !limits.max_total_work ||
      limits.max_depth > 52 || limits.max_paths > UINT32_MAX ||
      !limits.worker_count ||
      limits.worker_count > RepresentedIntervalMaximumWorkerCount) {
    result.report = Failure(RepresentedIntervalStatus::InvalidInput);
    return result;
  }
  result.forecast.path_index_capacity = limits.max_paths;
  result.forecast.pair_capacity = limits.max_input_pairs;
  result.forecast.result_capacity = limits.max_results;
  result.forecast.pair_status_capacity = limits.max_input_pairs;
  result.forecast.worker_count = limits.worker_count;
  result.forecast.dfs_frame_capacity =
      static_cast<std::size_t>(limits.max_depth) + 1;
  std::size_t all_dfs_frames = 0;
  const std::size_t page_bytes = PageBytes();
  if (!MultiplySize(limits.max_paths, 3,
                    &result.forecast.vertex_ledger_capacity) ||
      !MultiplySize(limits.max_paths, sizeof(std::uint32_t),
                    &result.forecast.path_index_bytes) ||
      !MultiplySize(limits.max_input_pairs, sizeof(CanonicalPair),
                    &result.forecast.pair_bytes) ||
      !MultiplySize(limits.max_results,
                    2 * sizeof(RepresentedIntervalResult),
                    &result.forecast.result_bytes) ||
      !MultiplySize(result.forecast.vertex_ledger_capacity,
                    sizeof(VertexLedgerRow),
                    &result.forecast.vertex_ledger_bytes) ||
      !MultiplySize(result.forecast.dfs_frame_capacity,
                    limits.worker_count, &all_dfs_frames) ||
      !MultiplySize(all_dfs_frames, sizeof(Cell),
                    &result.forecast.dfs_frame_bytes) ||
      !MultiplySize(limits.worker_count, sizeof(ExactScratch),
                    &result.forecast.exact_scratch_bytes) ||
      !MultiplySize(limits.max_input_pairs, sizeof(Impl::PairStatus),
                    &result.forecast.pair_status_bytes) ||
      !MultiplySize(limits.worker_count, sizeof(Impl::WorkerSlot),
                    &result.forecast.worker_metadata_bytes) ||
      !page_bytes ||
      kWorkerStackBytes <
          static_cast<std::size_t>(PTHREAD_STACK_MIN) ||
      kWorkerStackBytes % page_bytes ||
      kWorkerStackBytes > SIZE_MAX - page_bytes ||
      !MultiplySize(limits.worker_count,
                    kWorkerStackBytes + page_bytes,
                    &result.forecast.worker_stack_bytes)) {
    result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
    return result;
  }
  std::size_t total = sizeof(Impl);
  const std::size_t regions[]{
      result.forecast.path_index_bytes, result.forecast.pair_bytes,
      result.forecast.result_bytes, result.forecast.vertex_ledger_bytes,
      result.forecast.dfs_frame_bytes, result.forecast.exact_scratch_bytes,
      result.forecast.pair_status_bytes,
      result.forecast.worker_metadata_bytes,
      result.forecast.worker_stack_bytes};
  for (const auto bytes : regions) {
    if (!AddSize(total, bytes, &total)) {
      result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
      return result;
    }
  }
  result.forecast.owned_host_bytes = total;
  if (total > limits.max_host_bytes) {
    result.report = Failure(RepresentedIntervalStatus::ResourceLimit);
    return result;
  }
  result.forecast.startup_host_bytes = total;
  return result;
}

RepresentedIntervalReport RepresentedIntervalCrossing::Initialize(
    RepresentedIntervalLimits limits) noexcept {
  if (impl_)
    return Failure(RepresentedIntervalStatus::AlreadyInitialized);
  const auto plan = Preflight(limits);
  if (plan.report.status != RepresentedIntervalStatus::Ok)
    return plan.report;
  try {
    auto next = std::make_unique<Impl>(limits);
    next->forecast = plan.forecast;
    next->path_indices.reserve(limits.max_paths);
    next->pairs.reserve(limits.max_input_pairs);
    next->vertex_ledger.reserve(plan.forecast.vertex_ledger_capacity);
    next->published.reserve(limits.max_results);
    next->staging.reserve(limits.max_results);
    const std::size_t all_dfs_frames =
        plan.forecast.dfs_frame_bytes / sizeof(Cell);
    next->pair_status.reset(new (std::nothrow)
        Impl::PairStatus[limits.max_input_pairs]);
    next->workers.reset(new (std::nothrow)
        Impl::WorkerSlot[limits.worker_count]);
    next->dfs_frames.reset(new (std::nothrow) Cell[all_dfs_frames]);
    next->exact_scratch.reset(new (std::nothrow)
        ExactScratch[limits.worker_count]);
    if (next->path_indices.capacity() != limits.max_paths ||
        next->pairs.capacity() != limits.max_input_pairs ||
        next->vertex_ledger.capacity() !=
            plan.forecast.vertex_ledger_capacity ||
        next->published.capacity() != limits.max_results ||
        next->staging.capacity() != limits.max_results ||
        !next->pair_status || !next->workers || !next->dfs_frames ||
        !next->exact_scratch)
      return Failure(RepresentedIntervalStatus::ResourceLimit);
    if (!next->StartWorkers())
      return Failure(RepresentedIntervalStatus::ResourceLimit);
    impl_ = std::move(next);
  } catch (...) {
    return Failure(RepresentedIntervalStatus::ResourceLimit);
  }
  return FreshReport();
}

RepresentedIntervalReport RepresentedIntervalCrossing::Certify(
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count) noexcept {
  return CertifyUsing(paths, path_count, pairs, pair_count, nullptr);
}

RepresentedIntervalReport RepresentedIntervalCrossing::CertifyUsing(
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    represented_interval_crossing::DeviceExecution* device) noexcept {
  if (!impl_)
    return Failure(RepresentedIntervalStatus::NotInitialized);
  auto& storage = *impl_;
  RepresentedIntervalReport report = FreshReport();
  report.input_paths = path_count;
  report.input_pairs = pair_count;
  bool expected_idle = false;
  if (!storage.busy.compare_exchange_strong(
          expected_idle, true, std::memory_order_acq_rel)) {
    report.status = RepresentedIntervalStatus::InvalidInput;
    report.message =
        "represented interval crossing does not accept concurrent calls";
    return report;
  }
  BusyRelease busy_release{&storage.busy};
  Impl::PathRoster roster{paths, path_count};
  return storage.CertifySlice(paths, path_count, pairs, pair_count, roster, device);
}

RepresentedIntervalReport represented_interval_crossing::DeviceAccess::Certify(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    DeviceExecution& device) noexcept {
  return crossing.CertifyUsing(paths, path_count, pairs, pair_count, &device);
}

represented_interval_crossing::BatchReport
represented_interval_crossing::DeviceAccess::CertifyBatch(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity, RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity, DeviceExecution& device) noexcept {
  return BatchAccess::CertifyUsing(crossing, paths, path_count, pairs, pair_count,
      batch_pair_capacity, scratch, scratch_capacity, &device);
}

bool RepresentedIntervalCrossing::Impl::DisjointFromOwned(
    const void* data, std::size_t bytes) const noexcept {
  const auto& storage = *this;
  if (!RangeDisjoint(data, bytes, &storage, sizeof(storage)) ||
      !RangeDisjoint(data, bytes, storage.exact_scratch.get(),
                     storage.forecast.exact_scratch_bytes) ||
      !RangeDisjoint(data, bytes, storage.dfs_frames.get(),
                     storage.forecast.dfs_frame_bytes) ||
      !RangeDisjoint(data, bytes, storage.pair_status.get(),
                     storage.forecast.pair_status_bytes) ||
      !RangeDisjoint(data, bytes, storage.workers.get(),
                     storage.forecast.worker_metadata_bytes) ||
      !storage.WorkerStacksDisjoint(data, bytes))
    return false;
  struct Range {
    const void* data;
    std::size_t count;
    std::size_t element;
  };
  const Range ranges[]{
      {storage.path_indices.data(), storage.path_indices.capacity(),
       sizeof(std::uint32_t)},
      {storage.pairs.data(), storage.pairs.capacity(),
       sizeof(CanonicalPair)},
      {storage.vertex_ledger.data(), storage.vertex_ledger.capacity(),
       sizeof(VertexLedgerRow)},
      {storage.published.data(), storage.published.capacity(),
       sizeof(RepresentedIntervalResult)},
      {storage.staging.data(), storage.staging.capacity(),
       sizeof(RepresentedIntervalResult)}};
  for (const auto& range : ranges) {
    std::size_t owned_bytes = 0;
    if (!MultiplySize(range.count, range.element, &owned_bytes) ||
        !RangeDisjoint(data, bytes, range.data, owned_bytes))
      return false;
  }
  return true;
}

RepresentedIntervalReport RepresentedIntervalCrossing::Impl::CertifySlice(
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    PathRoster& roster, represented_interval_crossing::DeviceExecution* device) noexcept {
  auto& storage = *this;
  RepresentedIntervalReport report = FreshReport();
  report.input_paths = path_count;
  report.input_pairs = pair_count;
  if (roster.paths != paths || roster.count != path_count ||
      (roster.slice_capacity &&
       (roster.slice_offset > roster.ordered_pair_count ||
        pair_count > roster.ordered_pair_count - roster.slice_offset ||
        pair_count > roster.slice_capacity ||
        pairs != (roster.ordered_pairs ? roster.ordered_pairs + roster.slice_offset : nullptr)))) {
    report.status = RepresentedIntervalStatus::InvalidInput;
    report.message = "native lexical path roster changed during batch traversal";
    return report;
  }
  if (storage.phase.load(std::memory_order_acquire) !=
      Impl::Phase::Warm) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message =
        "represented interval crossing worker pool is unavailable";
    return report;
  }
  if ((path_count && !paths) || (pair_count && !pairs) ||
      path_count > storage.limits.max_paths ||
      pair_count > storage.limits.max_input_pairs) {
    report.status =
        path_count > storage.limits.max_paths ||
                pair_count > storage.limits.max_input_pairs
            ? RepresentedIntervalStatus::ResourceLimit
            : RepresentedIntervalStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }
  std::size_t path_bytes = 0, pair_bytes = 0;
  if (!MultiplySize(path_count, sizeof(*paths), &path_bytes) ||
      !MultiplySize(pair_count, sizeof(*pairs), &pair_bytes) ||
      !RangeDisjoint(paths, path_bytes, pairs, pair_bytes) ||
      !storage.DisjointFromOwned(paths, path_bytes) ||
      !storage.DisjointFromOwned(pairs, pair_bytes) ||
      (device && (!device->Disjoint(paths, path_bytes) ||
                  !device->Disjoint(pairs, pair_bytes)))) {
    report.status = RepresentedIntervalStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }

  if (!roster.authenticated) {
    ++roster.work.authentications;
    storage.path_indices.clear();
    storage.vertex_ledger.clear();
    for (std::size_t i = 0; i < path_count; ++i) {
      ++roster.work.path_rows;
      const auto status = ValidatePath(paths[i]);
      if (status != RepresentedIntervalStatus::Ok) {
        report.status = status;
        report.input_path = i;
        report.message = Message(status);
        return report;
      }
      storage.path_indices.push_back(static_cast<std::uint32_t>(i));
      for (const auto& vertex : paths[i].vertices) {
        storage.vertex_ledger.push_back(
            {vertex.key, {vertex.endpoint[0], vertex.endpoint[1]},
             paths[i].motion, i});
        ++roster.work.vertex_rows;
      }
    }
    ++roster.work.path_sorts;
    std::sort(storage.path_indices.begin(), storage.path_indices.end(),
              [&](std::uint32_t a, std::uint32_t b) {
                return Compare(paths[a].key, paths[b].key) < 0;
              });
    for (std::size_t i = 1; i < storage.path_indices.size(); ++i) {
      const auto previous = storage.path_indices[i - 1];
      const auto current = storage.path_indices[i];
      if (Same(paths[previous].key, paths[current].key) &&
          !CompatiblePath(paths[previous], paths[current])) {
        report.status = RepresentedIntervalStatus::IdentityMismatch;
        report.input_path = current;
        report.message = Message(report.status);
        return report;
      }
    }
    ++roster.work.vertex_sorts;
    std::sort(storage.vertex_ledger.begin(), storage.vertex_ledger.end(),
              VertexLedgerLess);
    for (std::size_t i = 1; i < storage.vertex_ledger.size(); ++i) {
      const auto& previous = storage.vertex_ledger[i - 1];
      const auto& current = storage.vertex_ledger[i];
      if (Compare(previous.key, current.key) == 0 &&
          (previous.motion != current.motion ||
           !SameBits(previous.endpoint[0], current.endpoint[0]) ||
           !SameBits(previous.endpoint[1], current.endpoint[1]))) {
        report.status = RepresentedIntervalStatus::IdentityMismatch;
        report.input_path = current.input_path;
        report.message = "inconsistent vertex trajectory identity";
        return report;
      }
    }
    roster.authenticated = true;
  }

  storage.pairs.clear();
  for (std::size_t i = 0; i < pair_count; ++i) {
    if (pairs[i].first >= path_count || pairs[i].second >= path_count ||
        pairs[i].first == pairs[i].second ||
        Same(paths[pairs[i].first].key, paths[pairs[i].second].key)) {
      report.status = RepresentedIntervalStatus::InvalidInput;
      report.input_pair = i;
      report.message = Message(report.status);
      return report;
    }
    const auto pair = represented_interval_crossing::CanonicalizePair(
        paths, pairs[i], i, [](const auto& a, const auto& b) { return Compare(a, b); });
    storage.pairs.push_back(pair);
  }
  std::sort(storage.pairs.begin(), storage.pairs.end(), PairLess);
  storage.pairs.erase(
      std::unique(storage.pairs.begin(), storage.pairs.end(), SamePair),
      storage.pairs.end());
  report.unique_pairs = storage.pairs.size();
  if (storage.pairs.size() > storage.limits.max_results) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  storage.staging.clear();
  try {
    storage.staging.resize(storage.pairs.size());
  } catch (...) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message = Message(report.status);
    storage.staging.clear();
    return report;
  }
  for (std::size_t pair = 0; pair < storage.pairs.size(); ++pair)
    storage.pair_status[pair].complete = false;
  if (device) {
    if (!roster.device_scene)
      roster.device_scene.emplace(
          represented_interval_crossing::AuthenticatedScene::ConstructionKey{},
          device, paths, path_count,
          roster.prefetch_disjoint ? roster.ordered_pairs : nullptr,
          roster.prefetch_disjoint ? roster.ordered_pair_count : 0,
          roster.prefetch_disjoint ? roster.slice_capacity : 0);
    auto& scene = *roster.device_scene;
    const auto cohort_capacity = device->NumericCohortCapacity();
    if (cohort_capacity && scene.slice_capacity_ && pair_count) {
      if (cohort_capacity < pair_count) {
        report.status = RepresentedIntervalStatus::ResourceLimit;
        report.message = "Native CUDA numerical cohort cannot hold this publication slice";
        storage.staging.clear();
        return report;
      }
      if (!scene.cohort_ || roster.slice_offset >= scene.cohort_->begin_ + scene.cohort_->count_) {
        const auto remaining = scene.ordered_pair_count_ - roster.slice_offset;
        const auto window = remaining <= cohort_capacity ? remaining :
            (cohort_capacity / scene.slice_capacity_) * scene.slice_capacity_;
        // Align nonfinal windows with the original publication slices. No row
        // is prefetched twice and no numerical budget is raised.
        scene.cohort_.emplace(
            represented_interval_crossing::AuthenticatedNumericCohort::ConstructionKey{},
            roster.slice_offset, window);
      }
    }
    const represented_interval_crossing::AuthenticatedWork work(*roster.device_scene,
        storage.pairs.data(), storage.pairs.size(), storage.limits,
        storage.staging.data(), storage.pair_status.get(), roster.slice_offset);
    const auto execution = device->Execute(work);
    if (execution.status != RepresentedIntervalStatus::Ok) {
      report.status = execution.status;
      report.input_pair = execution.input_pair;
      report.message = execution.message;
      storage.staging.clear();
      return report;
    }
  }
  if (!storage.RunWorkers(paths, storage.pairs.size())) {
    report.status = RepresentedIntervalStatus::ResourceLimit;
    report.message =
        "represented interval crossing persistent worker execution failed";
    storage.staging.clear();
    return report;
  }
  for (std::size_t pair_index = 0;
       pair_index < storage.pairs.size(); ++pair_index) {
    const auto& pair = storage.pairs[pair_index];
    if (!storage.pair_status[pair_index].complete) {
      report.status = RepresentedIntervalStatus::ResourceLimit;
      report.input_pair = pair.input_pair;
      report.message =
          "represented interval crossing worker result is incomplete";
      storage.staging.clear();
      return report;
    }
    const auto& result = storage.staging[pair_index];
    if (result.work > storage.limits.max_total_work - report.work) {
      report.status = RepresentedIntervalStatus::ResourceLimit;
      report.input_pair = pair.input_pair;
      report.total_work_limit = storage.limits.max_total_work;
      report.rejected_pair_work = result.work;
      report.message = Message(report.status);
      storage.staging.clear();
      return report;
    }
    report.work += result.work;
    if (result.classification ==
        RepresentedIntervalClassification::CertifiedSeparated)
      ++report.certified_separated;
    else if (result.classification ==
             RepresentedIntervalClassification::CertifiedCrossingContact)
      ++report.certified_crossing_contact;
    else
      ++report.unresolved;
  }
  storage.published.swap(storage.staging);
  storage.staging.clear();
  storage.complete = true;
  return report;
}

represented_interval_crossing::BatchReport
represented_interval_crossing::BatchAccess::Certify(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity,
    RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity) noexcept {
  return CertifyUsing(crossing, paths, path_count, pairs, pair_count,
      batch_pair_capacity, scratch, scratch_capacity, nullptr);
}

represented_interval_crossing::BatchReport
represented_interval_crossing::BatchAccess::CertifyUsing(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity, RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity, DeviceExecution* device) noexcept {
  const auto compare = [](const auto& first, const auto& second) {
    return Compare(first, second);
  };
  auto report = detail::ValidateInput(
      crossing.initialized(), crossing.forecast(), crossing.results(),
      paths, path_count, pairs, pair_count, batch_pair_capacity,
      scratch, scratch_capacity, compare, RangeDisjoint,
      [&](std::size_t path_bytes, std::size_t pair_bytes, std::size_t scratch_bytes) {
        return !device || (device->Disjoint(paths, path_bytes) &&
            device->Disjoint(pairs, pair_bytes) && device->Disjoint(scratch, scratch_bytes));
      });
  if (report.status != RepresentedIntervalStatus::Ok) return report;

  auto& storage = *crossing.impl_;
  // Keep the same native first-slice report on overlapping/unavailable calls.
  const auto first_count = std::min(batch_pair_capacity, pair_count);
  auto admission = FreshReport();
  admission.input_paths = path_count;
  admission.input_pairs = first_count;
  bool expected_idle = false;
  if (!storage.busy.compare_exchange_strong(
          expected_idle, true, std::memory_order_acq_rel)) {
    admission.status = RepresentedIntervalStatus::InvalidInput;
    admission.message =
        "represented interval crossing does not accept concurrent calls";
    report.native_report = admission;
    report.native_called = true;
    return detail::Failure(report, admission.status, admission.message);
  }
  BusyRelease busy_release{&storage.busy};
  if (storage.phase.load(std::memory_order_acquire) !=
      RepresentedIntervalCrossing::Impl::Phase::Warm) {
    admission.status = RepresentedIntervalStatus::ResourceLimit;
    admission.message =
        "represented interval crossing worker pool is unavailable";
    report.native_report = admission;
    report.native_called = true;
    return detail::Failure(report, admission.status, admission.message);
  }
  // Caller scratch is private, not an expired/native result buffer. Unlike
  // the old adapter's current-view check, authenticate every retained region.
  std::size_t scratch_bytes = 0;
  if (!detail::Bytes(scratch, scratch_capacity, &scratch_bytes) ||
      !RangeDisjoint(scratch, scratch_bytes, &crossing, sizeof(crossing)) ||
      !storage.DisjointFromOwned(scratch, scratch_bytes))
    return detail::Failure(report, RepresentedIntervalStatus::InvalidInput,
        "Crossing batch scratch aliases native owned storage");

  RepresentedIntervalCrossing::Impl::PathRoster roster{paths, path_count};
  // detail::ValidateInput authenticated this complete immutable canonical pair
  // roster before any slice. Its bounds stay lexical to this compound call.
  roster.ordered_pairs = pairs;
  roster.ordered_pair_count = pair_count;
  roster.slice_capacity = batch_pair_capacity;
  if (device && device->NumericCohortCapacity()) {
    // Ordinary validation keeps its original per-slice alias/error boundary.
    // A wider numerical borrow needs the stronger whole-roster proof. If that
    // proof is unavailable, retain per-slice GPU execution and its exact error
    // order; this is never a retry after a CUDA or arithmetic failure.
    roster.prefetch_disjoint = detail::NumericCohortRanges(paths, path_count, pairs, pair_count,
        [&](const void* data, std::size_t bytes) { return storage.DisjointFromOwned(data, bytes); },
        RangeDisjoint);
  }
  report = detail::Execute(
      paths, pairs, pair_count, batch_pair_capacity, scratch,
      [&](const RepresentedTrianglePair* slice, std::size_t count) {
        roster.slice_offset = slice && pairs ? static_cast<std::size_t>(slice - pairs) : 0;
        return storage.CertifySlice(paths, path_count, slice, count, roster, device);
      },
      [&]() {
        // Public results() deliberately hides publication while busy. This
        // native-owned view is consumed synchronously after workers join.
        return RepresentedIntervalResultView{
            storage.published.data(), storage.published.size(),
            storage.complete};
      }, compare);
  report.path_roster_work = roster.work;
  return report;
}

RepresentedIntervalForecast RepresentedIntervalCrossing::forecast() const
    noexcept {
  return impl_ ? impl_->forecast : RepresentedIntervalForecast{};
}

RepresentedIntervalResultView RepresentedIntervalCrossing::results() const
    noexcept {
  if (!impl_ || impl_->busy.load(std::memory_order_acquire))
    return {};
  return {impl_->published.data(), impl_->published.size(), impl_->complete};
}

represented_interval_crossing::NormalReuseComparison
represented_interval_crossing::CompareNormalReuse(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits) noexcept {
  NormalReuseComparison result;
  result.worker_exact_scratch_bytes = sizeof(ExactScratch);
  result.normal_storage_bytes = 6 * sizeof(ExactVec3);
  const RepresentedTrianglePath* first_canonical = nullptr;
  const RepresentedTrianglePath* second_canonical = nullptr;
  result.status = QualifyPairInputs(first, second, limits, &first_canonical, &second_canonical);
  if (result.status != RepresentedIntervalStatus::Ok) return result;
  const auto& a = *first_canonical;
  const auto& b = *second_canonical;
  const RepresentedIntervalPairKey key{{a.key, b.key}};
  Cell dfs[53];
  const auto dfs_capacity = static_cast<std::size_t>(limits.max_depth) + 1;
  ExactScratch scratch;
  const auto recomputed = CertifyPair<NormalReuse::Recompute, SeparationProof::RelativeFaces, ExactPathReuse::Original, CommonPointReuse::Original, NativeStorage::Wide>(
      a, b, limits, key, dfs, dfs_capacity, &scratch, &result.recomputed.counters);
  const auto memoized = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces, ExactPathReuse::Original, CommonPointReuse::Original, NativeStorage::Wide>(
      a, b, limits, key, dfs, dfs_capacity, &scratch, &result.memoized.counters);
  // Compare the exact existing native publication representation, including
  // its field-wise initialization, without changing the publication format.
  StoreResult(recomputed, &result.recomputed.result);
  StoreResult(memoized, &result.memoized.result);
  result.status = RepresentedIntervalStatus::Ok;
  return result;
}

represented_interval_crossing::RelativeSeparationComparison
represented_interval_crossing::CompareRelativeSeparation(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits) noexcept {
  RelativeSeparationComparison result;
  const RepresentedTrianglePath* a = nullptr;
  const RepresentedTrianglePath* b = nullptr;
  result.status = QualifyPairInputs(first, second, limits, &a, &b);
  if (result.status != RepresentedIntervalStatus::Ok) return result;
  result.domain = ProjectionDomain::FromPaths(*a, *b, limits.max_depth).report();
  const RepresentedIntervalPairKey key{{a->key, b->key}};
  Cell dfs[53]; ExactScratch scratch;
  const auto capacity = static_cast<std::size_t>(limits.max_depth) + 1;
  const auto legacy = CertifyPair<NormalReuse::Memoize, SeparationProof::LegacyAabb, ExactPathReuse::Original, CommonPointReuse::Original, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch);
  const auto current = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces, ExactPathReuse::Original, CommonPointReuse::Original, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch, nullptr, &result.counters);
  StoreResult(legacy, &result.legacy);
  StoreResult(current, &result.current);
  return result;
}

represented_interval_crossing::ExactPathReuseComparison
represented_interval_crossing::CompareExactPathReuse(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits) noexcept {
  ExactPathReuseComparison result;
  const RepresentedTrianglePath* a = nullptr;
  const RepresentedTrianglePath* b = nullptr;
  result.status = QualifyPairInputs(first, second, limits, &a, &b);
  if (result.status != RepresentedIntervalStatus::Ok) return result;
  result.domain = ProjectionDomain::FromPaths(*a, *b, limits.max_depth).report();
  const RepresentedIntervalPairKey key{{a->key, b->key}};
  Cell dfs[53]; ExactScratch scratch;
  const auto capacity = static_cast<std::size_t>(limits.max_depth) + 1;
  const auto original = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces, ExactPathReuse::Original, CommonPointReuse::Original, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch, &result.original.exact, nullptr, &result.original.reused);
  const auto current = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces, ExactPathReuse::Optimized, CommonPointReuse::Original, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch, &result.current.exact, nullptr, &result.current.reused);
  StoreResult(original, &result.original.result);
  StoreResult(current, &result.current.result);
  return result;
}

represented_interval_crossing::CommonPointReuseComparison
represented_interval_crossing::CompareCommonPointReuse(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits) noexcept {
  CommonPointReuseComparison result;
  const RepresentedTrianglePath* a = nullptr;
  const RepresentedTrianglePath* b = nullptr;
  result.status = QualifyPairInputs(first, second, limits, &a, &b);
  if (result.status != RepresentedIntervalStatus::Ok) return result;
  result.domain = ProjectionDomain::FromPaths(*a, *b, limits.max_depth).report();
  const RepresentedIntervalPairKey key{{a->key, b->key}};
  Cell dfs[53]; ExactScratch scratch;
  const auto capacity = static_cast<std::size_t>(limits.max_depth) + 1;
  const auto original = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces,
      ExactPathReuse::Optimized, CommonPointReuse::Original, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch, nullptr, nullptr, nullptr,
      &result.original.counters);
  const auto current = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces,
      ExactPathReuse::Optimized, CommonPointReuse::Optimized, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch, nullptr, nullptr, nullptr,
      &result.current.counters);
  StoreResult(original, &result.original.result);
  StoreResult(current, &result.current.result);
  return result;
}

represented_interval_crossing::NativeStorageComparison
represented_interval_crossing::CompareNativeStorage(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits) noexcept {
  NativeStorageComparison result;
  result.wide_scratch_bytes = sizeof(ExactScratch);
  result.narrow_scratch_bytes = sizeof(NarrowKernel::ExactScratch);
  const RepresentedTrianglePath* a = nullptr;
  const RepresentedTrianglePath* b = nullptr;
  result.status = QualifyPairInputs(first, second, limits, &a, &b);
  if (result.status != RepresentedIntervalStatus::Ok) return result;
  result.domain = NativeStorageDomain::FromPaths(*a, *b, limits.max_depth).report();
  const RepresentedIntervalPairKey key{{a->key, b->key}};
  Cell dfs[53]; ExactScratch scratch;
  const auto capacity = static_cast<std::size_t>(limits.max_depth) + 1;
  const auto original = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces,
      ExactPathReuse::Optimized, CommonPointReuse::Optimized, NativeStorage::Wide>(
      *a, *b, limits, key, dfs, capacity, &scratch, nullptr, nullptr, nullptr, nullptr,
      &result.original.counters);
  const auto current = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces,
      ExactPathReuse::Optimized, CommonPointReuse::Optimized, NativeStorage::Adaptive>(
      *a, *b, limits, key, dfs, capacity, &scratch, nullptr, nullptr, nullptr, nullptr,
      &result.current.counters);
  StoreResult(original, &result.original.result);
  StoreResult(current, &result.current.result);
  return result;
}

represented_interval_crossing::FixedPolicyComparison
represented_interval_crossing::CompareFixedIntegerPolicy(
    const RepresentedTrianglePath& first, const RepresentedTrianglePath& second,
    RepresentedIntervalLimits limits) noexcept {
  FixedPolicyComparison result;
  using FixedKernel = native::CellKernel<512, native::FixedIntegerPolicy<512>>;
  result.fixed_scratch_bytes = sizeof(FixedKernel::ExactScratch);
  static_assert(sizeof(FixedKernel::ExactScratch) <= 8192);
  const RepresentedTrianglePath* a = nullptr;
  const RepresentedTrianglePath* b = nullptr;
  result.status = QualifyPairInputs(first, second, limits, &a, &b);
  if (result.status != RepresentedIntervalStatus::Ok) return result;
  result.domain = NativeStorageDomain::FromPaths(*a, *b, limits.max_depth).report();
  const RepresentedIntervalPairKey key{{a->key, b->key}};
  Cell dfs[53]; ExactScratch wide_scratch;
  const auto capacity = static_cast<std::size_t>(limits.max_depth) + 1;
  const auto original = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces,
      ExactPathReuse::Optimized, CommonPointReuse::Optimized, NativeStorage::Wide>(
          *a, *b, limits, key, dfs, capacity, &wide_scratch);
  StoreResult(original, &result.original);
  if (result.domain.eligible) {
    native::ArithmeticContext context;
    FixedKernel kernel(context);
    FixedKernel::ExactScratch scratch;
    const auto current = kernel.CertifyPair(*a, *b, limits, key, dfs, capacity, &scratch);
    result.fixed_executed = true;
    result.arithmetic_failed = !context.valid();
    StoreResult(current, &result.current);
  } else {
    // Deliberately execute the wide route independently; never retry an
    // already-executed fixed result or reuse the comparison's reference value.
    const auto current = CertifyPair<NormalReuse::Memoize, SeparationProof::RelativeFaces,
        ExactPathReuse::Optimized, CommonPointReuse::Optimized, NativeStorage::Wide>(
            *a, *b, limits, key, dfs, capacity, &wide_scratch);
    StoreResult(current, &result.current);
  }
  return result;
}

}  // namespace tlfea::contact
