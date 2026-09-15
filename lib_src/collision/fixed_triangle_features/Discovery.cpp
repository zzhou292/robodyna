// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../FixedTriangleFeatureDiscovery.h"

#include "Geometry.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>

namespace tlfea::contact {
namespace ft = fixed_triangle_features;
namespace {

constexpr std::size_t kWorkerStackBytes = 1u << 20;

bool CheckedBytes(std::size_t count, std::size_t width,
                  std::size_t* total) noexcept {
  if (count && width > (std::numeric_limits<std::size_t>::max() - *total) /
                           count)
    return false;
  *total += count * width;
  return true;
}

bool CheckedProduct(std::size_t count, std::size_t width,
                    std::size_t* result) noexcept {
  if (count && width > std::numeric_limits<std::size_t>::max() / count)
    return false;
  *result = count * width;
  return true;
}

unsigned Popcount15(std::uint16_t value) noexcept {
  unsigned result = 0;
  for (unsigned slot = 0; slot < 15; ++slot)
    result += (value >> slot) & 1u;
  return result;
}

bool Disjoint(const void* a, std::size_t a_bytes, const void* b,
              std::size_t b_bytes) noexcept {
  if (!a || !b)
    return false;
  const auto x = reinterpret_cast<std::uintptr_t>(a);
  const auto y = reinterpret_cast<std::uintptr_t>(b);
  return a_bytes <= UINTPTR_MAX - x && b_bytes <= UINTPTR_MAX - y &&
         (x + a_bytes <= y || y + b_bytes <= x);
}

bool Same(Vec3 a, Vec3 b) noexcept {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

struct TriangleLedgerEntry {
  CurrentFixedTriangle value;
};

struct VertexLedgerEntry {
  FacetVertexKey key;
  Vec3 value;
};

struct EdgeLedgerEntry {
  FacetEdgeKey key;
  Vec3 endpoints[2];
};

struct PairEvaluationStage {
  FixedTriangleIntersection intersection;
  ft::PairFeatureResult feature;
  FixedTriangleFeatureTaskMask mask;
  std::size_t raw_feature_offset = 0;
  std::size_t expected_feature_count = 0;
  FixedTriangleDiscoveryStatus intersection_status =
      FixedTriangleDiscoveryStatus::Ok;
  FixedTriangleDiscoveryStatus feature_status =
      FixedTriangleDiscoveryStatus::Ok;
  bool intersects = false;
};

bool TriangleLedgerLess(const TriangleLedgerEntry& a,
                        const TriangleLedgerEntry& b) noexcept {
  return ft::Compare(a.value.key, b.value.key) < 0;
}

bool VertexLedgerLess(const VertexLedgerEntry& a,
                      const VertexLedgerEntry& b) noexcept {
  return ft::Compare(a.key, b.key) < 0;
}

bool EdgeLedgerLess(const EdgeLedgerEntry& a,
                    const EdgeLedgerEntry& b) noexcept {
  return ft::Compare(a.key, b.key) < 0;
}

bool SameCandidateValue(const FixedTriangleFeatureCandidate& a,
                        const FixedTriangleFeatureCandidate& b) noexcept {
  if (!ft::SameFeatureKey(a, b) || a.distance_m != b.distance_m ||
      a.representation_error_m != b.representation_error_m)
    return false;
  for (unsigned i = 0; i < 2; ++i) {
    if (!Same(a.points[i], b.points[i]) ||
        a.edge_parameters[i] != b.edge_parameters[i])
      return false;
  }
  if (a.key.kind == FixedTriangleCandidateKind::VertexFace) {
    const unsigned target_a = a.local_features[0] == 3 ? 0 : 1;
    const unsigned target_b = b.local_features[0] == 3 ? 0 : 1;
    if (ft::Compare(a.triangles[target_a], b.triangles[target_b]) == 0)
      for (unsigned i = 0; i < 3; ++i)
        if (a.face_weights[i] != b.face_weights[i])
          return false;
  }
  return true;
}

Vec3 EdgeEndpoint(const CurrentFixedTriangle& triangle,
                  const FacetVertexKey& key) noexcept {
  for (unsigned i = 0; i < 3; ++i)
    if (ft::Compare(triangle.vertex_keys[i], key) == 0)
      return triangle.vertices[i];
  return {};
}

const char* Message(FixedTriangleDiscoveryStatus status) noexcept {
  switch (status) {
    case FixedTriangleDiscoveryStatus::Ok:
      return "OK";
    case FixedTriangleDiscoveryStatus::AlreadyInitialized:
      return "Feature discovery is already initialized";
    case FixedTriangleDiscoveryStatus::NotInitialized:
      return "Feature discovery is not initialized";
    case FixedTriangleDiscoveryStatus::InvalidInput:
      return "Invalid fixed-triangle geometry or topology";
    case FixedTriangleDiscoveryStatus::OutOfRange:
      return "Triangle pair index is out of range";
    case FixedTriangleDiscoveryStatus::DegenerateTriangle:
      return "Consumed current triangle is degenerate";
    case FixedTriangleDiscoveryStatus::NonFiniteResult:
      return "Fixed-triangle arithmetic is not representable";
    case FixedTriangleDiscoveryStatus::ResourceLimit:
      return "Complete fixed-triangle candidate inventory exceeds capacity";
    case FixedTriangleDiscoveryStatus::IdentityMismatch:
      return "Repeated immutable feature identity disagrees";
  }
  return "Unknown fixed-triangle discovery status";
}

FixedTriangleDiscoveryReport FreshReport() noexcept {
  FixedTriangleDiscoveryReport result;
  std::memset(&result, 0, sizeof(result));
  result.input_pair = SIZE_MAX;
  result.input_task = SIZE_MAX;
  result.message = Message(result.status);
  return result;
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

}  // namespace

struct FixedTriangleFeatureDiscovery::Impl {
  enum class Phase : unsigned {
    Constructing,
    Warm,
    Running,
    Failed,
    Stopping,
  };

  struct WorkerSlot {
    Impl* owner = nullptr;
    pthread_t thread{};
    void* stack_mapping = nullptr;
    sem_t start{};
    bool start_initialized = false;
    bool started = false;
    bool failed = false;
  };

  FixedTriangleFeatureLimits limits;
  FixedTriangleFeatureForecast forecast;
  std::unique_ptr<TriangleLedgerEntry[]> triangle_ledger;
  std::unique_ptr<VertexLedgerEntry[]> vertex_ledger;
  std::unique_ptr<EdgeLedgerEntry[]> edge_ledger;
  std::unique_ptr<FixedTriangleFeatureCandidate[]> raw_features;
  std::unique_ptr<FixedTriangleFeatureCandidate[]> features;
  std::unique_ptr<PairEvaluationStage[]> pair_status;
  std::unique_ptr<FixedTriangleIntersection[]> raw_intersections;
  std::unique_ptr<FixedTriangleIntersection[]> intersections;
  std::unique_ptr<WorkerSlot[]> workers;
  sem_t completed{};
  bool completed_initialized = false;
  std::size_t started_workers = 0;
  std::atomic<std::size_t> next_pair{0};
  std::atomic<bool> stop{false};
  std::atomic<bool> pool_failed{false};
  std::atomic<bool> busy{false};
  std::atomic<Phase> phase{Phase::Constructing};
  const CurrentFixedTriangle* job_triangles = nullptr;
  const FixedTrianglePair* job_pairs = nullptr;
  std::size_t job_pair_count = 0;
  std::size_t feature_count = 0;
  std::size_t intersection_count = 0;
  bool complete = false;

  ~Impl() { Shutdown(); }

  bool InputDisjoint(const void* input,
                     std::size_t bytes) const noexcept {
    if (!input || !bytes)
      return false;
    const auto separated = [input, bytes](const auto& storage,
                                          std::size_t count) {
      return !count ||
             Disjoint(input, bytes, storage.get(),
                      count * sizeof(*storage.get()));
    };
    return Disjoint(input, bytes, this, sizeof(*this)) &&
           separated(triangle_ledger, limits.max_triangle_references) &&
           separated(vertex_ledger, limits.max_vertex_references) &&
           separated(edge_ledger, limits.max_edge_references) &&
           separated(raw_features, limits.max_raw_feature_candidates) &&
           separated(features, limits.max_feature_candidates) &&
           separated(pair_status, limits.max_input_pairs) &&
           separated(raw_intersections, limits.max_raw_intersections) &&
           separated(intersections, limits.max_intersections) &&
           separated(workers, limits.worker_count) &&
           WorkerStacksDisjoint(input, bytes);
  }

  void PublishEmpty() noexcept {
    feature_count = 0;
    intersection_count = 0;
    complete = true;
  }

  bool WorkerStacksDisjoint(const void* input,
                            std::size_t bytes) const noexcept {
    if (!workers)
      return false;
    const std::size_t mapping_bytes =
        kWorkerStackBytes + PageBytes();
    if (mapping_bytes < kWorkerStackBytes)
      return false;
    for (unsigned i = 0; i < limits.worker_count; ++i)
      if (!workers[i].stack_mapping ||
          !Disjoint(input, bytes, workers[i].stack_mapping,
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
      try {
        owner.EvaluateJobs();
      } catch (...) {
        worker->failed = true;
        owner.pool_failed.store(true, std::memory_order_release);
      }
      if (sem_post(&owner.completed) != 0) {
        worker->failed = true;
        owner.pool_failed.store(true, std::memory_order_release);
        owner.phase.store(Phase::Failed, std::memory_order_release);
        return nullptr;
      }
    }
  }

  void EvaluateJobs() {
    for (;;) {
      const std::size_t pair =
          next_pair.fetch_add(1, std::memory_order_relaxed);
      if (pair >= job_pair_count)
        return;
      // Dynamic scheduling chooses only which worker owns this ordinal.
      // Every ordinal has one writer and all report/publication folds below
      // revisit pair_status in canonical input order after all sem_wait joins.
      auto& stage = pair_status[pair];
      const auto input = job_pairs[pair];
      const auto& first = job_triangles[input.first];
      const auto& second = job_triangles[input.second];
      stage.intersection_status = ft::ClassifyPairIntersection(
          first, second, &stage.intersection, &stage.intersects);
      if (stage.intersection_status !=
          FixedTriangleDiscoveryStatus::Ok)
        continue;
      stage.feature_status = ft::EvaluatePairFeaturesMaskedOnce(
          first, second, stage.mask,
          raw_features.get() + stage.raw_feature_offset,
          stage.expected_feature_count, &stage.feature);
    }
  }

  bool RunWorkers(const CurrentFixedTriangle* triangles,
                  const FixedTrianglePair* pairs,
                  std::size_t pair_count) noexcept {
    Phase expected = Phase::Warm;
    if (!phase.compare_exchange_strong(
            expected, Phase::Running, std::memory_order_acq_rel))
      return false;
    job_triangles = triangles;
    job_pairs = pairs;
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
    failed = failed ||
        pool_failed.load(std::memory_order_acquire);
    phase.store(failed ? Phase::Failed : Phase::Warm,
                std::memory_order_release);
    return !failed;
  }

  bool StartWorkers() noexcept {
    const std::size_t page_bytes = PageBytes();
    if (!page_bytes || kWorkerStackBytes < PTHREAD_STACK_MIN ||
        kWorkerStackBytes % page_bytes ||
        sem_init(&completed, 0, 0) != 0)
      return false;
    completed_initialized = true;
    const std::size_t mapping_bytes =
        kWorkerStackBytes + page_bytes;
    for (unsigned i = 0; i < limits.worker_count; ++i) {
      auto& worker = workers[i];
      worker.owner = this;
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
      auto* stack = static_cast<unsigned char*>(
          worker.stack_mapping) + page_bytes;
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
          : pthread_create(&worker.thread, &attributes,
                           &Impl::WorkerEntry, &worker);
      pthread_attr_destroy(&attributes);
      if (create_status != 0)
        return false;
      worker.started = true;
      ++started_workers;
    }
    phase.store(Phase::Warm, std::memory_order_release);
    // Dispatch one empty generation so every thread, stack and semaphore path
    // is live before Initialize publishes the object.
    return RunWorkers(nullptr, nullptr, 0);
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
          kWorkerStackBytes + page_bytes;
      for (unsigned i = 0; i < limits.worker_count; ++i) {
        if (workers[i].start_initialized)
          sem_destroy(&workers[i].start);
        workers[i].start_initialized = false;
        if (workers[i].stack_mapping && page_bytes)
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

FixedTriangleFeatureDiscovery::FixedTriangleFeatureDiscovery() noexcept =
    default;
FixedTriangleFeatureDiscovery::~FixedTriangleFeatureDiscovery() = default;

FixedTriangleFeaturePreflight FixedTriangleFeatureDiscovery::Preflight(
    FixedTriangleFeatureLimits limits) noexcept {
  FixedTriangleFeaturePreflight result{};
  result.report = FreshReport();
  result.forecast.triangle_ledger_capacity =
      limits.max_triangle_references;
  result.forecast.vertex_ledger_capacity =
      limits.max_vertex_references;
  result.forecast.edge_ledger_capacity =
      limits.max_edge_references;
  result.forecast.raw_feature_capacity = limits.max_raw_feature_candidates;
  result.forecast.feature_publication_capacity =
      limits.max_feature_candidates;
  result.forecast.pair_intersection_capacity = limits.max_input_pairs;
  result.forecast.raw_intersection_capacity = limits.max_raw_intersections;
  result.forecast.intersection_publication_capacity =
      limits.max_intersections;
  result.forecast.pair_status_capacity = limits.max_input_pairs;
  result.forecast.worker_count = limits.worker_count;
  std::size_t bytes = sizeof(Impl);
  std::size_t maximum_tasks = 0;
  std::size_t maximum_triangle_references = 0;
  std::size_t pair_status_bytes = 0;
  std::size_t worker_metadata_bytes = 0;
  std::size_t worker_stack_bytes = 0;
  const std::size_t page_bytes = PageBytes();
  const bool worker_shape_ok =
      page_bytes &&
      kWorkerStackBytes >= static_cast<std::size_t>(PTHREAD_STACK_MIN) &&
      kWorkerStackBytes % page_bytes == 0 &&
      kWorkerStackBytes <= SIZE_MAX - page_bytes &&
      CheckedProduct(limits.max_input_pairs,
                     sizeof(PairEvaluationStage),
                     &pair_status_bytes) &&
      CheckedProduct(limits.worker_count, sizeof(Impl::WorkerSlot),
                     &worker_metadata_bytes) &&
      CheckedProduct(limits.worker_count,
                     kWorkerStackBytes + page_bytes,
                     &worker_stack_bytes);
  result.forecast.pair_status_bytes =
      worker_shape_ok ? pair_status_bytes : SIZE_MAX;
  result.forecast.worker_metadata_bytes =
      worker_shape_ok ? worker_metadata_bytes : SIZE_MAX;
  result.forecast.worker_stack_bytes =
      worker_shape_ok ? worker_stack_bytes : SIZE_MAX;
  const bool pair_product_ok =
      CheckedProduct(limits.max_input_pairs, 15, &maximum_tasks) &&
      CheckedProduct(limits.max_input_pairs, 2,
                     &maximum_triangle_references);
  const bool arithmetic_ok =
      pair_product_ok &&
      CheckedBytes(limits.max_triangle_references,
                   sizeof(TriangleLedgerEntry), &bytes) &&
      CheckedBytes(limits.max_vertex_references,
                   sizeof(VertexLedgerEntry), &bytes) &&
      CheckedBytes(limits.max_edge_references,
                   sizeof(EdgeLedgerEntry), &bytes) &&
      CheckedBytes(limits.max_raw_feature_candidates,
                   sizeof(FixedTriangleFeatureCandidate), &bytes) &&
      CheckedBytes(limits.max_feature_candidates,
                   sizeof(FixedTriangleFeatureCandidate), &bytes) &&
      CheckedBytes(limits.max_input_pairs,
                   sizeof(PairEvaluationStage), &bytes) &&
      CheckedBytes(limits.max_raw_intersections,
                   sizeof(FixedTriangleIntersection), &bytes) &&
      CheckedBytes(limits.max_intersections,
                   sizeof(FixedTriangleIntersection), &bytes) &&
      CheckedBytes(limits.worker_count,
                   sizeof(Impl::WorkerSlot), &bytes) &&
      CheckedBytes(limits.worker_count,
                   kWorkerStackBytes + page_bytes, &bytes);
  result.forecast.owned_host_bytes = arithmetic_ok ? bytes : SIZE_MAX;
  result.forecast.startup_host_bytes =
      result.forecast.owned_host_bytes;
  if (!limits.max_input_pairs || !limits.max_triangle_references ||
      !limits.max_vertex_references || !limits.max_edge_references ||
      !limits.max_raw_feature_candidates || !limits.max_raw_intersections ||
      !limits.worker_count ||
      limits.worker_count > FixedTriangleFeatureMaximumWorkerCount ||
      !pair_product_ok ||
      limits.max_raw_feature_candidates >
          maximum_tasks ||
      limits.max_raw_intersections > limits.max_input_pairs ||
      limits.max_feature_candidates >
          limits.max_raw_feature_candidates ||
      limits.max_intersections > limits.max_raw_intersections) {
    result.report.status = FixedTriangleDiscoveryStatus::InvalidInput;
  } else if (!worker_shape_ok || !arithmetic_ok ||
             bytes > limits.max_host_bytes) {
    result.report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
  }
  result.report.message = Message(result.report.status);
  return result;
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::Initialize(
    FixedTriangleFeatureLimits limits) noexcept {
  if (impl_) {
    auto report = FreshReport();
    report.status = FixedTriangleDiscoveryStatus::AlreadyInitialized;
    report.message = Message(report.status);
    return report;
  }
  const auto preflight = Preflight(limits);
  if (preflight.report.status != FixedTriangleDiscoveryStatus::Ok)
    return preflight.report;
  auto next = std::unique_ptr<Impl>(new (std::nothrow) Impl);
  if (!next) {
    auto report = FreshReport();
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Feature discovery control allocation failed";
    return report;
  }
  next->limits = limits;
  next->forecast = preflight.forecast;
  next->triangle_ledger.reset(new (std::nothrow)
      TriangleLedgerEntry[limits.max_triangle_references]);
  next->vertex_ledger.reset(new (std::nothrow)
      VertexLedgerEntry[limits.max_vertex_references]);
  next->edge_ledger.reset(new (std::nothrow)
      EdgeLedgerEntry[limits.max_edge_references]);
  next->raw_features.reset(new (std::nothrow)
      FixedTriangleFeatureCandidate[limits.max_raw_feature_candidates]);
  if (limits.max_feature_candidates)
    next->features.reset(new (std::nothrow)
        FixedTriangleFeatureCandidate[limits.max_feature_candidates]);
  next->pair_status.reset(new (std::nothrow)
      PairEvaluationStage[limits.max_input_pairs]);
  next->raw_intersections.reset(new (std::nothrow)
      FixedTriangleIntersection[limits.max_raw_intersections]);
  if (limits.max_intersections)
    next->intersections.reset(new (std::nothrow)
        FixedTriangleIntersection[limits.max_intersections]);
  next->workers.reset(new (std::nothrow)
      Impl::WorkerSlot[limits.worker_count]);
  if (!next->triangle_ledger || !next->vertex_ledger ||
      !next->edge_ledger || !next->raw_features ||
      (limits.max_feature_candidates && !next->features) ||
      !next->pair_status || !next->raw_intersections ||
      (limits.max_intersections && !next->intersections) ||
      !next->workers) {
    auto report = FreshReport();
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Feature discovery bounded arena allocation failed";
    return report;
  }
  std::memset(next->raw_features.get(), 0,
              limits.max_raw_feature_candidates *
                  sizeof(*next->raw_features.get()));
  if (limits.max_feature_candidates)
    std::memset(next->features.get(), 0,
                limits.max_feature_candidates *
                    sizeof(*next->features.get()));
  std::memset(next->pair_status.get(), 0,
              limits.max_input_pairs *
                  sizeof(*next->pair_status.get()));
  std::memset(next->raw_intersections.get(), 0,
              limits.max_raw_intersections *
                  sizeof(*next->raw_intersections.get()));
  if (limits.max_intersections)
    std::memset(next->intersections.get(), 0,
                limits.max_intersections *
                    sizeof(*next->intersections.get()));
  if (!next->StartWorkers()) {
    auto report = FreshReport();
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message =
        "Feature discovery persistent worker startup failed";
    return report;
  }
  impl_ = std::move(next);
  return FreshReport();
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::Discover(
    const CurrentFixedTriangle* triangles, std::size_t triangle_count,
    const FixedTrianglePair* pairs, std::size_t pair_count) noexcept {
  return DiscoverImpl(
      triangles, triangle_count, pairs, pair_count, nullptr, false);
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::DiscoverMasked(
    const CurrentFixedTriangle* triangles, std::size_t triangle_count,
    const FixedTrianglePair* pairs, std::size_t pair_count,
    const FixedTriangleFeatureTaskMask* masks) noexcept {
  return DiscoverImpl(
      triangles, triangle_count, pairs, pair_count, masks, true);
}

FixedTriangleDiscoveryReport FixedTriangleFeatureDiscovery::DiscoverImpl(
    const CurrentFixedTriangle* triangles, std::size_t triangle_count,
    const FixedTrianglePair* pairs, std::size_t pair_count,
    const FixedTriangleFeatureTaskMask* masks, bool masked) noexcept {
  auto report = FreshReport();
  if (!impl_) {
    report.status = FixedTriangleDiscoveryStatus::NotInitialized;
    report.message = Message(report.status);
    return report;
  }
  bool expected_idle = false;
  if (!impl_->busy.compare_exchange_strong(
          expected_idle, true, std::memory_order_acq_rel)) {
    report.status = FixedTriangleDiscoveryStatus::InvalidInput;
    report.message =
        "Feature discovery does not accept concurrent calls";
    return report;
  }
  struct BusyRelease {
    std::atomic<bool>* value;
    ~BusyRelease() { value->store(false, std::memory_order_release); }
  } busy_release{&impl_->busy};
  if (impl_->phase.load(std::memory_order_acquire) !=
      Impl::Phase::Warm) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Feature discovery worker pool is unavailable";
    return report;
  }
  // All previously borrowed views expire at this entry.  Staging remains
  // separate, so every failure below preserves the last complete publication.
  if (pair_count == 0) {
    impl_->PublishEmpty();
    return report;
  }
  if (!triangles || !triangle_count || !pairs ||
      (masked && !masks)) {
    report.status = FixedTriangleDiscoveryStatus::InvalidInput;
    report.message = Message(report.status);
    return report;
  }
  std::size_t triangle_bytes = 0;
  std::size_t pair_bytes = 0;
  std::size_t mask_bytes = 0;
  if (!CheckedProduct(triangle_count, sizeof(*triangles),
                      &triangle_bytes) ||
      !CheckedProduct(pair_count, sizeof(*pairs), &pair_bytes) ||
      (masked &&
       !CheckedProduct(pair_count, sizeof(*masks), &mask_bytes)) ||
      !impl_->InputDisjoint(triangles, triangle_bytes) ||
      !impl_->InputDisjoint(pairs, pair_bytes) ||
      (masked && !impl_->InputDisjoint(masks, mask_bytes)) ||
      !Disjoint(triangles, triangle_bytes, this, sizeof(*this)) ||
      !Disjoint(pairs, pair_bytes, this, sizeof(*this)) ||
      (masked &&
       (!Disjoint(masks, mask_bytes, this, sizeof(*this)) ||
        !Disjoint(masks, mask_bytes, triangles, triangle_bytes) ||
        !Disjoint(masks, mask_bytes, pairs, pair_bytes)))) {
    report.status = FixedTriangleDiscoveryStatus::InvalidInput;
    report.message = "Fixed-triangle input range aliases owned storage";
    return report;
  }
  if (pair_count > impl_->limits.max_input_pairs) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }
  if (!CheckedProduct(pair_count, 15, &report.potential_tasks)) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed-triangle potential task count overflow";
    return report;
  }
  if (!CheckedProduct(pair_count, 2, &report.triangle_references)) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed-triangle reference count overflow";
    return report;
  }
  if (report.triangle_references >
      impl_->limits.max_triangle_references) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed-triangle ledger capacity exceeded";
    return report;
  }

  std::size_t triangle_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i) {
    if (pairs[i].first >= triangle_count ||
        pairs[i].second >= triangle_count) {
      report.status = FixedTriangleDiscoveryStatus::OutOfRange;
      report.input_pair = i;
      report.message = Message(report.status);
      return report;
    }
    impl_->triangle_ledger[triangle_write++].value =
        triangles[pairs[i].first];
    impl_->triangle_ledger[triangle_write++].value =
        triangles[pairs[i].second];
  }
  std::sort(impl_->triangle_ledger.get(),
            impl_->triangle_ledger.get() + triangle_write,
            TriangleLedgerLess);
  std::size_t unique_triangles = 0;
  for (std::size_t i = 0; i < triangle_write; ++i) {
    if (unique_triangles &&
        ft::Compare(impl_->triangle_ledger[unique_triangles - 1].value.key,
                    impl_->triangle_ledger[i].value.key) == 0) {
      if (!ft::SameTriangleValue(
              impl_->triangle_ledger[unique_triangles - 1].value,
              impl_->triangle_ledger[i].value)) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    const auto status =
        ft::ValidateTriangle(impl_->triangle_ledger[i].value);
    if (status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = status;
      report.arithmetic_reason =
          FixedTriangleArithmeticReason::TriangleValidation;
      report.message = Message(status);
      return report;
    }
    if (unique_triangles != i)
      impl_->triangle_ledger[unique_triangles] =
          impl_->triangle_ledger[i];
    ++unique_triangles;
  }
  report.triangles = unique_triangles;
  if (!CheckedProduct(unique_triangles, 3,
                      &report.vertex_references) ||
      !CheckedProduct(unique_triangles, 3,
                      &report.edge_references)) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed feature ledger count overflow";
    return report;
  }
  if (report.vertex_references >
          impl_->limits.max_vertex_references ||
      report.edge_references > impl_->limits.max_edge_references) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = "Fixed feature ledger capacity exceeded";
    return report;
  }

  std::size_t vertex_write = 0;
  std::size_t edge_write = 0;
  for (std::size_t i = 0; i < unique_triangles; ++i) {
    const auto& triangle = impl_->triangle_ledger[i].value;
    for (unsigned local = 0; local < 3; ++local) {
      impl_->vertex_ledger[vertex_write++] =
          {triangle.vertex_keys[local], triangle.vertices[local]};
      impl_->edge_ledger[edge_write++] = {
          triangle.edge_keys[local],
          {EdgeEndpoint(triangle,
                        triangle.edge_keys[local].endpoints[0]),
           EdgeEndpoint(triangle,
                        triangle.edge_keys[local].endpoints[1])}};
    }
  }
  std::sort(impl_->vertex_ledger.get(),
            impl_->vertex_ledger.get() + vertex_write,
            VertexLedgerLess);
  std::size_t unique_vertices = 0;
  for (std::size_t i = 0; i < vertex_write; ++i) {
    if (unique_vertices &&
        ft::Compare(impl_->vertex_ledger[unique_vertices - 1].key,
                    impl_->vertex_ledger[i].key) == 0) {
      if (!Same(impl_->vertex_ledger[unique_vertices - 1].value,
                impl_->vertex_ledger[i].value)) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_vertices != i)
      impl_->vertex_ledger[unique_vertices] =
          impl_->vertex_ledger[i];
    ++unique_vertices;
  }
  report.vertices = unique_vertices;

  std::sort(impl_->edge_ledger.get(),
            impl_->edge_ledger.get() + edge_write, EdgeLedgerLess);
  std::size_t unique_edges = 0;
  for (std::size_t i = 0; i < edge_write; ++i) {
    if (unique_edges &&
        ft::Compare(impl_->edge_ledger[unique_edges - 1].key,
                    impl_->edge_ledger[i].key) == 0) {
      const auto& old = impl_->edge_ledger[unique_edges - 1];
      const auto& next = impl_->edge_ledger[i];
      if (!Same(old.endpoints[0], next.endpoints[0]) ||
          !Same(old.endpoints[1], next.endpoints[1])) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_edges != i)
      impl_->edge_ledger[unique_edges] = impl_->edge_ledger[i];
    ++unique_edges;
  }
  report.edges = unique_edges;

  for (std::size_t i = 0; i < pair_count; ++i) {
    auto& stage = impl_->pair_status[i];
    std::memset(&stage, 0, sizeof(stage));
    const auto mask = masked ? masks[i] : FixedTriangleFeatureTaskMask{};
    const auto unsupported =
        static_cast<std::uint16_t>(
            mask.local_tasks & ~FixedTriangleFeatureTaskBits);
    const auto local = ft::PairLocalFeatureTaskMask(
        triangles[pairs[i].first], triangles[pairs[i].second]);
    const auto remote =
        static_cast<std::uint16_t>(
            mask.local_tasks & ~local.local_tasks);
    if (unsupported || remote) {
      const auto invalid =
          static_cast<std::uint16_t>(unsupported | remote);
      report.status = FixedTriangleDiscoveryStatus::InvalidInput;
      report.input_pair = i;
      for (unsigned slot = 0; slot < 16; ++slot)
        if (invalid & static_cast<std::uint16_t>(1u << slot)) {
          report.input_task = slot;
          break;
        }
      report.message =
          "Feature task mask omits a nonlocal or nonexistent task";
      return report;
    }
    report.local_masked_tasks += Popcount15(mask.local_tasks);
    stage.mask = mask;
    stage.raw_feature_offset = report.raw_feature_candidates;
    const std::size_t count = ft::CountPairFeatureCandidates(
        triangles[pairs[i].first], triangles[pairs[i].second]);
    stage.expected_feature_count = count;
    if (count > std::numeric_limits<std::size_t>::max() -
                    report.raw_feature_candidates) {
      report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
      report.input_pair = i;
      report.message = "Fixed-triangle feature count overflow";
      return report;
    }
    report.raw_feature_candidates += count;
  }
  if (report.raw_feature_candidates >
      impl_->limits.max_raw_feature_candidates) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  if (!impl_->RunWorkers(triangles, pairs, pair_count)) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message =
        "Feature discovery persistent worker execution failed";
    return report;
  }

  std::size_t feature_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i) {
    const auto& stage = impl_->pair_status[i];
    if (stage.intersection_status !=
        FixedTriangleDiscoveryStatus::Ok) {
      report.status = stage.intersection_status;
      report.input_pair = i;
      report.arithmetic_reason =
          FixedTriangleArithmeticReason::IntersectionPredicate;
      report.message = Message(report.status);
      return report;
    }
    const auto& pair = stage.feature;
    report.feature_tasks += pair.feature_tasks;
    report.exact_executed_tasks += pair.feature_tasks;
    if (stage.feature_status != FixedTriangleDiscoveryStatus::Ok) {
      report.status = stage.feature_status;
      report.input_pair = i;
      report.input_task = pair.input_task;
      report.arithmetic_reason = pair.arithmetic_reason;
      report.message = Message(report.status);
      return report;
    }
    if (pair.feature_tasks !=
            15 - Popcount15(stage.mask.local_tasks) ||
        pair.feature_count != stage.expected_feature_count ||
        stage.raw_feature_offset != feature_write) {
      report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
      report.input_pair = i;
      report.message = "Fixed-triangle task materialization disagrees";
      return report;
    }
    feature_write += pair.feature_count;
    if (stage.intersects) {
      if (report.raw_intersections == SIZE_MAX) {
        report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
        report.input_pair = i;
        report.message = "Fixed-triangle intersection count overflow";
        return report;
      }
      ++report.raw_intersections;
    }
  }
  if (feature_write != report.raw_feature_candidates) {
    report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
    report.message = "Complete fixed-triangle feature count disagrees";
    return report;
  }
  if (report.local_masked_tasks >
          report.potential_tasks ||
      report.exact_executed_tasks !=
          report.potential_tasks - report.local_masked_tasks ||
      report.feature_tasks != report.exact_executed_tasks) {
    report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
    report.message = "Fixed-triangle task accounting disagrees";
    return report;
  }
  if (report.raw_intersections >
      impl_->limits.max_raw_intersections) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }

  std::size_t intersection_write = 0;
  for (std::size_t i = 0; i < pair_count; ++i)
    if (impl_->pair_status[i].intersects)
      impl_->raw_intersections[intersection_write++] =
          impl_->pair_status[i].intersection;

  std::sort(impl_->raw_features.get(),
            impl_->raw_features.get() + feature_write, ft::FeatureLess);
  std::size_t unique_features = 0;
  for (std::size_t i = 0; i < feature_write; ++i) {
    if (unique_features &&
        ft::SameFeatureKey(impl_->raw_features[unique_features - 1],
                           impl_->raw_features[i])) {
      if (!SameCandidateValue(impl_->raw_features[unique_features - 1],
                              impl_->raw_features[i])) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_features != i)
      impl_->raw_features[unique_features] = impl_->raw_features[i];
    ++unique_features;
  }

  std::sort(impl_->raw_intersections.get(),
            impl_->raw_intersections.get() + intersection_write,
            ft::IntersectionLess);
  std::size_t unique_intersections = 0;
  for (std::size_t i = 0; i < intersection_write; ++i) {
    if (unique_intersections &&
        ft::SameIntersectionPair(
            impl_->raw_intersections[unique_intersections - 1],
            impl_->raw_intersections[i])) {
      const auto& old =
          impl_->raw_intersections[unique_intersections - 1];
      const auto& next = impl_->raw_intersections[i];
      if (old.kind != next.kind ||
          old.local_exclusion != next.local_exclusion) {
        report.status = FixedTriangleDiscoveryStatus::IdentityMismatch;
        report.message = Message(report.status);
        return report;
      }
      continue;
    }
    if (unique_intersections != i)
      impl_->raw_intersections[unique_intersections] =
          impl_->raw_intersections[i];
    ++unique_intersections;
  }

  report.feature_candidates = unique_features;
  report.intersections = unique_intersections;
  if (unique_features > impl_->limits.max_feature_candidates ||
      unique_intersections > impl_->limits.max_intersections) {
    report.status = FixedTriangleDiscoveryStatus::ResourceLimit;
    report.message = Message(report.status);
    return report;
  }
  for (std::size_t i = 0; i < unique_features; ++i)
    impl_->features[i] = impl_->raw_features[i];
  for (std::size_t i = 0; i < unique_intersections; ++i)
    impl_->intersections[i] = impl_->raw_intersections[i];
  impl_->feature_count = unique_features;
  impl_->intersection_count = unique_intersections;
  impl_->complete = true;
  return report;
}

FixedTriangleFeatureForecast FixedTriangleFeatureDiscovery::forecast()
    const noexcept {
  return impl_ ? impl_->forecast : FixedTriangleFeatureForecast{};
}

FixedTriangleFeatureView FixedTriangleFeatureDiscovery::features()
    const noexcept {
  if (!impl_ || impl_->busy.load(std::memory_order_acquire) ||
      !impl_->complete)
    return {};
  return {impl_->feature_count ? impl_->features.get() : nullptr,
          impl_->feature_count, true};
}

FixedTriangleIntersectionView FixedTriangleFeatureDiscovery::intersections()
    const noexcept {
  if (!impl_ || impl_->busy.load(std::memory_order_acquire) ||
      !impl_->complete)
    return {};
  return {impl_->intersection_count ? impl_->intersections.get() : nullptr,
          impl_->intersection_count, true};
}

}  // namespace tlfea::contact
