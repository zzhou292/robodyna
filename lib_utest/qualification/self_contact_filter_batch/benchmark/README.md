# Standalone CPU/CUDA filter batch benchmark

This driver compares the unchanged public CPU accepted-pair and linear-prism
certificate wrappers with one persistent standalone CUDA `Batch`. It does not
advance a vehicle, authorize contact, select an owner or time a complete solver.
No GTest target or per-pair allocation is used. Source and binary qualification
must pass before any timing is used.

Default: 4,096 pairs / 8,192 copied facets, two warmups and 20 measured repetitions
per mode. `--pairs 1..65536` and `--repeats 1..1000` are explicit bounded controls.
There are five modes: accepted classification and all four linear axis limits.
The scalar CPU and CUDA calls alternate paired ABBA order. Every call must retain
all status/category/separated/axis fields, unused fields, complete result extent
and actual upload generation. Full fieldwise comparisons and per-class counts
are outside the timed operations; equal digests alone do not admit a result.

## What the timings include

Source construction, full CPU facet pre-expansion, input hashing, stream/context
setup, owner initialization, initial scene upload and repeated scene refresh are
reported separately. Scene refresh copies the same unchanged complete scene;
its cost is not added to each retained-scene query measurement. The CPU timer
contains only the complete scalar batch of public wrapper calls with preallocated
results. The CUDA timer includes CPU pair admission, pair upload, kernel, complete
result readback and the adapter's existing stream synchronization. This is not a
kernel-only comparison. No timing-only device synchronization is added.

Each mode also reports reference/oracle setup, warmup and verification cost.
Independent ordinals retain repeated pairs as separate outputs. Per-class counts
include unresolved/invalid numerical results; the benchmark never interprets a
filter result as physical acceptance. Arithmetic environments incompatible with
the adapter's RN/gradual-underflow/masked-trap contract fail explicitly.

## Reused input corpus

The default source is the owning no-GTest `../Corpus.h` finite synthetic corpus,
including static/moving gaps, touching/ULP boundaries, coincidence, rigid groups,
degenerate geometry, extreme exponents and deterministic dyadic inputs. Geometry
is copied unchanged into independent ordinal pairs; no source-index behavior is
added to production. `MakeCorpus(false)` omits its explicitly malformed rows.

An optional explicit `TL_FILTER_FIXTURE_ROOT` can reuse the maintained GTest-free
`YarisGeometry.h` and `PathFixture.h` from the native benchmark source checkout.
With it, half the default roster is generic; one quarter uses each existing
Yaris translation/shared-vertex coordinate family. Both endpoints retain their
original bits. Half-thickness 0.001 m and absent rigid groups are declared
synthetic benchmark properties, not original car-card claims. All model/ownership
keys are outside this numerical adapter. The fixture commit and both header
SHA-256 hashes are emitted. No constants are copied into this module and no
unstated fallback fixture root is used.

## Build and execution

Build only after the root grants the heavy lane:

```sh
cmake -S <TL>/lib_utest/qualification/self_contact_filter_batch/benchmark \
  -B <fresh-build> -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DTL_FILTER_ROOT=<TL> -DTL_FILTER_FIXTURE_ROOT=<optional-native-fixture-root>
cmake --build <fresh-build> --target self_contact_filter_batch_benchmark --parallel 4
<fresh-build>/self_contact_filter_batch_benchmark --pairs 4096 --repeats 20
```

Use the workspace guard and fresh receipts/logs. Build limits: eight affinity CPUs,
four compiler workers, 16 GiB sampled RSS, 32 GiB available RAM. GPU run: two CPUs,
10 GiB sampled RSS, GPU0, 8 GiB free reserve, at most 6 GiB device growth and 32 GiB
available RAM. Keep other GPU work running; shared-device contention is part of
these observed wall times. Record the exact source pins, fixture header hashes,
binary/cache, flags and input/result digests. Do not launch alongside an active
vehicle acceptance or infer vehicle throughput from the component ratio.

This source checkpoint is unbuilt and unexecuted. It is not evidence that CUDA
filter parity or performance has passed.
