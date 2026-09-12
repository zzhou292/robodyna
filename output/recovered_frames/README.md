# Interrupted sampled review recovery

This additive reader publishes `recovered-samples.json`, schema
`robo_dyna.recovered_sampled_review.v1`. It is not a normal physical run
manifest or restart. The original planned request and observation profile are
retained as provenance; the interval ledger is explicitly unavailable, and no
accepted-interval continuity or requested-horizon completion is certified.

`Recover` requires an unpublished interrupted archive, a real empty destination
outside the original, caller-selected source/mapping authority and a stop reason.
This first profile requires no durable interval files. It validates the source
bundle, exact frame context, initial and consecutive scheduled complete
frame/activity pairs, optional seven-file wall receipt and composition. Missing
trailing metadata pairs stop the retained sample prefix; corrupt complete pairs
reject. Typed readers check all binary hashes, sizes, finite values, identities,
stamps, field layouts and packed activity padding. It never guesses missing rows,
attempts, structural limits, frames, activity, energy or contact work.

Only named typed-owner files are copied into the new directory. Copies recheck
original hashes and use at most one 32 MiB buffer. No hard links or original
writes occur. The descriptor is published last; failed copies remain incomplete
evidence without a recovery descriptor. Replay revalidates exact inventory and
all typed records before returning, then rechecks each sought sample. Its API has
frames and recorded stamps, not a normal run Index or completion marker.

The source allowance plus context, metadata, maximum sample/copy/wall workspace
is bounded by the existing 512 MiB ReplayLimits. The copied archive respects the
original selected total byte cap, with a full 1 MiB descriptor reservation.
Source hashes establish caller-selected provenance/coherence, not validation of
physics or continuity of unrecorded intervals.

Root build (no CUDA required):

```sh
cmake -S <app>/output/recovered_frames -B <build> -DChrono_DIR=<chrono-r1>/lib/cmake/Chrono -DROBO_DYNA_TL_ROOT=<TL> -DCMAKE_BUILD_TYPE=Release
cmake --build <build> --parallel 4
ctest --test-dir <build> --output-on-failure
<build>/recover_physical_samples <interrupted-archive> <pre-created-empty-new-archive> <matching-source-viewer-input.json> "Hard GPU-growth guard interrupted recording; interval buffer unavailable"
```

The authority receipt may belong to another run with the same exact original
source and mapping; its run ID and archive manifest are not reused. The recovery
CLI prints the distinct descriptor size/SHA after opening the full copied review
through the new reader. The renderer's explicit recovered-input mode uses this
descriptor and preserves original PID palette, recorded plasticity/activity,
selected wall and recorded physical time.

Author checks: six small-file host functions and eleven production/test/CLI
syntax units pass under one CPU/512 MiB. Host linking reused existing bounded
record libraries with current changed/shared objects; root owning CMake and full
100-sample recovery remain the decisive integration gate. No GPU, native solver
or full source run was executed by the author.
