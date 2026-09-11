# TYPE13 startup qualification

Compile the explicit source package to an absent destination:

```sh
python3 -B -m modelio.type13_source --archive SOURCE_ZIP --assets CANONICAL_DIR --output NEW.json
```

The command reports the exact expected byte count and SHA256 for the C++ loader.
There is no implicit search path or simulation startup. See SOURCE_CONTRACT.md
for the pinned scope, units, defaults, original-coordinate identities and budgets.

Author checks: four small Python source tests and two C++ converter tests PASS;
all reader/test units pass host syntax. The complete original compiler produced
5,150,841 bytes, SHA256
`c15fc2096317ac0206397ac50776f8456ddd23495e0c65aeee98e093ebd0b1b1`,
in 2.23 s at 70,144 KiB maximum RSS (one CPU, 512 MiB cap). This fixture is
`crash-work/reports/yaris-type13-startup-declaration-1.json` in the workspace.

Root owning host gate, using its existing Chrono/TL source paths:

```sh
cmake -S modelio/type13 -B BUILD_PATH -DChrono_DIR=CHRONO_CONFIG \
  -DROBO_DYNA_TL_ROOT=TL_SOURCE \
  -DROBO_DYNA_TYPE13_DECLARATION=ORIGINAL_COMPILED_DECLARATION
cmake --build BUILD_PATH --parallel 1
ctest --test-dir BUILD_PATH --output-on-failure
```

Target `robo_dyna_type13_source_check` owns two original converter/atomicity
functions and two actual source functions. Actual tests compare all 7,494 raw/SI
nodes and 4,442 beam associations against the independently authenticated TL
original fixture, then check 15 rehashed corruptions, capacity/budget failures,
preserved destination and retry. Shared namespace/geometry/reader utilities are
linked through their existing owning targets. No CUDA/runtime target is added.
The TL startup's separate native/CUDA gate qualifies the RKINI3/R4BUF3/RMASS
values; this app test does not execute the complete SDI converter or a vehicle
simulation.
