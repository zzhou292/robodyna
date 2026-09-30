# Owning connectivity gates

All changes are new files below `case/vehicle_startup/connectivity`; the base is
app9b0b352. No source, mechanics, participant or frozen fixture is changed.

Run these commands under the root workstation guard. Use fresh build, guard,
XML and report paths. The complete actual fixture reproduces its existing GPU
search, so it requires the same CUDA access as the physical-attachments gate.

```bash
cmake -S <app>/case/vehicle_startup/connectivity -B <fresh-build> \
  -DCMAKE_BUILD_TYPE=Release \
  -DChrono_DIR=/home/jsonzhou/Desktop/chrono-work/crash-work/install/chrono-vsg-r0/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DROBO_DYNA_CONNECTIVITY_ORIGINAL=ON \
  -DROBO_DYNA_VEHICLE_CANONICAL=/home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_VEHICLE_SCOPE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-full-shell-scope-10.json \
  -DROBO_DYNA_VEHICLE_DECLARATIONS=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-vehicle-declarations-1.json \
  -DROBO_DYNA_VEHICLE_GLASS_RESOLUTION=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-vehicle-section-resolution-glass-1.json \
  -DROBO_DYNA_VEHICLE_GLASS_SHA256=ea40b817b66c73e961c502e5ff0d4dffd5d354b65e1339e6a093164adeb32158 \
  -DROBO_DYNA_VEHICLE_TYPE13_DECLARATION=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-type13-startup-declaration-1.json \
  -DROBO_DYNA_CONNECTIVITY_REPORT=<fresh-create-only-report.json>
cmake --build <fresh-build> --parallel 1
ctest --test-dir <fresh-build> --output-on-failure
```

Owning targets are `robo_dyna_vehicle_connectivity_values_check` (four host
functions) and `robo_dyna_vehicle_connectivity_original_check` (two original
functions). CTest names are `vehicle_connectivity_values` and
`vehicle_connectivity_original`. The latter reuses the existing authenticated
`actual_fixture.py` ZIP/member/context fixture wrapper; no new extraction
script is introduced. `GTEST_OUTPUT=xml:<fresh-directory>/` retains per-target
function evidence through the wrapper's inherited environment.

Before graph allocation the original test prints the complete source, graph,
scratch and phase-max reservation. It records measured component counts,
371,413 typed source relations, ordered support slots, graph/total bytes and
complete JSON size/SHA in GTest properties. It validates every original CIN
support slot (including 125 repeated triangles), all 4,442 beam endpoint
pairs, native/source shell/solid identities, exact caps and shared lifetime.
All required-joint and omitted-source obligations remain Pending regardless
of the measured component count. Full JSON must fit the unchanged 32 MiB cap;
a failure must preserve evidence and be corrected without dropping rows.

The author performed no original-source execution, native or CUDA build/run.
Small evidence lives under `crash-work/reports/vehicle-connectivity-*`:
four tests in `author-functions-1.xml`, initial complete syntax in
`syntax-1.log`, and final narrowing-warning-clean syntax in `syntax-2.log`.
The initial warning and its correction are retained as separate evidence.
