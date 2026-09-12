# V5 connectivity source qualification

Base app 7a04816. This source/report change adds no mechanics, source admission,
geometry, constraints or timestep policy. Tiny tests and syntax are author
evidence; the commands below belong to the root workstation guard.

From `/home/jsonzhou/Desktop/chrono-work`, configure a fresh build using the
integrated app path (or this isolated worktree before integration):

```sh
cmake -S robo-dyna/case/vehicle_startup/connectivity \
  -B crash-work/build/vehicle-v5-connectivity-root-1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DChrono_DIR=/home/jsonzhou/Desktop/chrono-work/crash-work/install/chrono-vsg-r0/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DROBO_DYNA_CONNECTIVITY_ORIGINAL=ON \
  -DROBO_DYNA_CONNECTIVITY_SUPPORTS=ON \
  -DROBO_DYNA_VEHICLE_CANONICAL=/home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_VEHICLE_SCOPE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-full-shell-scope-10.json \
  -DROBO_DYNA_VEHICLE_DECLARATIONS=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-vehicle-declarations-1.json \
  -DROBO_DYNA_VEHICLE_GLASS_RESOLUTION=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-vehicle-section-resolution-glass-1.json \
  -DROBO_DYNA_VEHICLE_GLASS_SHA256=ea40b817b66c73e961c502e5ff0d4dffd5d354b65e1339e6a093164adeb32158 \
  -DROBO_DYNA_VEHICLE_TYPE13_DECLARATION=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-type13-startup-declaration-1.json \
  -DROBO_DYNA_CONNECTIVITY_REPORT=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/vehicle-connectivity-v1-regression-2.json \
  -DROBO_DYNA_CONNECTIVITY_V5_REPORT=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/vehicle-connectivity-v5-full-1.json

cmake --build crash-work/build/vehicle-v5-connectivity-root-1 --parallel 2 \
  --target robo_dyna_vehicle_connectivity_values_check \
           robo_dyna_vehicle_connectivity_original_check robo_dyna_vehicle_connectivity_v5_check

ctest --test-dir crash-work/build/vehicle-v5-connectivity-root-1 \
  --output-on-failure -R '^vehicle_connectivity_(values|original|v5)$' \
  --output-junit /home/jsonzhou/Desktop/chrono-work/crash-work/reports/vehicle-v5-connectivity-root-tests-1.xml
```

The existing actual-fixture wrapper supplies exact staged source, auxiliary
and wall members. CUDA is needed only for that existing CIN startup search.
There is no dynamics owner or trajectory in this gate. Use fresh report names
on a repeat because report publication is create-only. Existing source caps,
32 MiB complete JSON cap and 64 MiB extra phase cap remain unchanged.

Expected functions:8 tiny, 2 legacy original, 3 V5. V5 records measured component
counts and all seven region labels, complete source slots/counts, shared joint
lifetime, and cap-minus-one/late replacement behavior. The whole report remains
weak incidence/potential transfer, with released joint DOFs and directed CIN
meaning explicitly preserved. A passing graph is not constrained-DOF rank or
proof of current activity/force transmission, self-contact or omitted physics.
