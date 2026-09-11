# Combined rigid/CIN owner preallocation forecast

`FENodalState::ForecastAssemblyCin` returns a value containing a report and the
existing complete assembly-owner layout. It reuses ForecastRigidStorage,
ForecastCinStorage, StateLayout, force-stage capture layout and CinOwnerHostFits
with the real private control/object sizes. No memory, clock identity, source
coefficient, material history or degree of freedom is created. Raw mass/inertia
and witness arrays may be null for this metadata-only query; the actual
Initialize overload still requires and validates every physical value.

The selected query requires prepared PART/plain assembly and CIN with the exact
same physical domain backing. Its optional presence flag corresponds to whether
the actual extended constructor receives an explicit rotational presence mask.
The complete host budget retains the existing limit policy. Exact device bytes
exclude CUDA runtime overhead. Host values partition into retained CIN source,
owner payload and temporary witness identity index; vector capacities remain
subject to the constructor's existing runtime capacity guard. The external
prepared rigid binding/ledger are not retained by the owner and are not charged
again here. A composing app must retain and account for them separately.

Author: 3 host functions PASS under 1 CPU/512 MiB using fresh changed/forecast
units and cached qualified fixture/source libraries; CUDA fixture host syntax
passes. No author native/GPU execution. Root gate:

```sh
cmake -S lib_utest/qualification/physical_owner_forecast -B /path/to/build \
  -DPHYSICAL_OWNER_FORECAST_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build /path/to/build --target physical_owner_forecast_host_test \
  physical_owner_forecast_cuda_test --parallel 1
ctest --test-dir /path/to/build -R '^physical_owner_forecast_' --output-on-failure
```

Host tests cover exact complete host/device bounds and one-byte-short retry,
source profiles, invalid counts/phase, optional presence and force-stage capture.
The new root-owned actual CUDA test compares the forecast against the genuine
combined owner allocation and checks preallocation rejection/retry. It reuses
the established source fixture including genuine PART zero M/J, plain groups,
CIN dependent inverse zeros and ordinary solid rotational absence.
