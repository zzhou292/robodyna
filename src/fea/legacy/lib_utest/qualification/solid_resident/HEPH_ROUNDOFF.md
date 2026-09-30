# Loaded HEPH comparison: bounded spectral cancellation

The original 10 N owner fixture, fixed `dt=2^-20`, material, histories, native
donors and mechanical code are unchanged. This test-only allowance applies to
the eight-step, active, near-identity resident trajectory. It is not a general
large-strain bound, a runtime admission rule, or a new `NearlyEqual` policy.
The original 2,412 constructor comparisons and older family gates retain their
previous criteria. `solid24_force/NativeComparison.h` is an unchanged extraction
of the existing comparison functions for reuse by the new host test.

## Same-packet diagnosis

Root `e898b3e` captured all 187 HEPH channels using the actual owner-prepared
positions/velocities. Initial material history matched exactly. In interval one:

- All geometry, gradients, rates and total strains matched host/native bits.
- GPU retained fields matched host bits except HG viscosity by one ULP, already
  inside its existing criterion. Stress was identical on CPU and GPU.
- The three native stresses exceeded host/GPU by approximately `2^-28` Pa at
  stress levels 0.755/0.651/0.651 Pa. Remaining failed channels were their force
  and energy consequences, including duplicate diagnostic stress fields.

`SIGEPS42` evaluates O(24 MPa) derivatives and subtracts their mean. The shared
Eigen helper returns ascending principal values; native `VALPVEC_V` returns
max/middle/min. Reversing only the diagnostic mean reduction on the captured
packet changes `0x1.6e36000000001p+24` to `0x1.6e36p+24`, exactly `-2^-28` Pa.
It reproduces the native normal stresses within 2.22e-16 Pa. Both orders retain
the same mathematical law; changing the force implementation is unnecessary.

The independent reference avoids an eigen decomposition. For the selected
single alpha=2 term, with `C=I+strain` (engineering shears divided by two),

```
J = sqrt(det(C))
sigma = mu * J^(-5/3) * dev(C) + bulk * (J-1) * I
```

The prepared binary64 bulk and exact captured binary64 strains enter Python
Decimal at 80 and 120 digits. Both precisions round to the same six stored
stress values. Maximum native error is about 9.04e-8 Pa and maximum host error
about 9.41e-8 Pa; a tolerance relative only to the sub-Pa residual cannot cover
the material-scale operations. `verify_heph_capture.py` reproduces this check
without importing production or native code.

## Allowance and propagation

`PrepareHephRoundoff` admits only finite native packets with active=1,
`L1(total_strain)<=1/16`, `0.9<=J<=1.1`, positive volumes and nonnegative dt.
Those conditions are comparison scope only. Its conservative stress operation
scale is the same one used by the qualified near-zero S6Z control:

```
b_new = 128 * epsilon * (bulk + 2*mu) * (1 + L1(total_strain))  [Pa]
```

The factor bounds the short near-identity spectral/volume/material cancellation
at issue and is checked by the independent invariant packet. It is not asserted
to enclose every cubic eigen problem or arbitrary large strain. The actual
fixture is far inside the declared strain/volume limits; outside them the
trajectory must fail its comparison preparation rather than broaden admission.

For `SFINT3` and `SRROTA3`, the per-component force allowance is
`b_new * current_volume * max_slot_L1(gradient) * max_row_L1(frame)` [N].
Maxima preserve the bound under source-slot permutation. Geometry and frame
comparisons themselves receive no extra allowance.

In native material work, the old/new normal pressure sum averages three
old/new stress pairs. If `b_sum=b_old+b_new`, its error is at most `b_sum`;
each normal work term has at most `2*b_sum`, each shear term `b_sum`. Thus:

```
B = .5 * (abs(average_volume*dt) * (2*L1(normal_rate)+L1(shear_rate))
          + abs(volume_increment))
b_work = B*b_sum + 32*epsilon*B*L1(old_stress,new_stress)  [J]
```

Accepted energy error and `b_work` propagate through the literal storage-volume
division. Its finite-sum allowance and the existing independent HG-work
criterion propagate through the two HG energy additions. HG work/history,
viscosity, density, Q, ET, SSP, active flag, geometry and stamps keep their old
checks. Only `Accept()` moves the prepared budget into the accepted budget;
failed candidates/retries cannot accumulate additional slack.

Three host functions exercise the captured complete packet, reject 0.01 Pa
stress / 0.001 N force perturbations and work/energy perturbations exceeding
their derived allowances, assert every unaffected field has zero added budget,
and verify out-of-scope rejection plus unchanged/retry/carry behavior. The
owning eight-step CUDA/native test and existing later-participant rejection
test exercise actual accepted and rejected owner transactions.

## Evidence and owning gate

Root reports retained under `crash-work/reports`:
`solid-resident-heph-diag-tests-1.log`, `solid-resident-heph-analysis-1.log`,
`solid-resident-heph-mean-test-1.log`; the mean-order probe source is retained as
`solid-resident-heph-mean-probe.cpp`. `HephCapturedPacket.h` records the complete
diagnostic-log SHA256 and exact owner/native values.

Configure the existing resident qualifier with `SOLID_RESIDENT_CUDA=ON` and
`SOLID_RESIDENT_ORIGINAL=ON`; build `solid_resident_roundoff` and
`solid_resident_cuda`, then run both plus `solid_resident_roundoff_invariant`.
Native/GPU execution remains the root qualification lane.
