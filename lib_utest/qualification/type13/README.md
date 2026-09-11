# TYPE13 startup qualification

This target checks only resolved property normalization, explicit-N3 frame
startup, and endpoint mass/inertia coefficients. Native recurrence, hysteresis
history, failure, stability timestep, scatter, tied interfaces and owner
admission are outside the target. No TYPE25 substitution is used.

`type13_startup_test` has eight small host functions: original four-curve/six-
channel ownership, last-segment maximum slope, late invalid output preservation
and retry, count/scope/overflow rejection, separate source inertia floor, both
native skew fallbacks, the dimensional N3 condition and coefficient failure.
Padding bytes are inspected only to show the same failed destination was not
written; successful object representations are not a serialization contract.

`type13_native_test` compiles exact authenticated RKINI3, R4BUF3 and RMASS
fragments with initialized wrapper inputs. It compares native slope/frame/M/J
results and an independent long-double dimensional M/J calculation. Only warning
I/O is omitted from the selected nondegenerate source branch. The R4BUF3 wrapper
retains both source alignment conditions and all three selected-frame arithmetic
paths; it does not claim the complete engine or degenerate warning-path behavior.

`type13_source_native_test` compares all 4,442 original PID2000486 beam frames and
coefficient outputs against the native oracle. The original 7,494-node fixture
(7,493 physical endpoints plus N3) is authenticated separately. Its original
coordinate doubles are hex literals; each native-to-SI multiplication must match
the existing canonical SI bytes. First/final, minimum/maximum length and closest
N3 alignment identities are asserted. Source membership is not tied contact
pairing and this test does not connect these endpoints to the shell owner.

`type13_startup_cuda_test` has two optional actual-device functions for original
property/reference/M/J parity and late rejection followed by exact retry. Root
schedules native compilation and CUDA execution. Author evidence is only eight
host tests PASS plus native C++ syntax and both fixture/source hash verifiers;
the initial host build used one CPU, 179,980 KiB peak RSS and a 512 MiB cap.

Owning root gate (choose the guarded build path externally):

```sh
cmake -S lib_utest/qualification/type13 -B BUILD_PATH \
  -DTYPE13_NATIVE_CHECKS=ON -DTYPE13_CUDA_CHECKS=ON \
  -DCMAKE_Fortran_COMPILER=FORTRAN_PATH -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_PATH --parallel 1
ctest --test-dir BUILD_PATH --output-on-failure
```

Existing workstation Fortran/CUDA environment and resource guard apply. Owning
Bazel targets are `//lib_utest/qualification/type13:type13_startup_check` and
`//lib_utest/qualification/type13:type13_startup_cuda_check`. Complete donors and
exact extracts are verified by `native/verify_sources.py`; original fixture
bytes by `source_fixture/verify_fixture.py`.
