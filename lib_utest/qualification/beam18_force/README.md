# Beam18 circular-four-point recurrence qualification

The original142 scope reuses the already authenticated beam reference fixture
and the existing 46-point curve2100270 fixture. `prepare_fixture.py` checks the
four original rate/curve/failure cards in addition to the reference generator's
raw working coordinates, SI bits, identities, radii and elastic/density values.
There is no app dependency or generated durable duplicate mesh/curve table.

The independent oracle compiles complete MAIN_BEAM18, MULAW_IB, SIGEPS44PI,
PEVEC3, PDEFO3, PCURV3, PDLEN3, PDAMP3, PFINT3, PFCUM3, PMCUM3 and PIBUF3.
Complete PFORC3 is retained as call-order evidence. Only symbol namespaces
change. Reader/curve and reference initialization use the already owned native
LAW44 and beam18 qualification libraries. The private scalar NEL1/NPT4 ELBUF
context binds all consumed fields and every declared field referenced by the
complete callers. Excluded material/failure callees stop if entered.

The packet preserves raw working-unit native history/cursors across calls.
It compares 85 channels: four point stress/total-strain/PLA histories, section
seed, undamped force/moment, filter, EINT[2]/WPLA, endpoint nodal RHS/couples,
frame, length, rates, stiffness, unscaled monitor, damped resultants, native
endpoint/N3 stiffness scatter and OFF. All four cursors compare exactly.
Three diagnostic work increments are independently formed from the native
cumulative values; their comparison uses the absolute old/new scale because
subtraction can cancel. Public point yield/ET/unrounded dPLA are source and
direct host checked, not separately exposed by this full caller packet.

PDLEN3 with NODADT1 supplies native STI/STIR. Its unscaled element monitor is
observed in a separate unchanged NODADT0 call, with temporary output buffers.
This dual observation does not claim a native whole-solver schedule. TT0 calls
the actual virgin PIBUF3 setup and exact zero-time force sequence; the ordinary
recurrence wrapper retains a separate positive-time guard. Native geometry
starts from raw source coordinates; later prescribed SI packets are mapped to
the declared native working units, whose rounding is included in comparison.

Host controls cover exact elastic work, three-stress projection, cursor knots,
late point and complete-output rejection/retry, source identity, two-node
assembly with N3 untouched, physical damping, history carry and rigid-frame
covariance. Native controls cover SI/working units, 32 steps with unloading,
short beams and the time-floor branch, plus all142 source beams through four
intervals. CUDA controls cover 16-step native recurrence with rejected scratch
and retry, and all142 original constructors/first intervals.

Root commands (native/CUDA execution is not an author claim):

```
cmake -S lib_utest/qualification/beam18_force -B <isolated-build> \
  -DBEAM18_FORCE_NATIVE=ON -DBEAM18_FORCE_CUDA=ON \
  -DBEAM18_FORCE_SOURCE_FIXTURE=<crash-work>/reports/yaris-beam18-geometry-1
cmake --build <isolated-build> -j1
ctest --test-dir <isolated-build> --output-on-failure
```

Owning Bazel labels are `//lib_src/materials/law44/beam:point`,
`//lib_src/elements/beam18:force`, and
`//lib_utest/qualification/beam18_force:host_check`. The existing reference and
LAW44 point qualification suites remain affected regression obligations.
