# Zero-coefficient rigid-dependent member values

An explicit `NonnegativeDependent` policy lets the existing two-member and
ordinary rigid-member value functions accept source M/J equal to zero.
Rigid-dependent motion is recovered from its prepared primary; these functions
never divide by member M/J. Reactions retain native M*a-F and J*alpha-C.
The default `PositiveIndependent` policy preserves existing strict admission.
Negative/nonfinite coefficients and unknown policies reject without publication.
No equations, regularizers, native donor statements or tolerance change.

Two host functions check actual primary motion, zero-coefficient reactions,
legacy rejection and whole-packet late failure/retry. Two independent native
functions evolve their own accepted histories across64 loaded intervals using
the existing authenticated RGBODFP/RGBODV wrappers; they cover four-member
zero M/J and two-member zero J finite rotation. One CUDA function compares both
64-interval recurrences to host values.

Root gate `rigid-dependent-root-tests-1` passes these5 new functions, the linked
legacy rigid value/native regressions, and source identity. Build attempt1
omitted the existing NativeSchedule fixture required by a legacy test;
attempt2 links that unchanged fixture and succeeds.

These are math packets with explicit supplied primary values and durations,
not a proof of full physical inventory or live owner admission. The next
common-owner increment selects this policy only for the authentic prepared
PART binding and retains the native primary/member state in the same transaction.
