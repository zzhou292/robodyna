# Observed post-KINCHK values

This explicit profile implements the consumed KINCHK result for supplied
observed nodes when RWALL, RBE2, RBE3 and cyclic roles are excluded. It preserves
the complete five IKINE fields and ITF decode, and assigns KINET from the current
condition field. IRUPT is preserved caller association: KINCHK has no IRUPT
argument. The result does not admit coefficients, an owner, a global generated
condition map, or the following INIVCHK velocity initialization.

Repeated and mixed incompatible condition flags reproduce the per-node native
possible-conflict predicates under this no-wall profile. Native warnings do not
switch CIN/PEN. Global hierarchy/warning counts remain unspecified. App source
receipts must authenticate the profile; the TL values cannot infer absence from
an incomplete source census.

Host checks cover the complete fields and separate CIN/PEN association,
excluded condition bits, phase/identity failures, last decode word, exact byte
limits, borrowed input and failure-atomic retry. The native gate uses complete
KINCHK, its original KININI decode, master-only RBODY hierarchy/duplicate warning
controls and real excluded wall/RBE/cyclic branches. It is a supplied-context
oracle, not a reconstruction of the full starter registry.

Configure this directory with `-DTL_TIED_POST_KINCHK_NATIVE=ON` for the owning
Fortran gate. Targets are `tied_post_kinchk_host_test` and
`tied_post_kinchk_native_test`; all CTest entries start `tied_post_kinchk_`.
The author ran host and source-identity checks only. Root subsequently passed
all3 host +4 native functions and source identity (`tied-post-kinchk-root-tests-1`,
2026-09-11). The18 affected classifier host/native functions plus source identity
also pass (`post-kinchk-affected-classification-tests-1`). Both owning targets
build in `law36-post-kinchk-owning-bazel-build-1`. No production correction was
needed. These observations do not admit coefficients or a runtime owner.
