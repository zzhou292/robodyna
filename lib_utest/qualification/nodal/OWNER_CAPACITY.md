# Shared nodal owner capacity

The connected-component staging limit is 2,048 physical nodes under the existing
1 MiB explicit device allocation ceiling. All device buffers and host staging
use the active count. The owner still creates six device allocations at startup;
there is no allocation or container growth while stepping or reading results.
Extended device storage costs `411*N + sizeof(Control)` bytes. Legacy translation
storage costs `193*N + sizeof(Control)` bytes. The owner validates both its count
and the caller's byte budget before allocating or publishing anything.

`MaxNodalStateNodes` controls only this owner. The old `MaxTranslationNodes=128`
constant remains a compatibility bound for existing shell/contact contributors.
Those contributors must separately qualify their larger buffers, work coverage,
force reductions and publication before a larger shell case can run. In
particular, increasing owner capacity does not admit a 1,030-node contact case.

Existing ordered node loops visit the entire admitted active range. This change
does not change their arithmetic or promise parallel integration throughput.
The rotation operation remains the qualified isotropic-node recurrence, not a
rigid-group anisotropic integrator. Group constraints are a separate participant.

`utest_nodal_capacity_cuda` checks 1,030 and 2,048 nodes against analytic constant
force/torque results in both collocated and staggered modes. It checks every node,
first-half-kick timing, allocation scaling, one-byte-short admission, invalid
startup at the final node, and a final-node arithmetic failure followed by exact
retry without consuming the initial half kick. Existing nodal tests retain
the 128-node contributor fixture and its unchanged byte budget.

Execution evidence belongs in the workspace's guarded reports; adding this
test source is not itself a passing runtime claim.
