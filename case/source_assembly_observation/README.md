# Source assembly kinetic and kick observations

`ObserveInitial` and `ObserveInterval` map the authenticated six-part source to
TL's qualified rigid kinetic/kick value functions. They own no CUDA state,
clock, material history, integration policy or output archive. The case must
supply the same immutable binding used by its participants, actual owner/token
readbacks and the exact assembled load snapshot before sealing the trial.

The adapter checks the complete source group order and accepted/candidate time
schedule. Kinetic energy is partitioned into ordinary native nodes, physical
group members and aggregate groups. Effective stored kinetic is ordinary plus
aggregate; native TOTAL J is counted once. Physical/added member inertia,
primary mass/inertia and principal correction remain separate diagnostics.
Publication's raw native nodal kinetic channels are checked against an
independent long-double all-node sum.

Per-component native impulse and work checks use the real applied force/couple
and candidate reaction force/couple. The first half kick is distinct from the
full physical interval. Group work/phase checks use TL's existing value API.
Group replacement deltas are accumulated directly, preserving a small group
correction even when ordinary-node work is much larger. All outputs are staged
on bounded stack storage and assigned only on success; no step allocation occurs.
The largest temporary group spans contain 256 members and the membership map
contains at most 2,048 nodes. Source storage remains owned by its binding.

The later kinetic metric pairs midpoint velocities with lagged force-stage
principal axes. It is not a collocated physical-energy reconstruction.
`effective_residual` is native kick bookkeeping plus replacement and rounding,
not an independent check of aggregate dynamics. Shell/contact work, geometry,
material history and the complete acceptance receipt belong to the case.
Reaction work is not dissipation; native plastic work is not added again to
shell internal work.

Ten host functions pass in `source-assembly-observation-tests-1`: complete actual
915-shell/1,030-node/6-group initial partitions, full tensor/native scalar J,
applied and reaction force/couple work, first/later timing, corrupt final-node
and group reactions, zero-average reversal, sub-ULP impulse, nonfinite/alias
rollback and small group correction under large ordinary work. Supplied-motion
mapping fixtures are explicitly value tests, not claimed accepted trajectories.
The new case's actual loaded CUDA gate must still supply these observations
from its authenticated owner before common publication.
