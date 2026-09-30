# Deterministic mapped wall evaluation

The mapped contact operator reuses the exact existing point law and local share
reduction, parallelized across independent compact nodes. Parent certificate
sums retain original local-node order and now run independently. Integer minima
select the original first parent and failure kind. The existing node-status scan
still precedes parent errors; global certified and signed node sums remain serial.
Force/stiffness assembly, rigid response, step admission and interval work retain
their existing formulas and ordering. This is execution work for the full-model
run, not a new contact formulation.

The mapped Summary adds 16 bytes for integer arbitration/admission state through
the existing bounded layout. No per-step allocation, full-node scratch or new
public contact configuration is added. Kernel boundaries use the existing owner
stream. Launch errors stop later stages and pass through existing poisoning and
readback. The legacy operator still uses its one-block schedule and default
64-lane copy helpers.

The synthetic fixture reuses the existing vehicle wall fixture at 137 nodes and
194 mixed Q4/T3 parents, with coincident layered parents and more than one block
in both domains. It is not an original Yaris trajectory. Three CUDA functions
compare against frozen baseline `b939d76`: point/parent masks and signed motion;
late node/parent failures and retry; reset and multi-block accepted-base copy,
including rejection and candidate preservation. All named mechanics/certificate
records and failure identities must agree. Unpublished parent scratch can finish
in parallel after a lower parent fails. Configure checks pin the frozen oracle
and unchanged arithmetic leaves; its only oracle adaptations are its namespace,
include path and a qualified call to its own ResetResult.

Configure this directory with Release, CUDA architecture120 and the installed
NVCC, build `mapped_wall_evaluation_check`, then run CTest under the workstation
guard. After this focused gate, require the existing physical_mesh_wall CUDA
owner/activity/retry tests, affected legacy wall tests, owning Bazel build and
full loaded-model timing/device-growth check before a long run. No small-fixture
pass alone establishes a full-model speedup.
