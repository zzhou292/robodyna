# Local MPI verification

`local_mpi_test` runs a declared native test program on one to eight local ranks.
Use this shared harness for Vehicle co-simulation and SynChrono transport coupons;
do not embed another MPI installation or make a system `mpirun` assumption.

The SDK provider supplies `@mpi_sdk//:cpp`, `:runtime` and `sdk.json`. The harness
uses the declared `prefix/bin/mpirun.openmpi`, explicit plugin/loader paths and
one CPU thread per rank. Remote-launch environment overrides are removed; the
command selects localhost, the isolated process launcher and shared-memory MPI.
GPU visibility is disabled for these CPU verification jobs.
The pinned Ubuntu layout stores MCA plugins in the `openmpi/lib/openmpi3`
subdirectory. `cli_test` executes the actual launcher's `--prefix --help launch`
path to catch missing option-parser plugins before any ranks are started.

The ordinary workstation guard must enclose the Bazel test command. Inside the
test, the existing unchanged `run_bounded.py` monitors the actual MPI session
under four GiB RSS, the inherited CPU/rank allowance and a short timeout. Its
private create-only lock serializes only that test's owned command; it does not
replace shared workstation admission. The guard retains the session leader and
uses pidfds to clean ordinary descendants across process groups before reporting
success. Its documented deliberate-session-escape limitation is unchanged.

Request, selected interpreter, executable/SDK hashes, log, guard and completion
receipts are retained in `TEST_UNDECLARED_OUTPUTS_DIR`, including failures.
The process-control implementation and its platform-interpreter capability check
are reused from the qualified runtime utilities. `environment_test` checks host
configuration admission only; a real `local_mpi_test` remains required to qualify
the actual MPI SDK and protocol.
