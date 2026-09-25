# Native scene observation

These modules are qualification-only. They never run within the TL solver or
provide OpenRadioss-computed mechanics to production. They inspect the pinned
Linux x86-64 GNU double-precision Engine in its own guarded GDB child and stop
that child after the required source entries are observed. No existing process
is attached, paused or terminated. This is not a physical trajectory or timing run.

`abi.py` parses the original fixed-form routine signatures. `prepare.py` pins
those exact bytes, checks the initial scalar common-block layout, and writes
the argument dictionary/GDB commands into a fresh probe directory. `gdb_access.py`
implements bounded read-only call-entry access for the declared platform.
`observe.py` names each captured stage, units/array storage and completion state.
Completed stages disable their breakpoints and flush diagnostic evidence promptly.

Observe first interface controls, all initial primary-search worker slices and
their removal indices, classification normals/adjacency/role/gap/coefficient arrays,
boundary bisectors/global-main identities, raw nodal masses/velocities/coordinates,
and the first positive force-entry packet with native histories and time phases.
Arrays retain Fortran column-major order. Stored normals/bisectors are REAL4;
positions/coefficients/history are MYREAL8. Integer IDs remain native one-based
identities rather than being silently converted to app ordinals.

`I25TRIVOX.NRTM` is a local worker extent. Initial slices retain ESHIFT/ITASK and
must cover every primary search role exactly once before inventory completion.
The full classification NRTM includes generated opposite sides and is a distinct
namespace. The source caller shifts IRECT/STFM/gaps/removal offsets but leaves
MSEGTYP unshifted; the probe preserves this literal pointer contract. Do not
relabel the first worker's two rows as the complete16-role interface.

Only positive-penetration FOR3 lanes and their actual CAND_N_N history rows are
read. Inactive pre-response interpolation/history scratch is not an observation.
The original observation1 is retained but superseded for full inventory and
inactive packet-channel interpretation; its topology/controls remain separately
scoped raw evidence. Corrected observations use a fresh invocation/directory.

NOINT is the source interface ID. FLAGREMNOD/IPARI63 is the removal-mode selector.
The prior authenticated same-binary normal/friction probes establish the fixed
SCR18/SMS/IMPL1 control offsets reused here; the recipe must bind the identical
Engine binary. The generated ABI pins original routine and common-layout source.
The probe records actual values without treating input labels as resolved controls.

This first observation does not yet cover every producer: nodal rotary inertia,
global pre-normalization stiffness operands, unconsumed friction coefficient
storage, full per-step histories and evolving main normals require separate
source binding/qualification. No missing value is initialized to a guessed
physical default. A nonfinite requested array or unsupported extent fails and
preserves all preceding records. Unrelated native scratch is not requested.

Use a fresh directory containing byte-identical copies of the successfully
generated restart and corresponding Engine input; never run against the frozen
Starter output directory. Prepare with the alignment donor root, the existing
qualified QEPH original common include root and the pinned app worktree. The
launcher binds every input/source/binary hash and applies the usual4-core/10GiB
host guard and shared workstation lock. GDB ptrace permission may require the
same authorized sandbox escalation used by earlier reference probes.

Both `native-observations.jsonl` and `native-observation-summary.json` are retained.
Summary completion requires all named stages; exit2 means incomplete/rejected
observation. Partial records and all failed guard receipts remain diagnostic evidence.
