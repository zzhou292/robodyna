# Immutable CIN attachment map

`TiedCinAttachmentModel` binds one complete post-KINCHK CIN scope to a declared
`NodalNodeDomain`. Every row retains its original NSV row, one-based IRECT rank,
declared shell EID/PID, secondary node and four ordered master slots. A triangle
repeats its third master in the fourth slot. The declared mechanical shell is
independent of coincident INCOQ material/thickness witnesses.

Domain lookup requires exact source identity and represented SI coordinate bits,
including signed zero. The model owns shared immutable post-KINCHK and domain
handles, and prepares a reference `Patch` through the existing qualified geometry
function. It rejects incomplete CIN coverage, penalty/conflicting conditions,
missing nodes, conflicting source associations and rejected reference patches.
No row is silently dropped.

This is a reference map. Current patch geometry and master activity/release remain
explicit pending obligations. It supplies no physical coefficients, owner,
constraint DOFs, source inventory closure or mechanics clock.

The preflight charges the public model handle, `sizeof(Data)`, a conservative
64-byte shared-control reserve, the complete row array, two bounded identity
indexes, and retained post/domain backing. Shared backing is counted once using
explicit storage identity. Replacement also reserves the old result and any
distinct old backing before borrowed input is read. These are owned payload and
scratch reservations; additional allocator metadata and process RSS are outside
this convention. Limits remain 65,536 attachments and 64 MiB.

The six host functions cover shuffled lookup, exact bits, repeated slots, complete
11,165-row synthetic scope, late failure/retry, byte/count caps and distinct
backing. The native function reuses the existing pinned patch oracle for mapped
Q4/T3 geometry, load, motion and coefficient values. Its synthetic coefficients
are qualification inputs only. A separately supplied current patch cannot mutate
the retained reference. No new native formula or donor extract is introduced;
`tied_shell_patch/Native.cmake` factors the existing oracle target unchanged.

```sh
cmake -S lib_utest/qualification/tied_cin_attachment -B <build> \
  -DTL_TIED_CIN_ATTACHMENT_NATIVE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build <build> --target tied_cin_attachment_host_test tied_cin_attachment_native_test -j1
ctest --test-dir <build> -R '^tied_cin_attachment_' --output-on-failure
python3 -B lib_utest/qualification/tied_shell_patch/native/verify_sources.py
```

Author evidence: all six host functions and native C++ syntax passed under the
one-CPU/512-MiB lane. Native compilation and execution belong to the owning gate.
