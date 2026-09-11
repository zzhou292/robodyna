# Ordered solid source slots

Eight-slot translational force assembly reuses the existing tied-triangle
staging utility and strict single-node force arithmetic. The old four-slot
force/couple entry and strict distinct-node entry retain their contracts.
Translation-only assembly never reads or writes couples, including signed zero
or absent rotation storage. The positive scalar helper stages original slot
stiffness additions; it accepts already computed element terms and defines no
new stiffness formula. Both operations reject before any destination write.

Root qualification includes five new host functions, three unchanged tied
assembly host functions, one new CUDA function and the unchanged tied CUDA
function. Controls distinguish sequential addition from combining repeated
terms first, catch overwritten repeated stiffness, exercise final-slot invalid
connectivity/NaN/overflow, preserve unused nodes and rotation channels, compare
distinct forces against the strict existing scatter for both signs, and retry.
These are value/assembly tests; they do not admit the rear H8 family to the
resident model or establish its constitutive force or native STI formula.

Configure this directory with `-DTL_ORDERED_SOLID_SLOTS_CUDA=ON`, build and run
CTest through the shared workstation guard. Owning Bazel host target is
`//lib_utest/qualification/ordered_solid_slots:host`.

The typed rear force adapter must retain SCUMU3's quarter of the summed point
STI at every original slot, including repeated slots. Later resident integration
must stage/check both force and stiffness for its parent and discard the whole
owner trial after any failure, as the existing coordinator already requires.
No new owner, time integration, arena, atomics or physical mass is introduced.
