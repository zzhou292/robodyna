# QBAT ordered measurement read tile

Source candidate based on TL92dd532e. Scope is `FinalizeMapped`, previously one
thread over already prepared 168-byte measurement packets. A 64-thread block
loads bounded rows into 10,440 bytes of shared storage; only lane zero performs
the unchanged parent/local/channel additions and Control publication.

The complete source-order candidate-status prescan precedes every measurement
read. Invalid packet validity is checked before payload; inactive valid rows
retain their work/history contribution. Each tile has two uniform barriers,
and first-invalid measurement stops the leader with the exact diagnostic prefix.
Displacement fallback and final finite checks remain shared existing functions.
No reduction tree, floating atomic, arena/packet ABI, model policy or history change.

`MeasurementValues.h` extracts the original arithmetic verbatim into one helper;
`Measurement.h` similarly shares the original status prescan. Three small files
under `mapped/measurement` own transient layout, read/load, and CUDA scheduling.

Qualification reuses existing geometry/device fixtures and complete-Control
comparison. Frozen baseline headers keep an independent literal ordered fold.
Three host tests include a protected unreadable numerical payload for invalid
packets; six CUDA tests cover tile boundaries, active/removed histories, complete
status priority, invalid packet prefix, cancellation, NaN/overflow, repair,
displacement fallback and the actual production mapped launch versus the
independent older full serial caller. Source proof reverses all three changed
production bodies and pins unchanged layout/force/fixture dependencies.

Source-only until the guarded build/CUDA/owner/resource and matched vehicle
gates execute. Existing retained memory forecasts must stay identical. Inspect
actual compiler inputs and resources. Compare fresh counterbalanced 101-step
runs with byte-exact archives; revert the scheduling experiment if it loses.
