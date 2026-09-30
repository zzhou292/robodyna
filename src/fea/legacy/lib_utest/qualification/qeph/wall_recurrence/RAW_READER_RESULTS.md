# Strict raw reader results

All five new host functions and 38 retained model/raw/analysis/support functions
passed on first execution. The tests cover complete synthetic raw records with
failed derivative checks, incomplete nonfinite records, hash/path/budget and
exact model/probe/contact corruption, including rehashed mutations that reach
semantic validation. Expected failures preserve the complete caller output.
No native interval calls, spectral selection or CUDA are part of this gate.

The guarded one-worker build took 7.25 seconds and reached a
largest sampled process-group RSS of 370589696 bytes. Tests
used one CPU and a 1 GiB RSS cap. All resource guards passed. Reports are
`crash-work/reports/qeph-wall-raw-reader-*-1.json` and
`qeph-wall-raw-reader-xml-1/` in the parent workspace.

The pre-execution [input map](raw-reader-source-map.json) pins 183 inputs,
SHA-256 `b20f0285756cd1eeb864fe8ac5d56970b9be246eaa057f124fc403b74fa0efc6`.
The `qeph-wall-raw-reader-1` source/runtime checkpoint retains actual binaries
and reports. Native/compiler dependencies remain delegated as declared;
this is not a hermetic image. The full six raw jobs already exist, but actual
reading/analysis of that set and an authenticated selected timestep are pending.

[RAW_READER.md](RAW_READER.md) defines exact external fingerprint, inventory,
model and payload validation. The reader never treats stored numerical pass
flags or a self-supplied source hash as proof of impact admission.
