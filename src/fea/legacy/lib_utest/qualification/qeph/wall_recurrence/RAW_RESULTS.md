# CW1 raw capture/report utility results

All seven new host functions and all 31 retained analytical/model/support
functions passed on first execution. The tests cover partial capture,
interruption, exact hashes and binary64 precision, create-only output and
shared byte budgets, and four actual contact/native samples. They do not
execute the six full native jobs or establish a stability/impact decision.

The guarded single-worker build took 10.607 seconds and reached a largest
sampled process-group RSS of 374,194,176 bytes. Tests used one CPU, no GPU and
a 1 GiB RSS ceiling; all resource guards passed. The seven new test bodies
took 0.005 seconds, too brief for exact peak-memory measurement by sampling.
Reports are `crash-work/reports/qeph-wall-raw-*-1.json` and
`qeph-wall-raw-xml-1/` in the parent workspace.

The pre-execution [source map](raw-source-map.json) pins 165 declared inputs
(SHA-256 `52001ede7f74138c6df941f891f30e43121a648e4a0d0406f3f77af675cb724e`).
The `qeph-wall-raw-1` source/runtime checkpoint preserves the actual binaries,
build metadata and passing reports. Toolchain/native dependencies retain their
declared delegated scope; this is not a hermetic runtime image.

[RAW_CAPTURE.md](RAW_CAPTURE.md) records the exact capture/report contract.
Authenticated reading, derived analysis/reporting and six-job selection remain
separate pending gates. Raw collection completeness never grants simulation
admission. The 96 MiB shared limit includes all raw and derived artifacts.
