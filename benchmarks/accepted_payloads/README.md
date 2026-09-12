# Exact accepted payload comparison

Reusable, read-only qualification of two closed vehicle run outputs. Streams file
hashes in 1 MiB chunks and compares every archive file and the viewer receipt.
Every non-timing summary field is compared using its original JSON value text,
retaining negative zero and numerical spelling. The only other permitted change
is an explicitly declared total host forecast delta. Unknown field differences,
duplicate keys, nonfinite JSON constants, missing payloads and symlinks fail.

This compares results; it does not independently establish model validity,
restart completeness, device allocation counts, or statistical timing confidence.
Those remain owner/forecast and repeated benchmark gates.

```
python3 -B robo-dyna/benchmarks/accepted_payloads/test_compare.py
python3 -B robo-dyna/benchmarks/accepted_payloads/main.py BASELINE CANDIDATE \
  --expected-host-delta 0 --report NEW_REPORT.json
```

The report is create-only. Existing simulation files are never modified.
