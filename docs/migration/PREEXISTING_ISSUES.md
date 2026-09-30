# Pre-existing checks requiring scope review

The inherited `output/physical_run/tests/replay_preflight/verify_sources.py`
checks whole-file hashes for an earlier source-only replay-admission change against
baseline `1ea39473365a3853c346d80df10c6efd9317e706`. It rejects current Prepare.cpp,
which is byte-identical in the qualified100ms app and the consolidated import:
`57cf2b99d7238aab0bd2b4bd6317b8a4a5d16ee0abbad2a7bfa20169ff372449`.
The later100ms horizon change predates this migration. Keep the historical proof
unchanged; do not refresh its hashes to claim that old reversal evidence covers
new source. Current functional replay-preflight tests and exact import/short-run
comparisons are separate migration gates. Numerical runtime validation was not
disabled. This issue is not classified as a new physics regression.
