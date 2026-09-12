# Bounded monitoring history

`run_bounded.py` evaluates every live sample against its existing limits. It
retains at most 16,384 sample records: the first 2,048 and the rolling last
14,336. Before the capacity is reached, all samples remain present. Afterward,
the middle is dropped; retained records remain in chronological order without
overlap. This bounds the guard's retained telemetry even on long commands.

The existing `peak_sampled_rss_bytes` now uses an independent whole-run maximum,
including evicted samples. `sample_history` explicitly reports the versioned
policy, capacities, total/retained/dropped counts, retained first/tail counts,
and whether `samples` is a complete trace. Empty preflight reports carry zero
counts and a zero peak. Existing sample fields and limit priority are unchanged.
Consumers must check this metadata before treating the retained array as a full
timeline or computing whole-run statistics from it. In particular, its maximum
RSS may be lower than `peak_sampled_rss_bytes`.

Cooperative-stop diagnostics retain their original triggering sample separately,
even if it is later evicted from the regular history. Retention does not thin
sampling, suppress a limit check, alter a deadline, or expand any resource cap.
Final JSON encoding needs bounded additional space for the retained records.
The policy caps record count rather than promising an exact Python byte size.

Owned-group RSS still excludes the guard process itself; GPU readings still
measure the whole selected device and are refreshed every two seconds. This
change fixes growing monitor history. It does not establish absence of solver,
driver or allocator leaks, nor capture peaks between samples. Old report files
and historical receipts are not rewritten.

Run the small host-only guard suite from the TL checkout:

```sh
python3 -B -m unittest discover -s tools -p 'test_*bounded*.py' -v
python3 -B -m unittest discover -s tools -p test_cooperative_stop.py -v
```

The helper's smaller test capacities exercise exact rollover and peak eviction;
the command uses the fixed default capacities above. No GPU is needed by these
tests.
