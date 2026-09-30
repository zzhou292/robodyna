# Native contact Gather/Apply batching

Candidate from TL7695f855. Only the final Gather/Apply pair loses an intermediate
control D2H and stream drain. Kernel bodies, all earlier count/capacity fences,
physical arithmetic, enumeration, selectors and persistent storage are unchanged.
ApplyNodes already checks the stable Gather failure key before every force/STI
write; every Gather lane finishes first on the same authenticated owner stream.

The private host AssemblyTail helper preserves the original first-launch error
path. A failed Apply launch is exceptional: cudaPeekAtLastError leaves its host
thread error pending, so clear it before observing earlier Gather completion.
If Gather rejected semantically, return its original report; if the copy/drain
fails, return DeviceFailure and poison; if Gather passed, report/poison the saved
Apply launch error through the existing Fence. This is not a general clearing of
pending errors or a retry of failed device work. Normal success clears nothing.

Five focused CUDA tests compare a literal frozen two-fence tail using unchanged
production Gather/Apply and existing assembly corpus/endpoint/incidence utilities.
They cover exact force/STI/control values, repeated slots and native cohorts,
empty rows, early/late/multiple/overflow failures, poisoned private output,
a real invalid second launch following earlier semantic rejection, repaired
retry, first/second launch failure and copy/drain failure poisoning. A CUDA
transport failure may leave different private attempt arrays after queued Apply;
neither path may publish. Existing full runtime owner suites provide actual
common discard/selector and native-force coverage. No test changes tolerances.

Success saves one Control copy/drain per interface (two per two-interface step).
This is a structural count, not a measured speedup. The frozen API profile shows
pageable D2H waiting; removing only cudaStreamSynchronize would not remove it.
Full vehicle admission, exact archive/retry and fresh timing remain required.

The initial source gate intentionally pins7695 exactly. A later composition with
a separately qualified launch-grid change must use donor-union evidence, not
silently rewrite the original kernel identity proof.
