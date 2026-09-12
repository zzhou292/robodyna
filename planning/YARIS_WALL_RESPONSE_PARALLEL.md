# Ordered mapped wall response

Selected from the measured original V5 wall profile: CheckResponse mean 0.558229 s versus Scatter 0.168955 s and Evaluate 0.1036 s. Diagnostic archive/viewer hashes exactly match baseline; diagnostic code remains outside production.

The bounded implementation preserves each rigid group's original compact-node trace order using startup CSR. Independent ordinary nodes compute the exact original outward product and reduce finite nonnegative maxima. Every failure runs the complete frozen serial response before the unchanged step check, preserving first-error and partial output behavior. Activity, scatter, contact law and transaction semantics are unchanged.

At 359785/779 it adds 1464752 B per host/device arena, plus existing sizeof-based private metadata accounting, without a new allocation. See [qualification and root commands](../lib_utest/qualification/mapped_wall_response/README.md). Author host/source checks pass; native/CUDA/full-source performance qualification belongs to root.
