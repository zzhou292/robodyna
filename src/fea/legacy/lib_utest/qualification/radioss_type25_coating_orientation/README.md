# Native coating orientation value

The production leaf is `startup/CoatingOrientation.h`, exposed by the existing
fixed-main startup target. It evaluates the native SEG_INS geometry after shell
membership in a selected solid is established. It owns no mesh, source identity,
contact policy, physical clock, allocation or runtime cache.

Input uses actual ordered native NDS positions, including repeated low-order
slots, and the first three segment corners as indices in that packet. Supported
native counts are 8/10/16/20. Unused tail positions are not read. The caller must
provide the genuine chosen solid and native working coordinates; this operation
does not choose the first incident solid, infer arity, classify an entire surface,
correct node order or establish physical source authority.

The original center sum/division, cross-product operands and left-associated dot
product are preserved. Returned `center_triangle_determinant` is original VOL,
not geometric volume divided by six. Positive VOL selects Reversed; all other
finite values, including either zero sign, select Forward. Nonfinite consumed
inputs/intermediates and unsupported packets reject without changing output.
Existing precise production CMake requirements apply to C++ and CUDA consumers.

Qualification retains the complete pinned original SEG_INS routine. The generator
renames it, adds an observation output declaration and copies its existing VOL
immediately before return. Reversing those instrumentation edits must recover
the original routine byte-for-byte. No arithmetic or branch is rewritten. Its
wrapper supplies genuine membership indices and observes native INS/VOL; repeated
position packets qualify the ordered geometric sum, not original source-ID mapping.
The native routines and Fortran library are qualification-only dependencies.

Four host groups compare 64 shared operand packets, exact sign/determinant bits,
reversal, zero boundaries, unused tails, failures and retry. Two actual CUDA groups
repeat the same inputs across block widths/order and test private output preservation.
A separate production-only consumer checks public linkage/precision without native
or GTest symbols. Owning CMake native/CUDA checks and Bazel header/source checks
must pass before integration. This is not full coating topology or vehicle admission.
