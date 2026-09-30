# Independent early surface oracle

The wrapper executes complete pinned CREATE_ELEMENT_FROM_PART,
CREATE_SURFACE_FROM_ELEMENT, SURFACE_BUFFER, SOLID_SURFACE_BUFFER,
SURF_SEGMENT and SHELL_SURFACE_BUFFER, followed by original MY_ORDERS.
BUILD_CNEL raw8/Q4/T3 count, prefix and corner-major fill blocks are copied
exactly. Bounds are 256 nodes, 64 rows per family, 256 parts and 512 emitted faces.
PENTA is its declared pre-INITIA raw8 reader expansion within native NUMELS8.
Higher-order solids, QUAD families and remeshing stay unselected; their external
calls have explicit fatal diagnostics, rather than numerical substitutes.

C++ packs genuine supplied reader tables and dense internal PART identities.
The Fortran wrapper constructs only the inverse PART roster in source row order;
original CREATE_ELEMENT_FROM_PART performs selection and sorting. A SOLID clause
supplies its admitted sorted unique original rows directly. No production
surface math or production Build/Preflight result participates in expected data.

Read-only hooks observe JJ/JS and the destination buffer address before each
original SURF_SEGMENT call, plus INDEX after original five-word sorting. These
recover raw solid face ordinal and original buffer ordinal without reconstructing
geometry in C++. Unused observation channels are API-zero, not native scratch
observations. External EID/PID restoration uses the actual returned family and
family-local ELEM only. Emitted-solid flags are an explicit observation that a
native segment was emitted; no FLAG_ELEM_INTER25 argument exists here. This does
not claim the later I25GAPM erosion disposition or grant source/runtime authority.

COMMON state and observation storage are serialized by the C++ adapter. All
original donors retain their byte hashes, Git blobs and AGPL attribution.
Source generation/check is separate from the owning compiler/native test gate.
