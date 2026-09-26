# Original single-list PART surface defaults

2026-09-26. Read-only source closure for the existing authenticated
OriginalSelection chain. No product edits, compiler/native execution or reader
reimplementation. Exact donors are pinned in the adjacent evidence JSON.

The actual selection has three retained evidence blocks: the automatic single
surface contact, plain SET_PART_ADD root, and one SET_PART_LIST_TITLE child.
The source chain is 1000002 -> 5000002 with861 distinct positive PIDs. These are
case observations/test expectations, not shipping lookup constants. The first
binder must require one positive child, exactly that one list, ordinary blank
option/control tails and shared authenticated source/include authority. Existing
OriginalSelection allows a more general additive graph; its successful flattening
alone is not this narrower native surface certificate.

## Native path and resolved options

1. `convertcontacts.cxx:250–261` maps SSTYP2 to the native /SET identity returned
   by GetRadiossSetIdFromLsdSet for SET_PART. It does not set per-clause surface
   orientation/all/edge options. Set-ID remapping is semantic lookup, not a claim
   that original/native numerical IDs are identical.
2. `ConvertSet` uses /SET/GENERAL (`convertsets.h:37–47`). A plain PART list
   selects KEY_type PART (`convertsets.cxx:102–112`) and the ordinary list arm
   creates one clause, fills IDs and idsmax (`:418–434`). It sets no opt_O,
   opt_A or opt_E. TITLE changes naming only for this ordinary list path.
3. The separate SET_PART_ADD conversion (`:448–492`, `:769–833`) uses KEY_type
   SET and maps its positive child IDs. With one positive child and no negative
   generate-range syntax, it creates one ordinary additive SET clause, with no
   orientation/all/edge overrides. The negative range branch is outside scope.
4. `radioss2024/SETS/set_support.cfg:98–110` explicitly defaults opt_D/O/G/B/A/E/I/C
   to0. Independently, `cpp_get_intv.cpp:90–123` writes integer0 when an indexed
   value is unavailable. The complete HM_GET_INT_ARRAY_INDEX wrapper
   (`hm_get_int_array_index.F:177–227`) merely forwards that value; it applies no
   hidden offset or option transformation. Thus absent options resolve O=A=E=0,
   not uninitialized storage or a guessed default.
5. HM_SET reads these exact fields (`hm_set.F:270–280`). SET_OPERATOR chooses
   SET_ADD whenever D and I are not1 (`set_operator.F:65–72`). PART dispatch calls
   CREATE_ELEMENT_FROM_PART then CREATE_SURFACE_FROM_ELEMENT (`hm_set.F:314–334`).
   The latter explicitly starts IEXT=EXT_SURF=1 and changes it only for E=1 or
   A=1 (`create_surface_from_element.F:98–104`). Therefore this clause is genuine
   PART/EXT1 with OPT_O0. EXT1 is not EXT2, and ALL3 is not admitted by this proof.

## Single child is not a flattened multi-list equivalence claim

SET_INIT starts NB_SURF_SEG0 (`set_init.F:91`). For the root SET clause,
CREATE_SET_CLAUSE loops over child sets and invokes INSERT_CLAUSE_IN_SET using
SET_ADD (`create_set_clause.F:410–428`). With exactly one child, the destination
is empty, so INSERT copies SURF_NODES, SURF_ELTYP and SURF_ELEM in their original
order (`insert_clause_in_set.F:673–695`). The root's final insertion is the same
empty-destination copy. No union, reconstruction or new exterior-face extraction
occurs. The only surface-reconstruction arm is SET_DELETE (`:794–836`), absent
here. This preserves the child's already-extracted native faces and sort order.

With multiple children, the nonempty branch calls UNION_SURFACE (`:613–631`).
That merges already-extracted surface arrays; it is not proven equivalent to
extracting once from the union of physical PARTs, because shared-solid-face
removal and first-shell suppression can change. Such graphs, repeated lists,
delete/intersection/generate/collect variants remain explicitly unready in the
first binder. Do not reuse the generic OriginalSelection flattening as proof.

## Zero remeshing and 2D triangle family

`contrl.F:679–682` computes NADMESH as the sum of three actual /ADMESH counts.
The complete admitted source inventory has no adaptive/remesh inputs; all40
cached private converter translation units at the pinned revision emit no
/ADMESH entity. Fresh direct conversion with no native preload/INCLUDE_RADIOSS
therefore has zero counters. Unknown import routes remain unready.

`convertelements.cxx:97–107` maps a repeated-slot ordinary ELEMENT_SHELL to
/SH3N, otherwise /SHELL. The converter mapping does not create /TRIA for this
3D source profile. `contrl.F:644,661–662` counts SH3N and TRIA separately, with
NUMELTRIA specifically the native 2D TRIA family. With the same closed fresh
namespace, NUMELTRIA=0 even though many physical T3 shells exist. Neither a
physical triangle count nor the later combined NUMELTG can establish this zero.

These source controls qualify the bounded numerical extraction already authored
by the surface module. They do not establish early reader storage order,
ambiguous first-shell ownership, physical inclusion beyond named V5, mixed
I25SURFI/SH2 topology, erosion, or runtime admission; those remain separate gates.
