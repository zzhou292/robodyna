# Explicit moving-main source declaration

Version1 remains the qualified fixed-wall primary surface plus additional moving
secondary-node group. Its exported scene fields and native deck bytes remain
unchanged. Version2 requires contact_surface=all_shells and exports every wall
triangle and patch quadrilateral in the same finite SURF/SEG roster. It retains
the genuine ILEV1 node union, source IDs, material, geometry and time controls.
This adds moving main geometry; it is not a second solver or timestep change.

Version2 has a distinct exported schema. The C++ source reader retains its typed
surface selection and checks agreement with the original declaration. Existing
fixed ContactSource rejects it before numerical source allocation. Structural
PhysicalSource remains independent of contact selection. A separately admitted
moving-main factory/runtime is required before the new scene can run in TL.

All-shell selection changes native secondary thickness ownership: patch nodes
now lie on the selected main surface. New native source/history observations are
therefore required. Old fixed-wall histories and coefficients must not be reused
as the reference for this new declared profile. No moving-contact qualification
or full vehicle claim follows from successfully writing or reading this source.
