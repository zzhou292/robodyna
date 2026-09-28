# Contact activity operands

This component stages native contact data after the transaction has authenticated the physical activity snapshot. It does not own an accepted selector, source-content generation, solver clock, force law or contact history. The transaction alone publishes a changed slot with the common physical owner.

The immutable source plan supplies complete parent/node support, all-corner containing parents and a separate emitting-parent/main relation. Emission IDs are one-based expanded main IDs. A newly failed QEPH or T3 parent contributes each declared emission; distinct emitters are counted separately. A main survives I_DEL1 while any eligible active parent contains all its corners. Node support excludes bare mass and constraint membership, exactly as the qualified source plan declares.

Source policy controls secondary removal. I_DEL1 with disconnected-node removal marks orphan coefficients negative and then applies the native zero normalization. I_DEL0 preserves its original coefficient tables. Disabled deletion with enabled solid erosion is rejected as an inconsistent declaration. Secondary output contains coefficients only; the transaction applies them to candidate history without resetting initial_contact_flag.

Signed NB_ELM_M counters are retained per slot. With erosion enabled, native initialization sets them to zero for nonnegative mains, or one plus the final second-support presence for negative mains. The disabled-erosion profile keeps the descriptive counter table at zero. Every affected event decrements the counter when erosion is enabled. The actual negative-to-positive exposure branch remains unsupported, including an intermediate two-to-one crossing inside several events. Complete current support loss takes precedence and removes the main instead. Removed-event multiplicity and newly nonzero-to-zero main counts are separate diagnostics.

Borrowed slot zero belongs to the transaction. The component allocates one alternate operand slot and its own small coefficient/counter tables and GPU source incidence. It uploads the source plan once and retains no duplicate host incidence. A failed stage never writes the selected accepted slot. DiscardStaged requires both the accepted and alternate indices and cannot invalidate the named accepted slot. Source generation changes are decided by the transaction only when derived operand content changes, including real counter changes.

For moving normals, neighbor IDs and their companion edge slots are cleared together and the complete free-main roster is generated in ascending native main order. The fixed profile has no normal/free buffers and reports free_count zero. Main and secondary SI stiffness use the existing explicit unit factors.

Preflight accepts descriptor-only borrowed capacities with null pointers; Initialize checks actual ranges. startup_host_bytes includes retained Plan output, operand-owned host state, upload buffers and a 4 KiB preparation reservation. It excludes Plan construction scratch, snapshot storage and caller source-staging allocations. The caller should compare startup phases instead of adding the retained Plan twice. borrowed_device_bytes is descriptive existing transaction storage; owned_device_bytes is the additional allocation.

Qualification uses the original OpenRadioss FIND_SURFACE_INTER, TAGOFF3N, CHECK_ACTIVE_ELEM_EDGE, CHECK_SURFACE_STATE, CHECK_NODAL_STATE, REMOVE_NEIGHBOUR_SEGMENT and I25FREE_BOUND only as test oracles. Production contains no OpenRadioss linkage. The root transaction qualification supplies real owner/receipt and common-commit tests; this component also has small numerical GPU coupons and exact native output comparisons.

The borrowed view also exposes `main_node_activity` over `node_count` physical
nodes. Each operand slot owns its mask, so a failed stage or discard cannot
change the accepted view. The immutable membership mask is the union of all
expanded main corners, including opposite sides. For I_DEL1, member bytes are
one exactly while any eligible global element supports that physical node;
I_DEL0 and nonmember bytes are always one. This represents native CHKMSR main
roles only. It does not remove nodes, mass, constraints or secondary roles.
`keep_disconnected_nodes` affects secondary coefficients independently and does
not suppress main-role retirement. A changed member mask counts as changed
source content even if all surface coefficients were already zero.

The forecast includes two node-mask slabs and one membership slab (3N device
bytes before alignment), plus two startup upload arrays (2N host bytes before
alignment). Existing `node_active` remains private scratch. The component gate
executes the hash-pinned original CHKMSR3NB prefix through its TYPE25 early
return with a serial barrier adapter, as well as original global TAGOFF3N.
It checks shared supports, unrelated-node neutrality, source changes without
coefficient changes, I_DEL0 retention, exact budgets and discard/failure/retry.
