# Parallel post-node CIN input checks

The 101-step full-vehicle profile measured 11.27 ms per CompleteInputs launch.
Witness and row checks are independent before any transfer or entry-inertia copy.
This change keeps pointer, node, numerical-mass, witness, and row phase priority.
Each row executes the existing checks in order; integer minimum selects the first
failing row, whose unchanged leaf is evaluated once to obtain its exact report.
The original public serial function reuses those leaves in the same order.
No allocation, floating-point reduction, physical state, or payload limit changes.

The exact 90a4a806 force header is retained as a source-proof baseline. Existing
frozen full-caller host/CUDA tests are reused, with 129-row boundary and combined
fault cases. Historical source-identity manifests already differ at the baseline
because their old nonempty-only header predates explicitly-empty CIN support;
the new proof does not modify those donors or erase historical receipts.

Root qualification: configure with CIN_POST_NODE_INPUTS_CUDA=ON, build, and run
all three CTest targets. Host checks total six GTests; CUDA checks total four.
