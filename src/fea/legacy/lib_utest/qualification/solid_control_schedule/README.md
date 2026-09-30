# Solid control schedule host gate

Build this CMake directory for the immutable source-selection/schedule tests and
legacy/extended model regressions. No CUDA or dynamics run is required. The
synthetic six-parent fixture deliberately uses a different native member order
than TL family order, two H24 parents in one packet, all five current families,
and explicit controlled/uncontrolled rows. It is not an actual Yaris packet
export. Source-declared physics remains blocked by the existing resident planner.
