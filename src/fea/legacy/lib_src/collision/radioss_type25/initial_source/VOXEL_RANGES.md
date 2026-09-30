# Complete native voxel ranges

The qualified x-only sweep reached 2,745,614,538 encounters and 11,060,785
tasks on the retained vehicle, exceeding the unchanged 8,388,608-task cap
before candidate counting. The native grid was 393 by 156 by 126. This
successor changes candidate enumeration only.

Keep the existing native `Cell` calculation, global box, main six-cell bounds,
internal-main exclusion, all pair filters and final `(secondary row, main)`
sort. Sort active secondary nodes by the exact integer flattened native cell
key, with x varying fastest. The existing double key storage represents every
key exactly because the admitted voxel extent is at most 8,000,000.

For each main, visit every (z,y) row in its inclusive native cell box. Binary
search the sorted keys for that row's inclusive x interval. Empty intervals
produce no task. Adjacent occupied intervals with contiguous sorted ordinals
may merge; this adds no node outside the native box. Split each resulting run
at the existing TaskWidth. Count all encounters and tasks before task writes,
retain the existing exclusive scan and task/pair caps, and repeat the same
range traversal to fill tasks. No new device allocation or cap is introduced.

The shared count/scan/fill and canonical pair sorting remain unchanged. A
secondary belongs to exactly one voxel, each visited row is unique, and each
occupied interval is visited once, so this enumerates exactly the former
x sweep after its y/z filter. Task or lane order cannot alter row arithmetic:
the complete pair array is sorted before the existing row producer.

Qualification adds dense overlapping and y/z-separated source packets with
full sorted pair equality against whole native BUC/TRIVOX/STO, plus complete
seed/gap parity. It also checks exact task-cap admission and one-task-short
failure without publishing a seed. Existing source, native and CUDA coupons
remain in the owning gate. Full vehicle census runs only after that gate.
