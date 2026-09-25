# Inherited source-manifest refresh

The extended-solid manifest in selected base6bb pinned an older
`lib_src/elements/publication/PhysicalStartup.cpp` hash `121dbb02d7971c61ea35745da09bef2fcf5c9b9f79a37470c39e31055d164976`. P4 does not edit that file;
its bytes were compared directly with `git show6bb1cee1:lib_src/elements/publication/PhysicalStartup.cpp`.
The owning manifest now records that unchanged selected-base source at
`c4ea8a50ad4d6d72ca8701361fdba2992e682a855cdb283b4953b4875ca83cca`. No source-proof assertion or numerical oracle
was removed. The failed initial static receipt is preserved as
`solid-parallel-assembly-freeze-1.json`.
