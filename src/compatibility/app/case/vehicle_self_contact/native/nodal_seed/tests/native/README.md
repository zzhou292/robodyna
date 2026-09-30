# Private LAW42 contact-slot reference

This qualification-only target reuses the original GS/PARMAT/generic statements
already extracted by TL solid_law42_caller NativeInitial. The generic slice now
extends through PM32. It additionally executes the unchanged bulk/PM100 reader
statements, final reader LAW42 exception, the complete conditional LAW42_UPD
parameter-write block with its original formats, and UPDMAT's final PM107 update.
The remainder of LAW42_UPD's stability scan has no PM write and is not compiled.

The wrapper explicitly selects one Ogden term, alpha2, nu.463 and no Prony.
The original GammaInf=ONE caller statement is retained; no unsupported Prony
or scalar-reader validation branch is inferred. Positive finite native mu is
required. All four returned fields are observed original array slots, not
computed from production Parameters or a translated expected expression.

`robo_dyna_law42_contact_slots_native` compiles generated Constants.F90 and
ContactSlots.F. The C ABI is `law42_contact_slots_native(parameters,slots)`;
inputs are `{mu_native,nu}`, outputs `{PM20,PM32,PM100,PM107}`. Call only from
qualification. It provides no original-case identity or mechanical admission.
