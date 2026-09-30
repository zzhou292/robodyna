# Shared native spring frame and projection

These small value helpers retain the R4EVEC3 frame transport and R4CUM3 endpoint
projection previously qualified by TYPE25. Their arguments use internally
consistent caller units; they select no material, mass, damping or clock policy.
TYPE25 and TYPE13 retain their own public input admission and staged outputs.

Source provenance remains in `lib_utest/qualification/type25/native`; TYPE13's
recurrence verifier also verifies those complete source/extraction hashes.
Adapted OpenRadioss source is AGPL-3.0-or-later, Copyright (C) 2026 Siemens;
see [the license](../type13/LICENSE.md).
