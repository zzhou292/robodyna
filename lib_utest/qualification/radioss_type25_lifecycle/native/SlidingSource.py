"""Complete native sliding/retention routines and unchanged membership blocks."""
from Sources import ROOT, routine


def routines(source):
    donor = source["i25slid.F"]
    return {"PrepSliding1.F": routine(donor, "I25PREP_SLID_1"),
            "PrepSliding2.F": routine(donor, "I25PREP_SLID_2"),
            "KeepContact.F": routine(donor, "I25KEEPF")}


def memberships(source):
    # COMP_2 takes two complete phase snapshots. These are original Fortran
    # loops over source-shaped buffers, not translated production predicates.
    donor = source["i25comp_2.F"]
    start = "      DO I = JTASK, I_STOK_GLO, NTHREAD\n"
    assert donor.count(start) == 2
    blocks = []
    for tail in donor.split(start)[1:]:
        end = "      ENDDO\n"
        assert end in tail
        blocks.append(start + tail.split(end, 1)[0] + end)
    return tuple(blocks)
