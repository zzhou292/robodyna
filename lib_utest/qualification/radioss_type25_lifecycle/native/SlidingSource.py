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


def phase_blocks(source):
    clear = "        INTBUF_TAB(NIN)%ISLIDE(4*NSNF+1:4*NSNL)=0\n"
    assert source["i25main_slid.F"].count(clear) == 1
    main = source["i25mainf.F"]
    start = "      DO N=NSNFT, NSNLT\n        IF(INTBUF_TAB%IRTLM(4*(N-1)+1) < 0) \n"
    assert main.count(start) == 1
    finish = start + main.split(start, 1)[1].split("      END DO\n", 1)[0] + "      END DO\n"
    return clear, finish
