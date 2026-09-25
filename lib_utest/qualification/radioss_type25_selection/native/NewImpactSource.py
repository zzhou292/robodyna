"""Complete new-impact stage and transitive native linear-intersection helpers."""
from Sources import ROOT, routine


def routines(source):
    original = source["i25dst3_22.F"]
    result = {"CorNewImpact.F": routine(source["i25cor3.F"], "I25COR3_22"),
              "DstNewImpact.F": routine(original, "I25DST3_22"),
              "GlobNewImpact.F": routine(original, "I25GLOB_22")}
    for name in ["INTERSECA_25", "INTERSECB_25", "INTERSECV0_25"]:
        result[name + ".F"] = routine(original, name)
    return result


def observed(original):
    result = dict(original)
    text = result["DstNewImpact.F"]
    assert text.count("      USE TRI7BOX\n") == 1
    text = text.replace("      USE TRI7BOX\n", "      USE TRI7BOX\n      USE SELECTION_OBSERVATIONS\n")
    entry = "      DO I=1,JLT\nC\n        IF(STIF(I) <= ZERO)CYCLE\nC\n        IA    = SUBTRIA(I)"
    assert text.count(entry) == 1
    text = text.replace(entry,
        "      impact_side_selector(1,:)=SUBTRIA\n"
        "      impact_side_selector(2,:)=SUBTRIB\n" + entry)
    primary = "          L      = CAND_E(I)\n"
    opposite = "          L         = ISHEL(I)\n"
    winner = "            IRTLM(1,N) = -MGLOB\n"
    for needle in [primary, opposite, winner]:
        assert text.count(needle) == 1, needle
    text = text.replace(primary, "          impact_side_choice(I)=1\n" + primary)
    text = text.replace(opposite, "          impact_side_choice(I)=2\n" + opposite)
    text = text.replace(winner, "            impact_won(I)=1\n" + winner)
    assert text.count("      RETURN\n") == 1
    text = text.replace("      RETURN\n", "      impact_raw_lb=LB\n      impact_raw_lc=LC\n"
        "      impact_distance_squared=DD\n      impact_primary_far=FARA\n"
        "      impact_opposite_far=FARB\n      impact_primary_gap=INGAPA\n"
        "      impact_opposite_gap=INGAPB\n      impact_primary_penetration=PENA\n"
        "      impact_opposite_penetration=PENB\n      impact_intersection(1,:)=INTERSECTA\n"
        "      impact_intersection(2,:)=INTERSECTB\n      impact_recontact=ICONT_R\n      RETURN\n")
    result["DstNewImpact.F"] = text
    result["NewImpactWrapper.F90"] = (ROOT / "NewImpactWrapper.F90").read_text()
    result["com08_c.inc"] = "      REAL(C_DOUBLE) DT1\n      COMMON /SELECTION_CLOCK/ DT1\n"
    return result
