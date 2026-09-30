"""Complete continuation routines with observation-only return/winner hooks."""
from Sources import ROOT, routine


def routines(source):
    return {"CorContinuation.F": routine(source["i25cor3.F"], "I25COR3_21"),
            "DstContinuation.F": routine(source["i25dst3_21.F"], "I25DST3_21"),
            "GlobContinuation.F": routine(source["i25dst3_1.F"], "I25GLOB")}


def observed(original):
    result = dict(original)
    text = result["DstContinuation.F"]
    assert text.count("      USE TRI7BOX\n") == 1
    text = text.replace("      USE TRI7BOX\n", "      USE TRI7BOX\n      USE SELECTION_OBSERVATIONS\n")
    selected = "        IF(IT/=0.AND.PENT(I,IT)==ZERO) IT=0\n"
    assert text.count(selected) == 1
    text = text.replace(selected, selected + "        continuation_selected(I)=IT\n")
    winner = "            IRTLM(1,N) = MGLOB\n"
    assert text.count(winner) == 1
    text = text.replace(winner, "            continuation_won(I)=1\n" + winner)
    assert text.count("      RETURN\n") == 1
    text = text.replace("      RETURN\n", "      continuation_distance_squared = DD\n"
        "      continuation_ingap = INGAP\n"
        "      continuation_axes(1,:)=IBCX\n"
        "      continuation_axes(2,:)=IBCY\n"
        "      continuation_axes(3,:)=IBCZ\n      RETURN\n")
    result["DstContinuation.F"] = text
    result["ContinuationWrapper.F90"] = (ROOT / "ContinuationWrapper.F90").read_text()
    # The selected local process is assigned explicitly by the wrapper.
    result["task_c.inc"] = "      INTEGER ISPMD\n      COMMON /SELECTION_PROCESS/ ISPMD\n"
    result["lockon.inc"] = ""
    result["lockoff.inc"] = ""
    return result
