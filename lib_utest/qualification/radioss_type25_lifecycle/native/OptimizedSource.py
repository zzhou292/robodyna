"""Complete OPTCD and its caller's source-exact release marker block."""
from Sources import routine


def original(source):
    return routine(source["i25optcd.F"], "I25OPTCD")


def observed(text):
    # Observe original retained ordinals at the actual successful array write;
    # repeated equal pairs must not be matched after the fact by their values.
    assert text.count("      USE TRI7BOX\n") == 1
    text = text.replace("      USE TRI7BOX\n",
        "      USE LIFECYCLE_FOREIGN\n      USE LIFECYCLE_OBSERVATIONS\n")
    point = "          cand_opt_n(next) = cand_n(i)\n"
    assert text.count(point) == 1
    text = text.replace(point, "          optcd_ordinal(next) = i\n" + point)
    capacity = "      if(i_opt_stok+offset(nthread+1)<=sizopt) then \n"
    assert text.count(capacity) == 1
    return text.replace(capacity,
        "      optcd_required_count = i_opt_stok+offset(nthread+1)\n" + capacity)


def release(source):
    text = source["i25main_opt_tri.F"]
    start = "      DO N = NSNF,NSNL \nC       release node for future impact (at next cycles)\n"
    assert text.count(start) == 1
    return start + text.split(start, 1)[1].split("      ENDDO\n", 1)[0] + "      ENDDO\n"
