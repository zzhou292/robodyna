#!/usr/bin/env python3
"""Complete pinned local normal activation and free-roster native reference."""
import argparse,re
from pathlib import Path
from Sources import ROOT,read,routine,constants

def generated():
    source=read()
    tag=routine(source["i25norm.F"],"I25TAGN")
    free=routine(source["i25free_bound.F"],"I25FREE_BOUND")
    def namespace(text):
        text=text.replace("USE INTBUFDEF_MOD","USE NA_REMOTE").replace("USE TRI7BOX","USE NA_SORT").replace(
          "use nodal_arrays_mod","use na_nodes").replace("use my_alloc_mod","use na_memory").replace(
          "use my_dealloc_mod, only : my_dealloc","use na_memory, only : my_dealloc").replace(
          "CALL MY_BARRIER","CALL NA_BARRIER")
        return re.sub(r"\bmy_barrier\b","na_barrier",text,flags=re.I)
    result={"Tag.F":namespace(tag),"Free.F":free,
      "Constants.F90":constants(source["constant_mod.F"],[tag.upper(),free.upper()]).replace("selection_constants","na_constants"),
      "Boundary.F90":(ROOT/"Boundary.F90").read_text(),"Wrapper.F90":(ROOT/"Wrapper.F90").read_text(),
      "i25edge_c.inc":source["i25edge_c.inc"],
      "implicit_f.inc":"      USE ISO_C_BINDING\n      USE na_constants\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n",
      "param_c.inc":"      INTEGER,PARAMETER::NLEDGE=15\n",
      "com01_c.inc":"      INTEGER NSPMD,NINTER25,ISPMD\n      COMMON /NA_PARTITION/NSPMD,NINTER25,ISPMD\n",
      "com04_c.inc":"      INTEGER NUMNOD,NUMELS\n      COMMON /NA_COUNTS/NUMNOD,NUMELS\n",
      "task_c.inc":"      INTEGER NTHREAD\n      COMMON /NA_THREADS/NTHREAD\n"}
    return result
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path);p.add_argument("--check",action="store_true");a=p.parse_args()
    for name,text in generated().items():
        if a.output:
            target=a.output/name
            if a.check:assert target.read_text()==text
            else:target.parent.mkdir(parents=True,exist_ok=True);target.write_text(text)
    print("Complete native FREE_BOUND and local TAGN reference prepared")
