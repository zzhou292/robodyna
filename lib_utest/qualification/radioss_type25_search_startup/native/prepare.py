#!/usr/bin/env python3
"""Independent complete serial source initialization; no production math read."""
import argparse
from pathlib import Path
from Sources import ROOT,read,routine,between,constants

def generated():
    source=read()
    removal=source["get_list_remnode.F90"]
    margin=source["margin.F90"]
    init=routine(source["i7remnode.F"],"I7REMNODE_INIT")
    inverse=routine(source["i7remnode.F"],"I25REMNOR")
    body=routine(source["i25buc_vox1.F"],"I25BUC_VOX1")
    first="      DD=ZERO\n"
    last="      MARGE = BUMULT*DD\n"
    assert body.count(first)==1 and body.count(last)==1
    margin_block=body[body.index(first):body.index(last)+len(last)]
    reader=source["hm_read_inter_type25.F"]
    multiplier=between(reader,"        BUMULT=ZERO  \n","        FRIGAP(4)=BUMULT")
    machine=source["machine.inc"]
    base=next(line for line in machine.splitlines() if line.strip().startswith("BMUL0") and "=" in line)
    assert base.split("!",1)[0].strip()=="BMUL0 = 0.20"
    xsave=source["i25xsave.F90"]
    extent=between(xsave,"          c_max = zero\n","          return\n")
    all_math=[removal,margin,init,inverse,margin_block,multiplier,base,extent]
    output={"Constants.F90":constants(source["constant_mod.F"],all_math).replace("selection_constants","constant_mod"),
      "Boundary.F90":(ROOT/"Boundary.F90").read_text(),
      "Removal.F90":removal,"SmallMargin.F90":margin,"RemovalInit.F":init,"Inverse.F":inverse,
      "Wrapper.F90":(ROOT/"Wrapper.F90").read_text()}
    text=(ROOT/"Margin.F.in").read_text()
    for name,value in [("MARGIN",margin_block),("BASE",base+"\n"),("MULTIPLIER",multiplier)]:
        assert text.count("@"+name+"@")==1;text=text.replace("@"+name+"@",value)
    output["Margin.F"]=text
    text=(ROOT/"Extent.F90.in").read_text();assert text.count("@EXTENT@")==1
    output["Extent.F90"]=text.replace("@EXTENT@",extent)
    output["implicit_f.inc"]="      USE ISO_C_BINDING\n      USE constant_mod\n      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n"
    # Initial allocation writes zeros; selected INACTI5 has no ICONT assignment.
    allocated=source["intbuf_ini_starter.F"]
    start='          CALL MY_ALLOC(INTBUF_TAB(NIN)%ICONT_I,INTBUF_TAB(NIN)%S_ICONT_I,"INTBUF_TAB(NIN)%ICONT_I")\n'
    stop='          INTBUF_TAB(NIN)%ICONT_I(1:INTBUF_TAB(NIN)%S_ICONT_I) = 0 '
    assert allocated.count(start)==1 and allocated.count(stop)==1
    initialize=allocated[allocated.index(start):allocated.index(stop)+len(stop)]+"\n"
    text=(ROOT/"InitialFlags.F.in").read_text();assert text.count("@INITIALIZE@")==1
    output["InitialFlags.F"]=text.replace("@INITIALIZE@",initialize)
    pwr=routine(source["i25pwr3.F"],"I25PWR3")
    selected=between(pwr,"             ELSE IF(INACTI==5) THEN\n","             ELSE IF(INACTI==-1) THEN")
    assert "ICONT_I" not in selected
    return output
if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path);parser.add_argument("--check",action="store_true")
    args=parser.parse_args()
    for name,text in generated().items():
        if args.output:
            target=args.output/name
            if args.check:assert target.read_text()==text
            else:target.parent.mkdir(parents=True,exist_ok=True);target.write_text(text)
    print("Pinned native serial removals, expanded-main margin, extent and flag branch prepared")
