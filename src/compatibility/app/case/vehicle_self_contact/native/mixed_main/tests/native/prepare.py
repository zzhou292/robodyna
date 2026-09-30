#!/usr/bin/env python3
"""Independent original I25GAPM shell MAX/partner overlay on native solid output."""
import argparse,importlib.util
from pathlib import Path
def generate(tl):
    path=tl/'lib_utest/qualification/radioss_type25_coated_coefficients/native/prepare.py'
    spec=importlib.util.spec_from_file_location('post_gapm_coating_source',path)
    original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
    text=original.generate() # Authenticates the complete original donor chain.
    text=text.replace('RD_COATED_COEFFICIENT','RD_POST_GAPM_SHELL_OVERLAY')
    start=text.index('            IF (ICONTR==1 ) THEN\n')
    stop=text.index('      IF(F(3)==3)THEN',start)
    # Caller passes the independently native-evaluated solid stage. Replace
    # only that complete earlier stage, keeping each original shell arm and
    # encoded partner write unchanged. No C++ expected numeric formula.
    text=text[:start]+'      STF(1)=V(5)\n      GAP_N(1,1)=V(6)\n'+text[stop:]
    text=text.replace('      MSEGTYP(1)=4','      IF(F(4)/=0)MSEGTYP(1)=4')
    assert text.count('STF(I)=MAX(STF(I),STC)')==2
    assert text.count('STF(J)  = STC')==2
    return text
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--tl-root',type=Path,required=True)
    p.add_argument('--output',type=Path);p.add_argument('--check',action='store_true');a=p.parse_args();text=generate(a.tl_root)
    if a.output:
        out=a.output/'Overlay.F'
        if a.check:assert out.read_text()==text
        else:out.parent.mkdir(parents=True,exist_ok=True);out.write_text(text)
    print('Original shell overlay and encoded partner blocks verified')
