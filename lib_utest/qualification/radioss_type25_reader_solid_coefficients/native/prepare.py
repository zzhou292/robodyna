#!/usr/bin/env python3
"""Pinned I25GAPM signed exterior/internal arithmetic with actual VOLINT."""
from pathlib import Path
import argparse
import importlib.util
ROOT=Path(__file__).resolve().parent

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result

def generate():
    base=module('reader_coefficients',ROOT.parent.parent/'radioss_type25_coefficients/native/prepare.py')
    geometry=module('reader_geometry',ROOT.parent.parent/'radioss_type25_main_geometry/native/prepare.py')
    donor=base.sources();geo=geometry.source()
    gapm=donor['i25sti3.F'].split('      SUBROUTINE I25GAPM(',1)[1]
    start=gapm.index('            IF (ICONTR==1 ) THEN\n')
    stop=gapm.index('\n          ELSE\n',start)+1
    body=gapm[start:stop]
    assert 'STF(I) = HALF*(STF2+STF1)' in body
    assert body.count('CALL VOLINT(VOL2)')==1
    body=body.replace('CALL VOLINT(VOL2)','CALL RD_READER_VOLUME(XC,YC,ZC,VOL2)')
    start=gapm.index('          GAP_N(1,I)=VOL/AREA\n')
    lengths=gapm[start:gapm.index('C--------Correction',start)]
    sign='          IF(IELEM_M(2,I) > 0) STF(I) = - STF(I)\n'
    assert gapm.count(sign)==1
    volume=geometry.between(geo['volint.F'],'      X17 =','      RETURN')
    text=(ROOT/'Reference.F.in').read_text()
    for key,value in {'CONSTANTS':geometry.constants(geo['constant_mod.F']),
                      'CONTRIBUTION':body,'LENGTH':lengths,'SIGN':sign,'VOLUME':volume}.items():
        assert '@'+key+'@' in text
        text=text.replace('@'+key+'@',value)
    return text
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();text=generate()
    if args.output:
        path=args.output/'Reference.F'
        if args.check:assert path.read_text()==text
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
    print('Original I25GAPM signed exterior/internal branches and complete VOLINT verified')
