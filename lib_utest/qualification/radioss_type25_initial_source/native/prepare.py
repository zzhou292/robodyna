"""Whole pinned Starter inventory; expected data only, never production arithmetic."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
PREFIX = 'initial_inventory_'
SYMBOLS = ('I25BUC_VOX1', 'INSOL25', 'I25TRIVOX1', 'I25STO', 'I25S1S2',
           'I25COR3T', 'I25PEN3A', 'TRI7BOX', 'MESSAGE_MOD', 'NAMES_AND_TITLES_MOD',
           'INTBUFDEF_MOD', 'MARGIN_REDUCTION_MOD', 'MY_ALLOC_MOD', 'MY_DEALLOC_MOD',
           'MY_MOVE_ALLOC_MOD', 'ARRAY_MOD', 'ELEMENT_MOD', 'PRECISION_MOD',
           'CONSTANT_MOD', 'UPGRADE_MULTIMP', 'OMP_GET_THREAD_NUM', 'OMP_GET_NUM_THREADS',
           'MY_ORDERS', 'MY_ORDERS_', 'MY_ORDERS__', 'TRI_DIRECT', 'BITGET', 'PREPARE_INT25', 'FRONT_MOD')


def rename(text):
    pattern = r'\b('+'|'.join(SYMBOLS)+r')\b'
    changed = re.sub(pattern, lambda match: PREFIX+match.group(), text, flags=re.I)
    restored = re.sub(r'\b'+PREFIX+r'('+ '|'.join(SYMBOLS)+r')\b',
                      lambda match: match.group(1), changed, flags=re.I)
    assert restored == text, 'Numerical bodies allow reversible symbol renames only'
    return changed


def generated(tl):
    helper = tl/'lib_utest/qualification/radioss_type25_selection/native/Sources.py'
    spec = importlib.util.spec_from_file_location('initial_inventory_sources', helper)
    sources = importlib.util.module_from_spec(spec); spec.loader.exec_module(sources)
    donors = {}
    for item in json.loads((ROOT/'source-manifest.json').read_text())['files']:
        raw = (ROOT/item['path']).read_bytes()
        assert len(raw) == item['bytes']
        assert hashlib.sha256(raw).hexdigest() == item['sha256']
        assert hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest() == item['git_blob']
        donors[Path(item['path']).name] = raw.decode()
    bodies = {}
    for routine, file in [('I25BUC_VOX1','i25buc_vox1.F'),('INSOL25','i25buc_vox1.F'),
                          ('I25TRIVOX1','i25trivox1.F'),('I25STO','i25sto.F'),('I25S1S2','i25sto.F'),
                          ('I25COR3T','i25cor3t.F'),('I25PEN3A','i25pen3a.F'),('PREPARE_INT25','build_cnel.F')]:
        bodies[routine+'.F'] = sources.routine(donors[file], routine)
    bodies['SmallMargin.F90'] = donors['margin.F90']
    bodies['Tri7box.F'] = donors['tri7box.F']
    bodies['Sort.c'] = donors['my_orders.c']
    out = {name: rename(body) for name, body in bodies.items()}
    constants = sources.constants(donors['constant_mod.F'], [v.upper() for v in bodies.values()])
    out['Constants.F90'] = constants.replace('selection_constants', PREFIX+'constant_mod')
    mvsiz = '       INTEGER MVSIZ\n       PARAMETER (MVSIZ = 512)\n'
    assert mvsiz in donors['starter_mvsiz_p.inc']; out['mvsiz_p.inc'] = mvsiz
    assert 'PARAMETER(LVOXEL = 8000000)' in donors['tri7box.F']
    base = [line.split('!', 1)[0].strip() for line in donors['machine.inc'].splitlines()
            if line.strip().startswith('BMUL0') and '=' in line]
    assert base == ['BMUL0 = 0.20']
    out['implicit_f.inc'] = ('      USE ISO_C_BINDING\n      USE '+PREFIX+'constant_mod\n'
                            '      IMPLICIT NONE\n#define my_real REAL(C_DOUBLE)\n')
    out['com04_c.inc'] = ('      INTEGER NUMNOD,NUMELS,NUMELS8,NUMELS10,NUMELS16,NUMELS20,NPART,NUMNOR,NINTER25,NSNT25,NRTMX25\n'
        '      COMMON /INITIAL_INVENTORY_COUNTS/ NUMNOD,NUMELS,NUMELS8,NUMELS10,NUMELS16,NUMELS20,NPART,NUMNOR,NINTER25,NSNT25,NRTMX25\n')
    out['com01_c.inc'] = '      INTEGER,PARAMETER::NINTER=1\n'
    out['spmd_c.inc'] = '      INTEGER,PARAMETER::NSPMD=1\n'
    out['scr06_c.inc'] = '      REAL(C_DOUBLE) BMUL0\n      PARAMETER(BMUL0=0.20)\n'
    out['units_c.inc'] = '      INTEGER,PARAMETER::IOUT=6\n'
    # MVSIZ is storage512. The selected GNU/Linux default batch width is128:
    # machine IBUILTIN18 -> ARCHINFO(18,1) -> CONTRL NVSIZ, absent -grp_size.
    assert re.search(r'\bIBUILTIN\s*=\s*18\b', donors['machine.inc'])
    assert re.search(r'\bARCHINFO\(18,1\)\s*=\s*128\b', donors['archloops.inc'])
    assert re.search(r'\bNVSIZ\s*=\s*ARCHINFO\(IBUILTIN,1\)', donors['contrl.F'], re.I)
    assert 'NPARI' in donors['param_c.inc'] and 'NVSIZ' in donors['param_c.inc']
    out['param_c.inc'] = '      INTEGER,PARAMETER::NPARI=200,NVSIZ=128\n'
    start=donors['inint3.F'].index('         NGROUS=1+(I_STOK-1)/NVSIZ')
    stop=donors['inint3.F'].index('           IWPENE0 = 0',start)
    caller=donors['inint3.F'][start:stop]
    assert '           DO NG=1,NGROUS' in caller and 'CALL I25COR3(' in caller and 'CALL I25PEN3(' in caller
    assert caller.rindex('ENDDO') > caller.index('CALL I25PEN3(')
    assert 'CALL I25PWR3(' in donors['inint3.F'][stop:stop+100]
    for name in ('Boundary.F90','Wrapper.F90'):
        out[name] = rename((ROOT/name).read_text())
    out['FullHistory.F90']=(ROOT/'FullHistory.F90').read_text()
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tl-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    for name, text in generated(args.tl_root).items():
        path = args.output/name
        if args.check:
            assert path.read_text() == text
        else:
            path.parent.mkdir(parents=True, exist_ok=True); path.write_text(text)
    print('Whole native BUC/TRIVOX/STO/COR3T/PEN3A and native sort prepared; source arithmetic unchanged')
