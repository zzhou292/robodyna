"""Create source-derived native observation ABI and GDB command file; no execution."""
import argparse
import hashlib
import json
from pathlib import Path

from .abi import arguments, verify_common_prefix

ROUTINES = {
    'I25CDCOR3':'engine/source/interfaces/int25/i25mainf.F',
    'I25MAINF':'engine/source/interfaces/int25/i25mainf.F',
    'I25COMP_2':'engine/source/interfaces/int25/i25comp_2.F',
    'I25MAIN_TRI':'engine/source/interfaces/intsort/i25main_tri.F',
    'I25TRIVOX':'engine/source/interfaces/intsort/i25trivox.F',
    'I25COR3_22':'engine/source/interfaces/int25/i25cor3.F',
    'I25DST3_22':'engine/source/interfaces/int25/i25dst3_22.F',
    'I25FOR3':'engine/source/interfaces/int25/i25for3.F',
}


def prepare(donors, common_root, output, app_root, mode="observe"):
    if mode not in ("observe","sequence"):raise ValueError("Unknown native probe mode")
    sources=[]; abi={}
    def read(path):
        data=path.read_bytes()
        sources.append(dict(path=str(path.resolve()),bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
        return data.decode()
    for name, relative in ROUTINES.items():
        abi[name]=arguments(read(donors/relative),name)
    common_fields={
        'COM01':('N2D','NCPRI','IALE','NGROUP','NCYCLE','IRUN','IGER','LBUFEL','IRODDL','IEULER',
                 'IHSH','ITESTV','ITURB','ILAG','ISECUT','IDAMP','IRXDP','NMULT','INTEG8','ISIGI','NSPMD'),
        'COM04':('NUMMAT','NUMNOD'),
        'COM08':('TT','DT1','DT2','DT12','DT2OLD','TSTOP'),
        'PARAM':('NPROPM','NVSIZ','NPROPG','NPARG','LVEUL','NIXFR1','NIXFR2','NPARI'),
        'SCR05':('ICRAY','IRFORM','ITFORM','TH_VERS','IRESP'),
    }
    for name, fields in common_fields.items():
        verify_common_prefix(read(common_root/(name.lower()+'_c.inc')),name,fields)
    # GNU x86-64 ABI pointer positions are independently checked against the
    # earlier native profile probe's NIN stack argument and IPARI register.
    if abi['I25MAINF'][2] != 'IPARI' or abi['I25MAINF'][18] != 'NIN':
        raise ValueError('Main interface ABI differs from authenticated prior probe')
    document={'schema':'robo_dyna.native_scene_observation_abi.v1',
              'platform':'Linux x86-64 little-endian GNU Fortran MYREAL8; entry breakpoints',
              'routines':abi,'source_pins':sources,'common_prefixes':common_fields}
    with (output/'probe-abi.json').open('x') as f:
        json.dump(document,f,indent=2);f.write('\n')
    commands=['set pagination off','set confirm off','set print thread-events off',
              'set disable-randomization off','set language c','python',
              'import sys',f'sys.path.insert(0, {str(app_root.resolve())!r})',
              f'from benchmarks.native_contact_scene.reference.{mode} import install, finish',
              'install("probe-abi.json")','end','run','python','finish()','end']
    if mode=='observe':commands += ['kill']
    commands += ['quit']
    with (output/'inspect.gdb').open('x') as f:f.write('\n'.join(commands)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('donors','common-root','output','app-root'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument("--mode",choices=("observe","sequence"),default="observe")
    args=parser.parse_args()
    prepare(args.donors,args.common_root,args.output,args.app_root,args.mode)
