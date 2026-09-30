"""Prepare a source-pinned own-child Starter mass/coefficient observation."""
import argparse
import hashlib
import json
from pathlib import Path

from .abi import arguments, verify_common_prefix


def prepare(donors, output, app_root):
    pins=[]
    def read(relative):
        path=donors/relative
        data=path.read_bytes()
        pins.append(dict(path=str(path.resolve()),bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
        return data.decode()
    routines={name:arguments(read('starter/source/elements/initia/'+file),name)
              for name,file in (('INITIA','initia.F'),('SPMD_MSIN','spmd_msin.F'))}
    verify_common_prefix(read('starter/share/includes/com04_c.inc'),'COM04',('NUMMAT','NUMNOD'))
    document={'schema':'robo_dyna.native_startup_observation_abi.v1',
              'platform':'Linux x86-64 little-endian GNU Fortran MYREAL8',
              'routines':routines,'source_pins':pins}
    with (output/'startup-abi.json').open('x') as stream:
        json.dump(document,stream,indent=2);stream.write('\n')
    commands=['set pagination off','set confirm off','set print thread-events off',
              'set disable-randomization off','set language c','python','import sys',
              f'sys.path.insert(0, {str(app_root.resolve())!r})',
              'from benchmarks.native_contact_scene.reference.startup import install, finish',
              'install("startup-abi.json")','end','run','python','finish()','end','quit']
    with (output/'inspect-startup.gdb').open('x') as stream:stream.write('\n'.join(commands)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('donors','output','app-root'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    prepare(args.donors,args.output,args.app_root)
