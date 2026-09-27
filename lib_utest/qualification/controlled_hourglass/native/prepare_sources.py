"""Reuse authenticated complete SHOUR_CTL; add reversible read-only observations."""
from pathlib import Path
import importlib.util,argparse,json
HERE=Path(__file__).resolve().parent
QUAL=HERE.parents[1]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);obj=importlib.util.module_from_spec(spec);spec.loader.exec_module(obj);return obj
ic1=module('controlled_source',QUAL/'solid24_icontrol/native/prepare_sources.py')
heph=module('heph_source',QUAL/'solid24_force/native/prepare_sources.py')
def generated():
    meta,prepared=ic1.generated()
    original=prepared['shour_ctl.F90']
    end=original.index('          call IC1_NATIVE_HOUR_WORK(')
    end=original.index('\n',original.index('hy4(i)*hgy4(i)',end))+1
    rate=','.join('hg'+k+str(h)+'(i)' for k in 'xyz' for h in range(1,5))
    force=','.join('h'+k+str(h)+'(i)' for k in 'xyz' for h in range(1,5))
    hook='          call CONTROLLED_LEAF_MODES(i, ['+rate+'], ['+force+'])\n'
    changed=original[:end]+hook+original[end:]
    assert changed.replace(hook,'',1)==original
    prepared['shour_ctl.F90']=changed
    return {n:prepared[n] for n in ['shour_ctl.F90','mvsiz_mod.F90','slot_gs.inc','slot_bulk.inc','slot_parmat.inc','slot_generic.inc','slot_pm107.inc']}
def prepare(output,check):
    heph.prepare(output,check)
    for name,content in generated().items():
        path=output/name;raw=content.encode('latin1')
        if check:assert path.read_bytes()==raw,name
        else:path.write_bytes(raw)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();prepare(a.output,a.check)
    print(json.dumps({'status':'source_passed','native_revision':'a62b27e6baa555d222a580d6218867d0be4d70b5','numerical_rewrites':False,'observation_hook_reversible':True}))
