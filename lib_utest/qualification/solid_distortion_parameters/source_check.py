"""Authenticate the existing native oracle; never regenerate numerical leaves."""
from pathlib import Path
import argparse,importlib.util,json,subprocess
p=argparse.ArgumentParser();p.add_argument('--oracle-root',required=True,type=Path);a=p.parse_args()
root=a.oracle_root.resolve()
assert subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()=='94fc2db3b3de4d04509d5f5908c02844eb305775'
path=root/'lib_utest/qualification/solid24_icontrol/native/prepare_sources.py'
spec=importlib.util.spec_from_file_location('qualified_icontrol',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
meta,prepared=module.generated()
assert meta['native_revision']=='a62b27e6baa555d222a580d6218867d0be4d70b5'
ini=prepared['sdistror_ini.F90'];criterion=prepared['scre_sig3.F']
for token in ['fqmax = ep02','if (nu > 0.48999)','else if (nu>0.4)',
              'c1 = max(pm(32,mx),pm(100,mx))+onep333*pm(22,mx)',
              'aj2=half*(sig(i,1)**2+sig(i,2)**2+sig(i,3)**2)',
              'll(i) = vol(i)**third','f_es = max(f_min,ep02*es)','f_es = min(one,f_es)']:
    assert token in ini,token
assert 'SMIN = MIN(SIG(I,1),SIG(I,2),SIG(I,3),' in criterion
assert 'IF (SIG_C<-SMIN) ISTAB(I) = 1' in criterion
assert 'IF (OFFG(I)==ZERO) CYCLE' in criterion
constants=root/next(row['path'] for row in meta['reused_sources'] if row.get('source')=='common_source/modules/constant_mod.F')
text=constants.read_text()
for token in ['ONEP333 = ONEP33  + THREEEM3','ONEP33  = ONEP3  + ZEP03',
              'ONEP3   = ONE + ZEP3','ZEP3      = THREE  / TEN',
              'ZEP03     = THREE  / EP02','THREEEM3  = THREE  / EP03']:
    assert token in text,token
print(json.dumps({'status':'source_passed','native_revision':meta['native_revision'],
 'oracle_checkout':'94fc2db3','fqmax_source_assignment':100,'fqmax_observed_by_probe':False,
 'native_component_criterion':True,'production_profile_enabled':False,
 'compiled':False,'executed':False}))
