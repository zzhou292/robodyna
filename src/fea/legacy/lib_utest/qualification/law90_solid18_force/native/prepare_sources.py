#!/usr/bin/env python3
"""Authenticate full enclosing donors; stage exact LAW90 selected caller regions."""
import argparse,hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parent
M='engine/source/materials/mat_share/'
S='engine/source/elements/solid/'
SLICES={
 'density_update.inc':(S+'solide/srho3.F',144,149),
 'density.inc':(M+'mmain.F90',693,693),
 'density_reference.inc':(M+'mmain.F90',670,670),
 'reference_density_default.inc':('starter/source/materials/mat/hm_read_mat.F90',1548,1548),
 'average_volume.inc':(M+'mmain.F90',683,683),
 'selected_total.inc':(M+'mulaw.F90',917,924),
 'engineering_strain.inc':(M+'mulaw.F90',984,988),
 'internal_work.inc':(M+'mulaw.F90',2996,3010),
 'stored_energy.inc':(M+'mmain.F90',1997,2003),
 'viscosity_precision.inc':(M+'mqviscb.F',121,127),
 'viscosity_rates.inc':(M+'mqviscb.F',141,153),
 'viscosity_length.inc':(M+'mqviscb.F',179,190),
 'viscosity_pressure.inc':(M+'mqviscb.F',196,220),
 'viscosity_dt.inc':(M+'mqviscb.F',259,262),
 'viscosity_stiffness.inc':(M+'mqviscb.F',381,399),
 'point_length.inc':(S+'solide8e/s8ederi_2.F',132,144),
 'tag_global_default.inc':('starter/source/elements/elbuf_init/ini_mlaw_vars.F',67,69),
 'tag_local_pla_default.inc':('starter/source/elements/elbuf_init/ini_mlaw_vars.F',133,133),
 'tag_local_rate_default.inc':('starter/source/elements/elbuf_init/ini_mlaw_vars.F',138,138),
 'tag_global_zero.inc':('starter/source/elements/elbuf_init/zerovars_auto.F',77,83),
 'tag_local_rate_zero.inc':('starter/source/elements/elbuf_init/zerovars_auto.F',343,343),
 'tag_local_zero.inc':('starter/source/elements/elbuf_init/zerovars_auto.F',349,351),
 'tag_global_apply.inc':('starter/source/elements/elbuf_init/initvars_auto.F',179,184),
 'tag_local_apply.inc':('starter/source/elements/elbuf_init/initvars_auto.F',403,406),
 'tag_local_rate_apply.inc':('starter/source/elements/elbuf_init/initvars_auto.F',411,411),
 'tag_law90.inc':('starter/source/materials/mat/mat090/hm_read_mat90.F',212,213),
}
LEAVES={'s8efint3.F','s8efmoy3.F','s8etotsh10.F','srrota3.F'}
def prepare(output,check):
 m=json.loads((ROOT/'source-manifest.json').read_text())
 if m['revision']!='a62b27e6baa555d222a580d6218867d0be4d70b5':raise ValueError('revision')
 r=m['license'];b=(ROOT/r['path']).read_bytes()
 if len(b)!=r['bytes'] or hashlib.sha256(b).hexdigest()!=r['sha256']:raise ValueError('license')
 data={};names={'constant_mod','precision_mod','elbuftag_mod'}
 for r in m['sources']:
  b=(ROOT/r['path']).read_bytes()
  if len(b)!=r['bytes'] or hashlib.sha256(b).hexdigest()!=r['sha256'] or hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()!=r['git_blob_sha1']:raise ValueError(r['source'])
  value=b.decode('latin1');data[r['source']]=value
  names.update(x.lower() for x in re.findall(r'COMMON\s*/\s*(\w+)\s*/',value,re.I))
  if Path(r['source']).name in LEAVES:
   names.update(x.lower() for x in re.findall(r'^\s*SUBROUTINE\s+(\w+)',value,re.I|re.M))
 outputs={Path(s).name:v for s,v in data.items() if '/share/' in s or '/modules/' in s or Path(s).name in LEAVES}
 for name,(s,first,last) in SLICES.items():outputs[name]=''.join(data[s].splitlines(keepends=True)[first-1:last])
 for name,value in outputs.items():
  for inc in re.findall(r'^\s*#include\s+"([^"]+)"',value,re.M):
   if inc not in outputs:raise ValueError('missing include '+inc+' from '+name)
 tokens=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
 for name,value in outputs.items():
  b=tokens.sub(lambda m:'LAW90_FORCE_'+m.group().upper(),value).encode('latin1');p=output/name
  if check:
   if not p.is_file() or p.read_bytes()!=b:raise ValueError('prepared '+name)
  else:output.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();prepare(a.output,a.check)
