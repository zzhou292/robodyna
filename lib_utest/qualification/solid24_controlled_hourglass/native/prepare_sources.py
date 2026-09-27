"""Authenticate complete H24 sources; observe intermediate values without changing mechanics."""
from pathlib import Path
import argparse,importlib.util,json,hashlib
HERE=Path(__file__).resolve().parent;QUAL=HERE.parents[1]
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod
ic1=load('ic1_adapter_sources',QUAL/'solid24_icontrol/native/prepare_sources.py')
def insert(text,marker,hook):
 assert text.count(marker)==1,marker
 changed=text.replace(marker,marker+hook,1);assert changed.replace(hook,'',1)==text
 return changed
def generated():
 _,result=ic1.generated()
 shour=result['shour_ctl.F90'];end=shour.index('          call IC1_NATIVE_HOUR_WORK(')
 end=shour.index('\n',shour.index('hy4(i)*hgy4(i)',end))+1
 rate=','.join('hg'+k+str(h)+'(i)' for k in 'xyz' for h in range(1,5))
 force=','.join('h'+k+str(h)+'(i)' for k in 'xyz' for h in range(1,5))
 hook='          call H24_ADAPTER_MODES(i, ['+rate+'], ['+force+'])\n'
 changed=shour[:end]+hook+shour[end:];assert changed.replace(hook,'',1)==shour;result['shour_ctl.F90']=changed
 pins=json.loads((HERE/'wrapper-pins.json').read_text())
 for name,expected in pins.items():
  raw=(QUAL/name).read_bytes();assert hashlib.sha256(raw).hexdigest()==expected,name
  original=raw.decode();text=original
  if name.endswith('NativeCaller.F90'):
   text=insert(text,'  use IC1_NATIVE_OBSERVATIONS\n','  use H24_ADAPTER_OBSERVATIONS\n')
   text=insert(text,'  stages=0;status=1;center_contacts=0;corner_contacts=0\n','  call H24_RESET()\n')
   text=insert(text,'  stages=1\n','  call H24_GEOMETRY(geometry)\n')
   text=insert(text,'  hg_sti=sti(1);stages=7\n','  call H24_FORCE(force,0,2)\n')
   text=insert(text,'  stages=31\n','  call H24_FORCE(force,48,4)\n')
   text=insert(text,'  values=result;status=0\n','  call H24_PUBLISH()\n')
  else:
   text=insert(text,'      USE HEPH_NATIVE_PACKETS\n','      USE H24_ADAPTER_OBSERVATIONS\n')
   text=insert(text,'     . SVIS)\n','      CALL H24_FORCE(F,24,3)\n')
  result[Path(name).name]=text
 return result
def prepare(out,check):
 result=generated()
 if not check:out.mkdir(parents=True,exist_ok=True)
 for name,value in result.items():
  path=out/name;raw=value.encode('latin1')
  if check:assert path.read_bytes()==raw,name
  else:path.write_bytes(raw)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();prepare(a.output,a.check)
 print(json.dumps({'status':'source_passed','native_revision':'a62b27e6baa555d222a580d6218867d0be4d70b5','numerical_rewrites':False,'full_native_sequence_retained':True}))
