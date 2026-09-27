# Authenticate complete S6 control/assembly and compose qualified native caller bindings.
from pathlib import Path
import argparse,hashlib,json,re
HERE=Path(__file__).resolve().parent;QUAL=HERE.parents[1]
def generated():
 meta=json.loads((HERE/'source-manifest.json').read_text());assert meta['native_revision']=='a62b27e6baa555d222a580d6218867d0be4d70b5'
 out={}
 for row in meta['owned_sources']:
  data=(HERE/row['path']).read_bytes();assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
  assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()==row['git_blob_sha1']
  text=data.decode('latin1')
  names={'s6chour_ctl_mod':'S6_CONTROL_NATIVE_MOD','s6chour_ctl':'S6_CONTROL_NATIVE','shour_ctl_mod':'IC1_NATIVE_SHOUR_CTL_MOD','shour_ctl':'IC1_NATIVE_SHOUR_CTL','constant_mod':'HEPH_NATIVE_CONSTANT_MOD','precision_mod':'HEPH_NATIVE_PRECISION_MOD','mvsiz_mod':'IC1_NATIVE_MVSIZ_MOD','s6cumu3':'S6_CONTROL_ASSEMBLY_NATIVE'}
  pat=re.compile(r'\b(?:'+'|'.join(sorted(names,key=len,reverse=True))+r')\b',re.I)
  converted=pat.sub(lambda m:names[m.group().lower()],text)
  back=converted
  for original,replacement in sorted(names.items(),key=lambda v:len(v[1]),reverse=True):back=re.sub(r'\b'+replacement+r'\b',original,back,flags=re.I)
  assert back.lower()==text.lower()
  if row['source'].endswith('s6chour_ctl.F90'):
   anchor='          f14(1:nel) =zero';assert converted.count(anchor)==1
   values=','.join('px'+str(n)+'h'+str(h)+'(i)' for n in range(1,5) for h in range(1,4))
   hook='          do i=1,nel\n            call S6_CONTROL_PROJECT(i,['+values+'])\n          enddo\n'
   observed=converted.replace(anchor,hook+anchor,1);assert observed.replace(hook,'',1)==converted;converted=observed
  out[Path(row['source']).name]=converted
 wrappers={}
 for path,digest in meta['wrapper_pins'].items():
  raw=(QUAL/path).read_bytes();assert hashlib.sha256(raw).hexdigest()==digest,path;wrappers[Path(path).name]=raw.decode()
 caller=wrappers['NativeForce.F90']
 for old,new in [('SOLID6Z_FORCE_CALLER','S6_CONTROL_CALLER'),('EVALUATE_VALUES','EVALUATE_CONTROL_VALUES'),('SOLID6Z_FORCE_RESULTANTS','S6_CONTROL_RESULTANTS'),('SOLID6Z_FORCE_NATIVE','S6_CONTROL_NATIVE_CALLER'),('solid6z_force_native','s6_control_native_caller'),('STABILIZATION(28)','STABILIZATION(44)')]:caller=caller.replace(old,new)
 caller=caller.replace('  G%X=0','  if(STEP(2)/=.1D0.or.STEP(3)/=1D0)return\n  G%X=0',1)
 out['NativeCaller.F90']=caller
 initial=wrappers['NativeInitial.F90']
 for old,new in [('SOLID6Z_FORCE_INITIAL_NATIVE','S6_CONTROL_INITIAL_NATIVE'),('solid6z_force_initial_native','s6_control_initial_native'),('SOLID6Z_FORCE_CALLER','S6_CONTROL_CALLER'),('EVALUATE_VALUES','EVALUATE_CONTROL_VALUES'),('STABILIZATION(28)','STABILIZATION(44)')]:initial=initial.replace(old,new)
 out['NativeInitial.F90']=initial
 original=wrappers['NativeResultants.F90'];start=original.index('  call SOLID6Z_FORCE_S6ZHOUR3(')
 prefix=original[:start].replace('SOLID6Z_FORCE_RESULTANTS','S6_CONTROL_RESULTANTS').replace('  use SOLID6Z_FORCE_S6ZHOUR3_MOD\n','')
 prefix=prefix.replace('  use SOLID6Z_FORCE_S6ZRROTA3_MOD','  use S6_CONTROL_NATIVE_MOD\n  use CONTROLLED_LEAF_OBSERVATIONS,only:rate,force,work,work_calls,mode_calls\n  use S6_CONTROL_OBSERVATIONS\n  use SOLID6Z_FORCE_S6ZRROTA3_MOD')
 prefix=prefix.replace('STABILIZATION(28)','STABILIZATION(44)').replace('FHOUR(1,3,4)','FHOUR(1,12)')
 prefix=prefix.replace('  integer :: MAT', '  real(kind=8)::STIN(MVSIZ),SLOTS(6),RAW_STIN\n  integer :: MAT')
 prefix=prefix.replace('  PM=0','  call S6_CONTROL_RESET()\n  work=0;rate=0;force=0;work_calls=0;mode_calls=0\n  call ic1_native_slots(PARAMETERS,SLOTS)\n  PM=0',1)
 prefix=prefix.replace('  PM(21,1)=PARAMETERS(2)','  PM(20,1)=SLOTS(1)\n  PM(21,1)=PARAMETERS(2)')
 prefix=prefix.replace('  PM(22,1)=G0','  PM(22,1)=SLOTS(3)\n  PM(32,1)=SLOTS(4);PM(100,1)=SLOTS(5);PM(107,1)=SLOTS(6)\n  STIN=POINT(33)')
 prefix=prefix.replace('SSP=POINT(20)*SSP_SCALE','SSP=POINT(20)').replace('FHOUR(1,K,M)=','FHOUR(1,K+3*(M-1))=')
 prefix=prefix.replace('  interface\n','''  interface
    subroutine ic1_native_slots(P,S) bind(C,name='ic1_native_slots')
      use iso_c_binding
      real(c_double),intent(in)::P(4)
      real(c_double),intent(out)::S(6)
    end subroutine
''',1)
 call='  call S6_CONTROL_NATIVE( &\n';args=[]
 for field in ['X','V']:
  for k in range(1,4):args.extend('G%'+field+'(:,'+str(n)+','+str(k)+')' for n in range(1,7))
 for k in range(1,4):args.extend('F(:,'+str(n)+','+str(k)+')' for n in range(1,7))
 args+=['PM','250','1','42','MAT','DN','RHO(1:1)','G%VOLUME','SSP','FHOUR','OFF','VOL0','EINT','DT','STIN','1']
 for i in range(0,len(args),4):call+='    '+', '.join(args[i:i+4])+(', &\n' if i+4<len(args) else ')\n')
 tail='''  if(work_calls/=1.or.mode_calls/=1.or.projection_calls/=1)error stop 'S6 controlled observation count'
  RAW_STIN=STIN(1)
  HISTORY(1:9)=POINT(1:9);HISTORY(8)=EINT(1)
  do K=1,3
    do M=1,4
      HISTORY(9+4*(K-1)+M)=FHOUR(1,K+3*(M-1))
    enddo
  enddo
  STABILIZATION(1:12)=rate;STABILIZATION(13:24)=force
  STABILIZATION(25:26)=[work,RAW_STIN];STABILIZATION(27:38)=projection
  do N=1,6
    do K=1,3
      FORCES(18+3*(N-1)+K)=F(1,N,K)
    enddo
  enddo
'''
 rotation=original[original.index('  call SOLID6Z_FORCE_S6ZRROTA3('):original.index('  do N=1,6',original.index('  call SOLID6Z_FORCE_S6ZRROTA3('))]
 out['NativeResultants.F90']=prefix+call+tail+rotation+'''  call S6_CONTROL_ASSEMBLE(F,STIN,FORCES(37:54),STABILIZATION(39:44))
end subroutine
'''
 return out
def prepare(output,check):
 if not check:output.mkdir(parents=True,exist_ok=True)
 for name,value in generated().items():
  path=output/name;raw=value.encode('latin1')
  if check:assert path.read_bytes()==raw,name
  else:path.write_bytes(raw)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--check',action='store_true');a=p.parse_args();prepare(a.output,a.check)
 print(json.dumps({'status':'source_passed','controlled_native_repaired':False,'s6_distortion_enabled':False}))
