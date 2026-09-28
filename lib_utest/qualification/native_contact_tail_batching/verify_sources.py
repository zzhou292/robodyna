#!/usr/bin/env python3
"""Source-only proof of the one batched tail against the immutable baseline."""
import pathlib,subprocess,json,hashlib
here=pathlib.Path(__file__).resolve().parent;root=here.parents[2]
base="7695f85554611d07883ee6fd96fadbde26773e38"
def frozen(path):return subprocess.check_output(['git','-C',str(root),'show',base+':'+path],text=True)
path='lib_src/collision/radioss_type25/runtime/Transaction.cpp'
old=frozen(path);new=(root/path).read_text()
first='  status=p.Fence(rd::Gather(p.device,schedule,incidence.incidence(),view,cin,p.stream));if(status.status!=TransactionStatus::Ok)return p.Fail(status);'
second='  status=p.Fence(rd::Apply(p.device,view,cin,p.stream));if(status.status!=TransactionStatus::Ok)return p.Fail(status);'
replacement="""  status=rd::AssembleTail(
      [&]{return rd::Gather(p.device,schedule,incidence.incidence(),view,cin,p.stream);},
      [&]{return rd::Apply(p.device,view,cin,p.stream);},
      [&](cudaError_t error){return p.Fence(error);},[]{return cudaGetLastError();});
  if(status.status!=TransactionStatus::Ok)return p.Fail(status);"""
assert old.count(first+'\n'+second)==1
expected=old.replace('#include "NormalStage.h"','#include "NormalStage.h"\n#include "AssemblyTail.h"',1).replace(first+'\n'+second,replacement)
assert new==expected,'Transaction changed outside approved tail'
allowed={path,'lib_src/collision/radioss_type25/runtime/AssemblyTail.h','lib_src/collision/RadiossType25Transaction.cmake'}
changed=set(subprocess.check_output(['git','-C',str(root),'diff',base,'--name-only','--','lib_src','lib_utils'],text=True).splitlines())
assert changed==allowed,changed
records=[]
for name in ['Kernels.cu','NormalStage.cu','Layout.h','Storage.h']:
 p='lib_src/collision/radioss_type25/runtime/'+name
 current=(root/p).read_text();assert current==frozen(p),p
 records.append({'path':p,'sha256':hashlib.sha256(current.encode()).hexdigest()})
assert 'runtime/*.h' in (root/'lib_src/collision/BUILD.bazel').read_text()
print(json.dumps({'status':'passed','baseline':base,'production_files':len(allowed),
 'all_device_kernels_and_storage_unchanged':True,'all_earlier_transaction_stages_unchanged':True,
 'numerical_execution':False,'unchanged_source_pins':records}))
