#!/usr/bin/env python3
from pathlib import Path
import subprocess,sys,re
root=Path(__file__).resolve().parents[3]
here=Path(__file__).resolve().parent
out=Path(sys.argv[1]);out.mkdir(exist_ok=False)
flags=['g++','-std=c++17','-O0','-fno-fast-math','-ffp-contract=off','-I'+str(root),'-I/usr/local/cuda/include','-I/usr/include/eigen3']
def run(name,cmd):
 r=subprocess.run(cmd,capture_output=True,text=True)
 (out/(name+'.log')).write_text(r.stdout+r.stderr)
 print(name,r.returncode,flush=True)
 if r.returncode: print(r.stdout+r.stderr,flush=True)
 r.check_returncode()
run('build',flags+[str(here/'HostTest.cpp'),str(here/'IncidenceTest.cpp'),str(root/'lib_src/collision/nodal_wall_mapped/Layout.cpp'),'-lgtest_main','-lgtest','-pthread','-o',str(out/'host')])
run('host',[str(out/'host'),'--gtest_output=xml:'+str(out/'host.xml')])
for name in ['Initialize.cpp','Sources.cpp','Forecast.cpp']:
 run(name,flags+['-fsyntax-only',str(root/'lib_src/collision/nodal_wall_mapped'/name)])

# C++ name/type checking only. Kernel launch/device semantics remain root-owned.
shape=out/'CudaShape.h'
shape.write_text('#include <cuda_runtime.h>\n#undef __global__\n#undef __device__\n#undef __shared__\n#define __global__\n#define __device__\n#define __shared__\nstruct ShapeIndex {unsigned x=0;};\nstatic ShapeIndex threadIdx,blockIdx,blockDim,gridDim;\ninline void __syncthreads(){}\ninline void atomicMin(unsigned long long* p,unsigned long long v){if(v<*p)*p=v;}\n')
wall=root/'lib_src/collision/nodal_wall_mapped'
headers=['ObserverReduction.cuh','Evaluation.cuh','Scatter.cuh','IntervalReduction.cuh','AssemblyValidation.cuh','Response.cuh']
mapped={str(wall/name):str(out/name) for name in headers}
def transform(path):
 def include(match):
  target=(path.parent/match.group(1)).resolve()
  if not target.exists(): target=root/match.group(1)
  return '#include "'+mapped.get(str(target),str(target))+'"'
 return re.sub(r'<<<.*?>>>','',re.sub(r'#include "([^"]+)"',include,path.read_text()),flags=re.S)
for name in headers: (out/name).write_text(transform(wall/name))
for path in (wall/'Operations.cu',here/'CudaTest.cu',here/'OwnerTest.cu'):
 target=out/(path.stem+'.cpp');target.write_text(transform(path))
 run(path.stem+'-shape',flags+['-include',str(shape),'-fsyntax-only',str(target)])
run('source-identity',[sys.executable,'-B',str(here/'verify_sources.py')])
run('configure',['cmake','-S',str(here),'-B',str(out/'cmake'),'-DCMAKE_BUILD_TYPE=Release'])
