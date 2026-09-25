#!/usr/bin/env python3
"""Pin the original serial body, not a second independently maintained formula."""
from pathlib import Path
import hashlib,json,re
here=Path(__file__).resolve().parent
root=here.parents[2]
record=json.loads((here/'reference.json').read_text())
old=(here/'reference/Assembly.cu.txt').read_text()
assert hashlib.sha256(old.encode()).hexdigest()==record['sha256']
serial=(root/'lib_src/elements/solids/resident/AssemblySerial.cuh').read_text()
def body(text,name):
 start=text.index(name+'(');start=text.index('{',start);depth=1;end=start+1
 while depth:
  depth+=(text[end]=='{')-(text[end]=='}');end+=1
 return re.sub(r'\s+','',text[start:end])
for name in ['CheckNodes','AddFamily','AddRearFamily','Assemble']:
 assert body(old,name)==body(serial,name),name
current=(root/'lib_src/elements/solids/resident/Assembly.cu').read_text()
assert 'atomicAdd' not in current and 'atomicExch(&state.assembly.fallback, 1u)' in current
assert 'if (!blocks) blocks = 1' in current
assert 'if (error != cudaSuccess) return error' in current
assert 'assembly_serial::Assemble(storage, slab, view, cin)' in current
print('PASS: four original serial numerical/control bodies and exceptional routing retained')
