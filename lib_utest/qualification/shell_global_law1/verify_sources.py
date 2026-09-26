#!/usr/bin/env python3
"""Only the shared evaluator's coefficient input changes; old math stays pinned."""
from pathlib import Path
import hashlib,json,runpy
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def body(s,name):
    a=s.index(' noexcept {',s.index(name))+len(' noexcept ')
    depth=0
    for i in range(a,len(s)):
        if s[i]=='{': depth+=1
        if s[i]=='}':
            depth-=1
            if depth==0:return s[a:i+1]
    raise ValueError('Unclosed evaluator')
def main():
    pins=json.loads((HERE/'source-proof.json').read_text())
    for path,pin in pins['functions'].items():
        s=(ROOT/path).read_text()
        if path.endswith('/qeph/QephForce.h'):
            proof=runpy.run_path(str(HERE.parent/'qeph_private_trial/verify_sources.py'))
            proof['verify']()
            actual=proof['original_global'](s)
        else:
            actual=body(s,'inline Status EvaluateForceWithThickness(')
        explicit='  auto coefficient_input=r.input;\n  coefficient_input.thickness=coefficient_thickness;\n'
        if actual.count(explicit)!=1:raise ValueError('Unexpected coefficient routing')
        actual=actual.replace(explicit,'').replace('PrepareMaterial(coefficient_input,','PrepareMaterial(r.input,')
        if hashlib.sha256(actual.encode()).hexdigest()!=pin['body_sha256']:raise ValueError('Legacy force body changed: '+path)
        legacy=body(s,'inline Status EvaluateForce(')
        if legacy!='{\n  return detail::EvaluateForceWithThickness(r,base,interval,r.input.thickness,output);\n}':raise ValueError('Legacy route changed')
    print('Two legacy force bodies and reference-thickness routes match '+pins['base'])
if __name__=='__main__':main()
