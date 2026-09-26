"""Authenticate a storage-only refactor against pinned pre-refactor numerical bodies."""
from pathlib import Path
import hashlib
import json

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
MANIFEST_SHA256="297e025fbe6daab948702df790c2c90d92cd2c62c8614e298c77549b3f6e8ed5"


def body(text,name):
    start=text.index('{',text.index(name+'('))
    depth=1
    end=start+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}')
        end+=1
    return text[start:end]


def replace_once(text,old,new):
    if text.count(old)!=1:
        raise ValueError('Unexpected storage boundary: '+old)
    return text.replace(old,new,1)


def original_global(text):
    value=body(text,'EvaluateForceWithThicknessIntoTrial')
    value=replace_once(value,'  candidate=ForceTrial{}; // Reset every field when retrying the same trial slab.\n','')
    value=replace_once(value,'  candidate.kinematics=geometry.values;',
        '  ForceTrial candidate;\n  candidate.kinematics=geometry.values;')
    return replace_once(value,'  return Status::kSuccess;\n}',
        '  output=candidate;\n  return Status::kSuccess;\n}')


def original_layered(text):
    value=body(text,'EvaluateLayeredForceIntoTrial')
    value=replace_once(value,'{\n','{\n  const auto& base=accepted.shell;\n')
    value=replace_once(value,'  candidate=ForceTrial{};\n','')
    value=replace_once(value,'  candidate.kinematics=geometry.values;',
        '  Trial staged;\n  auto& candidate=staged.force;\n  candidate.kinematics=geometry.values;')
    value=replace_once(value,'  if(adapter.Update(', '  typename Adapter::Result section;\n  if(adapter.Update(')
    value=replace_once(value,'  StiffnessDiagnostics(geometry,',
        '  Adapter::Publish(section,staged);\n  StiffnessDiagnostics(geometry,')
    return replace_once(value,'  return Status::kSuccess;\n}',
        '  output=staged;\n  return Status::kSuccess;\n}')


def original_elastic(text):
    value=body(text,'EvaluateLayeredLaw1IntoTrial')
    value=replace_once(value,'  if(!sections::MatchesLayeredMaterial',
        '  const auto& base=accepted.shell;\n  if(!sections::MatchesLayeredMaterial')
    value=replace_once(value,'MatchesLayeredSectionResultants(accepted_section,',
        'MatchesLayeredSectionResultants(accepted.section,')
    value=replace_once(value,'UpdateShellLayeredLaw1(parameters,accepted_section,',
        'UpdateShellLayeredLaw1(parameters,accepted.section,')
    value=replace_once(value,'  candidate=ForceTrial{};\n','')
    value=replace_once(value,'  candidate.kinematics=geometry.values;',
        '  LayeredLaw1ForceTrial staged;\n  auto& candidate=staged.force;\n  candidate.kinematics=geometry.values;')
    value=replace_once(value,'  if(!sections::UpdateShellLayeredLaw1(',
        '  sections::ShellLayeredLaw1Result section;\n  if(!sections::UpdateShellLayeredLaw1(')
    value=replace_once(value,'  detail::StiffnessDiagnostics(geometry,',
        '  staged.proposed_section=section.history;\n  detail::StiffnessDiagnostics(geometry,')
    return replace_once(value,'  return Status::kSuccess;\n}',
        '  output=staged;\n  return Status::kSuccess;\n}')


def verify():
    raw=(HERE/'source-proof.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=MANIFEST_SHA256:
        raise ValueError('Unreviewed private-trial proof manifest')
    proof=json.loads(raw)
    for item,normalizer in zip(proof['numerical_bodies'],
            (original_global,original_layered,original_elastic)):
        text=(ROOT/item['path']).read_text()
        actual=hashlib.sha256(normalizer(text).encode()).hexdigest()
        if actual!=item['original_body_sha256']:
            raise ValueError('Numerical body changed: '+item['path'])
    for item in proof['reviewed_storage_bodies']:
        value=body((ROOT/item['path']).read_text(),item['function'])
        if hashlib.sha256(value.encode()).hexdigest()!=item['body_sha256']:
            raise ValueError('Reviewed storage/dispatch body changed: '+item['function'])
    return {'status':'passed','baseline':proof['baseline'],'numerical_bodies':3,
            'scope':'Exact old arithmetic/validation after reversing only explicit staging/reference plumbing; numerical tests remain required'}


if __name__=='__main__':
    print(json.dumps(verify()))
