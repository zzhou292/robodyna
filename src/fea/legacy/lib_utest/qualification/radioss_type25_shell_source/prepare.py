"""Emit a deterministic input/expectation fixture from actual pinned native records."""
import argparse
import hashlib
import json
from pathlib import Path


def prepare():
    root=Path(__file__).with_name('evidence')
    manifest=json.loads((root/'manifest.json').read_text())
    for pin in manifest['files']:
        data=(root/pin['name']).read_bytes()
        if len(data)!=pin['bytes'] or hashlib.sha256(data).hexdigest()!=pin['sha256']:
            raise ValueError('Observed source fixture changed:'+pin['name'])
    declared=json.loads((root/'declared-scene.json').read_text())
    initial=[json.loads(s) for s in (root/'native-startup.jsonl').read_text().splitlines()]
    initial=next(r['arrays'] for r in initial if r['stage']=='initia_return')
    observations={r['stage']:r['observation'] for s in (root/'native-observations.jsonl').read_text().splitlines()
                  for r in (json.loads(s),)}
    mesh=declared['mesh'];ids=[n['id'] for n in mesh['nodes']];dense={identifier:i for i,identifier in enumerate(ids)}
    physical=sorted(mesh['patch'],key=lambda e:e['id'])+sorted(mesh['wall'],key=lambda e:e['id'])
    coefficient=declared['scene']['material']['young_n_mm2'];thickness=declared['scene']['thickness_mm']
    lines=['// Generated from pinned actual Starter/Engine source records; do not edit.',
           '#pragma once','namespace observed {',f'inline constexpr unsigned Nodes={len(ids)};',
           'inline const s::PhysicalShell Shells[]{']
    for e in physical:
        nodes=e['nodes'];slots=nodes if len(nodes)==4 else nodes+[nodes[-1]]
        layout='Quad4' if len(nodes)==4 else 'Triangle3'
        lines.append('{'+str(e['id'])+',ShellLayout::'+layout+',{'+','.join(str(dense[n]) for n in slots)+'},'+
                     ','.join(repr(v) for v in (coefficient,thickness,thickness,thickness,0.))+'},')
    lines.append('};')
    native_ids=observations['main']['arrays']['ITAB']
    chunks=observations['inventory']['chunks'];primary=[];primary_k=[]
    for chunk in chunks:
        arrays=chunk['arrays']
        for i,stiffness in enumerate(arrays['STF']):
            ids_at_face={native_ids[j-1] for j in arrays['IRECT'][4*i:4*i+4]}
            matching=[j for j,e in enumerate(physical) if set(e['nodes'])==ids_at_face]
            if len(matching)!=1:raise ValueError('Ambiguous source-to-main face mapping')
            primary.append(matching[0]);primary_k.append(stiffness)
    arrays=chunks[0]['arrays']
    secondary=[dense[native_ids[j-1]] for j in arrays['NSV']]
    lines.append('inline const std::uint32_t Primary[]{'+','.join(map(str,primary))+'};')
    lines.append('inline const s::Secondary Secondary[]{'+','.join('{'+str(n)+',1.}' for n in secondary)+'};')
    def array(name,values,kind='double'):
        lines.append('inline const '+kind+' '+name+'[]{'+','.join(map(repr,values))+'};')
    raw={identifier:i for i,identifier in enumerate(initial['ITAB'])}
    for name in ('ETNOD','NSHNOD','STIFINT'):array(name,[initial[name][raw[i]] for i in ids], 'int' if name=='NSHNOD' else 'double')
    array('PrimaryK',primary_k);array('SecondaryK',arrays['STFN']);array('SecondaryGap',arrays['GAP_S'])
    lines.append('} // namespace observed')
    return '\n'.join(lines)+'\n'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();text=prepare()
    if args.check:
        if args.output.read_text()!=text:raise ValueError('Generated observed fixture drift')
    else:args.output.write_text(text)
