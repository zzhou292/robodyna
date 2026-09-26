"""Pinned v3 expected contact + rigid phases; never source-factory inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
from . import trajectory as t

BASE_STAGES={'initial_rows_after_begin','coefficient_controls','i25mainf_entry','i25mainf_return',
             'i25dst3_3_entry','i25dst3_3_return','i25for3_entry','i25for3_return','i25cdcor3_entry','i25cdcor3_return'}
EXTRA_STAGES={'engine_restart_contact_source','native_search_entry','native_sto_entry','native_pen3_entry',
              'native_pen3_return','native_sto_return','native_search_return','native_optcd_entry','native_optcd_return',
              'native_comp2_entry','native_comp2_return','coupled_positive_for3_entry','rgbodfp_entry','rgbodfp_return',
              'rgbodv_entry','rgbodv_return'}


def secondary_source_ids(native,raw):
    return tuple(native[t.integer(index,1,len(native))-1] for index in raw)


def bind_rigid_clock(actual,base,recovery):
    for key in ('NCYCLE','NSPMD','IRESP'):
        t.require(t.integer(actual[key])==t.integer(base[key]),'Rigid integer clock differs from same-cycle MAINF')
    for key in ('TT','DT1'):
        t.require(t.real(actual[key])==t.real(base[key]),'Rigid force-base clock differs from same-cycle MAINF')
    expected=(1.5e-7 if base['NCYCLE']==0 else 3e-7) if recovery else t.real(base['DT12'])
    t.require(t.real(actual['DT12'])==expected,'Rigid force/recovery kick phase differs')


def convert(workspace,manifest):
    t.require(manifest['schema']=='robo_dyna.rigid_trajectory_inputs.v1','Unknown rigid input manifest')
    body=dict(manifest,schema='robo_dyna.native_trajectory_inputs.v1');raw,pins=t.pinned_inputs(body,workspace)
    t.require(set(raw)=={'sequence','summary','coupled_summary','engine_seal','startup_seal','declaration'},'Incomplete rigid evidence')
    summary=t.parse(raw['summary']);coupled=t.parse(raw['coupled_summary']);seal=t.parse(raw['engine_seal'])
    t.require(summary['complete'] and coupled['extra_complete'] and summary['failure'] is None and coupled['failure'] is None and
              summary['exit_code']==coupled['exit_code']==0 and seal['status']=='passed_reference_observation','Unsealed rigid native observation')
    declaration=t.parse(raw['declaration']);startup=t.parse(raw['startup_seal'])
    t.require(declaration['schema']=='robo_dyna.native_contact_scene_export.v3' and startup['status']=='passed_startup_observation','Wrong rigid source phase')
    physical=t.ids([n['id'] for n in declaration['mesh']['nodes']]);primary=t.integer(declaration['reference_rigid_body']['primary']['id'],1)
    t.require(primary not in physical and len(physical)==18,'Rigid primary must remain separate from physical18')
    sequence=t.records(raw['sequence']);t.require(all(r['stage'] in BASE_STAGES|EXTRA_STAGES for r in sequence),'Unknown coupled native stage')
    source=t.one(sequence,'engine_restart_contact_source');native=t.ids(source['ITAB']);arrays=source['source']['arrays']
    secondaries=secondary_source_ids(native,t.array(arrays['NSV'],18,'i'))
    t.require(source['source']['controls']['NRTM']==24 and arrays['MSEGLO']==list(range(1,25)),'Rigid main identity differs')
    domain=t.Domain(physical,native,secondaries,24,1000,3e-7,(primary,));domain.validate()
    by={}
    for record in sequence:by.setdefault(record['stage'],[]).append(record['observation'])
    t.require([r['clock']['NCYCLE'] for r in by['i25mainf_entry']]==list(range(1001)),'Incomplete physical cycle grid')
    first=by['i25mainf_entry'][0]['arrays'];mass=dict(zip(native,t.array(first['MS'],19)))
    reference,metadata=t.serialize([r for r in sequence if r['stage'] in BASE_STAGES],domain,mass)
    entries={(r['clock']['NCYCLE'],r['controls']['IFLAG']):r for r in by['rgbodfp_entry']}
    returns={(r['clock']['NCYCLE'],r['controls']['IFLAG']):r for r in by['rgbodfp_return']}
    recovery={r['clock']['NCYCLE']:r for r in by['rgbodv_return']}
    t.require(len(entries)==len(returns)==2002 and len(recovery)==1001,'Missing rigid entry/return phases')
    bases={r['clock']['NCYCLE']:r['clock'] for r in by['i25mainf_entry']}
    order={};members=sorted(declaration['reference_rigid_body']['member_node_ids'])
    for row in sequence:
        if row['stage'].startswith(('rgbodfp_','rgbodv_')):
            r=row['observation'];cycle=t.integer(r['clock']['NCYCLE'],0,1000)
            bind_rigid_clock(r['clock'],bases[cycle],row['stage'].startswith('rgbodv_'))
            order.setdefault(cycle,[]).append((r['rigid_phase'],row['stage'].rsplit('_',1)[1]))
            t.require(r['ITAB']==list(native) and r['source_ids']['primary_source_id']==primary and
                sorted(r['source_ids']['member_source_ids'])==members,'Rigid source join changed')
    wanted=[(phase,event) for phase in ['RGBODFP_IFLAG1','RGBODFP_IFLAG2','RGBODV'] for event in ['entry','return']]
    t.require(list(order)==list(range(1001)) and all(value==wanted for value in order.values()),'Native rigid phase order differs')
    internal={node:i for i,node in enumerate(native)};root=internal[primary]
    def physical_values(values,width):
        t.array(values,19*width)
        return [values[width*internal[node]+k] for node in physical for k in range(width)]
    rigid=bytearray(b'T25RIG01'+struct.pack('<III',18,1,1001))
    for cycle in range(1001):
        accepted=entries[cycle,1];forced=returns[cycle,2];recovered=recovery[cycle]
        # Recovery is the upcoming kick, unlike the force-history prior kick.
        t.require(recovered['clock']['DT12']==(1.5e-7 if cycle==0 else 3e-7),'Rigid recovery kick phase differs')
        a=accepted['arrays'];b=forced['arrays'];c=recovered['arrays']
        values=physical_values(a['VR'],3)
        values += a['X'][3*root:3*root+3]+a['V'][3*root:3*root+3]+a['VR'][3*root:3*root+3]
        # RBY first9 is native column-major principal frame at force stage n.
        values += b['RBY'][:9]
        values += physical_values(c['A'],3)+physical_values(c['AR'],3)
        values += b['AF'][3*root:3*root+3]+b['AM'][3*root:3*root+3]
        t.array(values,186);rigid.extend(struct.pack('<Q186d',cycle,*values))
        t.require(all(struct.pack('<d',a['MS'][i])==struct.pack('<d',mass[node]) for i,node in enumerate(native)), 'Rigid accepted mass changed')
    t.require(len(rigid)<=t.MAX_BINARY,'Rigid expected binary exceeds cap')
    initial_inertia=entries[0,1]['arrays']['IN']
    principal=t.array(returns[0,2]['arrays']['RBY'][9:12],3)
    t.require(all(value>0 for value in principal),'Initial rigid principal moments must be positive')
    t.require(all(struct.pack('<3d',*t.array(returns[cycle,2]['arrays']['RBY'][9:12],3))==struct.pack('<3d',*principal)
                  for cycle in range(1001)), 'Rigid principal moments changed within fixed-mass source scope')
    metadata.update(schema='robo_dyna.native_rigid_reference_trajectory.v1',intervals=1000,frames=1001,dt_s=3e-7,
        node_source_ids=physical,secondary_source_ids=secondaries,auxiliary_primary_source_id=primary,
        initial_force_principal_moments_native_tonne_mm2=principal,
        mass_native_tonne=[mass[node] for node in physical],inertia_native_tonne_mm2=[initial_inertia[internal[node]] for node in physical],
        reference_sha256=hashlib.sha256(reference).hexdigest(),rigid_reference_sha256=hashlib.sha256(rigid).hexdigest(),input_pins=pins,
        scope='Expected observations only; no shipping source coefficients or restart state',
        phase_contract='Frame n holds accepted spin/primaryX/V/omega, force-n principal frame, and upcoming-kick recovered A/AR. Contact history n is MAINF return n. Commit n+1 uses physical/group state n+1 and force-n frame.',
        inherited_boolean_disposition='matches_last_main_fields is ignored; returned record booleans were inherited from entry')
    return reference,bytes(rigid),metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True);parser.add_argument('--manifest',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();reference,rigid,metadata=convert(args.workspace,t.parse(t.read(args.manifest)))
    args.output.mkdir(exist_ok=False)
    (args.output/'native-reference.bin').write_bytes(reference);(args.output/'rigid-reference.bin').write_bytes(rigid)
    (args.output/'reference-metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':main()
