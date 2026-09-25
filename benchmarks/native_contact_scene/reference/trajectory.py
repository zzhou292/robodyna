"""Convert pinned native observations to expected T25REF01 data, never physics seeds."""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct

MAX_INPUT = 64 << 20
MAX_BINARY = 16 << 20
STRIDES = {'IRTLM': 4, 'PENE_OLD': 5, 'STIF_OLD': 2, 'SECND_FR': 6, 'TIME_S': 2, 'ICONT_I': 1}


def require(value, message):
    if not value:
        raise ValueError(message)


def integer(value, low=0, high=(1 << 31)-1):
    require(type(value) is int and low <= value <= high, 'Invalid native integer')
    return value


def real(value):
    require(type(value) in (int, float), 'Native real has nonnumeric type')
    result = float(value)
    require(math.isfinite(result) and (type(value) is float or result == value), 'Nonfinite or inexact integer real')
    return result


def array(value, size, code='d'):
    require(isinstance(value, list) and len(value) == size, 'Native array extent differs')
    return [real(x) for x in value] if code == 'd' else [integer(x, -(1 << 31)) for x in value]


def ids(value):
    require(isinstance(value, (list, tuple)) and 0 < len(value) <= 128, 'Native ID domain exceeds bound')
    out = tuple(integer(x, 1) for x in value)
    require(len(set(out)) == len(out), 'Native IDs are not unique')
    return out


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'Duplicate JSON key')
        result[key] = value
    return result


def parse(data):
    return json.loads(data, object_pairs_hook=unique_object,
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite JSON literal')))


def read(path, cap=MAX_INPUT):
    path = Path(path)
    require(path.is_file() and path.stat().st_size <= cap, 'Reference input exceeds byte bound')
    data = path.read_bytes()
    require(len(data) <= cap, 'Reference input changed size while reading')
    return data


def records(data):
    result = [parse(line) for line in data.splitlines() if line.strip()]
    require(0 < len(result) <= 20000, 'Native record count exceeds bound')
    return result


def one(values, stage):
    found = [r.get('observation', r) for r in values if r.get('stage') == stage]
    require(len(found) == 1, 'Expected exactly one native stage: ' + stage)
    return found[0]


@dataclass(frozen=True)
class Domain:
    node_ids: tuple
    native_ids: tuple
    secondary_ids: tuple
    main_count: int
    intervals: int
    dt_s: float

    def validate(self):
        require(set(ids(self.node_ids)) == set(ids(self.native_ids)), 'Native/declaration node domains differ')
        require(set(ids(self.secondary_ids)) <= set(self.node_ids), 'Unknown secondary source ID')
        integer(self.main_count, 1, 4096)
        integer(self.intervals, 1, 4096)
        require(real(self.dt_s) > 0, 'Reference dt must be positive')


def rows(value, count):
    require(isinstance(value, dict) and set(value) == set(STRIDES), 'Incomplete persistent native row fields')
    return {key: array(value[key], count*width, 'i' if key in ('IRTLM', 'ICONT_I') else 'd')
            for key, width in STRIDES.items()}


def clock(value, epoch, expected_time, dt):
    require(isinstance(value, dict), 'Missing native clock')
    require(integer(value['NCYCLE']) == epoch and integer(value['NSPMD'], 1) == 1,
            'Native cycle or partition scope differs')
    require(integer(value['IRESP']) in (0, 1, 2), 'Unsupported native precision profile')
    require(real(value['TT']) == expected_time, 'Native time is not the original fixed-step recurrence')
    require(real(value['DT1']) == (dt if epoch else 0), 'Native DT1 grid differs')
    require(real(value['DT12']) == (0 if epoch == 0 else dt/2 if epoch == 1 else dt), 'Native kick grid differs')


def same_clock(a, b):
    # Other fields include a mid-cycle estimator, not an accepted next step.
    require(all(a[k] == b[k] for k in ('TT', 'DT1', 'DT12', 'NCYCLE', 'NSPMD', 'IRESP')),
            'Packet belongs to a different force-base clock')


class Writer:
    def __init__(self, domain):
        self.data = bytearray(b'T25REF01' + struct.pack('<III', len(domain.node_ids),
                                                      len(domain.secondary_ids), domain.intervals+1))

    def put(self, code, values):
        values = list(values)
        require(len(self.data) + struct.calcsize('<'+code)*len(values) <= MAX_BINARY, 'Expected binary exceeds cap')
        self.data.extend(struct.pack('<'+code*len(values), *values))

    def frame(self, domain, epoch, before, after, packets):
        # Same explicit scalar order as qualified TL prepare_fixture.py and
        # ReferenceReader: no C++ padding, pointers, SI conversion or restart state.
        self.put('Q', [epoch]);self.put('d', [before['clock'][k] for k in ('TT', 'DT1', 'DT12')])
        index = {value: i for i, value in enumerate(domain.native_ids)}
        def physical(values, width):
            return [values[width*index[node]+j] for node in domain.node_ids for j in range(width)]
        for key in ('X', 'V'):self.put('d', physical(before['arrays'][key], 3))
        for state, key, width in ((before, 'A', 3), (after, 'A', 3), (before, 'STIFN', 1), (after, 'STIFN', 1)):
            self.put('d', physical(state['arrays'][key], width))
        history = after['rows']
        for i in range(len(domain.secondary_ids)):
            self.put('i', history['IRTLM'][4*i:4*i+4])
            p=history['PENE_OLD'][5*i:5*i+5];k=history['STIF_OLD'][2*i:2*i+2];f=history['SECND_FR'][6*i:6*i+6]
            self.put('d', [p[1], k[1], p[0], k[0], p[2], *f[3:], *f[:3], p[3], p[4], *history['TIME_S'][2*i:2*i+2]])
            self.put('i', [history['ICONT_I'][i]])
        self.put('I', [len(packets)])
        for packet in packets:
            self.put('I', [packet['jlt']]);self.put('i', packet['arrays']['CAND_N']);self.put('i', packet['arrays']['CAND_E'])


def serialize(sequence, domain, mass_by_id=None):
    """Reusable original serializer with bounded source/clock validation.

    The public converter below fixes the existing reader's18/18/1001 scope.
    Smaller domains are useful for direct byte-contract tests, not a new format.
    """
    domain.validate();n=len(domain.node_ids);nr=len(domain.secondary_ids)
    writer=Writer(domain);current=None;pending=None;packets=[];epoch=0;time=0.;initial=False
    counts={};active=set();episodes=[0]*nr;was_active=[False]*nr;starts=[[] for _ in range(nr)]
    coefficient=False
    allowed={'initial_rows_after_begin','coefficient_controls','i25mainf_entry','i25mainf_return',
             'i25dst3_3_entry','i25dst3_3_return','i25for3_entry','i25for3_return','i25cdcor3_entry','i25cdcor3_return'}
    require(0<len(sequence)<=20000, 'Native sequence record cap exceeded')
    for record in sequence:
        stage=record.get('stage');require(stage in allowed, 'Unknown native observation stage')
        data=record['observation'];counts[stage]=counts.get(stage,0)+1
        if stage=='initial_rows_after_begin':
            require(not initial and epoch==0 and current is None, 'Initial row snapshot is late or repeated')
            clock(data['clock'],0,0.,domain.dt_s);rows(data['rows'],nr);initial=True;continue
        if stage=='i25mainf_entry':
            require(initial and current is None and pending is None and epoch<=domain.intervals, 'Unbalanced or excess main entry')
            clock(data['clock'],epoch,time,domain.dt_s)
            require(integer(data['NVSIZ'],1)==128 and integer(data['nodes'],1)==n, 'Native main extent differs')
            a=data['arrays'];require(ids(a['ITAB'])==domain.native_ids, 'Native node permutation changed')
            for key,width in (('X',3),('V',3),('A',3),('STIFN',1),('MS',1)):array(a[key],width*n)
            if mass_by_id is not None:
                require(all(struct.pack('<d',a['MS'][i])==struct.pack('<d',mass_by_id[node])
                            for i,node in enumerate(domain.native_ids)), 'Native mass changed after startup')
            current=data;packets=[];continue
        require(current is not None, 'Native packet is outside its main call')
        clock(data['clock'],epoch,time,domain.dt_s)
        same_clock(data['clock'],current['clock'])
        if stage=='coefficient_controls':
            require(not coefficient and data['controls']=={'KMIN':0.0,'KMAX':1e30,'IGSTI':4,'ISTIF_MSDT':0},
                    'Unsupported or repeated coefficient control observation')
            coefficient=True;continue
        if stage=='i25mainf_return':
            require(pending is None and integer(data['nodes'],1)==n and integer(data['NVSIZ'],1)==128,
                    'Main returned with unfinished packet or mismatched extent')
            array(data['arrays']['A'],3*n);array(data['arrays']['STIFN'],n);history=rows(data['rows'],nr)
            if epoch<domain.intervals:
                for i in range(nr):
                    hit=history['IRTLM'][4*i]!=0
                    if hit and not was_active[i]:episodes[i]+=1;starts[i].append(epoch)
                    was_active[i]=hit
            writer.frame(domain,epoch,current,data,packets);epoch+=1;time+=domain.dt_s;current=None;continue
        count=integer(data['jlt'],1,128);require(integer(data['NVSIZ'],1)==128, 'Native vector width differs')
        if stage.endswith('_entry'):
            require(pending is None, 'Unexpected nested native packet');pending=(stage[:-6],data)
            if stage=='i25dst3_3_entry':
                require(integer(data['nsn'],1)==nr and len(packets)<16, 'Geometry cohort/row bound exceeded')
                a=data['arrays'];array(a['CAND_N'],count,'i');array(a['CAND_E'],count,'i')
                require(all(1<=x<=nr for x in a['CAND_N']) and all(1<=x<=domain.main_count for x in a['CAND_E']), 'Unknown geometry source index')
                packets.append(data)
            else:
                indices=array(data['source_occurrences_one_based'],count,'i');require(all(x>0 for x in indices),'Invalid native occurrence index')
                if stage=='i25for3_entry':
                    require(integer(data['nsn'],1)==nr, 'FOR3 row extent differs');a=data['arrays']
                    array(a['CAND_N_N'],count,'i');require(all(1<=x<=nr for x in a['CAND_N_N']),'Invalid FOR3 row')
                    pene=array(a['PENE'],count);array(a['STIF'],count)
                    if epoch<domain.intervals and any(p>0 for p in pene):active.add(epoch)
        else:
            require(pending is not None and pending[0]==stage[:-7] and pending[1]['jlt']==count,'Unbalanced native packet return')
            if 'nsn' in pending[1]:
                require(integer(data['nsn'],1)==nr, 'Returned native row extent differs')
            if 'source_occurrences_one_based' in pending[1]:
                require(array(data['source_occurrences_one_based'],count,'i')==pending[1]['source_occurrences_one_based'],
                        'Native packet occurrence order changed')
            if stage=='i25dst3_3_return':
                array(data['arrays']['PENE'],count);array(data['arrays']['STIF'],count)
                array(data['history']['IRTLM'],4*nr,'i');array(data['history']['PENE_OLD'],5*nr)
            elif stage=='i25cdcor3_return':
                for key in ('CAND_E_N','CAND_N_N'):array(data['arrays'][key],count,'i')
            else:
                a=data['arrays']
                for key in ('H1','H2','H3','H4','STIF'):array(a[key],count)
                indices=data['assembly_rows_one_based']
                expected=[i+1 for i in range(count) if ((a['H1'][i]+a['H2'][i])+a['H3'][i])+a['H4'][i]!=0]
                require(indices==expected,'Native post-response defined-channel mask differs')
                for key in ('N1','N2','N3'):array(data['active_force'][key],len(indices))
                for key in ('IRTLM','PENE_OLD','STIF_OLD','SECND_FR'):array(data['history'][key],STRIDES[key]*nr,'i' if key=='IRTLM' else 'd')
            pending=None
    require(initial and coefficient and current is None and pending is None and epoch==domain.intervals+1,'Native sequence is incomplete')
    return bytes(writer.data),dict(counts=counts,active_steps=len(active),first_active_step=min(active) if active else None,
                                  active_force_base_epochs=sorted(active),episodes_by_secondary=episodes,episode_start_force_base_epochs=starts)


def pinned_inputs(manifest, workspace):
    require(manifest.get('schema')=='robo_dyna.native_trajectory_inputs.v1','Unknown input pin schema')
    result={};pins=[]
    for key,item in manifest['files'].items():
        path=Path(item['path']);require(not path.is_absolute() and '..' not in path.parts,'Input pin must stay inside workspace')
        data=read(Path(workspace)/path)
        integer(item['bytes'],0,MAX_INPUT)
        require(len(data)==item['bytes'] and hashlib.sha256(data).hexdigest()==item['sha256'],'Pinned input changed: '+key)
        result[key]=data;pins.append(dict(key=key,**item))
    return result,pins


def convert(manifest, workspace):
    data,pins=pinned_inputs(manifest,workspace)
    for key,schema in [('startup_abi','robo_dyna.native_startup_observation_abi.v1'),
                       ('observation_abi','robo_dyna.native_scene_observation_abi.v1'),
                       ('sequence_abi','robo_dyna.native_scene_observation_abi.v1')]:
        abi=parse(data[key]);require(abi['schema']==schema, 'Unknown captured ABI')
        if key!='startup_abi':require(abi['history_strides']==STRIDES, 'Captured row ABI differs')
    require(data['observation_abi']==data['sequence_abi'], 'Sequence and observation use different ABIs')
    declaration=parse(data['declaration']);require(declaration['schema']=='robo_dyna.native_contact_scene_export.v2','This converter requires declared v2 source')
    require(declaration['scene']['contact_surface']=='all_shells','This qualification is the moving all-shell profile')
    node_ids=ids([node['id'] for node in declaration['mesh']['nodes']])
    require(data['declaration']==data['startup_declaration']==data['observation_declaration'],'Observations have different declared sources')
    require(hashlib.sha256(data['scene']).hexdigest()==declaration['source_sha256'],'Raw scene differs from declaration')
    for record in declaration['files']:
        key={'scene.json':'scene','contact_scene_0000.rad':'starter_deck','contact_scene_0001.rad':'engine_deck'}.get(record['path'])
        require(key is not None and len(data[key])==record['bytes'] and hashlib.sha256(data[key]).hexdigest()==record['sha256'],'Declared native deck changed')
    startup_summary=parse(data['startup_summary']);summary=parse(data['sequence_summary']);observation_summary=parse(data['observation_summary'])
    for value,schema in ((startup_summary,'robo_dyna.native_startup_observation.v1'),(summary,'robo_dyna.native_sequence_observation.v1'),(observation_summary,'robo_dyna.native_scene_observation.v1')):
        require(value.get('schema')==schema and value.get('complete') is True and value.get('failure') is None,'Incomplete native observation')
    require(type(startup_summary['exit_code']) is int and startup_summary['exit_code']==0 and type(summary['exit_code']) is int and summary['exit_code']==0,'Native child did not finish successfully')
    startup=records(data['startup']);observation=records(data['observation']);sequence=records(data['sequence'])
    initial=one(startup,'initia_return');require(integer(initial['nodes'],1)==len(node_ids),'Startup node count differs')
    a=initial['arrays'];startup_ids=ids(a['ITAB']);require(set(startup_ids)==set(node_ids),'Startup node identity differs')
    mass=dict(zip(startup_ids,array(a['MS'],len(node_ids))));inertia=dict(zip(startup_ids,array(a['IN'],len(node_ids))))
    require(all(x>0 for x in mass.values()) and all(x>0 for x in inertia.values()),'Missing positive startup mass/inertia')
    main=one(observation,'main');classification=one(observation,'classification')
    native_ids=ids(main['arrays']['ITAB']);nsn=integer(classification['nsn'],1,128)
    nsv=array(classification['arrays']['NSV'],nsn,'i');require(all(1<=x<=len(native_ids) for x in nsv),'Invalid native NSV')
    secondary_ids=ids([native_ids[x-1] for x in nsv]);main_count=integer(classification['nrtm'],1,4096)
    require(integer(main['nin'],1)==1 and integer(main['clock']['NSPMD'],1)==1 and integer(main['controls']['NSN'],1)==nsn and integer(main['controls']['NRTM'],1)==main_count,'Native interface domains differ')
    require((len(node_ids),nsn,integer(summary['cycle_count']),integer(summary['first_cycle']),integer(summary['last_cycle']))==(18,18,1001,0,1000),'Existing ReferenceReader requires18nodes/18rows/1001frames')
    require(set(secondary_ids)==set(node_ids),'This source profile requires the complete declared secondary union')
    domain=Domain(node_ids,native_ids,secondary_ids,main_count,1000,3e-7)
    require(declaration['scene']['time_step_cap_s']==domain.dt_s and declaration['scene']['end_time_s']==domain.intervals*domain.dt_s,'Declared horizon differs')
    binary,stats=serialize(sequence,domain,mass)
    first=next(r['observation'] for r in sequence if r['stage']=='i25mainf_entry');mapping={v:i for i,v in enumerate(native_ids)}
    expected=[v for node in declaration['mesh']['nodes'] for v in array(node['xyz_mm'],3)]
    observed=[first['arrays']['X'][3*mapping[node]+j] for node in node_ids for j in range(3)]
    require(all(struct.pack('<d',real(x))==struct.pack('<d',real(y)) for x,y in zip(expected,observed)),'Native initial coordinates differ from declared source')
    counts=stats['counts']
    for key,stage in [('geometry_packets','i25dst3_3_entry'),('responses','i25for3_entry'),('external_cdcor3_force_packets','i25cdcor3_entry')]:
        require(integer(summary[key])==counts.get(stage,0),'Observer summary packet count differs')
    require(integer(summary['records'])==len(sequence) and integer(summary['bytes'])==len(data['sequence']) and integer(summary['complete_row_snapshots'])==1001 and integer(summary['coefficient_control_calls'])==counts['i25dst3_3_entry'],'Observer completion counters differ')
    require(stats['first_active_step'] is not None,'No observed positive-penetration force-base step')
    metadata=dict(schema='robo_dyna.native_reference_trajectory.v1',scope='Expected observations only; never source coefficients, restart state or physics seeds',
        node_source_ids=node_ids,secondary_source_ids=secondary_ids,mass_native_tonne=[mass[x] for x in node_ids],
        inertia_native_tonne_mm2=[inertia[x] for x in node_ids],intervals=1000,frames=1001,dt_s=domain.dt_s,
        active_steps=stats['active_steps'],episodes_by_secondary=stats['episodes_by_secondary'],first_active_step=stats['first_active_step'],
        reference_sha256=hashlib.sha256(binary).hexdigest(),reference_bytes=len(binary),native_main_count=main_count,
        active_force_base_epochs=stats['active_force_base_epochs'],episode_start_force_base_epochs=stats['episode_start_force_base_epochs'],
        activity_scope='force-base epochs[0,intervals); positive pre-FOR3 PENE; episodes are after-main IRTLM(1) zero-to-nonzero transitions',
        row_identity_scope='Observed classification NSV through MAINF ITAB; fixed single-interface roster. Per-cycle NSV was not observed; stable row extents and ITAB are verified.',
        inputs=pins,format='Unchanged T25REF01 scalar little-endian ReferenceReader contract; native mm/tonne/s, original row/cohort order')
    return binary,metadata


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True);parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--check',action='store_true')
    args=parser.parse_args(argv);manifest=parse(read(args.manifest,1<<20));binary,metadata=convert(manifest,args.workspace)
    files={'native-reference.bin':binary,'reference-metadata.json':(json.dumps(metadata,indent=2,allow_nan=False)+'\n').encode()}
    if args.check:
        require(all((args.output/name).read_bytes()==value for name,value in files.items()),'Prepared expected reference changed')
    else:
        args.output.mkdir(exist_ok=False)
        for name,value in files.items():
            with (args.output/name).open('xb') as stream:stream.write(value)
    print(json.dumps(dict(status='passed',frames=metadata['frames'],active_steps=metadata['active_steps'],reference_sha256=metadata['reference_sha256'])))


if __name__=='__main__':main()
