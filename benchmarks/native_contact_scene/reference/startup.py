"""Read native raw mass, isotropic inertia and stiffness operands at defined returns.

Only GDB's own Starter child is observed. All arrays are captured after SPMD_MSIN
or INITIA returns, not from uninitialized entry output. Starter continues normally
and writes its own restart. Physical units remain native mm/tonne/s.
"""
import hashlib
import json
from pathlib import Path

import gdb

from .gdb_access import Call, common, values

_abi=None
_stream=None
_failure=None
_exit_code=None
_context=None
_records=[]
_breakpoints=[]


def failed(error):
    global _failure
    _failure=str(error)
    return True


def read_state(pointers, nodes):
    arrays={name:values(pointers[name],nodes,code) for name,code in
            (('MS','d'),('IN','d'),('ETNOD','d'),('NSHNOD','i'),
             ('VOLNOD','d'),('BVOLNOD','d'),('STIFINT','d'))}
    arrays['ITAB']=values(pointers['ITAB'],nodes,'i')
    if len(set(arrays['ITAB']))!=nodes or any(v<=0 for v in arrays['ITAB']):
        raise ValueError('Native original-node map is not a positive bijection')
    if any(v<=0 for v in arrays['MS']) or any(v<=0 for v in arrays['IN']):
        raise ValueError('This all-shell scene requires positive raw mass and isotropic inertia')
    return arrays


class Returned(gdb.FinishBreakpoint):
    def __init__(self, label, pointers, nodes):
        super().__init__(gdb.newest_frame(),internal=True)
        self.label,self.pointers,self.nodes=label,pointers,nodes

    def stop(self):
        global _context
        try:
            if len(_records)>=8:raise ValueError('Startup observation call bound exhausted')
            record={'stage':self.label,'nodes':self.nodes,
                    'arrays':read_state(self.pointers,self.nodes)}
            _records.append(record)
            _stream.write(json.dumps(record,allow_nan=False)+'\n');_stream.flush()
            print('Observed native '+self.label,flush=True)
            if self.label=='initia_return':_context=None
            return False
        except Exception as error:return failed(error)

    def out_of_scope(self):
        failed('Native startup frame unwound before a defined return')


class Entered(gdb.Breakpoint):
    def __init__(self, routine):
        super().__init__('*'+routine.lower()+'_',internal=True)
        self.routine=routine

    def stop(self):
        global _context
        try:
            nodes=common('com04_',1,'i')
            if not 0<nodes<=4096:raise ValueError('Tiny startup observation node bound exceeded')
            call=Call(_abi['routines'][self.routine])
            pointers={name:call.pointer(name) for name in
                      ('MS','IN','ETNOD','NSHNOD','VOLNOD','BVOLNOD','STIFINT')}
            if self.routine=='INITIA':
                if _context is not None:raise ValueError('Nested INITIA is unsupported')
                pointers['ITAB']=call.pointer('ITAB')
                _context=(nodes,pointers['ITAB'])
            else:
                if _context is None or _context[0]!=nodes:
                    raise ValueError('SPMD_MSIN is not within the captured INITIA node scope')
                pointers['ITAB']=_context[1]
            _breakpoints.append(Returned(self.routine.lower()+'_return',pointers,nodes))
            return False
        except Exception as error:return failed(error)


def exited(event):
    global _exit_code
    _exit_code=getattr(event,'exit_code',None)


def install(path):
    global _abi,_stream
    if Path('native-startup.jsonl').exists() or Path('native-startup-summary.json').exists():
        raise FileExistsError('Use a fresh native startup observation directory')
    _abi=json.loads(Path(path).read_text())
    if _abi['schema']!='robo_dyna.native_startup_observation_abi.v1':raise ValueError('Unknown startup ABI')
    for pin in _abi['source_pins']:
        data=Path(pin['path']).read_bytes()
        if len(data)!=pin['bytes'] or hashlib.sha256(data).hexdigest()!=pin['sha256']:
            raise ValueError('Pinned Starter donor changed')
    _stream=Path('native-startup.jsonl').open('x')
    gdb.events.exited.connect(exited)
    for name in ('INITIA','SPMD_MSIN'):_breakpoints.append(Entered(name))


def finish():
    counts={name:sum(r['stage']==name for r in _records)
            for name in ('spmd_msin_return','initia_return')}
    complete=(_failure is None and _exit_code==0 and _context is None and
              counts['spmd_msin_return']>0 and counts['spmd_msin_return']==counts['initia_return'])
    with Path('native-startup-summary.json').open('x') as stream:
        json.dump(dict(schema='robo_dyna.native_startup_observation.v1',complete=complete,
                       exit_code=_exit_code,failure=_failure,counts=counts,
                       scope='Reference-only raw startup operands; no TL solver acceptance'),stream,indent=2)
        stream.write('\n')
    _stream.close()
    if not complete:
        if gdb.selected_inferior().pid:gdb.execute('kill')
        gdb.execute('quit 2')
