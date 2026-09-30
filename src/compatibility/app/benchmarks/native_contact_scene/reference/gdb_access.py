"""Read-only Linux x86-64 GNU Fortran MYREAL8 call-entry access for this pinned Engine."""
import math
import struct

import gdb

MAX_VALUES = 1 << 18


def values(address, count, code):
    if type(count) is not int or not 0 <= count <= MAX_VALUES:
        raise ValueError('Native observation extent exceeds bounded scope')
    if not count:
        return []
    if not address:
        raise ValueError('Null native operand')
    payload=gdb.selected_inferior().read_memory(address,count*struct.calcsize(code)).tobytes()
    out=list(struct.unpack('<'+str(count)+code,payload))
    if code in ('d','f') and not all(math.isfinite(v) for v in out):
        raise ValueError('A requested source observation is nonfinite/undefined')
    return out


def common(symbol, offset, code):
    base=int(gdb.parse_and_eval(f'(void*)&{symbol}'))
    return values(base+offset*struct.calcsize(code),1,code)[0]


def clock():
    return dict(zip(('TT','DT1','DT2_estimator','DT12','DT2OLD','TSTOP'),
                    values(int(gdb.parse_and_eval('(void*)&com08_')),6,'d')),
                NCYCLE=common('com01_',4,'i'), NSPMD=common('com01_',20,'i'),
                IRESP=common('scr05_',4,'i'))


class Call:
    def __init__(self, names):
        self.index={name:i for i,name in enumerate(names)}

    def pointer(self, name):
        index=self.index[name]
        if index < 6:
            return int(gdb.parse_and_eval('$'+('rdi','rsi','rdx','rcx','r8','r9')[index]))
        rsp=int(gdb.parse_and_eval('$rsp'))
        return values(rsp+8*(index-5),1,'Q')[0]

    def array(self, name, count, code='d'):
        return values(self.pointer(name),count,code)

    def scalar(self, name, code='i'):
        return self.array(name,1,code)[0]
