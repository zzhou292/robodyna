"""Checked inverse of the sole mapped removal scheduling substitution."""
import hashlib

def legacy_operations(text):
    replacements = (
        ('#include "RemovalEvents.cuh"\n', ''),
        ('  if(m::removal_events::RemovedPotential(storage,side) && !threadIdx.x) {',
         '  if(storage.control.status==Code::Ok &&\n      m::RemovedPotential(storage,side)) {'),
        ('FinishCandidate<<<1,m::removal_events::Threads,0,state.stream>>>',
         'FinishCandidate<<<1,1,0,state.stream>>>'),
    )
    for current, original in replacements:
        assert text.count(current) == 1, current
        text = text.replace(current, original)
    assert hashlib.sha256(text.encode()).hexdigest() == '74d3a30b47603651a6a5bbfceb7ecbbb08bdec7c91887f77f1292491483daac0'
    return text
