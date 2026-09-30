"""Reversible screen schedule and literal original trace proof, not runtime code."""
from pathlib import Path
import re
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
def body(text, name):
    start = text.index('{', text.index(name+'('))
    end, depth = start+1, 1
    while depth:
        depth += (text[end] == '{')-(text[end] == '}')
        end += 1
    return text[start+1:end-1]
def same(a, b, label):
    assert re.sub(r'\s+', '', a) == re.sub(r'\s+', '', b), label

def legacy_kernels(current):
    old = (HERE/'reference/Screen.cu').read_text()
    value = current.replace('#include "GroupScreen.cuh"\n', '')
    cooperative = body(value, 'EvaluateGroups')
    same(cooperative, '''
      if (input.control->status != NodalStatus::Ok || input.screen[0].invalid_node != UINT32_MAX) return;
      __shared__ group_screen::Tile tile;
      group_screen::ScreenGroup(input, blockIdx.x, tile);
    ''', 'only group scheduling kernel changes')
    value = value.replace(cooperative, body(old, 'EvaluateGroups'))
    value = value.replace('EvaluateGroups<<<input.groups.group_count, group_screen::Threads, 0, stream>>>',
        'const auto blocks = 1+(input.groups.group_count-1)/groups::Threads;\n  EvaluateGroups<<<blocks, groups::Threads, 0, stream>>>')
    assert value == old, 'complete old screen, all phase guards and publication'
    return old

def legacy_build(current):
    """Reverse exactly the three private cooperative owner header exports."""
    addition = ', "cin_advance/GroupResponse.h", "cin_advance/GroupScreenValues.h", "cin_advance/GroupScreen.cuh"'
    assert current.count(addition) == 1
    return current.replace(addition, '')

def prove():
    old = (HERE/'reference/Rigid.h').read_text()
    now = (ROOT/'lib_src/solvers/cin_timestep/Rigid.h').read_text()
    expected = body(old, 'AddRigidMemberTrace').replace(
        'contact::EvaluateRigidNormalResponse(body, position, axis, response) != contact::Status::kOk',
        '!read_response(body, position, axis, response)')
    same(body(now, 'AddRigidMemberTraceWithResponse'), expected, 'literal ordered trace fold')
    same(body(now, 'RigidTraceLimit'), body(old, 'RigidTraceLimit'), 'unchanged final trace bound')
    same(body(now, 'AddRigidMemberTrace'), '''
      return detail::AddRigidMemberTraceWithResponse(body, position, translation, rotation,
          trace, detail::DirectRigidResponse{});
    ''', 'legacy direct fresh wrapper')
    assert 'EvaluateRigidNormalResponse(body, position, axis, output)' in now
    for name,path in [('ScreenValues.h','cin_timestep/ScreenValues.h'),
        ('Screen.h','cin_timestep/Screen.h'), ('ScreenSummary.h','cin_advance/ScreenSummary.h'),
        ('Groups.h','cin_advance/Groups.h'), ('Capture.h','cin_limiter/Capture.h')]:
        assert (HERE/'reference'/name).read_bytes() == (ROOT/'lib_src/solvers'/path).read_bytes(), path
    legacy_kernels((ROOT/'lib_src/solvers/cin_advance/Screen.cu').read_text())
    response = (ROOT/'lib_src/solvers/cin_advance/GroupResponse.h').read_text()
    assert 'MemberResponses next{};' in response
    assert 'EvaluateRigidNormalResponse(body, position, axes[axis], response)' in response
    assert 'Status::kOk) break;' in response
    assert response.index('if (!value.valid) return false;') < response.index('output =')
    fold = (ROOT/'lib_src/solvers/cin_advance/GroupScreenValues.h').read_text()
    for name in ('PrepareMember','FoldMember'):
        value = body(fold, name)
        assert value.index('node >= source.nodes') < value.index('source.cin_secondary[node]')
        assert value.index('source.cin_secondary[node]') < value.index('source.accepted+3*node')
    member = body(fold, 'FoldMember')
    assert member.index('if (state.stopped) return;') < member.index('source.rigid.members[member]')
    assert member.index('state.report.last_node = node;') < member.index('node >= source.nodes')
    begin = body(fold, 'BeginGroup')
    assert begin.index('range.count > groups.member_count-range.offset') < begin.index('groups.members[range.offset]')
    assert begin.index('out.report.visited = true;') < begin.index('PrepareRigidContactBodyFromAccepted')
    kernel = (ROOT/'lib_src/solvers/cin_advance/GroupScreen.cuh').read_text()
    assert 'atomic' not in kernel and 'cudaMalloc' not in kernel
    assert 'for (unsigned local = 0; local < count && !tile.group.stopped; ++local)' in kernel
    assert 'first += count;' in kernel and kernel.count('__syncthreads();') == 4
    assert 'sizeof(Tile) == 4776' in fold
if __name__ == '__main__':
    prove()
