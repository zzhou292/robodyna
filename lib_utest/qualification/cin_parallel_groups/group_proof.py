"""Checked compatibility views for prior scheduling receipts, never runtime code.

Earlier gates keep their complete old arithmetic/phase proofs. This module proves
the new reversible owner scheduling delta and extracted screen group body before
offering the exact prior view to those proofs. Frozen reference bytes never move.
"""
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

def legacy_owner(current):
    value = current.replace('#include "cin_advance/Groups.h"\n', '')
    value = value.replace('const cin_advance::Input input, bool groups_prepared = false)',
        'const cin_advance::Input input)')
    value = value.replace('!groups_prepared && g < groups.group_count', 'g < groups.group_count')
    addition = '''  const bool parallel_groups = input.group_reports && input.groups.group_count;
  if (parallel_groups) {
    error = groups::LaunchMotion(input, stream);
    if (error != cudaSuccess) return error;
  }
'''
    assert value.count(addition) == 1
    value = value.replace(addition, '')
    value = value.replace('CompleteCin<<<1,1,0,stream>>>(input, parallel_groups);',
        'CompleteCin<<<1,1,0,stream>>>(input);')
    value = value.replace('cin->screen, cin->group_reports}, stream)', 'cin->screen}, stream)')
    assert value == (HERE/'reference/ExplicitNodalCinStep.cu').read_text(), 'exact owner scheduling delta'
    return value

def legacy_values(current):
    old = (HERE/'reference/ScreenValues.h').read_text()
    for name in ('CheckSources', 'EvaluateNode'):
        same(body(old, name), body(current, name), name)
    loop = body(old, 'EvaluateGroups')
    start = loop.index('  for (std::uint32_t group')
    selected = loop[loop.index('{', start)+1:loop.rfind('}')]
    selected = selected.replace('invalid_node = groups.members[range.offset].node;',
        'out.visited = true; out.first_node = out.last_node = groups.members[range.offset].node;')
    selected = selected.replace('invalid_node = node;', 'out.last_node = node;')
    selected = selected.replace('return false;', 'return out;')
    selected = selected.replace('ScalarLimit limit;', '')
    selected = selected.replace('RigidTraceLimit(trace, factor, limit)', 'RigidTraceLimit(trace, factor, out.limit)')
    selected = selected.replace('Include(limit, groups.members[range.offset].node, group, next);', '')
    expected = 'GroupEvaluation out; const auto groups = source.rigid;'+selected+'out.valid = true; return out;'
    same(body(current, 'EvaluateGroup'), expected, 'exact group body/member trace order')
    same(body(current, 'EvaluateGroups'), '''
      for (std::uint32_t group = 0; group < source.rigid.group_count; ++group) {
        const auto value = EvaluateGroup(source, factor, group);
        if (value.visited) invalid_node = value.last_node;
        if (!value.valid) return false;
        Include(value.limit, value.first_node, group, next);
      }
      return true;
    ''', 'serial wrapper/carry/min order')
    return old

def legacy_kernels(current):
    old = (HERE/'reference/Screen.cu').read_text()
    for name in ('Begin', 'Reduce', 'Nodes'):
        same(body(current, name), body(old, name), 'screen '+name)
    finish = body(old, 'Finish')
    marker = '  cin_timestep::Result result;'
    split = finish.index(marker)
    same(body(current, 'Finish'), finish[:split]+'''  if (!parallel_groups) Publish(input, values[0], false);
''', 'same ordinary summary and deferred publication')
    tail = finish[split:]
    selected = '  if (!Complete(Sources(input), input.structural.factor, values[0], result, invalid_node)) {'
    replacement = '''  const auto source = Sources(input);
  const bool valid = parallel_groups
      ? groups::CompleteScreen(source, ordinary, input.group_reports, result, invalid_node)
      : Complete(source, input.structural.factor, ordinary, result, invalid_node);
  if (!valid) {'''
    assert selected in tail
    same(body(current, 'Publish'), tail.replace(selected, replacement), 'same failure/bound/key publication')
    return old

if __name__ == '__main__':
    legacy_owner((ROOT/'lib_src/solvers/ExplicitNodalCinStep.cu').read_text())
    legacy_values((ROOT/'lib_src/solvers/cin_timestep/ScreenValues.h').read_text())
    legacy_kernels((ROOT/'lib_src/solvers/cin_advance/Screen.cu').read_text())
