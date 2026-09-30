"""Literal MAT024 mode/rejection census, not new constitutive admission."""
import math
import re

from .keyword_cards import card, MAT1, MAT2, integral_flag


def material_fields(block):
    if block.keyword not in ('*MAT_024', '*MAT_PIECEWISE_LINEAR_PLASTICITY'):
        return None
    if len(block.cards) != 4:
        return dict(disposition='unexpected MAT024 card count', cards=len(block.cards))
    first=card(block.cards[0],MAT1,[int]+[float]*7)
    second=card(block.cards[1],MAT2,[float,float,int,int,integral_flag,float,float,float])
    values=dict(zip(first.names,first.values));values.update(zip(second.names,second.values))
    inline=[card(row,tuple(str(i) for i in range(8))).values for row in block.cards[2:]]
    lcss=values['lcss']
    mode=('positive_curve_reference' if lcss is not None and lcss>0 else
          'inline_points_supplied' if any(v is not None for row in inline for v in row) else
          'SIGY_ETAN_fields_without_positive_LCSS')
    invalid=[n for n in ('mid','ro','e','lcss','c','p')
             if values[n] is None or not math.isfinite(values[n]) or values[n]<=0]
    return dict(source_line=first.source_line, source_values=values,
                hardening_declaration=mode, inline_source_values=inline,
                strict_positive_admission_failures=invalid,
                disposition='literal source fields only; omitted/default/rate and hardening semantics need separate qualification')


def material_dispositions(index,parts):
    modes={};reasons={};optional={}
    for part in parts:
        if not part['retained_shell_part']:continue
        pid,mid,n=part['source_part_id'],part['source_material_id'],part['counts']['shells']
        block=index.one('material',mid)
        representative=dict(source_part_id=pid,source_material_id=mid,source_line=block.first_line)
        for rejection in part['existing_adapter_rejections']:
            # Source lines are retained per part; normalize only the summary key.
            reason=re.sub(r'line \d+: ', '', rejection['reason'])
            key=(rejection['family'],reason)
            row=reasons.setdefault(key,dict(family=key[0],reason=key[1],shells=0,parts=0,representatives=[]))
            row['shells']+=n;row['parts']+=1
            if len(row['representatives'])<3:row['representatives'].append(representative)
        fields=material_fields(block)
        if fields is None:continue
        values=fields.get('source_values',{})
        for name in ('fail','tdel','lcsr','vp','reserved6','reserved7','reserved8'):
            value=values.get(name)
            if value is None:continue
            row=optional.setdefault((name,value),dict(field=name,source_value=value,shells=0,parts=0,representatives=[]))
            row['shells']+=n;row['parts']+=1
            if len(row['representatives'])<3:row['representatives'].append(representative)
        missing=tuple(fields.get('strict_positive_admission_failures',()))
        key=(fields.get('hardening_declaration','unexpected_card_count'),missing,
             tuple(values.get(k) for k in ('lcss','sigy','etan','c','p')))
        row=modes.setdefault(key,dict(hardening_declaration=key[0],strict_positive_admission_failures=list(missing),
            source_values={k:values.get(k) for k in ('lcss','sigy','etan','c','p')},
            shells=0,parts=0,representatives=[]))
        row['shells']+=n;row['parts']+=1
        if len(row['representatives'])<3:row['representatives'].append(representative)
    return dict(rejection_summary=sorted(reasons.values(),key=lambda r:(-r['shells'],r['family'],r['reason'])),
        mat024_source_modes=sorted(modes.values(),key=lambda r:(-r['shells'],r['representatives'][0]['source_part_id'])),
        mat024_supplied_optional_fields=sorted(optional.values(),key=lambda r:(r['field'],r['source_value'])),
        interpretation='Reason counts can overlap. Positive LCSS is an explicit table reference; this report does not invent SIGY/ETAN, inline or zero-C/P semantics.')
