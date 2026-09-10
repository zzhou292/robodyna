"""Source declaration coverage; unsupported cards remain visible and selected."""
from collections import Counter

from .assembly_declarations import assembly_section
from .keyword_cards import parse_part, parse_material, parse_curve
from .full_shell_materials import material_fields, material_dispositions
from .law44_declarations import linear_law44_candidates


def block_record(block):
    return dict(keyword=block.keyword, file=block.filename, first_line=block.first_line,
                last_line=block.last_line, sha256=block.sha256,
                cards=[dict(source_line=c.source_line, text=c.text) for c in block.cards])


def declaration_coverage(source, geometry_rows, selected_parts, excluded):
    g,index,units=source.geometry,source.index,source.units
    parts, grouped= [], Counter()
    for pid, part in sorted(g.parts.items()):
        sid,mid=part['source_section_id'],part['source_material_id']
        pblock,sblock,mblock=index.one('part',pid),index.one('section',sid),index.one('material',mid)
        reasons=[];material=None;curve_id=None
        for label, parser, block in [('part',parse_part,pblock),
                ('section',lambda b:assembly_section(b,units),sblock),
                ('material',lambda b:parse_material(b,units),mblock)]:
            try:
                parsed=parser(block)
                if label=='material':material=parsed
            except (ValueError,OverflowError) as error:reasons.append(dict(family=label,reason=str(error)))
        if material:
            curve_id=material.hardening_curve_id
            try:parse_curve(index.one('curve',curve_id),units)
            except (ValueError,OverflowError) as error:reasons.append(dict(family='curve',reason=str(error)))
        counts=geometry_rows.get(pid,dict(shells=0,q4=0,t3=0,solids=0,beams=0))
        selected=pid in selected_parts
        if selected:
            status='existing_declaration_adapter_candidate' if not reasons else 'unsupported_declaration_retained'
            grouped[(sblock.keyword,g.sections[sid]['formulation_field_raw'],mblock.keyword,status)]+=counts['shells']
        else:status='explicit_tire_shell_exclusion' if pid in excluded else 'non_shell_part_outside_selection'
        parts.append(dict(part,counts=counts,retained_shell_part=selected,disposition=status,
                          existing_adapter_rejections=reasons,parsed_hardening_curve_id=curve_id,
                          source_part_sha256=pblock.sha256))
    tables={family:[dict(identity=identity,**block_record(blocks[0]))
                    for (kind,identity),blocks in sorted(index.entries.items()) if kind==family]
            for family in ('section','material','curve')}
    for row in tables['material']:
        row['mat024_literal_fields']=material_fields(index.one('material',row['identity']))
    families=[dict(section_keyword=k[0],source_elform=k[1],material_keyword=k[2],
                   disposition=k[3],shells=n) for k,n in sorted(grouped.items())]
    return dict(parts=parts,tables=tables,retained_shell_families=families,
                material_dispositions=material_dispositions(index,parts),
                linear_law44_candidates=linear_law44_candidates(index,parts,units),
                interpretation='Adapter candidates retain source options; native startup, capacities and physical equivalence are not qualified by this report')
