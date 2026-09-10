"""Literal shell interface inventory and source-PID incidence, no pair invention."""
from collections import Counter
from dataclasses import asdict

from .attachment_cards import parse_nodal_rigid,parse_list_set,parse_tied_contact
from .full_shell_declarations import block_record
from .full_shell_coverage import node_part_incidence
from ._legacy import require


def _partition(nodes,selected):
    inside=sorted(set(nodes).intersection(selected));outside=sorted(set(nodes)-selected)
    return dict(classification='internal' if inside and not outside else 'cross_boundary' if inside else 'outside',
                retained_node_ids=inside,nonretained_node_ids=outside)


def connection_coverage(source,selected_parts,selected_nodes,part_counts):
    index,g=source.index,source.geometry
    groups,welds,ties,unresolved,wanted=[],[],[],[],set()
    for (family,identity),blocks in sorted(index.entries.items()):
        if family=='nodal_rigid':
            record=dict(identity=identity,source=block_record(blocks[0]))
            try:
                rigid=parse_nodal_rigid(blocks[0]);nodes=parse_list_set(index.one('node_set',rigid.node_set_id),'node_set')
                require(all(n in g.node_index for n in nodes.members),'rigid group has unresolved source nodes')
                record.update(node_set_id=rigid.node_set_id,source_node_ids=list(nodes.members),
                              node_set_source=block_record(nodes.source),
                              unresolved_source_options=list(rigid.unsupported+nodes.unsupported),
                              **_partition(nodes.members,selected_nodes),
                              mechanics_status='native group family exists; complete source options and group interactions need admission')
                wanted.update(nodes.members)
            except (ValueError,OverflowError) as error:
                record.update(classification='unresolved',reason=str(error));unresolved.append(record)
            groups.append(record)
        elif family=='tied_contact':
            record=dict(source=block_record(blocks[0]),pairing_qualified=False)
            try:
                tied=parse_tied_contact(blocks[0])
                slave=parse_list_set(index.one('part_set',tied.slave_set_id),'part_set')
                master=parse_list_set(index.one('part_set',tied.master_set_id),'part_set')
                record.update(slave_part_ids=list(slave.members),master_part_ids=list(master.members),
                    retained_slave_part_ids=sorted(selected_parts.intersection(slave.members)),
                    retained_master_part_ids=sorted(selected_parts.intersection(master.members)),
                    slave_source=block_record(slave.source),master_source=block_record(master.source),
                    unresolved_source_options=list(tied.unsupported+slave.unsupported+master.unsupported),
                    load_path_status='Tied slave parts may be omitted beam welds; zero shell slaves does not discharge shell load-path closure')
            except (ValueError,OverflowError) as error:record['reason']=str(error);unresolved.append(record)
            ties.append(record)
    for weld in source.welds.records:
        require(all(n in g.node_index for n in weld.node_ids),'spotweld has unresolved source nodes')
        record=asdict(weld);record.update(_partition(weld.node_ids,selected_nodes))
        record['mechanics_status']='TYPE25 direct-import family exists; all source options/counts require admission'
        welds.append(record);wanted.update(weld.node_ids)
    # Keep every remaining structural/load/contact/setup block and its original
    # location/hash. Unparsed endpoint operators are not a resolved load path.
    other=[]
    for filename,summary in source.source_files.items():
        for block in summary['blocks']:
            k=block['keyword']
            if k.startswith(('*CONSTRAINED_','*CONTACT_','*AIRBAG_','*ELEMENT_MASS','*ELEMENT_DISCRETE',
                             '*LOAD_','*INITIAL_','*BOUNDARY_','*INCLUDE','*DEFINE_TRANSFORMATION')):
                other.append(dict(block,scope='retained source obligation; unqualified unless separately resolved'))
    for tied in ties:
        for side in ('slave', 'master'):
            tied[side+'_part_geometry']=[dict(source_part_id=pid,
                counts=part_counts.get(pid, {}),
                source_material_id=g.parts[pid]['source_material_id'],
                material_keyword=g.materials[g.parts[pid]['source_material_id']]['keyword'])
                for pid in tied.get(side+'_part_ids', [])]
    incidence=node_part_incidence(g,wanted)
    for row in incidence:
        row['retained_shell_part_ids']=sorted(selected_parts.intersection(row['parts_by_family'].get('shells',[])))
    return dict(nodal_rigid_groups=groups,spotwelds=welds,tied_contacts=ties,
                classifications={key:dict(Counter(r['classification'] for r in rows))
                                 for key,rows in [('nodal_rigid',groups),('spotweld',welds)]},
                interface_node_part_incidence=incidence,unresolved_literal_records=unresolved,
                other_source_obligations=other,
                incidence_scope='all literal vehicle-include nodal-rigid and SPOTWELD_ID endpoints; joints/tied pairs and auxiliary include operators remain unresolved',
                boundary_policy='unassigned: no external connection has been silently released',
                full_load_path_closure_qualified=False)
