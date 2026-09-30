"""Exact whole-shell selection and source part incidence, without a mass model."""
from collections import Counter, defaultdict
import hashlib
import struct

from ._legacy import FIELDS, require

# Exact original shell tire bodies AND tread. Rims, disks and hubs are
# distinct source parts and must never be removed by a title substring match.
TIRE_SHELL_PARTS = {2000211: '435_tirefrontleft', 2000360: '521_tirerearleft',
                    2000363: '524_tirerearright', 2000367: '518_tirefrontright',
                    2000487: '638_TireTread', 2000488: '639_TireTread',
                    2000489: '640_TireTread', 2000490: '641_TireTread'}


def tire_exclusions(geometry, policy):
    require(policy in ('retain_all', 'omit_original_tire_shells'), 'unknown full-shell tire policy')
    if policy == 'retain_all':
        return set()
    for pid, title in TIRE_SHELL_PARTS.items():
        part = geometry.parts.get(pid)
        require(part is not None and part['title'] == title and
                part['source_section_id'] == pid and part['source_material_id'] == pid and
                geometry.sections[pid]['keyword'] == '*SECTION_SHELL' and
                geometry.materials[pid]['keyword'] == '*MAT_ELASTIC',
                'original tire-shell identity differs; no inferred tire exclusion')
    return set(TIRE_SHELL_PARTS)


def coverage(geometry, excluded):
    require(set(excluded) <= set(geometry.parts), 'unknown excluded source part')
    rows, selected_nodes, selected_parts = {}, set(), set()
    selected, omitted = Counter(shells=0,q4=0,t3=0), Counter(shells=0,q4=0,t3=0)
    digests = {name: hashlib.sha256() for name in ('retained_shell_ids', 'excluded_shell_ids')}
    for family, records in geometry.elements.items():
        width = len(FIELDS[family])
        for offset in range(0, len(records), width):
            eid, pid = records[offset:offset+2]
            require(pid in geometry.parts, 'missing source element part')
            row = rows.setdefault(pid, dict(shells=0, q4=0, t3=0, solids=0, beams=0))
            row[family] += 1
            if family != 'shells':
                continue
            nodes = records[offset+2:offset+width]
            arity = len(set(nodes));require(arity in (3, 4), 'invalid original shell arity')
            row['q4' if arity == 4 else 't3'] += 1
            keep = pid not in excluded
            (selected if keep else omitted)['shells'] += 1
            (selected if keep else omitted)['q4' if arity == 4 else 't3'] += 1
            digests['retained_shell_ids' if keep else 'excluded_shell_ids'].update(struct.pack('<Q', eid))
            if keep:
                selected_parts.add(pid);selected_nodes.update(nodes)
    node_digest = hashlib.sha256()
    for nid in sorted(selected_nodes):node_digest.update(struct.pack('<Q', nid))
    nonshell = {}
    for family in ('solids', 'beams'):
        records=geometry.elements[family];width=len(FIELDS[family]);touching=Counter()
        for offset in range(0,len(records),width):
            nodes=records[offset+2:offset+(4 if family=='beams' else width)]
            if selected_nodes.intersection(nodes):touching[records[offset+1]]+=1
        nonshell[family]=dict(elements=len(records)//width, touching_retained_shell_nodes=sum(touching.values()),
                             touching_source_parts=[dict(part_id=p,elements=n) for p,n in sorted(touching.items())],
                             mass_removed_kg=None, inertia_removed_kg_m2=None,
                             disposition='not selected by shell scope; mass and load paths unresolved, not zero')
    summary=dict(retained=dict(selected,parts=len(selected_parts),nodes=len(selected_nodes)),
                 excluded=dict(omitted,parts=len(excluded)), non_shell=nonshell,
                 element_id_digest_encoding='unsigned64 little endian; original canonical shell record order',
                 node_id_digest_encoding='unsigned64 little endian; ascending retained source NID',
                 selected_node_ids_sha256=node_digest.hexdigest(),
                 **{k+'_sha256':v.hexdigest() for k,v in digests.items()})
    return summary, rows, selected_parts, selected_nodes


def node_part_incidence(geometry, wanted):
    result = {nid: defaultdict(set) for nid in wanted}
    for family, records in geometry.elements.items():
        width=len(FIELDS[family])
        for offset in range(0,len(records),width):
            pid=records[offset+1];stop=offset+(4 if family=='beams' else width)
            for nid in set(records[offset+2:stop]).intersection(wanted):result[nid][family].add(pid)
    return [dict(source_node_id=nid, canonical_node_index=geometry.node_index.get(nid),
                 parts_by_family={f:sorted(p) for f,p in sorted(result[nid].items())}) for nid in sorted(wanted)]
