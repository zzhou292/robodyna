"""Typed one-hop attachment inventory; never an assembled constraint closure.

Only one original, untransformed source include is resolved. Literal list sets
and nodal-rigid NSID links are selected by actual node membership. Full vehicle
closure, general/additive operators, optional/default mechanics and tied pairing
remain unqualified. Numeric coincidences across ID namespaces are not links.
"""
from dataclasses import dataclass

from ._legacy import require, fields
from .attachment_cards import ListSet, NodalRigid, TiedContact, parse_list_set, parse_nodal_rigid, parse_tied_contact
from .canonical_incidence import CanonicalIncidence, load_incidence
from .source_blocks import SourceBlock


@dataclass(frozen=True)
class AttachmentLimits:
    nodes: int = 512
    groups: int = 64
    incident_elements: int = 4096

    def __post_init__(self):
        for value, maximum in ((self.nodes, 512), (self.groups, 64), (self.incident_elements, 4096)):
            require(type(value) is int and 0 < value <= maximum, 'invalid attachment cap')


@dataclass(frozen=True)
class RigidGroup:
    rigid: NodalRigid
    node_set: ListSet
    selected_node_ids: tuple
    external_node_ids: tuple


@dataclass(frozen=True)
class TiedCandidate:
    contact: TiedContact
    slave_part_set: ListSet
    master_part_set: ListSet
    selected_part_in_slave_set: bool
    selected_part_in_master_set: bool
    actual_pairing_qualified: bool = False


@dataclass(frozen=True)
class IncidentPart:
    part_id: int
    section_id: int
    material_id: int
    title: str
    source: SourceBlock


@dataclass(frozen=True)
class NodeIncidence:
    node_id: int
    in_selected_part: bool
    incident_element_ids: tuple
    incident_part_ids: tuple


@dataclass(frozen=True)
class AttachmentScope:
    source_instance: tuple
    selected_part_id: int
    selected_node_ids: tuple
    rigid_groups: tuple
    tied_candidates: tuple
    group_member_node_ids: tuple
    selected_group_node_ids: tuple
    external_group_node_ids: tuple
    neighboring_part_ids: tuple
    incidence: CanonicalIncidence
    node_incidence: tuple
    incident_parts: tuple
    unsupported_set_blocks: tuple
    unresolved: tuple
    literal_one_hop_references_validated: bool = True
    full_attachment_closure_qualified: bool = False
    attachment_mechanics_implemented: bool = False
    tied_pairing_qualified: bool = False
    cut_boundary_supplied: bool = False
    simulation_ready: bool = False


def _part(block):
    require(block.keyword == '*PART' and len(block.cards) == 2, 'unsupported incident PART shape')
    pid, sid, mid = fields(block.cards[1].text[:30], [10]*3, [int]*3)[0]
    require(min(pid, sid, mid) > 0, 'invalid incident PART references')
    return IncidentPart(pid, sid, mid, block.cards[0].text, block)


def compile_attachment_scope(index, geometry, asset_dir, archive_path, reference, limits=AttachmentLimits()):
    require(index.filename == 'yaris-coarse-v1l.key' and index.sha256 == geometry.member_sha256,
            'attachment include/source mismatch')
    required = {'part', 'node_set', 'part_set', 'nodal_rigid', 'tied_contact'}
    require(getattr(index, 'retained_families', frozenset()) >= required,
            'attachment index lacks explicit family coverage')
    selected = frozenset(n.source_id for n in geometry.nodes)
    require(len(selected) == len(geometry.nodes) and len(selected) <= limits.nodes and
            0 < len(geometry.shells) <= 256, 'invalid selected attachment geometry/cap')
    cache = {}
    def resolve_set(namespace, identity):
        key = (namespace, identity)
        if key not in cache:
            cache[key] = parse_list_set(index.one(namespace, identity), namespace)
        return cache[key]

    groups = []
    union = set(selected)
    # Resolve every nodal-rigid record before membership selection. An unknown
    # referenced set operator cannot silently hide a group touching this part.
    for family, identity in sorted(index.entries):
        if family != 'nodal_rigid':
            continue
        rigid = parse_nodal_rigid(index.one(family, identity))
        node_set = resolve_set('node_set', rigid.node_set_id)
        touched = selected.intersection(node_set.members)
        if not touched:
            continue
        require(len(groups) < limits.groups, 'selected nodal-rigid group cap exceeded')
        union.update(node_set.members)
        require(len(union) <= limits.nodes, 'selected/external attachment node cap exceeded')
        groups.append(RigidGroup(rigid, node_set, tuple(sorted(touched)),
                                 tuple(sorted(set(node_set.members) - selected))))

    candidates = []
    for family, identity in sorted(index.entries):
        if family != 'tied_contact':
            continue
        contact = parse_tied_contact(index.one(family, identity))
        slave = resolve_set('part_set', contact.slave_set_id)
        master = resolve_set('part_set', contact.master_set_id)
        is_slave = geometry.part_id in slave.members
        is_master = geometry.part_id in master.members
        if is_slave or is_master:
            require(len(candidates) < limits.groups, 'tied candidate cap exceeded')
            # Literal part references in both sets must name unique source PARTs;
            # this does not compile their material or geometric closure.
            for pid in set(slave.members) | set(master.members):
                _part(index.one('part', pid))
            candidates.append(TiedCandidate(contact, slave, master, is_slave, is_master))
    incidence = load_incidence(asset_dir, archive_path, reference, geometry, sorted(union), limits.incident_elements)
    incident_parts = tuple(_part(index.one('part', pid)) for pid in sorted({e.part_id for e in incidence.elements}))
    node_incidence = tuple(NodeIncidence(nid, nid in selected,
                                        tuple(sorted(e.source_id for e in incidence.elements if nid in e.touched_node_ids)),
                                        tuple(sorted({e.part_id for e in incidence.elements if nid in e.touched_node_ids})))
                           for nid in sorted(union))
    require(all(n.incident_element_ids for n in node_incidence), 'attachment node lacks canonical structural incidence')
    members = {n for g in groups for n in g.node_set.members}
    external = members - selected
    # Shared selected nodes can also connect another part directly, even when
    # they are not members of a selected nodal-rigid group.
    neighbors = {pid for n in node_incidence for pid in n.incident_part_ids}
    neighbors.discard(geometry.part_id)
    unsupported = tuple(block for (family, _), blocks in sorted(index.entries.items())
                        if family in ('node_set', 'part_set') for block in blocks
                        if block.keyword not in ('*SET_NODE_LIST', '*SET_NODE_LIST_TITLE',
                                                 '*SET_PART_LIST', '*SET_PART_LIST_TITLE'))
    return AttachmentScope((geometry.archive_sha256, geometry.source_member), geometry.part_id,
                           tuple(sorted(selected)), tuple(groups), tuple(candidates), tuple(sorted(members)),
                           tuple(sorted(members & selected)), tuple(sorted(external)), tuple(sorted(neighbors)),
                           incidence, node_incidence, incident_parts, unsupported,
                           ('source/include transforms and assembled setup are outside this original-frame inventory',
                            'nodal-rigid optional/default cards are retained but their mechanics are unsupported',
                            'unreferenced general/additive sets are not evaluated; referenced unknown operators fail',
                            'other attachment/contact families, added masses and load-transfer mechanics are unqualified',
                            'one-hop incidence is not transitive closure; no neighbor geometry or cut constraints are generated',
                            'tied master/slave set membership does not establish edge projection or attachment pairs'))
