"""Typed assembly interfaces; source connections never become guessed forces."""
from dataclasses import asdict, replace

from ._legacy import require
from .attachment_cards import parse_list_set, parse_nodal_rigid, parse_tied_contact
from .canonical_incidence import load_incidence
from .keyword_cards import parse_part


def _frontier_incidence(asset_dir, archive_path, reference, geometry, frontier, limits):
    if not frontier:
        return None
    # Reuse the source-verified incidence collector within its existing 512-node
    # contract. The smallest complete part is a validation seed, not a cut part
    # or a set of invented massless owner nodes. Publish only frontier records.
    seed = min(geometry.parts, key=lambda p: len(p.nodes))
    wanted = sorted(frontier | {n.source_id for n in seed.nodes})
    require(len(wanted) <= 512, 'frontier incidence validation node cap exceeded')
    incidence = load_incidence(asset_dir, archive_path, reference, seed, wanted, limits.incident_elements)
    elements = tuple(replace(e, touched_node_ids=tuple(n for n in e.touched_node_ids if n in frontier))
                     for e in incidence.elements if frontier.intersection(e.touched_node_ids))
    require({n for e in elements for n in e.touched_node_ids} == frontier,
            'frontier node lacks source structural incidence')
    return replace(incidence, nodes=tuple(n for n in incidence.nodes if n.source_id in frontier),
                   elements=elements)


def compile_assembly_attachments(index, weld_inventory, geometry, asset_dir, archive_path, reference, limits):
    selected = frozenset(geometry.source_node_ids)
    part_nodes = {p.part_id: frozenset(n.source_id for n in p.nodes) for p in geometry.parts}
    require(index.sha256 == weld_inventory.source_sha256 == geometry.parts[0].member_sha256,
            'assembly attachment/source identity mismatch')
    cache = {}

    def resolve_set(namespace, identity):
        key = (namespace, identity)
        if key not in cache:
            cache[key] = parse_list_set(index.one(namespace, identity), namespace)
        return cache[key]

    def membership(node_ids):
        return [dict(part_id=pid, node_ids=sorted(ns.intersection(node_ids)))
                for pid, ns in sorted(part_nodes.items()) if ns.intersection(node_ids)]

    groups, welds, tied, frontier = [], [], [], set()
    for family, identity in sorted(index.entries):
        if family != 'nodal_rigid':
            continue
        rigid = parse_nodal_rigid(index.one(family, identity))
        node_set = resolve_set('node_set', rigid.node_set_id)
        # Unsupported referenced set operators fail even before selection: they
        # could otherwise conceal an interface touching the selected component.
        if not selected.intersection(node_set.members):
            continue
        require(len(groups) < limits.groups, 'assembly nodal-rigid group cap exceeded')
        external = set(node_set.members) - selected
        frontier.update(external)
        require(len(frontier) <= limits.frontier_nodes, 'assembly frontier node cap exceeded')
        groups.append(dict(rigid=asdict(rigid), node_set=asdict(node_set),
                           classification='outgoing' if external else 'internal',
                           selected_membership=membership(node_set.members), external_node_ids=sorted(external)))
    for weld in weld_inventory.records:
        if not selected.intersection(weld.node_ids):
            continue
        require(len(welds) < limits.spotwelds, 'assembly spotweld cap exceeded')
        external = set(weld.node_ids) - selected
        frontier.update(external)
        require(len(frontier) <= limits.frontier_nodes, 'assembly frontier node cap exceeded')
        welds.append(dict(weld=asdict(weld), classification='outgoing' if external else 'internal',
                          selected_membership=membership(weld.node_ids), external_node_ids=sorted(external)))
    for family, identity in sorted(index.entries):
        if family != 'tied_contact':
            continue
        contact = parse_tied_contact(index.one(family, identity))
        slave = resolve_set('part_set', contact.slave_set_id)
        master = resolve_set('part_set', contact.master_set_id)
        if not (set(part_nodes).intersection(slave.members) or set(part_nodes).intersection(master.members)):
            continue
        require(len(tied) < limits.groups, 'assembly tied candidate cap exceeded')
        for pid in set(slave.members) | set(master.members):
            parse_part(index.one('part', pid))
        selected_slave = sorted(set(part_nodes).intersection(slave.members))
        selected_master = sorted(set(part_nodes).intersection(master.members))
        tied.append(dict(contact=asdict(contact), slave_part_set=asdict(slave), master_part_set=asdict(master),
                         selected_slave_part_ids=selected_slave, selected_master_part_ids=selected_master,
                         candidate_pair_scope='may_include_internal_pairs' if selected_slave and selected_master
                                              else 'cross_boundary_only',
                         actual_pairing_qualified=False))
    incidence = _frontier_incidence(asset_dir, archive_path, reference, geometry, frontier, limits)
    external_parts = sorted({e.part_id for e in incidence.elements}) if incidence else []
    require(not set(external_parts).intersection(part_nodes), 'frontier node unexpectedly belongs to selected part')
    return dict(nodal_rigid_groups=groups, spotwelds=welds, tied_candidates=tied,
                external_node_ids=sorted(frontier), external_parts=[asdict(parse_part(index.one('part', pid)))
                                                                 for pid in external_parts],
                frontier_incidence=asdict(incidence) if incidence else None,
                whole_source_spotweld_record_count=len(weld_inventory.records),
                source_keyword_counts=dict(weld_inventory.keyword_counts),
                literal_nodal_rigid_and_spotweld_frontier_validated=True,
                full_attachment_closure_qualified=False, attachment_mechanics_implemented=False,
                unresolved=['tied edge projection, offsets, tolerance and actual external pairings',
                            'nodal-rigid and spotweld optional/default mechanics and failure',
                            'other attachment families, point/added masses and assembled setup controls',
                            'general contact, transformed include instances and full vehicle closure'])
