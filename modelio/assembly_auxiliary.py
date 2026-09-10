"""Literal external point-mass/joint evidence; never owner or mechanics input."""
from dataclasses import asdict
import math

from ._legacy import fields, require

POLICY = 'released_external_auxiliary_nodes_v1'


def auxiliary_frontier(index, missing, units):
    """V3 admits only an explicitly retained mass + spherical-joint node role.

    Complete original blocks remain evidence, including unrelated mass cards.
    Referencing another joint endpoint does not recursively select that node.
    """
    missing = set(missing)
    blocks, masses, joints = {}, [], []
    seen_masses, seen_joints = set(), set()
    for (family, _), entries in sorted(index.entries.items()):
        if family not in ('point_mass', 'joint'):
            continue
        for block in entries:
            if family == 'point_mass':
                for ci, card in enumerate(block.cards):
                    if not card.text.strip():
                        continue
                    values, mask = fields(card.text, [8, 8, 16], [int, int, float])
                    eid, nid, mass = values
                    if nid not in missing:
                        continue
                    require(block.keyword == '*ELEMENT_MASS' and mask == 0 and eid > 0 and
                            eid not in seen_masses and math.isfinite(mass) and mass > 0 and
                            not card.text[32:].strip(), 'unsupported external auxiliary point-mass card')
                    si = mass * units.mass_to_kg
                    require(math.isfinite(si) and si > 0, 'external auxiliary mass conversion overflow')
                    seen_masses.add(eid); blocks[block.first_line] = block
                    masses.append(dict(source_element_id=eid, source_node_id=nid, source_block_line=block.first_line,
                                       source_card_index=ci, supplied_mass_source=mass, supplied_mass_kg=si))
            else:
                # All original joint variants stay visible to this match scan.
                # Only the two-card spherical ID form is classified here.
                if len(block.cards) < 2:
                    continue
                tokens = block.cards[1].text.split()
                if not any(token.isdigit() and int(token) in missing for token in tokens):
                    continue
                require(block.keyword == '*CONSTRAINED_JOINT_SPHERICAL_ID' and len(block.cards) == 2,
                        'unsupported external auxiliary joint card')
                identity, imask = fields(block.cards[0].text[:10], [10], [int])
                nodes, nmask = fields(block.cards[1].text, [10, 10], [int, int])
                require(imask == nmask == 0 and identity[0] > 0 and identity[0] not in seen_joints and
                        all(n > 0 for n in nodes) and nodes[0] != nodes[1] and
                        not block.cards[0].text[10:].strip() and not block.cards[1].text[20:].strip(),
                        'unsupported external spherical-joint options')
                seen_joints.add(identity[0]); blocks[block.first_line] = block
                joints.append(dict(source_joint_id=identity[0], source_node_ids=nodes,
                                   source_block_line=block.first_line))
    require({m['source_node_id'] for m in masses} == missing and
            missing <= {n for j in joints for n in j['source_node_ids']},
            'frontier node lacks supported literal auxiliary mass/joint evidence')
    return dict(policy=POLICY, source_node_ids=sorted(missing),
                source_blocks=[asdict(b) for _, b in sorted(blocks.items())],
                point_masses=sorted(masses, key=lambda m: (m['source_block_line'], m['source_card_index'])),
                spherical_joints=sorted(joints, key=lambda j: j['source_block_line']),
                mechanics_qualified=False, selected_owner_nodes_added=False, selected_owner_mass_added=False,
                interpretation='Original external auxiliary mass and joint cards retained; complete outgoing groups released; no joint or mass mechanics admitted')
