"""Literal typed attachment cards; optional values never become solver defaults.

The narrow field names follow the pinned deck's own header cards. Additional
nodal-rigid data cards remain opaque fixed-width fields, including empty cards.
No mass, constraint equations, projection or attachment mechanics are evaluated.
"""
from dataclasses import dataclass

from ._legacy import require
from .source_blocks import SourceBlock


@dataclass(frozen=True)
class LiteralCard:
    source_line: int
    raw_text: str
    names: tuple
    fields: tuple
    blank_mask: int


@dataclass(frozen=True)
class ListSet:
    namespace: str
    identity: int
    title: str | None
    members: tuple
    cards: tuple
    source: SourceBlock
    unsupported: tuple


@dataclass(frozen=True)
class NodalRigid:
    identity: int
    node_set_id: int
    title: str | None
    cards: tuple
    source: SourceBlock
    unsupported: tuple


@dataclass(frozen=True)
class TiedContact:
    source_identity: tuple
    slave_set_id: int
    master_set_id: int
    slave_type: int
    master_type: int
    cards: tuple
    source: SourceBlock
    unsupported: tuple


def literal_card(source, names=()):
    # Preserve field text and source bytes independently. Eight fields suffice
    # for admitted named headers; wider optional cards are retained verbatim.
    count = max(len(names), 8, (len(source.text) + 9) // 10)
    names = tuple(names) + tuple('unparsed_' + str(i + 1) for i in range(len(names), count))
    values = tuple(source.text[10*i:10*i+10] for i in range(count))
    mask = sum(1 << i for i, value in enumerate(values) if not value.strip())
    return LiteralCard(source.source_line, source.text, names, values, mask)


def identity(text, label):
    require(text.strip().isascii() and text.strip().isdigit(), 'invalid ' + label)
    value = int(text)
    require(0 < value < 2**63, 'invalid ' + label)
    return value


def _title(block):
    start = int(block.keyword.endswith('_TITLE'))
    require(len(block.cards) > start, 'missing attachment identity card')
    return (block.cards[0].text if start else None), start


def parse_list_set(block, namespace, member_cap=32768):
    require(namespace in ('node_set', 'part_set'), 'invalid set namespace')
    stem = '*SET_NODE_LIST' if namespace == 'node_set' else '*SET_PART_LIST'
    require(block.keyword in (stem, stem + '_TITLE'), 'unsupported set operator/keyword ' + block.keyword)
    title, start = _title(block)
    raw = tuple(literal_card(c, ('sid', 'da1', 'da2', 'da3', 'da4', 'solver') if i == start else ())
                for i, c in enumerate(block.cards) if i >= start)
    sid = identity(raw[0].fields[0], namespace + ' ID')
    unsupported = ('set header optional fields are not evaluated',) if any(v.strip() for v in raw[0].fields[1:]) else ()
    members, seen = [], set()
    for row in raw[1:]:
        require(not row.raw_text[80:].strip(), 'set list exceeds eight member fields')
        for value in row.fields[:8]:
            if not value.strip():
                continue
            member = identity(value, 'set member ID')
            require(member not in seen, 'duplicate set member')
            require(len(members) < member_cap, 'set member cap exceeded')
            seen.add(member)
            members.append(member)
    require(members, 'empty literal list set')
    return ListSet(namespace, sid, title, tuple(members), raw, block, unsupported)


def parse_nodal_rigid(block):
    require(block.keyword in ('*CONSTRAINED_NODAL_RIGID_BODY', '*CONSTRAINED_NODAL_RIGID_BODY_TITLE'),
            'unsupported nodal-rigid keyword ' + block.keyword)
    title, start = _title(block)
    header = literal_card(block.cards[start], ('pid', 'cid', 'nsid', 'pnode', 'iprt', 'drflag', 'rrflag'))
    rigid_id = identity(header.fields[0], 'rigid-body ID')
    nsid = identity(header.fields[2], 'nodal-rigid node-set reference')
    cards = (header,) + tuple(literal_card(c) for c in block.cards[start+1:])
    unsupported = ['nodal-rigid mechanics and blank/default options are not evaluated']
    if any(value.strip() for i, value in enumerate(header.fields) if i not in (0, 2)):
        unsupported.append('supplied optional rigid header fields are retained without interpretation')
    if len(cards) > 1:
        unsupported.append('all additional rigid cards are retained but unparsed, including blank cards')
    return NodalRigid(rigid_id, nsid, title, cards, block, tuple(unsupported))


def parse_tied_contact(block):
    require(block.keyword == '*CONTACT_TIED_SHELL_EDGE_TO_SURFACE', 'unsupported tied-contact keyword ' + block.keyword)
    require(block.cards, 'missing tied-contact reference card')
    header = literal_card(block.cards[0], ('ssid', 'msid', 'sstyp', 'mstyp', 'sboxid', 'mboxid', 'spr', 'mpr'))
    ssid, msid, stype, mtype = (identity(header.fields[i], 'tied reference/type') for i in range(4))
    require(stype == 2 and mtype == 2, 'only explicit tied part-set reference types are inventoried')
    cards = (header,) + tuple(literal_card(c) for c in block.cards[1:])
    return TiedContact((block.filename, block.first_line), ssid, msid, stype, mtype, cards, block,
                       ('set membership is a candidate scope, never a node/edge pair',
                        'all remaining contact fields, defaults and projection/tolerance semantics are unparsed'))
