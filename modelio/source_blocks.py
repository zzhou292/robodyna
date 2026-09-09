"""Bounded streaming metadata index with exact bytes and blank data cards.

Geometry and other keyword bodies are hashed but never interpreted or retained.
All candidate identities are indexed before a requested reference is resolved,
so a later duplicate cannot disappear behind an early successful lookup.
"""
from dataclasses import dataclass
import hashlib

from ._legacy import fields, require, sha256

MAX_SOURCE_BYTES = 64 * 1024 * 1024
MAX_METADATA_BYTES = 4 * 1024 * 1024
MAX_LINE_BYTES = 4096
DECLARATION_FAMILIES = frozenset(('part', 'section', 'material', 'curve'))
ATTACHMENT_FAMILIES = frozenset(('node_set', 'part_set', 'nodal_rigid', 'tied_contact'))


@dataclass(frozen=True)
class SourceCard:
    source_line: int
    text: str


@dataclass(frozen=True)
class SourceBlock:
    keyword: str
    filename: str
    first_line: int
    last_line: int
    raw_text: str
    sha256: str
    cards: tuple


@dataclass
class SourceIndex:
    filename: str
    sha256: str
    source_bytes: int
    keyword_count: int
    entries: dict
    retained_families: frozenset = DECLARATION_FAMILIES

    def one(self, family, identity):
        matches = self.entries.get((family, identity), [])
        require(matches, f'unresolved {family} reference {identity}')
        require(len(matches) == 1, f'duplicate {family} identity {identity}')
        return matches[0]


def _family(keyword):
    if keyword == '*PART' or keyword.startswith('*PART_'):
        return 'part'
    if keyword.startswith('*SECTION_'):
        return 'section'
    if keyword.startswith('*MAT_'):
        return 'material'
    if keyword == '*DEFINE_CURVE' or keyword.startswith('*DEFINE_CURVE_'):
        return 'curve'
    if keyword.startswith('*SET_NODE_'):
        return 'node_set'
    if keyword.startswith('*SET_PART_'):
        return 'part_set'
    if keyword.startswith('*CONSTRAINED_NODAL_RIGID_BODY'):
        return 'nodal_rigid'
    if keyword.startswith('*CONTACT_TIED_SHELL_EDGE_TO_SURFACE'):
        return 'tied_contact'
    return None


def _identity(block, family):
    # A non-ID contact card's first field is SSID, not its identity. Keeping
    # location identity also prevents two contacts with the same SSID merging.
    if family == 'tied_contact':
        return block.first_line
    # PART has a mandatory title. The _TITLE suffix on other families adds one.
    index = int(family == 'part' or block.keyword.endswith('_TITLE'))
    require(len(block.cards) > index, f'{block.keyword}: missing identity card')
    identity = fields(block.cards[index].text[:10], [10], [int])[0][0]
    require(0 < identity < 2**63, f'{block.keyword}: invalid identity')
    return identity


def scan_metadata(stream, filename, *, families):
    """Explicit family retention; unknown variants stay in their ID namespace.

    One SourceIndex represents one untransformed include instance. This scanner
    never merges include instances or evaluates additive/general set operators.
    """
    families = frozenset(families)
    require(families and families <= DECLARATION_FAMILIES | ATTACHMENT_FAMILIES,
            'unsupported metadata family selection')
    entries, digest = {}, hashlib.sha256()
    size = retained = keyword_count = number = 0
    keyword = family = None
    raw_block, cards = [], []
    first = last = 0
    ended = False

    def finish():
        if family is None:
            return
        raw = b''.join(raw_block)
        block = SourceBlock(keyword, filename, first, last, raw.decode('ascii'),
                            sha256(raw), tuple(cards))
        identity = _identity(block, family)
        entries.setdefault((family, identity), []).append(block)

    while True:
        raw = stream.readline(MAX_LINE_BYTES + 1)
        if not raw:
            break
        number += 1
        size += len(raw)
        require(len(raw) <= MAX_LINE_BYTES and size <= MAX_SOURCE_BYTES,
                'source exceeds declared line/file byte cap')
        digest.update(raw)
        text = raw.decode('ascii')
        comment = text.lstrip().startswith('$')
        active = '' if comment else text.split('$', 1)[0].rstrip('\r\n').rstrip()
        if active.lstrip().startswith('*'):
            require(not ended, f'{filename}:{number}: keyword after *END')
            finish()
            keyword = active.strip().upper()
            require(keyword_count or keyword == '*KEYWORD', 'missing initial *KEYWORD')
            require(keyword != '*KEYWORD' or not keyword_count, 'duplicate *KEYWORD')
            keyword_count += 1
            require(keyword_count <= 10000, 'metadata keyword-count cap exceeded')
            candidate = _family(keyword)
            family = candidate if candidate in families else None
            raw_block, cards, first = [], [], number
            ended = keyword == '*END'
        else:
            require(keyword is not None or not active.strip(), 'data before *KEYWORD')
            require(not ended or not active.strip(), 'data after *END')
            if family is not None and not comment:
                cards.append(SourceCard(number, active))  # Empty lines are cards.
                require(len(cards) <= 4096, 'metadata block card-count cap exceeded')
        last = number
        if family is not None:
            retained += len(raw)
            require(retained <= MAX_METADATA_BYTES, 'retained metadata byte cap exceeded')
            raw_block.append(raw)
    finish()
    require(ended, 'missing final *END')
    return SourceIndex(filename, digest.hexdigest(), size, keyword_count, entries, families)


def scan_declarations(stream, filename):
    """Original E1 family selection and public signature are unchanged."""
    return scan_metadata(stream, filename, families=DECLARATION_FAMILIES)
