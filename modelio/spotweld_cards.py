"""Bounded literal SPOTWELD_ID records, including repeated records per block.

This inventory retains endpoint IDs and optional fields without interpreting
strength, failure, rotational coupling or blank solver defaults.
"""
from dataclasses import dataclass
import hashlib

from ._legacy import require
from .attachment_cards import identity, literal_card
from .canonical_incidence import _bounded_lines
from .source_blocks import SourceCard, MAX_METADATA_BYTES


@dataclass(frozen=True)
class Spotweld:
    identity: int
    node_ids: tuple
    filename: str
    keyword_line: int
    cards: tuple
    mechanics_qualified: bool = False


@dataclass(frozen=True)
class SpotweldInventory:
    source_sha256: str
    records: tuple
    keyword_counts: tuple


def scan_spotwelds(stream, filename, record_cap=4096):
    require(type(record_cap) is int and 0 < record_cap <= 4096, 'invalid spotweld record cap')
    digest = hashlib.sha256()
    records, seen, counts = [], set(), {}
    keyword = None
    pending = None
    keyword_line = retained = 0
    ended = False
    for number, raw in enumerate(_bounded_lines(stream), 1):
        digest.update(raw)
        text = raw.decode('ascii')
        comment = text.lstrip().startswith('$')
        active = '' if comment else text.split('$', 1)[0].rstrip('\r\n').rstrip()
        if active.lstrip().startswith('*'):
            require(pending is None, 'incomplete final spotweld record')
            require(not ended, 'keyword after *END in spotweld source')
            keyword = active.strip().upper()
            require(counts or keyword == '*KEYWORD', 'missing initial *KEYWORD')
            require(keyword != '*KEYWORD' or not counts, 'duplicate *KEYWORD')
            counts[keyword] = counts.get(keyword, 0) + 1
            keyword_line = number
            ended = keyword == '*END'
            require(not keyword.startswith('*CONSTRAINED_SPOTWELD') or
                    keyword == '*CONSTRAINED_SPOTWELD_ID', 'unsupported spotweld keyword variant')
            continue
        require(keyword is not None or not active.strip(), 'data before *KEYWORD')
        require(not ended or not active.strip(), 'data after *END')
        if keyword != '*CONSTRAINED_SPOTWELD_ID' or comment:
            continue
        retained += len(raw)
        require(retained <= MAX_METADATA_BYTES, 'spotweld metadata byte cap exceeded')
        source = SourceCard(number, active)
        if pending is None:
            first = literal_card(source, ('wid',))
            wid = identity(first.fields[0], 'spotweld ID')
            require(wid not in seen and len(records) < record_cap,
                    'duplicate spotweld ID or spotweld record cap exceeded')
            seen.add(wid)
            pending = (wid, first)
        else:
            second = literal_card(source, ('n1', 'n2', 'sn', 'ss', 'n', 'm', 'tf', 'ep'))
            nodes = tuple(identity(v, 'spotweld node ID') for v in second.fields[:2])
            require(nodes[0] != nodes[1], 'spotweld endpoints must be distinct')
            records.append(Spotweld(pending[0], nodes, filename, keyword_line, (pending[1], second)))
            pending = None
    require(ended and pending is None, 'missing *END or incomplete final spotweld record')
    return SpotweldInventory(digest.hexdigest(), tuple(records), tuple(sorted(counts.items())))
