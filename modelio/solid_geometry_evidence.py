"""Selected declaration provenance for the geometry fixture, without defaults."""
from dataclasses import asdict
import math
import struct

from ._legacy import fields, require
from .keyword_cards import card, PART


def _scalar(value):
    return dict(value=value, binary64_le=struct.pack('<d', value).hex())


def declaration_evidence(index, manifest, part_ids):
    """Current exporter contract: original plain low-density-foam declaration.

    Geometry selection is material-independent. This small declaration adapter
    only resolves the four fields needed to join the radiator's source evidence;
    all remaining cards/options remain exact raw text, without solver defaults.
    """
    parts, blocks = [], {}

    def retain(family, identity):
        block = index.one(family, identity)
        blocks[(family, identity)] = dict(family=family, identity=identity, **asdict(block))
        return block

    for pid in part_ids:
        block = retain('part', pid)
        require(block.keyword == '*PART' and len(block.cards) == 2, 'unsupported selected PART shape')
        data = card(block.cards[1], PART, [int] * 8)
        sid, mid = data.get('secid'), data.get('mid')
        require(data.get('pid') == pid and type(sid) is int and sid > 0 and
                type(mid) is int and mid > 0, 'invalid selected PART references')
        original = [p for p in manifest['parts'] if p['source_part_id'] == pid]
        require(len(original) == 1 and original[0]['source_section_id'] == sid and
                original[0]['source_material_id'] == mid and
                original[0]['title'] == block.cards[0].text.strip() and
                original[0]['source_line'] == block.first_line and
                original[0]['blank_field_mask'] == data.blank_field_mask and
                tuple(original[0]['raw_fields']) == tuple(v if v is not None else 0 for v in data.values),
                'canonical/source PART association mismatch')
        section, material = retain('section', sid), retain('material', mid)
        require(section.keyword == '*SECTION_SOLID', 'selected section is not original solid')
        require(material.keyword == '*MAT_LOW_DENSITY_FOAM' and material.cards,
                'unsupported selected density/curve declaration')
        values, mask = fields(material.cards[0].text[:40], [10]*4, [int, float, float, int])
        require(values[0] == mid and values[1] > 0 and values[3] > 0 and not mask,
                'missing selected density or curve identity')
        curve_id = values[3]
        curve = retain('curve', curve_id)
        require(curve.keyword == '*DEFINE_CURVE', 'unsupported selected raw curve variant')
        density = values[1]
        density_si = density * 1e12  # Original t/mm^3 -> kg/m^3, one multiplication.
        require(math.isfinite(density_si) and density_si > 0, 'density SI conversion overflow/underflow')
        parts.append(dict(part_id=pid, section_id=sid, material_id=mid, curve_id=curve_id,
                          density_source_field=material.cards[0].text[10:20],
                          density_source_line=material.cards[0].source_line,
                          density_tonne_per_mm3=_scalar(density), density_kg_per_m3=_scalar(density_si)))
    return dict(parts=parts, blocks=[blocks[k] for k in sorted(blocks)],
                interpretation='Original declarations and density units only; no material or element admission')
