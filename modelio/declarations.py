"""Immutable typed source records and explicit physical unit conversion."""
from dataclasses import dataclass
import math

from ._legacy import require
from .source_blocks import SourceBlock


@dataclass(frozen=True)
class UnitSystem:
    mass: str
    length: str
    time: str
    mass_to_kg: float
    length_to_m: float
    time_to_s: float

    def __post_init__(self):
        require(all(isinstance(name, str) and name for name in (self.mass, self.length, self.time)),
                'unit names must be explicit')
        require(all(math.isfinite(v) and v > 0 for v in
                    (self.mass_to_kg, self.length_to_m, self.time_to_s)),
                'unit scales must be explicit finite positive values')
        try:
            require(all(math.isfinite(v) and v > 0 for v in (self.density_to_si, self.stress_to_si)),
                    'derived unit scales must remain finite and positive')
        except (OverflowError, ZeroDivisionError) as error:
            raise ValueError('derived unit scales overflow or underflow') from error

    @property
    def density_to_si(self):
        return self.mass_to_kg / self.length_to_m**3

    @property
    def stress_to_si(self):
        return self.mass_to_kg / (self.length_to_m * self.time_to_s**2)


@dataclass(frozen=True)
class TypedCard:
    source_line: int
    raw_data_text: str
    names: tuple
    values: tuple
    blank_field_mask: int

    def get(self, name):
        return self.values[self.names.index(name)]


@dataclass(frozen=True)
class Part:
    part_id: int
    section_id: int
    material_id: int
    title: str
    cards: tuple
    source: SourceBlock


@dataclass(frozen=True)
class SectionShell:
    section_id: int
    source_elform: int
    through_thickness_points: int
    thickness_m: tuple
    cards: tuple
    source: SourceBlock


@dataclass(frozen=True)
class Mat024:
    material_id: int
    density_kg_m3: float
    young_pa: float
    poisson_ratio: float
    supplied_sigy_pa: object
    supplied_etan_pa: object
    hardening_curve_id: int
    rate_coefficient_per_s: float
    rate_exponent: float
    rate_type: object
    cards: tuple
    source: SourceBlock


@dataclass(frozen=True)
class HardeningCurve:
    curve_id: int
    plastic_strain: tuple
    stress_pa: tuple
    documented_defaults: tuple
    cards: tuple
    source: SourceBlock


@dataclass(frozen=True)
class PartDeclarations:
    part: Part
    section: SectionShell
    material: Mat024
    hardening_curve: HardeningCurve
    units: UnitSystem
