"""Bounded shell-scene declarations with explicit contact and coupling profiles."""
from dataclasses import dataclass
import math

from output.json_object import read_object


def require(condition, message):
    if not condition:
        raise ValueError(message)


def keys(value, fields, name):
    require(isinstance(value, dict) and set(value) == set(fields), f"{name}: invalid fields")


def number(value, name, minimum=None, maximum=None):
    require(type(value) in (int, float) and math.isfinite(value), f"{name}: finite number required")
    require(minimum is None or value >= minimum, f"{name}: below lower bound")
    require(maximum is None or value <= maximum, f"{name}: above upper bound")
    return float(value)


def axis(value, name):
    require(isinstance(value, list) and 2 <= len(value) <= 33, f"{name}:2..33 coordinates required")
    out = tuple(number(v, name, -10000, 10000) for v in value)
    require(all(a < b for a, b in zip(out, out[1:])), f"{name}: strictly increasing coordinates required")
    return out


@dataclass(frozen=True)
class Grid:
    x_mm: tuple
    y_mm: tuple
    z_mm: float
    dz_dx: float


@dataclass(frozen=True)
class Material:
    density_tonne_mm3: float
    young_n_mm2: float
    poisson: float
    yield_n_mm2: float
    plastic_hardening_n_mm2: float
    rate_c_per_s: float
    rate_p: float
    rate_filter_hz: float
    law: str = 'law44_linear'


@dataclass(frozen=True)
class RigidPatch:
    kind: str
    primary_initialization: str
    inertia_mode: int
    center_of_gravity: int
    tied_secondary_removal: int
    primary_velocity: str


@dataclass(frozen=True)
class TiedPatch:
    kind: str
    dependent: Grid
    ignore: int
    spotflag: int
    level: int
    search: int
    deletion: int
    search_distance_mm: float
    stiffness_scale: float
    viscosity: float
    stiffness_mode: int
    tied_secondary_removal: int


def tied_patch(value):
    controls = {'kind': 'tied_patch', 'ignore': 2, 'spotflag': 28, 'level': 0,
                'search': 0, 'deletion': 1, 'search_distance_mm': 0.,
                'stiffness_scale': 1., 'viscosity': .05, 'stiffness_mode': 2,
                'tied_secondary_removal': 1}
    keys(value, tuple(controls)+('dependent',), 'coupling')
    require(all(value[k] == v and type(value[k]) is type(v) for k, v in controls.items()),
            'Only explicit declared TYPE2 ILEV28/Irem_i2=1 controls are admitted')
    raw = value['dependent']
    keys(raw, ('x_mm', 'y_mm', 'z_mm', 'dz_dx'), 'dependent patch')
    grid = Grid(axis(raw['x_mm'], 'dependent x'), axis(raw['y_mm'], 'dependent y'),
                number(raw['z_mm'], 'dependent z', -10000, 10000),
                number(raw['dz_dx'], 'dependent slope', -.5, .5))
    require(len(grid.x_mm) == len(grid.y_mm) == 2 and grid.dz_dx == 0.,
            'The declared tied dependent is one flat Q4')
    return TiedPatch(dependent=grid, **controls)


def rigid_patch(value):
    fields = {'kind': 'rigid_patch', 'primary_initialization': 'converted_part',
              'inertia_mode': 2, 'center_of_gravity': 1,
              'tied_secondary_removal': 1, 'primary_velocity': 'patch_translation'}
    keys(value, fields, 'coupling')
    require(value == fields and all(type(value[k]) is type(v) for k, v in fields.items()),
            'Only explicit converted-part rigid patch controls are admitted')
    return RigidPatch(**value)


@dataclass(frozen=True)
class Scene:
    wall: Grid
    patch: Grid
    velocity_mm_s: tuple
    material: Material
    thickness_mm: float
    end_time_s: float
    nodal_scale: float
    animation_interval_s: float
    time_step_cap_s: float | None = None
    contact_surface: str = "fixed_wall"
    definition_version: int = 1
    coupling: RigidPatch | TiedPatch | None = None


def load(path):
    raw, _ = read_object(path, max_bytes=128 << 10)
    require(isinstance(raw, dict), 'Scene must be an object')
    version = {'robo_dyna.native_contact_scene.v1': 1,
               'robo_dyna.native_contact_scene.v2': 2,
               'robo_dyna.native_contact_scene.v3': 3,
               'robo_dyna.native_contact_scene.v4': 4}.get(raw.get('schema'))
    require(version is not None, 'Unknown scene schema')
    fields = ('schema', 'units', 'wall', 'patch', 'material', 'thickness_mm', 'run')
    keys(raw, fields + (('contact_surface',) if version >= 2 else ()) +
         (('coupling',) if version >= 3 else ()), 'scene')
    contact_surface = 'fixed_wall'
    if version >= 2:
        require(raw['contact_surface'] == 'all_shells', 'Moving surface versions require explicit all_shells contact')
        contact_surface = raw['contact_surface']
    require(raw['units'] == {'length': 'mm', 'mass': 'tonne', 'time': 's'}, 'Only explicit native mm/tonne/s is admitted')
    grids = []
    for name in ('wall', 'patch'):
        fields = ('x_mm', 'y_mm', 'z_mm', 'dz_dx') + (('velocity_mm_s',) if name == 'patch' else ())
        obj = raw[name]
        keys(obj, fields, name)
        grids.append(Grid(axis(obj['x_mm'], name), axis(obj['y_mm'], name),
                          number(obj['z_mm'], name, -10000, 10000), number(obj['dz_dx'], name, -.5, .5)))
    require(2*(len(grids[0].x_mm)-1)*(len(grids[0].y_mm)-1) >= 4, 'At least4 genuine main faces required')
    velocity = raw['patch']['velocity_mm_s']
    require(isinstance(velocity, list) and len(velocity) == 3, 'Three velocity components required')
    velocity = tuple(number(v, 'velocity', -100000, 100000) for v in velocity)
    require(velocity[2] < 0, 'This impact scene approaches its wall along negative z')
    material = raw['material']
    keys(material, ('law', 'density_tonne_mm3', 'young_n_mm2', 'poisson', 'yield_n_mm2',
                    'plastic_hardening_n_mm2', 'rate_c_per_s', 'rate_p', 'rate_filter_hz'), 'material')
    require(material['law'] == 'law44_linear', 'Only existing analytic LAW44 is admitted')
    density = number(material['density_tonne_mm3'], 'density', 1e-15, 1e-3)
    young = number(material['young_n_mm2'], 'Young modulus', 1e-6, 1e9)
    poisson = number(material['poisson'], 'Poisson ratio', 0, .499)
    constitutive = Material(density, young, poisson,
        number(material['yield_n_mm2'], 'yield stress', 1e-6, young),
        number(material['plastic_hardening_n_mm2'], 'plastic hardening modulus', 0, young),
        number(material['rate_c_per_s'], 'rate C', 1e-9, 1e12),
        number(material['rate_p'], 'rate P', .1, 100),
        number(material['rate_filter_hz'], 'rate filter', 1e-9, 1e12))
    thickness = number(raw['thickness_mm'], 'thickness', 1e-6, 100)
    run = raw['run']
    run_fields = ('end_time_s', 'nodal_scale', 'animation_interval_s')
    if isinstance(run, dict) and 'time_step_cap_s' in run:
        run_fields += ('time_step_cap_s',)
    keys(run, run_fields, 'run')
    end = number(run['end_time_s'], 'end time', 1e-9, .1)
    scale = number(run['nodal_scale'], 'nodal scale', .01, .9)
    cadence = number(run['animation_interval_s'], 'animation interval', 1e-9, end)
    require(end/cadence <= 10000, 'Animation count exceeds scene cap')
    step_cap = None
    if 'time_step_cap_s' in run:
        step_cap = number(run['time_step_cap_s'], 'time step cap', 1e-12, end)
    wall, patch = grids
    require(wall.dz_dx == 0, 'This first fixed-main definition uses a planar wall')
    require(min(patch.z_mm+patch.dz_dx*x for x in patch.x_mm) > wall.z_mm+thickness,
            'Scene starts separated; initial overlap requires a separately named case')
    coupling = rigid_patch(raw['coupling']) if version == 3 else tied_patch(raw['coupling']) if version == 4 else None
    if version == 4:
        dependent = coupling.dependent
        require(len(patch.x_mm) == len(patch.y_mm) == 2 and patch.dz_dx == 0.,
                'The declared TYPE2 master is one flat Q4')
        require(dependent.z_mm == patch.z_mm and
                patch.x_mm[0] < dependent.x_mm[0] < dependent.x_mm[1] < patch.x_mm[1] and
                patch.y_mm[0] < dependent.y_mm[0] < dependent.y_mm[1] < patch.y_mm[1],
                'Dependent corners must be coplanar and strictly inside the declared master')
    return Scene(wall, patch, velocity, constitutive, thickness, end, scale, cadence,
                 step_cap, contact_surface, version, coupling)
