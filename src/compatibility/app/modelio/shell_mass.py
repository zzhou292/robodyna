"""Provisional midsurface lamina mass; no source shell/lumping equivalence.

Q4 uses its bilinear map and a sufficient global Jacobian certificate. T3 uses
exact triangle moments. Quadrature data comes from a separately qualified donor.
"""
from dataclasses import dataclass
import math
import sys

from ._legacy import require, file_sha256
from .canonical_geometry import read_json

MOMENT_TOLERANCE = 2e-12
REFINEMENT_TOLERANCE = 1e-9


def subtract(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(v):
    return math.hypot(*v)


def _dot(a, b):
    return math.fsum(x * y for x, y in zip(a, b))


def metrics(p):
    """Historical probe metric: diagonal-02 triangle area, never a mass rule."""
    edges = [norm(subtract(p[(i + 1) % 4], p[i])) for i in range(4)]
    d02 = subtract(p[2], p[0])
    n012 = cross(subtract(p[1], p[0]), d02)
    n023 = cross(d02, subtract(p[3], p[0]))
    length0, length1 = norm(n012), norm(n023)
    angle = None
    if length0 > 0 and length1 > 0:
        angle = math.degrees(math.atan2(norm(cross(n012, n023)), _dot(n012, n023)))
    minimum, maximum = min(edges), max(edges)
    return dict(warp_deg=angle, edge_aspect=maximum / minimum if minimum > 0 else None,
                split_area_m2=.5 * (length0 + length1),
                min_diagonal_triangle_area_m2=.5 * min(length0, length1),
                min_edge_m=minimum, max_edge_m=maximum)


@dataclass(frozen=True)
class Rule:
    order: int
    nodes: tuple
    weights: tuple

    def __post_init__(self):
        require(type(self.order) is int and self.order in (4, 8, 16), 'unsupported quadrature order')
        require(len(self.nodes) == len(self.weights) == self.order, 'quadrature shape mismatch')
        require(all(type(x) in (int, float) and math.isfinite(x) and abs(x) < 1 for x in self.nodes) and
                all(type(w) in (int, float) and math.isfinite(w) and w > 0 for w in self.weights),
                'invalid quadrature node/weight')
        require(all(self.nodes[i] > self.nodes[i + 1] for i in range(self.order - 1)),
                'expected distinct descending donor nodes')
        require(all(abs(self.nodes[i] + self.nodes[-1 - i]) <= MOMENT_TOLERANCE and
                    abs(self.weights[i] - self.weights[-1 - i]) <= MOMENT_TOLERANCE
                    for i in range(self.order)), 'quadrature symmetry mismatch')
        for x in self.nodes:
            p0, p1 = 1., x
            for degree in range(2, self.order + 1):
                p0, p1 = p1, ((2 * degree - 1) * x * p1 - (degree - 1) * p0) / degree
            require(abs(p1) <= MOMENT_TOLERANCE, 'quadrature Legendre root residual mismatch')
        for degree in range(2 * self.order):
            exact = 0. if degree % 2 else 2. / (degree + 1)
            actual = math.fsum(w * x**degree for x, w in zip(self.nodes, self.weights))
            require(abs(actual - exact) <= MOMENT_TOLERANCE, 'quadrature polynomial-moment mismatch')


@dataclass(frozen=True)
class Quadrature:
    rules: tuple
    artifact_sha256: str
    donor: str
    source_files: tuple
    version: str
    rejected_reference: tuple

    def __post_init__(self):
        require(tuple(r.order for r in self.rules) == (4, 8, 16), 'require all 4/8/16 quadrature rules')


def _load_quadrature(path):
    d = read_json(path, 64 * 1024)
    require(d['schema'] == 'robo-dyna.shell-surface-quadrature.v1' and
            d['polynomial_moments_qualified'] is True and
            d['legendre_root_residuals_qualified'] is True and
            d['moment_absolute_tolerance'] == MOMENT_TOLERANCE, 'unqualified quadrature artifact')
    require(d['donor'] == 'boost::math::quadrature::gauss<double,N>' and d['donor_version'] == '1_74' and
            type(d['donor_version_integer']) is int and d['donor_version_integer'] == 107400 and
            d['donor_license'] == 'Boost Software License 1.0', 'unexpected quadrature donor/version/license')
    sources = d['reported_source_files']
    require(len(sources) == 6 and {s['path'] for s in sources} ==
            {'boost/math/quadrature/gauss.hpp', 'boost/math/special_functions/legendre.hpp',
             'boost/math/tools/roots.hpp', 'boost/math/policies/policy.hpp', 'boost/version.hpp', 'package-copyright'} and
            all(isinstance(s['path'], str) and isinstance(s['sha256'], str) and
                len(s['sha256']) == 64 and all(c in '0123456789abcdef' for c in s['sha256']) for s in sources),
            'missing quadrature source provenance')
    rules = tuple(Rule(r['order'], tuple(r['nodes']), tuple(r['weights'])) for r in d['rules'])
    rejected = d['rejected_reference_diagnostic']
    require(rejected['donor'] == 'chrono::ChQuadratureTables' and rejected['order'] == 16 and
            rejected['qualified'] is False and math.isfinite(rejected['max_legendre_root_residual']) and
            rejected['max_legendre_root_residual'] > MOMENT_TOLERANCE and
            math.isfinite(rejected['max_polynomial_moment_absolute_error']), 'missing retained reference failure')
    keys = ('donor', 'order', 'qualified', 'max_legendre_root_residual', 'max_polynomial_moment_absolute_error')
    return Quadrature(rules, file_sha256(path), d['donor'], tuple((s['path'], s['sha256']) for s in sources),
                      d['donor_version'], tuple((k, rejected[k]) for k in keys))


def load_quadrature(path):
    try:
        return _load_quadrature(path)
    except (KeyError, TypeError, IndexError, RecursionError, OverflowError) as error:
        raise ValueError('malformed quadrature artifact') from error


def _finite(values, message):
    require(all(math.isfinite(v) for v in values), message)


@dataclass(frozen=True)
class _Surface:
    points: tuple  # Coordinates relative to this element's first source vertex.
    size: float
    coefficients: tuple
    jacobian_dot_lower_bound: float
    jacobian_roundoff_margin: float


def _prepare(points):
    require(len(points) in (3, 4) and all(len(p) == 3 for p in points), 'expected T3 or Q4')
    _finite([x for p in points for x in p], 'nonfinite geometry')
    relative = tuple(subtract(p, points[0]) for p in points)
    size = max(abs(x) for p in relative for x in p)
    require(math.isfinite(size) and size > 0, 'zero/nonfinite surface scale')
    scaled = tuple(tuple(x / size for x in p) for p in relative)
    area_scale = size * size
    require(math.isfinite(area_scale) and area_scale > 0, 'surface area scale overflow/underflow')
    if len(points) == 3:
        normal = cross(scaled[1], scaled[2])
        margin = 64 * sys.float_info.epsilon * max(1., norm(scaled[1]) * norm(scaled[2]))
        require(norm(normal) > margin,
                'degenerate triangle')
        return _Surface(tuple(points), size, (), norm(normal), margin)
    signs = ((1, 1, 1, 1), (-1, 1, 1, -1), (-1, -1, 1, 1), (1, -1, 1, -1))
    a, b, c, d = tuple(tuple(.25 * math.fsum(s[i] * scaled[i][j] for i in range(4))
                              for j in range(3)) for s in signs)
    center = cross(b, c)
    center_norm = norm(center)
    require(center_norm > 0 and math.isfinite(center_norm), 'zero center Jacobian')
    direction = tuple(x / center_norm for x in center)
    derivatives = [(tuple(b[j] + d[j] * v for j in range(3)), tuple(c[j] + d[j] * u for j in range(3)))
                   for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    corners = [cross(xu, xv) for xu, xv in derivatives]
    lower = min(_dot(n, direction) for n in corners)
    # x_u cross x_v is affine in u,v: its projection on a FIXED direction is
    # affine too, so positive corner projections certify the entire square.
    # Corner norms alone do not do this. This is sufficient, not necessary;
    # valid surfaces outside this fixed-direction chart are explicitly rejected.
    # Bound the operations forming the cross products, not their potentially
    # cancellation-small results. The unit floor in normalized coordinates also
    # covers cancellation in the coefficient/derivative construction. Very thin
    # valid maps can therefore be rejected as outside this conditioned domain.
    margin = 64 * sys.float_info.epsilon * max(1., *(norm(xu) * norm(xv) for xu, xv in derivatives))
    require(math.isfinite(lower) and lower > margin,
            'bilinear Jacobian regularity certificate failed')
    return _Surface(tuple(points), size, (a, b, c, d), lower, margin)


def _integrate(surface, rule, origin):
    if len(surface.points) == 3:
        p = tuple(subtract(x, origin) for x in surface.points)
        area = .5 * surface.jacobian_dot_lower_bound * surface.size**2
        s = tuple(math.fsum(x[j] for x in p) for j in range(3))
        first = tuple(area * x / 3 for x in s)
        second = tuple(area / 12 * (s[i] * s[j] + math.fsum(x[i] * x[j] for x in p))
                       for i in range(3) for j in range(3))
        return area, first, second
    a, b, c, d = surface.coefficients
    base = subtract(surface.points[0], origin)
    samples = []
    for u, wu in zip(rule.nodes, rule.weights):
        for v, wv in zip(rule.nodes, rule.weights):
            xu = tuple(b[j] + d[j] * v for j in range(3))
            xv = tuple(c[j] + d[j] * u for j in range(3))
            area = wu * wv * norm(cross(xu, xv)) * surface.size**2
            require(math.isfinite(area) and area > 0, 'quadrature area overflow/underflow')
            p = tuple(base[j] + surface.size * (a[j] + b[j] * u + c[j] * v + d[j] * u * v)
                      for j in range(3))
            samples.append((area, p))
    area = math.fsum(w for w, _ in samples)
    first = tuple(math.fsum(w * p[j] for w, p in samples) for j in range(3))
    second = tuple(math.fsum(w * p[i] * p[j] for w, p in samples) for i in range(3) for j in range(3))
    return area, first, second


def _element(surface, rule, areal_density, origin_from_anchor):
    area, first, _ = _integrate(surface, rule, (0., 0., 0.))
    require(math.isfinite(area) and area > 0, 'nonpositive/nonfinite surface area')
    center = tuple(x / area for x in first)
    # Second geometric pass about each LOCAL centroid. Only then add the element
    # origin offset for common-anchor aggregation. A remote tiny element must
    # not lose its representable edges by subtracting a distant part anchor first.
    _, _, central = _integrate(surface, rule, center)
    mass = area * areal_density
    q = tuple(x * areal_density for x in central)
    _finite((area, mass) + center + q, 'nonfinite surface moments')
    require(mass > 0 and all(q[i] >= 0 for i in (0, 4, 8)) and math.fsum(q[i] for i in (0, 4, 8)) > 0,
            'surface moment underflow/invalid inertia')
    return dict(area_m2=area, mass_kg=mass, centroid_from_element_origin_m=center,
                centroid_from_anchor_m=tuple(x + y for x, y in zip(center, origin_from_anchor)),
                central_second_moment_kg_m2=q)


def _aggregate(elements):
    area = math.fsum(e['area_m2'] for e in elements)
    mass = math.fsum(e['mass_kg'] for e in elements)
    require(math.isfinite(mass) and mass > 0, 'invalid aggregate mass')
    center = tuple(math.fsum(e['mass_kg'] * e['centroid_from_anchor_m'][j] for e in elements) / mass
                   for j in range(3))
    offsets = [subtract(e['centroid_from_anchor_m'], center) for e in elements]
    q = tuple(math.fsum(e['central_second_moment_kg_m2'][3 * i + j] + e['mass_kg'] * r[i] * r[j]
                       for e, r in zip(elements, offsets)) for i in range(3) for j in range(3))
    _finite((area, mass) + center + q, 'nonfinite aggregate moments')
    return dict(area_m2=area, mass_kg=mass, centroid_from_anchor_m=center, central_second_moment_kg_m2=q)


def _difference(a, b, size):
    scale = b['mass_kg'] * size * size
    require(math.isfinite(scale) and scale > 0, 'moment normalization overflow/underflow')
    centroid_key = 'centroid_from_element_origin_m' if 'centroid_from_element_origin_m' in a else 'centroid_from_anchor_m'
    result = dict(area_relative=abs(a['area_m2'] - b['area_m2']) / b['area_m2'],
                  mass_relative=abs(a['mass_kg'] - b['mass_kg']) / b['mass_kg'],
                  centroid_over_size=norm(subtract(a[centroid_key], b[centroid_key])) / size,
                  central_second_moment_over_mass_size_squared=max(abs(x - y) for x, y in
                      zip(a['central_second_moment_kg_m2'], b['central_second_moment_kg_m2'])) / scale)
    _finite(tuple(result.values()), 'nonfinite refinement diagnostic')
    return result


def _publish(value, anchor):
    result = dict(value)
    q = value['central_second_moment_kg_m2']
    result['centroid_m'] = tuple(a + x for a, x in zip(anchor, value['centroid_from_anchor_m']))
    result['central_inertia_kg_m2'] = tuple(math.fsum(q[4 * k] for k in range(3) if k != i) if i == j else -q[3 * i + j]
                                          for i in range(3) for j in range(3))
    _finite(result['centroid_m'] + result['central_inertia_kg_m2'], 'published moments overflow')
    return result


def audit_shell_surface(geometry, declarations, quadrature, *, refinement_tolerance=REFINEMENT_TOLERANCE):
    """Return a fully staged audit; any invalid element prevents publication.

    The optional tolerance may only tighten the fixed 1e-9 production budget.
    It exists for independent nonconvergence tests, not automatic relaxation.
    Source authentication is the canonical loader's responsibility. Direct
    callers supply the same immutable identity/connectivity contract; cheap
    structural checks below reject contradictory identities, not source forgeries.
    """
    require(type(refinement_tolerance) in (int, float) and math.isfinite(refinement_tolerance) and
            0 < refinement_tolerance <= REFINEMENT_TOLERANCE, 'invalid refinement tolerance')
    require(geometry.part_id == declarations.part.part_id and geometry.section_id == declarations.section.section_id and
            geometry.material_id == declarations.material.material_id, 'mass declaration join mismatch')
    thickness = declarations.section.thickness_m
    require(len(thickness) == 4 and all(type(t) in (int, float) and math.isfinite(t) and t > 0 for t in thickness) and
            all(t == thickness[0] for t in thickness), 'requires four explicit equal positive thicknesses')
    density = declarations.material.density_kg_m3
    require(type(density) in (int, float) and math.isfinite(density) and density > 0, 'invalid source density')
    areal_density = density * thickness[0]
    require(math.isfinite(areal_density) and areal_density > 0, 'areal density overflow/underflow')
    require(geometry.nodes and geometry.shells and len(geometry.nodes) <= 512 and len(geometry.shells) <= 256,
            'geometry outside bounded surface-audit scope')
    anchor = geometry.nodes[0].position_m
    positions = tuple(subtract(n.position_m, anchor) for n in geometry.nodes)
    part_size = max(norm(p) for p in positions)
    require(math.isfinite(part_size) and part_size > 0, 'invalid part size')
    require(len({n.source_id for n in geometry.nodes}) == len(geometry.nodes) and
            all(type(n.source_id) is int and 0 < n.source_id < 2**64 for n in geometry.nodes),
            'duplicate/invalid source node identity')
    require(len({s.source_id for s in geometry.shells}) == len(geometry.shells), 'duplicate source element identity')
    surfaces, offsets = [], []
    for shell in geometry.shells:
        require(shell.arity in (3, 4) and len(shell.local_node_indices) == 4 and
                all(type(i) is int and 0 <= i < len(positions) for i in shell.local_node_indices),
                'invalid shell-to-node mapping')
        require(type(shell.source_id) is int and 0 < shell.source_id < 2**64 and len(shell.raw_record) == 6 and
                shell.raw_record[:2] == (shell.source_id, geometry.part_id) and
                shell.raw_record[2:] == tuple(geometry.nodes[i].source_id for i in shell.local_node_indices) and
                len(set(shell.local_node_indices)) == shell.arity and
                (shell.arity == 4 or shell.local_node_indices[2] == shell.local_node_indices[3]),
                'contradictory source shell identity/connectivity')
        world = tuple(geometry.nodes[i].position_m for i in shell.local_node_indices[:shell.arity])
        p = tuple(subtract(x, world[0]) for x in world)
        surfaces.append(_prepare(p))
        offsets.append(subtract(world[0], anchor))
    try:
        by_order = [[_element(s, rule, areal_density, origin) for s, origin in zip(surfaces, offsets)]
                    for rule in quadrature.rules]
        totals = [_aggregate(elements) for elements in by_order]
        element_reports = []
        for i, (shell, surface) in enumerate(zip(geometry.shells, surfaces)):
            differences = [_difference(by_order[j][i], by_order[j + 1][i], surface.size) for j in (0, 1)]
            require(max(differences[1].values()) <= refinement_tolerance,
                    f'element {shell.source_id}: surface quadrature did not converge')
            entry = dict(source_element_id=shell.source_id, arity=shell.arity,
                         jacobian_certificate='positive fixed-direction corner projections over full bilinear square'
                            if shell.arity == 4 else 'nondegenerate exact triangle',
                         scaled_jacobian_lower_bound=surface.jacobian_dot_lower_bound,
                         scaled_jacobian_roundoff_margin=surface.jacobian_roundoff_margin,
                         orders={str(r.order): _publish(by_order[j][i], anchor) for j, r in enumerate(quadrature.rules)},
                         refinement_4_to_8=differences[0], refinement_8_to_16=differences[1])
            if shell.arity == 4:
                entry['diagonal02_diagnostics'] = metrics(surface.points)
                entry['diagonal13_diagnostics'] = metrics(surface.points[1:] + surface.points[:1])
            element_reports.append(entry)
        differences = [_difference(totals[j], totals[j + 1], part_size) for j in (0, 1)]
        require(max(differences[1].values()) <= refinement_tolerance, 'aggregate surface quadrature did not converge')
        return dict(schema='robo-dyna.shell-surface-mass-audit.v1', simulation_ready=False,
                    source_mass_equivalence_qualified=False, mass_interpretation='uniform rho*t midsurface lamina proxy',
                    quadrature_converged=True, surface_jacobian_conditioning_passed=True,
                    source_frame=geometry.source_frame, anchor_m=anchor, part_size_m=part_size,
                    thickness_m=thickness[0], density_kg_m3=density, areal_density_kg_m2=areal_density,
                    source_shell_count=len(surfaces), source_node_count=len(geometry.nodes),
                    quadrature=dict(donor=quadrature.donor, artifact_sha256=quadrature.artifact_sha256,
                                    donor_version=quadrature.version, rejected_reference=dict(quadrature.rejected_reference),
                                    reported_source_files=quadrature.source_files, refinement_tolerance=refinement_tolerance,
                                    convergence_is_certified_error_bound=False),
                    orders={str(r.order): _publish(totals[j], anchor) for j, r in enumerate(quadrature.rules)},
                    refinement_4_to_8=differences[0], refinement_8_to_16=differences[1], elements=element_reports,
                    exclusions=['source ELFORM2 integration/lumping equivalence', 'section offset/centering defaults',
                                'through-thickness physical rotary inertia', 'numerical drilling inertia',
                                'separately declared/added/scaled mass', 'attachment and full-case transforms'])
    except (OverflowError, ZeroDivisionError) as error:
        raise ValueError('surface integration overflow/underflow') from error
