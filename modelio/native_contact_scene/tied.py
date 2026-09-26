"""Declared TYPE2 card/roster inputs, never prescribed classification or weights."""
from .definition import TiedPatch


def reference_tied_interface(scene, mesh):
    if scene.definition_version != 4:
        return None
    c = scene.coupling
    if not isinstance(c, TiedPatch) or (c.kind, c.ignore, c.spotflag, c.level, c.search,
            c.deletion, c.search_distance_mm, c.stiffness_scale, c.viscosity,
            c.stiffness_mode, c.tied_secondary_removal) != ('tied_patch', 2, 28, 0, 0, 1, 0., 1., .05, 2, 1):
        raise ValueError('Unqualified declared TYPE2 controls')
    masters = tuple(e for e in mesh.patch if e.part == 2)
    dependents = tuple(e for e in mesh.patch if e.part == 3)
    if len(masters) != 1 or len(dependents) != 1 or len(mesh.patch) != 2:
        raise ValueError('The first tied scene requires two complete physical Q4 parents')
    master_ids = set(masters[0].nodes)
    dependent_ids = set(dependents[0].nodes)
    if len(master_ids) != 4 or len(dependent_ids) != 4 or master_ids & dependent_ids:
        raise ValueError('Master and dependent physical node rosters must be disjoint')
    return {'interface_id': 2, 'master_surface_id': 2, 'secondary_group_id': 3,
            'master_source_element_id': masters[0].id,
            'master_node_ids': masters[0].nodes,
            'secondary_node_ids': tuple(n.id for n in mesh.nodes if n.id in dependent_ids),
            'ignore': c.ignore, 'spotflag': c.spotflag, 'level': c.level,
            'search': c.search, 'deletion': c.deletion, 'search_distance_mm': c.search_distance_mm,
            'stiffness_scale': c.stiffness_scale, 'viscosity': c.viscosity,
            'stiffness_mode': c.stiffness_mode, 'tied_secondary_removal': c.tied_secondary_removal,
            'scope': 'Declared inputs only; actual CIN classification, coefficients and pair exclusions require native observation'}
