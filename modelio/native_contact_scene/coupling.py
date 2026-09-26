"""Declared converter-style group metadata; never a physical member mass source."""
from .definition import rigid_patch


def reference_rigid_body(scene, mesh):
    if scene.definition_version != 3:
        if scene.coupling is not None:
            raise ValueError('Legacy source cannot contain a coupling declaration')
        return None
    if scene.coupling is None:
        raise ValueError('Version3 requires explicit rigid coupling')
    rigid_patch(vars(scene.coupling))
    # Native ConvertUtils::GetCentroid(PartRead): first encounter of each unique
    # node in source element/corner order, then divide once. No mass weighting.
    nodes = {node.id: node.xyz_mm for node in mesh.nodes}
    members = []
    seen = set()
    center = [0., 0., 0.]
    for element in mesh.patch:
        for node in element.nodes:
            if node in seen:
                continue
            seen.add(node)
            members.append(node)
            for axis in range(3):
                center[axis] += nodes[node][axis]
    if not members or set(members) != set(mesh.patch_nodes):
        raise ValueError('Rigid patch must cover its exact physical source nodes')
    center = tuple(value/len(members) for value in center)
    # These are literal converter inputs. INIRBY determines final aggregate and
    # member coefficients; this record does not assert independent physical mass.
    return {'body_id': 1, 'source_part_id': 2,
            'primary': {'id': max(nodes)+1, 'xyz_mm': center},
            'centroid_node_order': tuple(members),
            'member_node_ids': mesh.patch_nodes,
            'converter_mass_tonne': 1e-20,
            'converter_inertia_tonne_mm2': (1e-20, 1e-20, 1e-20),
            'off_diagonal_inertia_tonne_mm2': (0., 0., 0.),
            'inertia_mode': scene.coupling.inertia_mode,
            'center_of_gravity': scene.coupling.center_of_gravity,
            'primary_velocity_mm_s': scene.velocity_mm_s}
