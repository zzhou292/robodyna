"""One physical mesh shared by the native exporter and future TL scene adapter."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Node:
    id: int
    xyz_mm: tuple


@dataclass(frozen=True)
class Element:
    id: int
    part: int
    nodes: tuple


@dataclass(frozen=True)
class Mesh:
    nodes: tuple
    wall: tuple
    patch: tuple
    wall_nodes: tuple
    patch_nodes: tuple


def _grid(grid, first_node, first_element, part, triangles):
    nodes = tuple(Node(first_node+j*len(grid.x_mm)+i, (x,y,grid.z_mm+grid.dz_dx*x))
                  for j, y in enumerate(grid.y_mm) for i, x in enumerate(grid.x_mm))
    elements = []
    for j in range(len(grid.y_mm)-1):
        for i in range(len(grid.x_mm)-1):
            a = first_node+j*len(grid.x_mm)+i
            quad = (a, a+1, a+len(grid.x_mm)+1, a+len(grid.x_mm))
            faces = ((quad[0],quad[1],quad[2]), (quad[0],quad[2],quad[3])) if triangles else (quad,)
            for face in faces:
                elements.append(Element(first_element+len(elements),part,face))
    return nodes, tuple(elements)


def build(scene):
    wall_nodes, wall = _grid(scene.wall, 1, 1, 1, True)
    patch_nodes, patch = _grid(scene.patch, len(wall_nodes)+1, len(wall)+1, 2, False)
    if scene.definition_version == 4:
        dependent_nodes, dependent = _grid(scene.coupling.dependent, len(wall_nodes)+len(patch_nodes)+1,
                                          len(wall)+len(patch)+1, 3, False)
        patch_nodes += dependent_nodes
        patch += dependent
    return Mesh(wall_nodes+patch_nodes, wall, patch,
                tuple(n.id for n in wall_nodes), tuple(n.id for n in patch_nodes))
