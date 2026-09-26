"""Reference-only Radioss2024 deck export. No process or production mechanics calls."""
from .cards import ints, reals, node_group
from .coupling import reference_rigid_body


def starter(scene, mesh):
    if (scene.definition_version, scene.contact_surface) not in ((1, 'fixed_wall'), (2, 'all_shells'), (3, 'all_shells')):
        raise ValueError('Unqualified scene contact surface declaration')
    rigid = reference_rigid_body(scene, mesh)
    material = scene.material
    unit = ''.join(s.rjust(20) for s in ('Mg','mm','s'))
    out = ['#RADIOSS STARTER', '/BEGIN', 'contact_scene', ints(2024), unit, unit,
           '/TITLE', ('Finite triangle wall and declared rigid shell patch' if rigid else
                      'Finite triangle wall and free elastoplastic shell patch'),
           '/MAT/LAW44/1', 'Explicit analytic LAW44 steel', reals(material.density_tonne_mm3),
           reals(material.young_n_mm2,material.poisson),
           reals(material.yield_n_mm2,material.plastic_hardening_n_mm2,1.,0.,1e30),
           reals(material.rate_c_per_s,material.rate_p)+ints(1,1)+reals(material.rate_filter_hz)+ints(0,0),
           reals(0.,0.,0.),
           '/PROP/TYPE1/1', 'Layered LAW44 QEPH and C0, NIP3 ITHICK1',
           ints(24,2,1,2,0)+' '*10+reals(1.), reals(0.,0.,0.,.015,.015),
           ints(3)+' '*10+reals(scene.thickness_mm,5./6.)+' '*10+ints(1,2,0)]
    for identifier, title in ((1,'Fixed finite wall'),(2,'Moving rigid patch' if rigid else 'Moving elastoplastic patch')):
        out += [f'/PART/{identifier}', title, ints(1,1,0)]
    out += ['/NODE'] + [ints(n.id)+reals(*n.xyz_mm) for n in mesh.nodes]
    if rigid:
        out += [ints(rigid['primary']['id'])+reals(*rigid['primary']['xyz_mm'])]
    out += ['/SH3N/1'] + [ints(e.id,*e.nodes) for e in mesh.wall]
    out += ['/SHELL/2'] + [ints(e.id,*e.nodes) for e in mesh.patch]
    out += node_group(1,'Fixed wall nodes',mesh.wall_nodes)
    out += node_group(2,'Moving patch nodes',mesh.patch_nodes)
    velocity_group = 2
    if rigid:
        # The auxiliary native primary belongs to the group initialization only,
        # never to the declared physical mesh or TYPE25 surface/secondary group.
        velocity_group = 3
        out += node_group(3,'Uniform initial group translation',mesh.patch_nodes+(rigid['primary']['id'],))
        out += ['/RBODY/1','Declared converted-part rigid patch',
                ints(rigid['primary']['id'],0,0,rigid['inertia_mode'])+
                reals(rigid['converter_mass_tonne'])+ints(2,0,rigid['center_of_gravity'],0),
                reals(*rigid['converter_inertia_tonne_mm2']),
                reals(*rigid['off_diagonal_inertia_tonne_mm2']),ints(0,2,0)]
    out += ['/BCS/1','Fixed wall translations and rotations','   111 111'+ints(0,1),
            '/INIVEL/TRA/1','Initial patch velocity',reals(*scene.velocity_mm_s)+ints(velocity_group,0),
            '/SURF/SEG/1',('Finite wall triangles' if scene.contact_surface == 'fixed_wall' else 'Complete declared wall and moving shell surface')]
    out += [ints(e.id,*e.nodes,0) for e in mesh.wall]
    if scene.contact_surface == 'all_shells':
        out += [ints(e.id,*e.nodes) for e in mesh.patch]
    # Surface1 selects ILEV1 self-contact on its complete declared roster.
    # Version1 adds moving patch secondaries; version2 already includes them
    # as genuine main-surface nodes and preserves the explicit group union. Fixed wall nodes remain authentic
    # secondary uses and are handled by native constraints/incidence/removal.
    # Labels below are intended inputs; actual normalized controls are observed
    # from the pinned Starter/Engine before any runtime profile is admitted.
    out += ['/INTER/TYPE25/1','Native aligned finite mesh contact',
            ints(1,0,4,0,2,1)+' '*10+ints(1,1000,0),
            ints(2)+' '*10+reals(1.,.4,1e30,1e30),
            reals(0.,1e30)+ints(1000,1)+reals(135.,.1),
            reals(1.,.1,0.,0.,1e30),
            ints(0,0,1,5)+reals(.05)+ints(0,0)+reals(0.),
            ints(2,0)+reals(1.)+ints(0,0)+reals(0.)+ints(0,0),
            reals(0.,0.,0.,0.,.1), reals(-.001), '/END']
    return '\n'.join(out)+'\n'


def engine(scene):
    # Ordinary unscaled nodal control. Optional DTIX limits initial/maximum dt;
    # smaller stability-limited steps remain legal. No target mass scaling.
    out = ['/RUN/contact_scene/1',reals(scene.end_time_s),
           '/DT',reals(scene.nodal_scale,0.),'/DT/NODA/STOP',reals(scene.nodal_scale,0.),
           '/ANIM/DT',reals(0.,scene.animation_interval_s),'/ANIM/VECT/DISP',
           '/ANIM/VECT/VEL','/ANIM/VECT/CONT','/ANIM/ELEM/ENER',
           '/TFILE/4',reals(scene.animation_interval_s),'/PARITH/OFF','/TH/TITLE']
    if scene.time_step_cap_s is not None:
        out += ['/DTIX',reals(scene.time_step_cap_s,scene.time_step_cap_s)]
    return '\n'.join(out)+'\n'
