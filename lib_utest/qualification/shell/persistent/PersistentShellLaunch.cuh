// Pinned K1/K3 argument mapping reused from the qualified S1/S2b fixture.
// Only private storage field names change; numerical bodies remain unchanged.
template<int ISMSTR,class D,class N,class C>
void LaunchGeometry(D d,N node,C c,int ne,HourglassParams hg,cudaStream_t stream){
    shell_geometry_kernel<ISMSTR><<<1,256,0,stream>>>(
      node(0),node(3),node(6),c(0),c(1),c(2),c(3),d(Off),d(Smstr),
      d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
      d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
      d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
      d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(AreaInverse),d(Vhx),d(Vhy),d(Z2),
      d(U),d(U+1),d(U+2),d(U+3),d(U+4),d(U+5),d(U+6),d(U+7),
      d(Sti),d(Stir),d(Modulus),d(StepThickness),c(2),c(3),ne,hg.H1,hg.H2);
}
template<int ISMSTR,class D,class N,class C>
void LaunchForces(D d,N node,C c,int ne,HourglassParams hg,double dt,cudaStream_t stream){
    shell_force_assembly_kernel<ISMSTR><<<1,256,0,stream>>>(
      c(0),c(1),c(2),c(3),
      d(Frame),d(Frame+1),d(Frame+2),d(Frame+3),d(Frame+4),d(Frame+5),d(Frame+6),d(Frame+7),d(Frame+8),
      d(Px1),d(Px2),d(Py1),d(Py2),d(Area),d(Vhx),d(Vhy),
      d(Vel),d(Vel+1),d(Vel+2),d(Vel+3),d(Vel+4),d(Vel+5),d(Vel+6),d(Vel+7),d(Vel+8),d(Vel+9),d(Vel+10),d(Vel+11),
      d(Angular),d(Angular+1),d(Angular+2),d(Angular+3),d(Angular+4),d(Angular+5),d(Angular+6),d(Angular+7),d(Angular+8),d(Angular+9),d(Angular+10),d(Angular+11),
      d(Off),d(StepThickness),d(StepThicknessSquared),
      d(Force),d(Force+1),d(Force+2),d(Force+3),d(Force+4),d(Moment),d(Moment+1),d(Moment+2),
      d(Hour),d(Sti),d(Stir),d(Sound),d(Rho),d(Modulus),d(Nu),d(A11),d(ShearModulus),d(SectionShear),d(Energy),
      node(9),node(10),node(11),node(12),node(13),node(14),node(15),node(16),hg,dt,3,ne,0,1);
}
