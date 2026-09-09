// Reused fixture slab layout from frozen S1. No donor numerical arithmetic.
// Element scalar planes. Multi-component fields occupy consecutive SoA planes.
enum Field {
  Off=0, Smstr=1, Frame=7, Vel=16, Angular=28, Px1=40, Px2=41,
  Py1=42, Py2=43, Area=44, AreaInverse=45, Vhx=46, Vhy=47, Z2=48,
  U=49, Sti=57, Stir=58, Young=59, Thickness=60, ThicknessSquared=61,
  Sound=62, Rho=63, Nu=64, A11=65, ShearModulus=66, ShearFactor=67,
  Force=68, Moment=73, Hour=76, Energy=81, ElementPlanes=83
};

