#ifndef TL_QUALIFICATION_SHELL_PHASE_NATIVE_H
#define TL_QUALIFICATION_SHELL_PHASE_NATIVE_H

/* Qualification oracle only. All numbers use SI and IEEE binary64.
 * Frames are supplied by the caller: frame[3*axis+world_component].
 * CNVEC3/CORTDIR3 and their frame/director evolution are outside this oracle.
 * x/v/omega use node-major xyz, in the four-node connectivity order.
 * Calls to this oracle and the CHVIS3 oracle must be externally serialized:
 * unchanged Fortran routines share native COMMON blocks across libraries.
 */
#ifdef __cplusplus
extern "C" {
#endif

typedef struct ShellPhaseState {
    double off;         /* Only active +1 or frozen-reference +2 admitted. */
    double smstr[6];     /* Native x2,y2,x3,y3,x4,y4, relative to node 1. */
    double gstr[8];      /* xx,yy,engineering xy,yz,xz,kxx,kyy,kxy. */
} ShellPhaseState;

typedef struct ShellPhaseInput {
    double x[12];
    double v[12];        /* Carried velocity over the completed interval. */
    double omega[12];    /* Carried world angular velocity. */
    double frame[9];     /* Proper orthonormal frame; axis 3 is plane normal. */
    double dt;          /* Completed interval, strictly positive. */
    double section_thickness;
    double current_thickness;
    double young;
    double poisson;
    double density;
    double sound_speed;
    double shear_factor; /* Native ISH=0, FSH=this scalar, fixed NPT=3. */
    double h[3];         /* Native GEO(13:15): H1,H2,H3. */
    double srh[3];       /* Native GEO(18:20): SRH1,SRH2,SRH3. */
    int ismstr;          /* 1 frozen reference or 2 current geometry. */
    int ithk;            /* 0 section thickness; 1 current physical thickness. */
} ShellPhaseInput;

typedef struct ShellPhaseOutput {
    ShellPhaseState state;
    double gradient[4]; /* PX1,PX2,PY1,PY2; divide by area for derivatives. */
    double area;
    double vhx;
    double vhy;
    double thk0;
    double thk02;
    double shear_factor;
    double vol0;
    double vol00;
    double material[7]; /* rho,E,nu,G,A11,A12,SSP selected by native CCOEF3. */
    double hg[6];       /* H1,H2,H3,SRH1,SRH2,SRH3 from native CCOEF3. */
    double dstrain[8];  /* xx,yy,engineering xy,yz,xz,kxx,kyy,kxy. */
    double local_v[12];
    double local_omega[12]; /* Tangential components native CCURV3; z projected. */
    double gathered_off;    /* CCOOR3 clips accepted off before CDERI3 updates. */
} ShellPhaseOutput;

/* n=1..2. Input/state/output arrays each have n elements. Output arrays must
 * not alias inputs or accepted state. On any error, all outputs are unchanged.
 * No accepted state is mutated: caller explicitly commits output[i].state.
 * 0 success; 1 invalid input/domain; 2 nonfinite/invalid native result.
 */
int crash_shell_phase_native(int n, const ShellPhaseInput* input,
                             const ShellPhaseState* accepted,
                             ShellPhaseOutput* trial);

#ifdef __cplusplus
}
#endif
#endif
