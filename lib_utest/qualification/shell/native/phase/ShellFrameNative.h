#ifndef TL_QUALIFICATION_SHELL_FRAME_NATIVE_H
#define TL_QUALIFICATION_SHELL_FRAME_NATIVE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ShellFrameInput {
    double x[12]; /* Four current world positions, node-major xyz, SI. */
} ShellFrameInput;

typedef struct ShellFrameOutput {
    double frame[9]; /* frame[3*axis+world_component], proper orthonormal. */
    /* Diagnostic endpoints of native direction buffers: A_first,A_last,
     * B_first,B_last. Full-buffer invariance is checked before success. */
    double direction_sentinels[4];
} ShellFrameOutput;

/* Test-only exact CNVEC3+CORTDIR3 context: ISHFRAM=0, IREP=0, IDRAPE=0,
 * IGTYP=1, one layer, MYREAL8. n=1..2, convex planar nondegenerate Q4s.
 * This computes a current frame; it owns no accepted history, startup policy,
 * director evolution, material transport or timestep. It does not qualify
 * finite-motion objectivity or map a Yaris shell formulation.
 *
 * All n outputs publish together. On failure they remain unchanged.
 * Caller supplies correctly sized, nonaliasing input/output arrays and keeps
 * them alive through this synchronous call. Calls to this API, phase and
 * CHVIS3 must be externally serialized because original COMMON is shared.
 * 0 success; 1 invalid input/domain; 2 invalid native result/sentinel change.
 */
int crash_shell_frame_native(int n, const ShellFrameInput* input,
                            ShellFrameOutput* output);

#ifdef __cplusplus
}
#endif
#endif
