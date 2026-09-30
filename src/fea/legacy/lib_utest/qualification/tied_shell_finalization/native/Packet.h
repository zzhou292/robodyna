#pragma once
// All input node/rank indices are one-based. Outputs publish only on status0.
// mode0 is the actual fresh-zero phase; mode1 labels only DPARA's suffix to
// independently expose native DMIN alias clearing (physical M/J remain zero).
extern "C" void native_tied_finalize(int nodes,int masters,int slaves,int mains,int event_capacity,int mode,
    const int* irect,const int* nsv,const int* msr,const int* selected,const double* st,const double* distance,
    int* counts,int* output_nsv,int* output_msr,int* output_selected,double* output_st,double* output_stb,
    double* output_dpara,int* output_irupt,double* output_nmas,int* events,double* event_values,int* status);
// Exact fresh LECINT branch controls: IS1=-1 counts every slave; ordinary
// IS1=2 counts only nodes whose registration count reaches two.
extern "C" void native_tied_connection_count(int nodes, int slaves, const int* nsv,
    int is1, int* counts, int* multi, int* status);
