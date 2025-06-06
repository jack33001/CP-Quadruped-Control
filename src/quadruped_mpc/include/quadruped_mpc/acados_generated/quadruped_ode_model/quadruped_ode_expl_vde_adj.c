/* This file was automatically generated by CasADi 3.7.0.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadruped_ode_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[3] = {25, 1, 1};
static const casadi_int casadi_s1[3] = {12, 1, 1};
static const casadi_int casadi_s2[3] = {0, 1, 1};
static const casadi_int casadi_s3[3] = {37, 1, 1};

/* quadruped_ode_expl_vde_adj:(i0[25],i1[25],i2[12],i3[0])->(o0[37]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a00, a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11;
  casadi_real a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23;
  casadi_real a24;
  a00=arg[2]? arg[2][10] : 0;
  a01=1.2500000000000000e+00;
  a02=arg[1]? arg[1][12] : 0;
  a02=(a01*a02);
  a03=(a00*a02);
  a04=arg[2]? arg[2][11] : 0;
  a05=arg[1]? arg[1][11] : 0;
  a05=(a01*a05);
  a06=(a04*a05);
  a03=(a03-a06);
  a06=arg[2]? arg[2][7] : 0;
  a07=(a06*a02);
  a08=arg[2]? arg[2][8] : 0;
  a09=(a08*a05);
  a07=(a07-a09);
  a09=(a03+a07);
  a10=arg[2]? arg[2][4] : 0;
  a11=(a10*a02);
  a12=arg[2]? arg[2][5] : 0;
  a13=(a12*a05);
  a11=(a11-a13);
  a09=(a09+a11);
  a13=arg[2]? arg[2][1] : 0;
  a14=(a13*a02);
  a15=arg[2]? arg[2][2] : 0;
  a16=(a15*a05);
  a14=(a14-a16);
  a09=(a09+a14);
  a09=(-a09);
  if (res[0]!=0) res[0][0]=a09;
  a09=arg[1]? arg[1][10] : 0;
  a01=(a01*a09);
  a04=(a04*a01);
  a09=arg[2]? arg[2][9] : 0;
  a16=(a09*a02);
  a04=(a04-a16);
  a08=(a08*a01);
  a16=arg[2]? arg[2][6] : 0;
  a17=(a16*a02);
  a08=(a08-a17);
  a17=(a04+a08);
  a12=(a12*a01);
  a18=arg[2]? arg[2][3] : 0;
  a19=(a18*a02);
  a12=(a12-a19);
  a17=(a17+a12);
  a15=(a15*a01);
  a19=arg[2]? arg[2][0] : 0;
  a20=(a19*a02);
  a15=(a15-a20);
  a17=(a17+a15);
  a17=(-a17);
  if (res[0]!=0) res[0][1]=a17;
  a09=(a09*a05);
  a00=(a00*a01);
  a09=(a09-a00);
  a16=(a16*a05);
  a06=(a06*a01);
  a16=(a16-a06);
  a06=(a09+a16);
  a18=(a18*a05);
  a10=(a10*a01);
  a18=(a18-a10);
  a06=(a06+a18);
  a19=(a19*a05);
  a13=(a13*a01);
  a19=(a19-a13);
  a06=(a06+a19);
  a06=(-a06);
  if (res[0]!=0) res[0][2]=a06;
  a06=arg[0]? arg[0][12] : 0;
  a13=5.0000000000000000e-01;
  a10=arg[1]? arg[1][6] : 0;
  a10=(a13*a10);
  a00=(a06*a10);
  a17=arg[0]? arg[0][11] : 0;
  a20=arg[1]? arg[1][5] : 0;
  a20=(a13*a20);
  a21=(a17*a20);
  a00=(a00+a21);
  a21=arg[0]? arg[0][10] : 0;
  a22=arg[1]? arg[1][4] : 0;
  a22=(a13*a22);
  a23=(a21*a22);
  a00=(a00+a23);
  if (res[0]!=0) res[0][3]=a00;
  a00=(a17*a10);
  a23=(a06*a20);
  a00=(a00-a23);
  a23=arg[1]? arg[1][3] : 0;
  a13=(a13*a23);
  a23=(a21*a13);
  a00=(a00-a23);
  if (res[0]!=0) res[0][4]=a00;
  a00=(a06*a22);
  a23=(a21*a10);
  a00=(a00-a23);
  a23=(a17*a13);
  a00=(a00-a23);
  if (res[0]!=0) res[0][5]=a00;
  a21=(a21*a20);
  a17=(a17*a22);
  a21=(a21-a17);
  a06=(a06*a13);
  a21=(a21-a06);
  if (res[0]!=0) res[0][6]=a21;
  a21=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][7]=a21;
  a21=arg[1]? arg[1][1] : 0;
  if (res[0]!=0) res[0][8]=a21;
  a21=arg[1]? arg[1][2] : 0;
  if (res[0]!=0) res[0][9]=a21;
  a21=arg[0]? arg[0][6] : 0;
  a06=(a21*a20);
  a17=arg[0]? arg[0][5] : 0;
  a00=(a17*a10);
  a06=(a06-a00);
  a00=arg[0]? arg[0][3] : 0;
  a23=(a00*a22);
  a06=(a06+a23);
  a23=arg[0]? arg[0][4] : 0;
  a24=(a23*a13);
  a06=(a06-a24);
  if (res[0]!=0) res[0][10]=a06;
  a06=(a23*a10);
  a24=(a00*a20);
  a06=(a06+a24);
  a24=(a21*a22);
  a06=(a06-a24);
  a24=(a17*a13);
  a06=(a06-a24);
  if (res[0]!=0) res[0][11]=a06;
  a00=(a00*a10);
  a23=(a23*a20);
  a00=(a00-a23);
  a17=(a17*a22);
  a00=(a00+a17);
  a21=(a21*a13);
  a00=(a00-a21);
  if (res[0]!=0) res[0][12]=a00;
  if (res[0]!=0) res[0][13]=a14;
  if (res[0]!=0) res[0][14]=a15;
  if (res[0]!=0) res[0][15]=a19;
  if (res[0]!=0) res[0][16]=a11;
  if (res[0]!=0) res[0][17]=a12;
  if (res[0]!=0) res[0][18]=a18;
  if (res[0]!=0) res[0][19]=a07;
  if (res[0]!=0) res[0][20]=a08;
  if (res[0]!=0) res[0][21]=a16;
  if (res[0]!=0) res[0][22]=a03;
  if (res[0]!=0) res[0][23]=a04;
  if (res[0]!=0) res[0][24]=a09;
  a09=arg[0]? arg[0][15] : 0;
  a04=arg[0]? arg[0][2] : 0;
  a09=(a09-a04);
  a03=(a09*a05);
  a16=arg[0]? arg[0][14] : 0;
  a08=arg[0]? arg[0][1] : 0;
  a16=(a16-a08);
  a07=(a16*a02);
  a03=(a03-a07);
  a07=7.5757575757575760e-02;
  a18=arg[1]? arg[1][7] : 0;
  a18=(a07*a18);
  a03=(a03+a18);
  if (res[0]!=0) res[0][25]=a03;
  a03=arg[0]? arg[0][13] : 0;
  a12=arg[0]? arg[0][0] : 0;
  a03=(a03-a12);
  a11=(a03*a02);
  a09=(a09*a01);
  a11=(a11-a09);
  a09=arg[1]? arg[1][8] : 0;
  a09=(a07*a09);
  a11=(a11+a09);
  if (res[0]!=0) res[0][26]=a11;
  a16=(a16*a01);
  a03=(a03*a05);
  a16=(a16-a03);
  a03=arg[1]? arg[1][9] : 0;
  a07=(a07*a03);
  a16=(a16+a07);
  if (res[0]!=0) res[0][27]=a16;
  a16=arg[0]? arg[0][18] : 0;
  a16=(a16-a04);
  a03=(a16*a05);
  a11=arg[0]? arg[0][17] : 0;
  a11=(a11-a08);
  a19=(a11*a02);
  a03=(a03-a19);
  a03=(a03+a18);
  if (res[0]!=0) res[0][28]=a03;
  a03=arg[0]? arg[0][16] : 0;
  a03=(a03-a12);
  a19=(a03*a02);
  a16=(a16*a01);
  a19=(a19-a16);
  a19=(a19+a09);
  if (res[0]!=0) res[0][29]=a19;
  a11=(a11*a01);
  a03=(a03*a05);
  a11=(a11-a03);
  a11=(a11+a07);
  if (res[0]!=0) res[0][30]=a11;
  a11=arg[0]? arg[0][21] : 0;
  a11=(a11-a04);
  a03=(a11*a05);
  a19=arg[0]? arg[0][20] : 0;
  a19=(a19-a08);
  a16=(a19*a02);
  a03=(a03-a16);
  a03=(a03+a18);
  if (res[0]!=0) res[0][31]=a03;
  a03=arg[0]? arg[0][19] : 0;
  a03=(a03-a12);
  a16=(a03*a02);
  a11=(a11*a01);
  a16=(a16-a11);
  a16=(a16+a09);
  if (res[0]!=0) res[0][32]=a16;
  a19=(a19*a01);
  a03=(a03*a05);
  a19=(a19-a03);
  a19=(a19+a07);
  if (res[0]!=0) res[0][33]=a19;
  a19=arg[0]? arg[0][24] : 0;
  a19=(a19-a04);
  a04=(a19*a05);
  a03=arg[0]? arg[0][23] : 0;
  a03=(a03-a08);
  a08=(a03*a02);
  a04=(a04-a08);
  a04=(a04+a18);
  if (res[0]!=0) res[0][34]=a04;
  a04=arg[0]? arg[0][22] : 0;
  a04=(a04-a12);
  a02=(a04*a02);
  a19=(a19*a01);
  a02=(a02-a19);
  a02=(a02+a09);
  if (res[0]!=0) res[0][35]=a02;
  a03=(a03*a01);
  a04=(a04*a05);
  a03=(a03-a04);
  a03=(a03+a07);
  if (res[0]!=0) res[0][36]=a03;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadruped_ode_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadruped_ode_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadruped_ode_expl_vde_adj_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadruped_ode_expl_vde_adj_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadruped_ode_expl_vde_adj_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadruped_ode_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadruped_ode_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_adj_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
