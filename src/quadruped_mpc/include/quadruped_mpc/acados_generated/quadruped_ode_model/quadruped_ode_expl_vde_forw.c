/* This file was automatically generated by CasADi 3.6.7.
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
  #define CASADI_PREFIX(ID) quadruped_ode_expl_vde_forw_ ## ID
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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[159] = {12, 12, 0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s2[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

/* quadruped_ode_expl_vde_forw:(i0[12],i1[12x12],i2[12x12],i3[12],i4[15])->(o0[12],o1[12x12],o2[12x12]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][6] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][7] : 0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][8] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[0]? arg[0][9] : 0;
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[0]? arg[0][10] : 0;
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[0]? arg[0][11] : 0;
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[3]? arg[3][0] : 0;
  a1=arg[3]? arg[3][3] : 0;
  a2=(a0+a1);
  a3=arg[3]? arg[3][6] : 0;
  a2=(a2+a3);
  a4=arg[3]? arg[3][9] : 0;
  a2=(a2+a4);
  a5=20.;
  a2=(a2/a5);
  if (res[0]!=0) res[0][6]=a2;
  a2=arg[3]? arg[3][1] : 0;
  a6=arg[3]? arg[3][4] : 0;
  a7=(a2+a6);
  a8=arg[3]? arg[3][7] : 0;
  a7=(a7+a8);
  a9=arg[3]? arg[3][10] : 0;
  a7=(a7+a9);
  a7=(a7/a5);
  if (res[0]!=0) res[0][7]=a7;
  a7=arg[3]? arg[3][2] : 0;
  a10=arg[3]? arg[3][5] : 0;
  a11=(a7+a10);
  a12=arg[3]? arg[3][8] : 0;
  a11=(a11+a12);
  a13=arg[3]? arg[3][11] : 0;
  a11=(a11+a13);
  a11=(a11/a5);
  a5=-9.8100000000000005e+00;
  a11=(a11+a5);
  if (res[0]!=0) res[0][8]=a11;
  a11=arg[4]? arg[4][1] : 0;
  a5=arg[4]? arg[4][13] : 0;
  a11=(a11-a5);
  a14=(a11*a7);
  a15=arg[4]? arg[4][2] : 0;
  a16=arg[4]? arg[4][14] : 0;
  a15=(a15-a16);
  a17=(a15*a2);
  a14=(a14-a17);
  a17=arg[4]? arg[4][4] : 0;
  a17=(a17-a5);
  a18=(a17*a10);
  a19=arg[4]? arg[4][5] : 0;
  a19=(a19-a16);
  a20=(a19*a6);
  a18=(a18-a20);
  a14=(a14+a18);
  a18=arg[4]? arg[4][7] : 0;
  a18=(a18-a5);
  a20=(a18*a12);
  a21=arg[4]? arg[4][8] : 0;
  a21=(a21-a16);
  a22=(a21*a8);
  a20=(a20-a22);
  a14=(a14+a20);
  a20=arg[4]? arg[4][10] : 0;
  a20=(a20-a5);
  a5=(a20*a13);
  a22=arg[4]? arg[4][11] : 0;
  a22=(a22-a16);
  a16=(a22*a9);
  a5=(a5-a16);
  a14=(a14+a5);
  a5=10.;
  a14=(a14/a5);
  if (res[0]!=0) res[0][9]=a14;
  a14=(a15*a0);
  a16=arg[4]? arg[4][0] : 0;
  a23=arg[4]? arg[4][12] : 0;
  a16=(a16-a23);
  a7=(a16*a7);
  a14=(a14-a7);
  a7=(a19*a1);
  a24=arg[4]? arg[4][3] : 0;
  a24=(a24-a23);
  a10=(a24*a10);
  a7=(a7-a10);
  a14=(a14+a7);
  a7=(a21*a3);
  a10=arg[4]? arg[4][6] : 0;
  a10=(a10-a23);
  a12=(a10*a12);
  a7=(a7-a12);
  a14=(a14+a7);
  a7=(a22*a4);
  a12=arg[4]? arg[4][9] : 0;
  a12=(a12-a23);
  a13=(a12*a13);
  a7=(a7-a13);
  a14=(a14+a7);
  a14=(a14/a5);
  if (res[0]!=0) res[0][10]=a14;
  a2=(a16*a2);
  a0=(a11*a0);
  a2=(a2-a0);
  a6=(a24*a6);
  a1=(a17*a1);
  a6=(a6-a1);
  a2=(a2+a6);
  a8=(a10*a8);
  a3=(a18*a3);
  a8=(a8-a3);
  a2=(a2+a8);
  a9=(a12*a9);
  a4=(a20*a4);
  a9=(a9-a4);
  a2=(a2+a9);
  a2=(a2/a5);
  if (res[0]!=0) res[0][11]=a2;
  a2=arg[1]? arg[1][6] : 0;
  if (res[1]!=0) res[1][0]=a2;
  a2=arg[1]? arg[1][7] : 0;
  if (res[1]!=0) res[1][1]=a2;
  a2=arg[1]? arg[1][8] : 0;
  if (res[1]!=0) res[1][2]=a2;
  a2=arg[1]? arg[1][9] : 0;
  if (res[1]!=0) res[1][3]=a2;
  a2=arg[1]? arg[1][10] : 0;
  if (res[1]!=0) res[1][4]=a2;
  a2=arg[1]? arg[1][11] : 0;
  if (res[1]!=0) res[1][5]=a2;
  a2=0.;
  if (res[1]!=0) res[1][6]=a2;
  if (res[1]!=0) res[1][7]=a2;
  if (res[1]!=0) res[1][8]=a2;
  if (res[1]!=0) res[1][9]=a2;
  if (res[1]!=0) res[1][10]=a2;
  if (res[1]!=0) res[1][11]=a2;
  a5=arg[1]? arg[1][18] : 0;
  if (res[1]!=0) res[1][12]=a5;
  a5=arg[1]? arg[1][19] : 0;
  if (res[1]!=0) res[1][13]=a5;
  a5=arg[1]? arg[1][20] : 0;
  if (res[1]!=0) res[1][14]=a5;
  a5=arg[1]? arg[1][21] : 0;
  if (res[1]!=0) res[1][15]=a5;
  a5=arg[1]? arg[1][22] : 0;
  if (res[1]!=0) res[1][16]=a5;
  a5=arg[1]? arg[1][23] : 0;
  if (res[1]!=0) res[1][17]=a5;
  if (res[1]!=0) res[1][18]=a2;
  if (res[1]!=0) res[1][19]=a2;
  if (res[1]!=0) res[1][20]=a2;
  if (res[1]!=0) res[1][21]=a2;
  if (res[1]!=0) res[1][22]=a2;
  if (res[1]!=0) res[1][23]=a2;
  a5=arg[1]? arg[1][30] : 0;
  if (res[1]!=0) res[1][24]=a5;
  a5=arg[1]? arg[1][31] : 0;
  if (res[1]!=0) res[1][25]=a5;
  a5=arg[1]? arg[1][32] : 0;
  if (res[1]!=0) res[1][26]=a5;
  a5=arg[1]? arg[1][33] : 0;
  if (res[1]!=0) res[1][27]=a5;
  a5=arg[1]? arg[1][34] : 0;
  if (res[1]!=0) res[1][28]=a5;
  a5=arg[1]? arg[1][35] : 0;
  if (res[1]!=0) res[1][29]=a5;
  if (res[1]!=0) res[1][30]=a2;
  if (res[1]!=0) res[1][31]=a2;
  if (res[1]!=0) res[1][32]=a2;
  if (res[1]!=0) res[1][33]=a2;
  if (res[1]!=0) res[1][34]=a2;
  if (res[1]!=0) res[1][35]=a2;
  a5=arg[1]? arg[1][42] : 0;
  if (res[1]!=0) res[1][36]=a5;
  a5=arg[1]? arg[1][43] : 0;
  if (res[1]!=0) res[1][37]=a5;
  a5=arg[1]? arg[1][44] : 0;
  if (res[1]!=0) res[1][38]=a5;
  a5=arg[1]? arg[1][45] : 0;
  if (res[1]!=0) res[1][39]=a5;
  a5=arg[1]? arg[1][46] : 0;
  if (res[1]!=0) res[1][40]=a5;
  a5=arg[1]? arg[1][47] : 0;
  if (res[1]!=0) res[1][41]=a5;
  if (res[1]!=0) res[1][42]=a2;
  if (res[1]!=0) res[1][43]=a2;
  if (res[1]!=0) res[1][44]=a2;
  if (res[1]!=0) res[1][45]=a2;
  if (res[1]!=0) res[1][46]=a2;
  if (res[1]!=0) res[1][47]=a2;
  a5=arg[1]? arg[1][54] : 0;
  if (res[1]!=0) res[1][48]=a5;
  a5=arg[1]? arg[1][55] : 0;
  if (res[1]!=0) res[1][49]=a5;
  a5=arg[1]? arg[1][56] : 0;
  if (res[1]!=0) res[1][50]=a5;
  a5=arg[1]? arg[1][57] : 0;
  if (res[1]!=0) res[1][51]=a5;
  a5=arg[1]? arg[1][58] : 0;
  if (res[1]!=0) res[1][52]=a5;
  a5=arg[1]? arg[1][59] : 0;
  if (res[1]!=0) res[1][53]=a5;
  if (res[1]!=0) res[1][54]=a2;
  if (res[1]!=0) res[1][55]=a2;
  if (res[1]!=0) res[1][56]=a2;
  if (res[1]!=0) res[1][57]=a2;
  if (res[1]!=0) res[1][58]=a2;
  if (res[1]!=0) res[1][59]=a2;
  a5=arg[1]? arg[1][66] : 0;
  if (res[1]!=0) res[1][60]=a5;
  a5=arg[1]? arg[1][67] : 0;
  if (res[1]!=0) res[1][61]=a5;
  a5=arg[1]? arg[1][68] : 0;
  if (res[1]!=0) res[1][62]=a5;
  a5=arg[1]? arg[1][69] : 0;
  if (res[1]!=0) res[1][63]=a5;
  a5=arg[1]? arg[1][70] : 0;
  if (res[1]!=0) res[1][64]=a5;
  a5=arg[1]? arg[1][71] : 0;
  if (res[1]!=0) res[1][65]=a5;
  if (res[1]!=0) res[1][66]=a2;
  if (res[1]!=0) res[1][67]=a2;
  if (res[1]!=0) res[1][68]=a2;
  if (res[1]!=0) res[1][69]=a2;
  if (res[1]!=0) res[1][70]=a2;
  if (res[1]!=0) res[1][71]=a2;
  a5=arg[1]? arg[1][78] : 0;
  if (res[1]!=0) res[1][72]=a5;
  a5=arg[1]? arg[1][79] : 0;
  if (res[1]!=0) res[1][73]=a5;
  a5=arg[1]? arg[1][80] : 0;
  if (res[1]!=0) res[1][74]=a5;
  a5=arg[1]? arg[1][81] : 0;
  if (res[1]!=0) res[1][75]=a5;
  a5=arg[1]? arg[1][82] : 0;
  if (res[1]!=0) res[1][76]=a5;
  a5=arg[1]? arg[1][83] : 0;
  if (res[1]!=0) res[1][77]=a5;
  if (res[1]!=0) res[1][78]=a2;
  if (res[1]!=0) res[1][79]=a2;
  if (res[1]!=0) res[1][80]=a2;
  if (res[1]!=0) res[1][81]=a2;
  if (res[1]!=0) res[1][82]=a2;
  if (res[1]!=0) res[1][83]=a2;
  a5=arg[1]? arg[1][90] : 0;
  if (res[1]!=0) res[1][84]=a5;
  a5=arg[1]? arg[1][91] : 0;
  if (res[1]!=0) res[1][85]=a5;
  a5=arg[1]? arg[1][92] : 0;
  if (res[1]!=0) res[1][86]=a5;
  a5=arg[1]? arg[1][93] : 0;
  if (res[1]!=0) res[1][87]=a5;
  a5=arg[1]? arg[1][94] : 0;
  if (res[1]!=0) res[1][88]=a5;
  a5=arg[1]? arg[1][95] : 0;
  if (res[1]!=0) res[1][89]=a5;
  if (res[1]!=0) res[1][90]=a2;
  if (res[1]!=0) res[1][91]=a2;
  if (res[1]!=0) res[1][92]=a2;
  if (res[1]!=0) res[1][93]=a2;
  if (res[1]!=0) res[1][94]=a2;
  if (res[1]!=0) res[1][95]=a2;
  a5=arg[1]? arg[1][102] : 0;
  if (res[1]!=0) res[1][96]=a5;
  a5=arg[1]? arg[1][103] : 0;
  if (res[1]!=0) res[1][97]=a5;
  a5=arg[1]? arg[1][104] : 0;
  if (res[1]!=0) res[1][98]=a5;
  a5=arg[1]? arg[1][105] : 0;
  if (res[1]!=0) res[1][99]=a5;
  a5=arg[1]? arg[1][106] : 0;
  if (res[1]!=0) res[1][100]=a5;
  a5=arg[1]? arg[1][107] : 0;
  if (res[1]!=0) res[1][101]=a5;
  if (res[1]!=0) res[1][102]=a2;
  if (res[1]!=0) res[1][103]=a2;
  if (res[1]!=0) res[1][104]=a2;
  if (res[1]!=0) res[1][105]=a2;
  if (res[1]!=0) res[1][106]=a2;
  if (res[1]!=0) res[1][107]=a2;
  a5=arg[1]? arg[1][114] : 0;
  if (res[1]!=0) res[1][108]=a5;
  a5=arg[1]? arg[1][115] : 0;
  if (res[1]!=0) res[1][109]=a5;
  a5=arg[1]? arg[1][116] : 0;
  if (res[1]!=0) res[1][110]=a5;
  a5=arg[1]? arg[1][117] : 0;
  if (res[1]!=0) res[1][111]=a5;
  a5=arg[1]? arg[1][118] : 0;
  if (res[1]!=0) res[1][112]=a5;
  a5=arg[1]? arg[1][119] : 0;
  if (res[1]!=0) res[1][113]=a5;
  if (res[1]!=0) res[1][114]=a2;
  if (res[1]!=0) res[1][115]=a2;
  if (res[1]!=0) res[1][116]=a2;
  if (res[1]!=0) res[1][117]=a2;
  if (res[1]!=0) res[1][118]=a2;
  if (res[1]!=0) res[1][119]=a2;
  a5=arg[1]? arg[1][126] : 0;
  if (res[1]!=0) res[1][120]=a5;
  a5=arg[1]? arg[1][127] : 0;
  if (res[1]!=0) res[1][121]=a5;
  a5=arg[1]? arg[1][128] : 0;
  if (res[1]!=0) res[1][122]=a5;
  a5=arg[1]? arg[1][129] : 0;
  if (res[1]!=0) res[1][123]=a5;
  a5=arg[1]? arg[1][130] : 0;
  if (res[1]!=0) res[1][124]=a5;
  a5=arg[1]? arg[1][131] : 0;
  if (res[1]!=0) res[1][125]=a5;
  if (res[1]!=0) res[1][126]=a2;
  if (res[1]!=0) res[1][127]=a2;
  if (res[1]!=0) res[1][128]=a2;
  if (res[1]!=0) res[1][129]=a2;
  if (res[1]!=0) res[1][130]=a2;
  if (res[1]!=0) res[1][131]=a2;
  a5=arg[1]? arg[1][138] : 0;
  if (res[1]!=0) res[1][132]=a5;
  a5=arg[1]? arg[1][139] : 0;
  if (res[1]!=0) res[1][133]=a5;
  a5=arg[1]? arg[1][140] : 0;
  if (res[1]!=0) res[1][134]=a5;
  a5=arg[1]? arg[1][141] : 0;
  if (res[1]!=0) res[1][135]=a5;
  a5=arg[1]? arg[1][142] : 0;
  if (res[1]!=0) res[1][136]=a5;
  a5=arg[1]? arg[1][143] : 0;
  if (res[1]!=0) res[1][137]=a5;
  if (res[1]!=0) res[1][138]=a2;
  if (res[1]!=0) res[1][139]=a2;
  if (res[1]!=0) res[1][140]=a2;
  if (res[1]!=0) res[1][141]=a2;
  if (res[1]!=0) res[1][142]=a2;
  if (res[1]!=0) res[1][143]=a2;
  a5=arg[2]? arg[2][6] : 0;
  if (res[2]!=0) res[2][0]=a5;
  a5=arg[2]? arg[2][7] : 0;
  if (res[2]!=0) res[2][1]=a5;
  a5=arg[2]? arg[2][8] : 0;
  if (res[2]!=0) res[2][2]=a5;
  a5=arg[2]? arg[2][9] : 0;
  if (res[2]!=0) res[2][3]=a5;
  a5=arg[2]? arg[2][10] : 0;
  if (res[2]!=0) res[2][4]=a5;
  a5=arg[2]? arg[2][11] : 0;
  if (res[2]!=0) res[2][5]=a5;
  a5=5.0000000000000003e-02;
  if (res[2]!=0) res[2][6]=a5;
  if (res[2]!=0) res[2][7]=a2;
  if (res[2]!=0) res[2][8]=a2;
  if (res[2]!=0) res[2][9]=a2;
  a9=1.0000000000000001e-01;
  a4=(a9*a15);
  if (res[2]!=0) res[2][10]=a4;
  a4=-1.0000000000000001e-01;
  a8=(a4*a11);
  if (res[2]!=0) res[2][11]=a8;
  a8=arg[2]? arg[2][18] : 0;
  if (res[2]!=0) res[2][12]=a8;
  a8=arg[2]? arg[2][19] : 0;
  if (res[2]!=0) res[2][13]=a8;
  a8=arg[2]? arg[2][20] : 0;
  if (res[2]!=0) res[2][14]=a8;
  a8=arg[2]? arg[2][21] : 0;
  if (res[2]!=0) res[2][15]=a8;
  a8=arg[2]? arg[2][22] : 0;
  if (res[2]!=0) res[2][16]=a8;
  a8=arg[2]? arg[2][23] : 0;
  if (res[2]!=0) res[2][17]=a8;
  if (res[2]!=0) res[2][18]=a2;
  if (res[2]!=0) res[2][19]=a5;
  if (res[2]!=0) res[2][20]=a2;
  a15=(a4*a15);
  if (res[2]!=0) res[2][21]=a15;
  if (res[2]!=0) res[2][22]=a2;
  a15=(a9*a16);
  if (res[2]!=0) res[2][23]=a15;
  a15=arg[2]? arg[2][30] : 0;
  if (res[2]!=0) res[2][24]=a15;
  a15=arg[2]? arg[2][31] : 0;
  if (res[2]!=0) res[2][25]=a15;
  a15=arg[2]? arg[2][32] : 0;
  if (res[2]!=0) res[2][26]=a15;
  a15=arg[2]? arg[2][33] : 0;
  if (res[2]!=0) res[2][27]=a15;
  a15=arg[2]? arg[2][34] : 0;
  if (res[2]!=0) res[2][28]=a15;
  a15=arg[2]? arg[2][35] : 0;
  if (res[2]!=0) res[2][29]=a15;
  if (res[2]!=0) res[2][30]=a2;
  if (res[2]!=0) res[2][31]=a2;
  if (res[2]!=0) res[2][32]=a5;
  a11=(a9*a11);
  if (res[2]!=0) res[2][33]=a11;
  a16=(a4*a16);
  if (res[2]!=0) res[2][34]=a16;
  if (res[2]!=0) res[2][35]=a2;
  a16=arg[2]? arg[2][42] : 0;
  if (res[2]!=0) res[2][36]=a16;
  a16=arg[2]? arg[2][43] : 0;
  if (res[2]!=0) res[2][37]=a16;
  a16=arg[2]? arg[2][44] : 0;
  if (res[2]!=0) res[2][38]=a16;
  a16=arg[2]? arg[2][45] : 0;
  if (res[2]!=0) res[2][39]=a16;
  a16=arg[2]? arg[2][46] : 0;
  if (res[2]!=0) res[2][40]=a16;
  a16=arg[2]? arg[2][47] : 0;
  if (res[2]!=0) res[2][41]=a16;
  if (res[2]!=0) res[2][42]=a5;
  if (res[2]!=0) res[2][43]=a2;
  if (res[2]!=0) res[2][44]=a2;
  if (res[2]!=0) res[2][45]=a2;
  a16=(a9*a19);
  if (res[2]!=0) res[2][46]=a16;
  a16=(a4*a17);
  if (res[2]!=0) res[2][47]=a16;
  a16=arg[2]? arg[2][54] : 0;
  if (res[2]!=0) res[2][48]=a16;
  a16=arg[2]? arg[2][55] : 0;
  if (res[2]!=0) res[2][49]=a16;
  a16=arg[2]? arg[2][56] : 0;
  if (res[2]!=0) res[2][50]=a16;
  a16=arg[2]? arg[2][57] : 0;
  if (res[2]!=0) res[2][51]=a16;
  a16=arg[2]? arg[2][58] : 0;
  if (res[2]!=0) res[2][52]=a16;
  a16=arg[2]? arg[2][59] : 0;
  if (res[2]!=0) res[2][53]=a16;
  if (res[2]!=0) res[2][54]=a2;
  if (res[2]!=0) res[2][55]=a5;
  if (res[2]!=0) res[2][56]=a2;
  a19=(a4*a19);
  if (res[2]!=0) res[2][57]=a19;
  if (res[2]!=0) res[2][58]=a2;
  a19=(a9*a24);
  if (res[2]!=0) res[2][59]=a19;
  a19=arg[2]? arg[2][66] : 0;
  if (res[2]!=0) res[2][60]=a19;
  a19=arg[2]? arg[2][67] : 0;
  if (res[2]!=0) res[2][61]=a19;
  a19=arg[2]? arg[2][68] : 0;
  if (res[2]!=0) res[2][62]=a19;
  a19=arg[2]? arg[2][69] : 0;
  if (res[2]!=0) res[2][63]=a19;
  a19=arg[2]? arg[2][70] : 0;
  if (res[2]!=0) res[2][64]=a19;
  a19=arg[2]? arg[2][71] : 0;
  if (res[2]!=0) res[2][65]=a19;
  if (res[2]!=0) res[2][66]=a2;
  if (res[2]!=0) res[2][67]=a2;
  if (res[2]!=0) res[2][68]=a5;
  a17=(a9*a17);
  if (res[2]!=0) res[2][69]=a17;
  a24=(a4*a24);
  if (res[2]!=0) res[2][70]=a24;
  if (res[2]!=0) res[2][71]=a2;
  a24=arg[2]? arg[2][78] : 0;
  if (res[2]!=0) res[2][72]=a24;
  a24=arg[2]? arg[2][79] : 0;
  if (res[2]!=0) res[2][73]=a24;
  a24=arg[2]? arg[2][80] : 0;
  if (res[2]!=0) res[2][74]=a24;
  a24=arg[2]? arg[2][81] : 0;
  if (res[2]!=0) res[2][75]=a24;
  a24=arg[2]? arg[2][82] : 0;
  if (res[2]!=0) res[2][76]=a24;
  a24=arg[2]? arg[2][83] : 0;
  if (res[2]!=0) res[2][77]=a24;
  if (res[2]!=0) res[2][78]=a5;
  if (res[2]!=0) res[2][79]=a2;
  if (res[2]!=0) res[2][80]=a2;
  if (res[2]!=0) res[2][81]=a2;
  a24=(a9*a21);
  if (res[2]!=0) res[2][82]=a24;
  a24=(a4*a18);
  if (res[2]!=0) res[2][83]=a24;
  a24=arg[2]? arg[2][90] : 0;
  if (res[2]!=0) res[2][84]=a24;
  a24=arg[2]? arg[2][91] : 0;
  if (res[2]!=0) res[2][85]=a24;
  a24=arg[2]? arg[2][92] : 0;
  if (res[2]!=0) res[2][86]=a24;
  a24=arg[2]? arg[2][93] : 0;
  if (res[2]!=0) res[2][87]=a24;
  a24=arg[2]? arg[2][94] : 0;
  if (res[2]!=0) res[2][88]=a24;
  a24=arg[2]? arg[2][95] : 0;
  if (res[2]!=0) res[2][89]=a24;
  if (res[2]!=0) res[2][90]=a2;
  if (res[2]!=0) res[2][91]=a5;
  if (res[2]!=0) res[2][92]=a2;
  a21=(a4*a21);
  if (res[2]!=0) res[2][93]=a21;
  if (res[2]!=0) res[2][94]=a2;
  a21=(a9*a10);
  if (res[2]!=0) res[2][95]=a21;
  a21=arg[2]? arg[2][102] : 0;
  if (res[2]!=0) res[2][96]=a21;
  a21=arg[2]? arg[2][103] : 0;
  if (res[2]!=0) res[2][97]=a21;
  a21=arg[2]? arg[2][104] : 0;
  if (res[2]!=0) res[2][98]=a21;
  a21=arg[2]? arg[2][105] : 0;
  if (res[2]!=0) res[2][99]=a21;
  a21=arg[2]? arg[2][106] : 0;
  if (res[2]!=0) res[2][100]=a21;
  a21=arg[2]? arg[2][107] : 0;
  if (res[2]!=0) res[2][101]=a21;
  if (res[2]!=0) res[2][102]=a2;
  if (res[2]!=0) res[2][103]=a2;
  if (res[2]!=0) res[2][104]=a5;
  a18=(a9*a18);
  if (res[2]!=0) res[2][105]=a18;
  a10=(a4*a10);
  if (res[2]!=0) res[2][106]=a10;
  if (res[2]!=0) res[2][107]=a2;
  a10=arg[2]? arg[2][114] : 0;
  if (res[2]!=0) res[2][108]=a10;
  a10=arg[2]? arg[2][115] : 0;
  if (res[2]!=0) res[2][109]=a10;
  a10=arg[2]? arg[2][116] : 0;
  if (res[2]!=0) res[2][110]=a10;
  a10=arg[2]? arg[2][117] : 0;
  if (res[2]!=0) res[2][111]=a10;
  a10=arg[2]? arg[2][118] : 0;
  if (res[2]!=0) res[2][112]=a10;
  a10=arg[2]? arg[2][119] : 0;
  if (res[2]!=0) res[2][113]=a10;
  if (res[2]!=0) res[2][114]=a5;
  if (res[2]!=0) res[2][115]=a2;
  if (res[2]!=0) res[2][116]=a2;
  if (res[2]!=0) res[2][117]=a2;
  a10=(a9*a22);
  if (res[2]!=0) res[2][118]=a10;
  a10=(a4*a20);
  if (res[2]!=0) res[2][119]=a10;
  a10=arg[2]? arg[2][126] : 0;
  if (res[2]!=0) res[2][120]=a10;
  a10=arg[2]? arg[2][127] : 0;
  if (res[2]!=0) res[2][121]=a10;
  a10=arg[2]? arg[2][128] : 0;
  if (res[2]!=0) res[2][122]=a10;
  a10=arg[2]? arg[2][129] : 0;
  if (res[2]!=0) res[2][123]=a10;
  a10=arg[2]? arg[2][130] : 0;
  if (res[2]!=0) res[2][124]=a10;
  a10=arg[2]? arg[2][131] : 0;
  if (res[2]!=0) res[2][125]=a10;
  if (res[2]!=0) res[2][126]=a2;
  if (res[2]!=0) res[2][127]=a5;
  if (res[2]!=0) res[2][128]=a2;
  a22=(a4*a22);
  if (res[2]!=0) res[2][129]=a22;
  if (res[2]!=0) res[2][130]=a2;
  a22=(a9*a12);
  if (res[2]!=0) res[2][131]=a22;
  a22=arg[2]? arg[2][138] : 0;
  if (res[2]!=0) res[2][132]=a22;
  a22=arg[2]? arg[2][139] : 0;
  if (res[2]!=0) res[2][133]=a22;
  a22=arg[2]? arg[2][140] : 0;
  if (res[2]!=0) res[2][134]=a22;
  a22=arg[2]? arg[2][141] : 0;
  if (res[2]!=0) res[2][135]=a22;
  a22=arg[2]? arg[2][142] : 0;
  if (res[2]!=0) res[2][136]=a22;
  a22=arg[2]? arg[2][143] : 0;
  if (res[2]!=0) res[2][137]=a22;
  if (res[2]!=0) res[2][138]=a2;
  if (res[2]!=0) res[2][139]=a2;
  if (res[2]!=0) res[2][140]=a5;
  a9=(a9*a20);
  if (res[2]!=0) res[2][141]=a9;
  a4=(a4*a12);
  if (res[2]!=0) res[2][142]=a4;
  if (res[2]!=0) res[2][143]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadruped_ode_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadruped_ode_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadruped_ode_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real quadruped_ode_expl_vde_forw_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadruped_ode_expl_vde_forw_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadruped_ode_expl_vde_forw_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadruped_ode_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s0;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadruped_ode_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadruped_ode_expl_vde_forw_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 3*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif