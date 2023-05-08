/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: colibri_v1.c
 *
 * Code generated for Simulink model 'colibri_v1'.
 *
 * Model version                  : 4.32
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu May  4 13:41:43 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "colibri_v1.h"
#include "rtwtypes.h"
#include <string.h>
#include <emmintrin.h>
#include <math.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Constant parameters (default storage) */
const ConstP rtConstP = {
  /* Expression: initial_state
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0782, 0.0, 0.1296, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  }
};

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_atan2d_snf(real_T u0, real_T u1);

/* Forward declaration for local functions */
static real_T xnrm2(int32_T n, const real_T x[112], int32_T ix0);
static real_T xdotc(int32_T n, const real_T x[112], int32_T ix0, const real_T y
                    [112], int32_T iy0);
static void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[112], int32_T iy0);
static real_T xnrm2_h(int32_T n, const real_T x[8], int32_T ix0);
static void xaxpy_k(int32_T n, real_T a, const real_T x[112], int32_T ix0,
                    real_T y[14], int32_T iy0);
static void xaxpy_kc(int32_T n, real_T a, const real_T x[14], int32_T ix0,
                     real_T y[112], int32_T iy0);
static real_T xdotc_m(int32_T n, const real_T x[64], int32_T ix0, const real_T
                      y[64], int32_T iy0);
static void xaxpy_kcg(int32_T n, real_T a, int32_T ix0, real_T y[64], int32_T
                      iy0);
static void xswap(real_T x[64], int32_T ix0, int32_T iy0);
static void xswap_k(real_T x[112], int32_T ix0, int32_T iy0);
static void xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
static void xrot(real_T x[64], int32_T ix0, int32_T iy0, real_T c, real_T s);
static void xrot_k(real_T x[112], int32_T ix0, int32_T iy0, real_T c, real_T s);
static void svd(const real_T A[112], real_T U[112], real_T s[8], real_T V[64]);
static real_T eps(real_T x);
static real_T xnrm2_hk(int32_T n, const real_T x[308], int32_T ix0);
static real_T xdotc_mz(int32_T n, const real_T x[308], int32_T ix0, const real_T
  y[308], int32_T iy0);
static void xaxpy_kcgn(int32_T n, real_T a, int32_T ix0, real_T y[308], int32_T
  iy0);
static real_T xnrm2_hk0(int32_T n, const real_T x[14], int32_T ix0);
static void xaxpy_kcgns(int32_T n, real_T a, const real_T x[308], int32_T ix0,
  real_T y[22], int32_T iy0);
static void xaxpy_kcgnsa(int32_T n, real_T a, const real_T x[22], int32_T ix0,
  real_T y[308], int32_T iy0);
static real_T xdotc_mzz(int32_T n, const real_T x[196], int32_T ix0, const
  real_T y[196], int32_T iy0);
static void xaxpy_kcgnsau(int32_T n, real_T a, int32_T ix0, real_T y[196],
  int32_T iy0);
static void xswap_kp(real_T x[196], int32_T ix0, int32_T iy0);
static void xswap_kpw(real_T x[308], int32_T ix0, int32_T iy0);
static void xrot_k5(real_T x[196], int32_T ix0, int32_T iy0, real_T c, real_T s);
static void xrot_k54(real_T x[308], int32_T ix0, int32_T iy0, real_T c, real_T s);
static void svd_j(const real_T A[308], real_T U[308], real_T s[14], real_T V[196]);
static void pinv(const real_T A[308], real_T X_0[308]);
static void q2e(const real_T q[4], real_T *roll, real_T *pitch, real_T *yaw);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xnrm2(int32_T n, const real_T x[112], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xdotc(int32_T n, const real_T x[112], int32_T ix0, const real_T y
                    [112], int32_T iy0)
{
  real_T d;
  int32_T k;
  d = 0.0;
  if (n >= 1) {
    for (k = 0; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  return d;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[112], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xnrm2_h(int32_T n, const real_T x[8], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_k(int32_T n, real_T a, const real_T x[112], int32_T ix0,
                    real_T y[14], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_kc(int32_T n, real_T a, const real_T x[14], int32_T ix0,
                     real_T y[112], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xdotc_m(int32_T n, const real_T x[64], int32_T ix0, const real_T
                      y[64], int32_T iy0)
{
  real_T d;
  int32_T k;
  d = 0.0;
  if (n >= 1) {
    for (k = 0; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  return d;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_kcg(int32_T n, real_T a, int32_T ix0, real_T y[64], int32_T
                      iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xswap(real_T x[64], int32_T ix0, int32_T iy0)
{
  int32_T k;
  for (k = 0; k < 8; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xswap_k(real_T x[112], int32_T ix0, int32_T iy0)
{
  int32_T k;
  for (k = 0; k < 14; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xrotg(real_T *a, real_T *b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xrot(real_T x[64], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  int32_T k;
  for (k = 0; k < 8; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xrot_k(real_T x[112], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  int32_T k;
  for (k = 0; k < 14; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void svd(const real_T A[112], real_T U[112], real_T s[8], real_T V[64])
{
  __m128d tmp;
  real_T b_A[112];
  real_T Vf[64];
  real_T work[14];
  real_T b_s[8];
  real_T e[8];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T qq_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg2;
  memcpy(&b_A[0], &A[0], 112U * sizeof(real_T));
  memset(&b_s[0], 0, sizeof(real_T) << 3U);
  memset(&e[0], 0, sizeof(real_T) << 3U);
  memset(&work[0], 0, 14U * sizeof(real_T));
  memset(&U[0], 0, 112U * sizeof(real_T));
  memset(&Vf[0], 0, sizeof(real_T) << 6U);
  for (i = 0; i < 8; i++) {
    qp1 = i + 2;
    qq_tmp = 14 * i + i;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2(14 - i, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      b_s[i] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - i) + 14;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 14;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_div_pd(tmp, _mm_set1_pd(b_s[i])));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= b_s[i];
        }
      }

      b_A[qq_tmp]++;
      b_s[i] = -b_s[i];
    } else {
      b_s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 9; qp1jj++) {
      qjj = (qp1jj - 1) * 14 + i;
      if (apply_transform) {
        xaxpy(14 - i, -(xdotc(14 - i, b_A, qq_tmp + 1, b_A, qjj + 1) /
                        b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 15; qq++) {
      qp1jj = (14 * i + qq) - 1;
      U[qp1jj] = b_A[qp1jj];
    }

    if (i + 1 <= 6) {
      nrm = xnrm2_h(7 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (fabs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          scalarLB = ((((7 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 9; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          scalarLB = ((((7 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 9; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 15; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 9; qq++) {
          xaxpy_k(13 - i, e[qq - 1], b_A, (i + 14 * (qq - 1)) + 2, work, i + 2);
        }

        for (qq = qp1; qq < 9; qq++) {
          xaxpy_kc(13 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + 14 *
                    (qq - 1)) + 2);
        }
      }

      for (qq = qp1; qq < 9; qq++) {
        Vf[(qq + (i << 3)) - 1] = e[qq - 1];
      }
    }
  }

  i = 6;
  e[6] = b_A[104];
  e[7] = 0.0;
  for (qp1 = 7; qp1 >= 0; qp1--) {
    qq = 14 * qp1 + qp1;
    if (b_s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 9; qp1jj++) {
        qjj = ((qp1jj - 1) * 14 + qp1) + 1;
        xaxpy(14 - qp1, -(xdotc(14 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U,
              qjj);
      }

      for (qjj = qp1 + 1; qjj < 15; qjj++) {
        qp1jj = (14 * qp1 + qjj) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qjj = 0; qjj < qp1; qjj++) {
        U[qjj + 14 * qp1] = 0.0;
      }
    } else {
      memset(&U[qp1 * 14], 0, 14U * sizeof(real_T));
      U[qq] = 1.0;
    }
  }

  for (qp1 = 7; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 6) && (e[qp1] != 0.0)) {
      qq = ((qp1 << 3) + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 9; qjj++) {
        qp1jj = (((qjj - 1) << 3) + qp1) + 2;
        xaxpy_kcg(7 - qp1, -(xdotc_m(7 - qp1, Vf, qq, Vf, qp1jj) / Vf[qq - 1]),
                  qq, Vf, qp1jj);
      }
    }

    memset(&Vf[qp1 << 3], 0, sizeof(real_T) << 3U);
    Vf[qp1 + (qp1 << 3)] = 1.0;
  }

  for (qp1 = 0; qp1 < 8; qp1++) {
    nrm = b_s[qp1];
    if (nrm != 0.0) {
      rt = fabs(nrm);
      nrm /= rt;
      b_s[qp1] = rt;
      if (qp1 + 1 < 8) {
        e[qp1] /= nrm;
      }

      qq = 14 * qp1 + 1;
      scalarLB = 14 + qq;
      vectorUB = qq + 12;
      for (qjj = qq; qjj <= vectorUB; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 13; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 8) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = fabs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        b_s[qp1 + 1] *= nrm;
        qq = ((qp1 + 1) << 3) + 1;
        scalarLB = 8 + qq;
        vectorUB = qq + 6;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&Vf[qjj - 1]);
          _mm_storeu_pd(&Vf[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qq + 7; qjj++) {
          Vf[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 8; qq++) {
    nrm = fmax(nrm, fmax(fabs(b_s[qq]), fabs(e[qq])));
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    qp1jj = i + 1;
    do {
      exitg1 = 0;
      qq = qp1jj;
      if (qp1jj == 0) {
        exitg1 = 1;
      } else {
        rt = fabs(e[qp1jj - 1]);
        if (rt <= (fabs(b_s[qp1jj - 1]) + fabs(b_s[qp1jj])) *
            2.2204460492503131E-16) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (i + 1 == qp1jj) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qq_tmp = i + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp >= qp1jj)) {
        qjj = qq_tmp;
        if (qq_tmp == qp1jj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp < i + 2) {
            rt = fabs(e[qq_tmp - 1]);
          }

          if (qq_tmp > qp1jj + 1) {
            rt += fabs(e[qq_tmp - 2]);
          }

          ztest = fabs(b_s[qq_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            b_s[qq_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp--;
          }
        }
      }

      if (qjj == qp1jj) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      rt = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          smm1 = e[qjj - 2];
          rt = -sqds * smm1;
          e[qjj - 2] = smm1 * ztest;
        }

        xrot(Vf, ((qjj - 1) << 3) + 1, ((i + 1) << 3) + 1, ztest, sqds);
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        smm1 = e[qjj - 1];
        rt = -sqds * smm1;
        e[qjj - 1] = smm1 * ztest;
        xrot_k(U, 14 * (qjj - 1) + 1, 14 * (qq - 1) + 1, ztest, sqds);
      }
      break;

     case 3:
      rt = b_s[i + 1];
      ztest = fmax(fmax(fmax(fmax(fabs(rt), fabs(b_s[i])), fabs(e[i])), fabs
                        (b_s[qq])), fabs(e[qq]));
      rt /= ztest;
      smm1 = b_s[i] / ztest;
      emm1 = e[i] / ztest;
      sqds = b_s[qq] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[qq] / ztest * sqds;
      for (qjj = qq + 1; qjj <= i + 1; qjj++) {
        xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > qq + 1) {
          e[qjj - 2] = rt;
        }

        emm1 = e[qjj - 1];
        rt = b_s[qjj - 1];
        e[qjj - 1] = emm1 * sqds - rt * smm1;
        ztest = smm1 * b_s[qjj];
        b_s[qjj] *= sqds;
        xrot(Vf, ((qjj - 1) << 3) + 1, (qjj << 3) + 1, sqds, smm1);
        b_s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&b_s[qjj - 1], &ztest, &sqds, &smm1);
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * b_s[qjj];
        b_s[qjj] = ztest * -smm1 + sqds * b_s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot_k(U, 14 * (qjj - 1) + 1, 14 * qjj + 1, sqds, smm1);
      }

      e[i] = rt;
      qp1++;
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = (qq << 3) + 1;
        scalarLB = 8 + qp1;
        vectorUB = qp1 + 6;
        for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&Vf[qjj - 1]);
          _mm_storeu_pd(&Vf[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qp1 + 7; qjj++) {
          Vf[qjj - 1] = -Vf[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 8) && (b_s[qq] < b_s[qp1])) {
        rt = b_s[qq];
        b_s[qq] = b_s[qp1];
        b_s[qp1] = rt;
        xswap(Vf, (qq << 3) + 1, ((qq + 1) << 3) + 1);
        xswap_k(U, 14 * qq + 1, 14 * (qq + 1) + 1);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }

  for (i = 0; i < 8; i++) {
    s[i] = b_s[i];
    memcpy(&V[i << 3], &Vf[i << 3], sizeof(real_T) << 3U);
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T eps(real_T x)
{
  real_T absx;
  real_T r;
  int32_T exponent;
  absx = fabs(x);
  if (rtIsInf(absx) || rtIsNaN(absx)) {
    r = (rtNaN);
  } else if (absx < 4.4501477170144028E-308) {
    r = 4.94065645841247E-324;
  } else {
    frexp(absx, &exponent);
    r = ldexp(1.0, exponent - 53);
  }

  return r;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xnrm2_hk(int32_T n, const real_T x[308], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xdotc_mz(int32_T n, const real_T x[308], int32_T ix0, const real_T
  y[308], int32_T iy0)
{
  real_T d;
  int32_T k;
  d = 0.0;
  if (n >= 1) {
    for (k = 0; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  return d;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_kcgn(int32_T n, real_T a, int32_T ix0, real_T y[308], int32_T
  iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xnrm2_hk0(int32_T n, const real_T x[14], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_kcgns(int32_T n, real_T a, const real_T x[308], int32_T ix0,
  real_T y[22], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_kcgnsa(int32_T n, real_T a, const real_T x[22], int32_T ix0,
  real_T y[308], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    int32_T scalarLB;
    int32_T tmp_0;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      tmp_0 = (iy0 + k) - 1;
      tmp = _mm_loadu_pd(&y[tmp_0]);
      _mm_storeu_pd(&y[tmp_0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&x[(ix0 + k) -
        1]), _mm_set1_pd(a)), tmp));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_0 = (iy0 + k) - 1;
      y[tmp_0] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static real_T xdotc_mzz(int32_T n, const real_T x[196], int32_T ix0, const
  real_T y[196], int32_T iy0)
{
  real_T d;
  int32_T k;
  d = 0.0;
  if (n >= 1) {
    for (k = 0; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  return d;
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xaxpy_kcgnsau(int32_T n, real_T a, int32_T ix0, real_T y[196],
  int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xswap_kp(real_T x[196], int32_T ix0, int32_T iy0)
{
  int32_T k;
  for (k = 0; k < 14; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xswap_kpw(real_T x[308], int32_T ix0, int32_T iy0)
{
  int32_T k;
  for (k = 0; k < 22; k++) {
    real_T temp;
    int32_T temp_tmp;
    int32_T tmp;
    temp_tmp = (ix0 + k) - 1;
    temp = x[temp_tmp];
    tmp = (iy0 + k) - 1;
    x[temp_tmp] = x[tmp];
    x[tmp] = temp;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xrot_k5(real_T x[196], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  int32_T k;
  for (k = 0; k < 14; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void xrot_k54(real_T x[308], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  int32_T k;
  for (k = 0; k < 22; k++) {
    real_T temp_tmp;
    real_T temp_tmp_0;
    int32_T temp_tmp_tmp;
    int32_T temp_tmp_tmp_0;
    temp_tmp_tmp = (iy0 + k) - 1;
    temp_tmp = x[temp_tmp_tmp];
    temp_tmp_tmp_0 = (ix0 + k) - 1;
    temp_tmp_0 = x[temp_tmp_tmp_0];
    x[temp_tmp_tmp] = temp_tmp * c - temp_tmp_0 * s;
    x[temp_tmp_tmp_0] = temp_tmp_0 * c + temp_tmp * s;
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void svd_j(const real_T A[308], real_T U[308], real_T s[14], real_T V[196])
{
  __m128d tmp;
  real_T b_A[308];
  real_T Vf[196];
  real_T work[22];
  real_T b_s[14];
  real_T e[14];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T exitg1;
  int32_T i;
  int32_T qjj;
  int32_T qp1;
  int32_T qp1jj;
  int32_T qq;
  int32_T qq_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg2;
  memcpy(&b_A[0], &A[0], 308U * sizeof(real_T));
  memset(&b_s[0], 0, 14U * sizeof(real_T));
  memset(&e[0], 0, 14U * sizeof(real_T));
  memset(&work[0], 0, 22U * sizeof(real_T));
  memset(&U[0], 0, 308U * sizeof(real_T));
  memset(&Vf[0], 0, 196U * sizeof(real_T));
  for (i = 0; i < 14; i++) {
    qp1 = i + 2;
    qq_tmp = 22 * i + i;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = xnrm2_hk(22 - i, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }

      b_s[i] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qq_tmp - i) + 22;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - i) + 22;
        scalarLB = ((((qjj - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qp1jj = qq; qp1jj <= vectorUB; qp1jj += 2) {
          tmp = _mm_loadu_pd(&b_A[qp1jj - 1]);
          _mm_storeu_pd(&b_A[qp1jj - 1], _mm_div_pd(tmp, _mm_set1_pd(b_s[i])));
        }

        for (qp1jj = scalarLB; qp1jj <= qjj; qp1jj++) {
          b_A[qp1jj - 1] /= b_s[i];
        }
      }

      b_A[qq_tmp]++;
      b_s[i] = -b_s[i];
    } else {
      b_s[i] = 0.0;
    }

    for (qp1jj = qp1; qp1jj < 15; qp1jj++) {
      qjj = (qp1jj - 1) * 22 + i;
      if (apply_transform) {
        xaxpy_kcgn(22 - i, -(xdotc_mz(22 - i, b_A, qq_tmp + 1, b_A, qjj + 1) /
                             b_A[qq_tmp]), qq_tmp + 1, b_A, qjj + 1);
      }

      e[qp1jj - 1] = b_A[qjj];
    }

    for (qq = i + 1; qq < 23; qq++) {
      qp1jj = (22 * i + qq) - 1;
      U[qp1jj] = b_A[qp1jj];
    }

    if (i + 1 <= 12) {
      nrm = xnrm2_hk0(13 - i, e, i + 2);
      if (nrm == 0.0) {
        e[i] = 0.0;
      } else {
        if (e[i + 1] < 0.0) {
          e[i] = -nrm;
        } else {
          e[i] = nrm;
        }

        nrm = e[i];
        if (fabs(e[i]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[i];
          scalarLB = ((((13 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 15; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          scalarLB = ((((13 - i) / 2) << 1) + i) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 15; qjj++) {
            e[qjj - 1] /= nrm;
          }
        }

        e[i + 1]++;
        e[i] = -e[i];
        for (qq = qp1; qq < 23; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 15; qq++) {
          xaxpy_kcgns(21 - i, e[qq - 1], b_A, (i + 22 * (qq - 1)) + 2, work, i +
                      2);
        }

        for (qq = qp1; qq < 15; qq++) {
          xaxpy_kcgnsa(21 - i, -e[qq - 1] / e[i + 1], work, i + 2, b_A, (i + 22 *
            (qq - 1)) + 2);
        }
      }

      for (qq = qp1; qq < 15; qq++) {
        Vf[(qq + 14 * i) - 1] = e[qq - 1];
      }
    }
  }

  i = 12;
  e[12] = b_A[298];
  e[13] = 0.0;
  for (qp1 = 13; qp1 >= 0; qp1--) {
    qq = 22 * qp1 + qp1;
    if (b_s[qp1] != 0.0) {
      for (qp1jj = qp1 + 2; qp1jj < 15; qp1jj++) {
        qjj = ((qp1jj - 1) * 22 + qp1) + 1;
        xaxpy_kcgn(22 - qp1, -(xdotc_mz(22 - qp1, U, qq + 1, U, qjj) / U[qq]),
                   qq + 1, U, qjj);
      }

      for (qjj = qp1 + 1; qjj < 23; qjj++) {
        qp1jj = (22 * qp1 + qjj) - 1;
        U[qp1jj] = -U[qp1jj];
      }

      U[qq]++;
      for (qjj = 0; qjj < qp1; qjj++) {
        U[qjj + 22 * qp1] = 0.0;
      }
    } else {
      memset(&U[qp1 * 22], 0, 22U * sizeof(real_T));
      U[qq] = 1.0;
    }
  }

  for (qp1 = 13; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 12) && (e[qp1] != 0.0)) {
      qq = (14 * qp1 + qp1) + 2;
      for (qjj = qp1 + 2; qjj < 15; qjj++) {
        qp1jj = ((qjj - 1) * 14 + qp1) + 2;
        xaxpy_kcgnsau(13 - qp1, -(xdotc_mzz(13 - qp1, Vf, qq, Vf, qp1jj) / Vf[qq
          - 1]), qq, Vf, qp1jj);
      }
    }

    memset(&Vf[qp1 * 14], 0, 14U * sizeof(real_T));
    Vf[qp1 + 14 * qp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 14; qp1++) {
    nrm = b_s[qp1];
    if (nrm != 0.0) {
      rt = fabs(nrm);
      nrm /= rt;
      b_s[qp1] = rt;
      if (qp1 + 1 < 14) {
        e[qp1] /= nrm;
      }

      qq = 22 * qp1 + 1;
      scalarLB = 22 + qq;
      vectorUB = qq + 20;
      for (qjj = qq; qjj <= vectorUB; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 21; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 14) {
      smm1 = e[qp1];
      if (smm1 != 0.0) {
        rt = fabs(smm1);
        nrm = rt / smm1;
        e[qp1] = rt;
        b_s[qp1 + 1] *= nrm;
        qq = (qp1 + 1) * 14 + 1;
        scalarLB = 14 + qq;
        vectorUB = qq + 12;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&Vf[qjj - 1]);
          _mm_storeu_pd(&Vf[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= qq + 13; qjj++) {
          Vf[qjj - 1] *= nrm;
        }
      }
    }
  }

  qp1 = 0;
  nrm = 0.0;
  for (qq = 0; qq < 14; qq++) {
    nrm = fmax(nrm, fmax(fabs(b_s[qq]), fabs(e[qq])));
  }

  while ((i + 2 > 0) && (qp1 < 75)) {
    qp1jj = i + 1;
    do {
      exitg1 = 0;
      qq = qp1jj;
      if (qp1jj == 0) {
        exitg1 = 1;
      } else {
        rt = fabs(e[qp1jj - 1]);
        if (rt <= (fabs(b_s[qp1jj - 1]) + fabs(b_s[qp1jj])) *
            2.2204460492503131E-16) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[qp1jj - 1] = 0.0;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (i + 1 == qp1jj) {
      qp1jj = 4;
    } else {
      qjj = i + 2;
      qq_tmp = i + 2;
      exitg2 = false;
      while ((!exitg2) && (qq_tmp >= qp1jj)) {
        qjj = qq_tmp;
        if (qq_tmp == qp1jj) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (qq_tmp < i + 2) {
            rt = fabs(e[qq_tmp - 1]);
          }

          if (qq_tmp > qp1jj + 1) {
            rt += fabs(e[qq_tmp - 2]);
          }

          ztest = fabs(b_s[qq_tmp - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            b_s[qq_tmp - 1] = 0.0;
            exitg2 = true;
          } else {
            qq_tmp--;
          }
        }
      }

      if (qjj == qp1jj) {
        qp1jj = 3;
      } else if (i + 2 == qjj) {
        qp1jj = 1;
      } else {
        qp1jj = 2;
        qq = qjj;
      }
    }

    switch (qp1jj) {
     case 1:
      rt = e[i];
      e[i] = 0.0;
      for (qjj = i + 1; qjj >= qq + 1; qjj--) {
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          smm1 = e[qjj - 2];
          rt = -sqds * smm1;
          e[qjj - 2] = smm1 * ztest;
        }

        xrot_k5(Vf, 14 * (qjj - 1) + 1, 14 * (i + 1) + 1, ztest, sqds);
      }
      break;

     case 2:
      rt = e[qq - 1];
      e[qq - 1] = 0.0;
      for (qjj = qq + 1; qjj <= i + 2; qjj++) {
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        smm1 = e[qjj - 1];
        rt = -sqds * smm1;
        e[qjj - 1] = smm1 * ztest;
        xrot_k54(U, 22 * (qjj - 1) + 1, 22 * (qq - 1) + 1, ztest, sqds);
      }
      break;

     case 3:
      rt = b_s[i + 1];
      ztest = fmax(fmax(fmax(fmax(fabs(rt), fabs(b_s[i])), fabs(e[i])), fabs
                        (b_s[qq])), fabs(e[qq]));
      rt /= ztest;
      smm1 = b_s[i] / ztest;
      emm1 = e[i] / ztest;
      sqds = b_s[qq] / ztest;
      smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
      emm1 *= rt;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      rt = (sqds + rt) * (sqds - rt) + shift;
      ztest = e[qq] / ztest * sqds;
      for (qjj = qq + 1; qjj <= i + 1; qjj++) {
        xrotg(&rt, &ztest, &sqds, &smm1);
        if (qjj > qq + 1) {
          e[qjj - 2] = rt;
        }

        emm1 = e[qjj - 1];
        rt = b_s[qjj - 1];
        e[qjj - 1] = emm1 * sqds - rt * smm1;
        ztest = smm1 * b_s[qjj];
        b_s[qjj] *= sqds;
        xrot_k5(Vf, 14 * (qjj - 1) + 1, 14 * qjj + 1, sqds, smm1);
        b_s[qjj - 1] = rt * sqds + emm1 * smm1;
        xrotg(&b_s[qjj - 1], &ztest, &sqds, &smm1);
        ztest = e[qjj - 1];
        rt = ztest * sqds + smm1 * b_s[qjj];
        b_s[qjj] = ztest * -smm1 + sqds * b_s[qjj];
        ztest = smm1 * e[qjj];
        e[qjj] *= sqds;
        xrot_k54(U, 22 * (qjj - 1) + 1, 22 * qjj + 1, sqds, smm1);
      }

      e[i] = rt;
      qp1++;
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = 14 * qq + 1;
        scalarLB = 14 + qp1;
        vectorUB = qp1 + 12;
        for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&Vf[qjj - 1]);
          _mm_storeu_pd(&Vf[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qp1 + 13; qjj++) {
          Vf[qjj - 1] = -Vf[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 14) && (b_s[qq] < b_s[qp1])) {
        rt = b_s[qq];
        b_s[qq] = b_s[qp1];
        b_s[qp1] = rt;
        xswap_kp(Vf, 14 * qq + 1, 14 * (qq + 1) + 1);
        xswap_kpw(U, 22 * qq + 1, 22 * (qq + 1) + 1);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      i--;
      break;
    }
  }

  for (i = 0; i < 14; i++) {
    s[i] = b_s[i];
    memcpy(&V[i * 14], &Vf[i * 14], 14U * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<Root>/Drone' */
static void pinv(const real_T A[308], real_T X_0[308])
{
  __m128d tmp;
  __m128d tmp_0;
  real_T U[308];
  real_T V[196];
  real_T s[14];
  real_T tol;
  int32_T ar;
  int32_T b_ic;
  int32_T ib;
  int32_T j;
  int32_T r;
  int32_T scalarLB;
  int32_T scalarLB_tmp;
  int32_T vcol;
  int32_T vectorUB;
  boolean_T p;
  p = true;
  for (r = 0; r < 308; r++) {
    X_0[r] = 0.0;
    if (p) {
      tol = A[r];
      if (rtIsInf(tol) || rtIsNaN(tol)) {
        p = false;
      }
    }
  }

  if (!p) {
    for (r = 0; r < 308; r++) {
      X_0[r] = (rtNaN);
    }
  } else {
    svd_j(A, U, s, V);
    tol = 22.0 * eps(s[0]);
    r = -1;
    vcol = 0;
    while ((vcol < 14) && (s[vcol] > tol)) {
      r++;
      vcol++;
    }

    if (r + 1 > 0) {
      vcol = 1;
      for (j = 0; j <= r; j++) {
        tol = 1.0 / s[j];
        scalarLB_tmp = 14 + vcol;
        vectorUB = vcol + 12;
        for (ar = vcol; ar <= vectorUB; ar += 2) {
          tmp_0 = _mm_loadu_pd(&V[ar - 1]);
          _mm_storeu_pd(&V[ar - 1], _mm_mul_pd(tmp_0, _mm_set1_pd(tol)));
        }

        for (ar = scalarLB_tmp; ar <= vcol + 13; ar++) {
          V[ar - 1] *= tol;
        }

        vcol += 14;
      }

      vcol = 0;
      for (j = 0; j <= 294; j += 14) {
        memset(&X_0[j], 0, 14U * sizeof(real_T));
      }

      for (j = 0; j <= 294; j += 14) {
        ar = -1;
        vcol++;
        scalarLB_tmp = 22 * r + vcol;
        for (ib = vcol; ib <= scalarLB_tmp; ib += 22) {
          scalarLB = j + 15;
          vectorUB = j + 13;
          for (b_ic = j + 1; b_ic <= vectorUB; b_ic += 2) {
            tmp_0 = _mm_loadu_pd(&V[(ar + b_ic) - j]);
            tmp = _mm_loadu_pd(&X_0[b_ic - 1]);
            _mm_storeu_pd(&X_0[b_ic - 1], _mm_add_pd(_mm_mul_pd(tmp_0,
              _mm_set1_pd(U[ib - 1])), tmp));
          }

          for (b_ic = scalarLB; b_ic <= j + 14; b_ic++) {
            X_0[b_ic - 1] += V[(ar + b_ic) - j] * U[ib - 1];
          }

          ar += 14;
        }
      }
    }
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/state' */
static void q2e(const real_T q[4], real_T *roll, real_T *pitch, real_T *yaw)
{
  real_T roll_tmp;
  real_T sinp;
  roll_tmp = q[2] * q[2];
  *roll = rt_atan2d_snf((q[0] * q[1] + q[2] * q[3]) * 2.0, 1.0 - (q[1] * q[1] +
    roll_tmp) * 2.0);
  sinp = (q[0] * q[2] - q[1] * q[3]) * 2.0;
  if (fabs(sinp) >= 1.0) {
    if (rtIsNaN(sinp)) {
      sinp = (rtNaN);
    } else if (sinp < 0.0) {
      sinp = -1.0;
    } else {
      sinp = (sinp > 0.0);
    }

    *pitch = 1.5707963267948966 * sinp;
  } else {
    *pitch = asin(sinp);
  }

  *yaw = rt_atan2d_snf((q[0] * q[3] + q[1] * q[2]) * 2.0, 1.0 - (q[3] * q[3] +
    roll_tmp) * 2.0);
}

/* Model step function */
void colibri_v1_step(void)
{
  real_T M[196];
  real_T A[112];
  real_T U[112];
  real_T X_1[112];
  real_T b_A[112];
  real_T V[64];
  real_T Q[14];
  real_T L1_p[12];
  real_T L1_p_dot[12];
  real_T L2[12];
  real_T L2_dot[12];
  real_T L3_p[12];
  real_T L3_p_dot[12];
  real_T LAB[12];
  real_T LAP[12];
  real_T S[9];
  real_T S_p[9];
  real_T F_motor_BL[3];
  real_T F_motor_BR[3];
  real_T F_motor_UL[3];
  real_T F_motor_UR[3];
  real_T b[3];
  int32_T ar;
  int32_T b_ic;
  int32_T ib;
  int32_T vcol;
  int8_T b_I[196];
  boolean_T p;
  static const real_T d[3] = { 0.0, 0.0, 6.7394700000000007 };

  static const real_T e[3] = { 0.0, 0.0, 7.19073 };

  __m128d tmp_4;
  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  __m128d tmp_8;
  __m128d tmp_9;
  __m128d tmp_a;
  __m128d tmp_b;
  __m128d tmp_c;
  __m128d tmp_d;
  __m128d tmp_e;
  __m128d tmp_f;
  real_T b_I_2[308];
  real_T tmp_3[308];
  real_T b_I_0[196];
  real_T b_I_1[196];
  real_T rtb_dxdt[28];
  real_T Q_0[22];
  real_T tmp_0[16];
  real_T tmp_1[16];
  real_T H_p_tmp[12];
  real_T H_tmp[12];
  real_T M_tmp[12];
  real_T M_tmp_0[12];
  real_T M_tmp_1[12];
  real_T tmp[12];
  real_T rtb_u[8];
  real_T A_tmp[4];
  real_T A_tmp_0[4];
  real_T A_tmp_1[4];
  real_T M_tmp_2[4];
  real_T M_tmp_3[4];
  real_T tmp_2[4];
  real_T F_motor_UR_0[3];
  real_T H_p_tmp_0[3];
  real_T A_tmp_2;
  real_T F_motor_UR_1;
  real_T F_motor_UR_2;
  real_T F_motor_UR_3;
  real_T F_motor_UR_4;
  real_T F_motor_UR_5;
  real_T L2_0;
  real_T L2_dot_0;
  real_T L2_dot_1;
  real_T L2_dot_2;
  real_T S_0;
  real_T S_1;
  real_T S_p_idx_0;
  real_T S_p_idx_0_0;
  real_T S_p_idx_1;
  real_T S_p_idx_1_0;
  real_T S_p_idx_2;
  real_T S_p_idx_2_0;
  real_T S_p_tmp;
  real_T S_p_tmp_0;
  real_T S_p_tmp_1;
  real_T S_p_tmp_2;
  real_T S_tmp;
  real_T S_tmp_0;
  real_T S_tmp_1;
  real_T S_tmp_2;
  real_T S_tmp_3;
  real_T S_tmp_4;
  real_T S_tmp_5;
  real_T rtb_RateLimiter2;
  real_T rtb_RateLimiter3;
  real_T rtb_RateLimiter4;
  real_T rtb_RateLimiter5;
  real_T rtb_Saturation7;
  real_T x_idx_0;
  int32_T M_tmp_tmp;
  int32_T i;
  int32_T scalarLB;
  int32_T vectorUB;
  static const real_T varargin_1[9] = { 0.687, 0.0, 0.0, 0.0, 0.687, 0.0, 0.0,
    0.0, 0.687 };

  static const real_T varargin_3[9] = { 0.733, 0.0, 0.0, 0.0, 0.733, 0.0, 0.0,
    0.0, 0.733 };

  static const real_T y[9] = { 0.0503, 0.0, 0.0, 0.0, 0.0037, 0.0, 0.0, 0.0,
    0.0529 };

  static const real_T b_y[9] = { 0.0016, 0.0, 0.0, 0.0, 0.0131, 0.0, 0.0, 0.0,
    0.0123 };

  static const real_T e_0[3] = { 0.0, 0.0, 7.19073 };

  static const real_T d_0[3] = { 0.0, 0.0, 6.7394700000000007 };

  static const int8_T f[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T g[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  int32_T scalarLB_tmp;

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/u'
   */
  x_idx_0 = rtU.u[0] * 1780.0;
  if (x_idx_0 < 10.0) {
    rtb_RateLimiter2 = 10.0;
  } else if (rtIsNaN(x_idx_0)) {
    rtb_RateLimiter2 = 10.0;
  } else {
    rtb_RateLimiter2 = x_idx_0;
  }

  x_idx_0 = rtU.u[1] * 1780.0;
  if (x_idx_0 < 10.0) {
    rtb_RateLimiter3 = 10.0;
  } else if (rtIsNaN(x_idx_0)) {
    rtb_RateLimiter3 = 10.0;
  } else {
    rtb_RateLimiter3 = x_idx_0;
  }

  x_idx_0 = rtU.u[2] * 1780.0;
  if (x_idx_0 < 10.0) {
    rtb_RateLimiter4 = 10.0;
  } else if (rtIsNaN(x_idx_0)) {
    rtb_RateLimiter4 = 10.0;
  } else {
    rtb_RateLimiter4 = x_idx_0;
  }

  x_idx_0 = rtU.u[3] * 1780.0;
  if (x_idx_0 < 10.0) {
    rtb_RateLimiter5 = 10.0;
  } else if (rtIsNaN(x_idx_0)) {
    rtb_RateLimiter5 = 10.0;
  } else {
    rtb_RateLimiter5 = x_idx_0;
  }

  x_idx_0 = rtU.u[7] * 1780.0;
  rtb_u[4] = rtU.u[4] * 0.52359877559829882;
  rtb_u[5] = rtU.u[5] * 0.52359877559829882;
  rtb_u[6] = rtU.u[6] * 0.52359877559829882;
  if (x_idx_0 < 10.0) {
    rtb_u[7] = 10.0;
  } else if (rtIsNaN(x_idx_0)) {
    rtb_u[7] = 10.0;
  } else {
    rtb_u[7] = x_idx_0;
  }

  /* Saturate: '<S3>/Saturation4' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  if (rtb_RateLimiter2 > 1780.0) {
    rtb_Saturation7 = 1780.0;
  } else {
    rtb_Saturation7 = rtb_RateLimiter2;
  }

  /* End of Saturate: '<S3>/Saturation4' */

  /* RateLimiter: '<S3>/Rate Limiter2' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY;
  if (x_idx_0 > 307.755) {
    rtb_RateLimiter2 = rtDW.PrevY + 307.755;
  } else if (x_idx_0 < -307.755) {
    rtb_RateLimiter2 = rtDW.PrevY - 307.755;
  } else {
    rtb_RateLimiter2 = rtb_Saturation7;
  }

  rtDW.PrevY = rtb_RateLimiter2;

  /* End of RateLimiter: '<S3>/Rate Limiter2' */

  /* Saturate: '<S3>/Saturation2' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  if (rtb_RateLimiter3 > 1780.0) {
    rtb_Saturation7 = 1780.0;
  } else {
    rtb_Saturation7 = rtb_RateLimiter3;
  }

  /* End of Saturate: '<S3>/Saturation2' */

  /* RateLimiter: '<S3>/Rate Limiter3' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_a;
  if (x_idx_0 > 307.755) {
    rtb_RateLimiter3 = rtDW.PrevY_a + 307.755;
  } else if (x_idx_0 < -307.755) {
    rtb_RateLimiter3 = rtDW.PrevY_a - 307.755;
  } else {
    rtb_RateLimiter3 = rtb_Saturation7;
  }

  rtDW.PrevY_a = rtb_RateLimiter3;

  /* End of RateLimiter: '<S3>/Rate Limiter3' */

  /* Saturate: '<S3>/Saturation5' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  if (rtb_RateLimiter4 > 1780.0) {
    rtb_Saturation7 = 1780.0;
  } else {
    rtb_Saturation7 = rtb_RateLimiter4;
  }

  /* End of Saturate: '<S3>/Saturation5' */

  /* RateLimiter: '<S3>/Rate Limiter4' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_ah;
  if (x_idx_0 > 307.755) {
    rtb_RateLimiter4 = rtDW.PrevY_ah + 307.755;
  } else if (x_idx_0 < -307.755) {
    rtb_RateLimiter4 = rtDW.PrevY_ah - 307.755;
  } else {
    rtb_RateLimiter4 = rtb_Saturation7;
  }

  rtDW.PrevY_ah = rtb_RateLimiter4;

  /* End of RateLimiter: '<S3>/Rate Limiter4' */

  /* Saturate: '<S3>/Saturation6' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function1'
   */
  if (rtb_RateLimiter5 > 1780.0) {
    rtb_Saturation7 = 1780.0;
  } else {
    rtb_Saturation7 = rtb_RateLimiter5;
  }

  /* End of Saturate: '<S3>/Saturation6' */

  /* RateLimiter: '<S3>/Rate Limiter5' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_h;
  if (x_idx_0 > 307.755) {
    rtb_RateLimiter5 = rtDW.PrevY_h + 307.755;
  } else if (x_idx_0 < -307.755) {
    rtb_RateLimiter5 = rtDW.PrevY_h - 307.755;
  } else {
    rtb_RateLimiter5 = rtb_Saturation7;
  }

  rtDW.PrevY_h = rtb_RateLimiter5;

  /* End of RateLimiter: '<S3>/Rate Limiter5' */

  /* Saturate: '<S3>/Saturation3' */
  if (rtb_u[4] > 0.52359877559829882) {
    rtb_Saturation7 = 0.52359877559829882;
  } else if (rtb_u[4] < -0.52359877559829882) {
    rtb_Saturation7 = -0.52359877559829882;
  } else {
    rtb_Saturation7 = rtb_u[4];
  }

  /* End of Saturate: '<S3>/Saturation3' */

  /* RateLimiter: '<S3>/Rate Limiter' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_as;
  if (x_idx_0 > 0.00511556003759538) {
    rtDW.PrevY_as += 0.00511556003759538;
  } else if (x_idx_0 < -0.00511556003759538) {
    rtDW.PrevY_as -= 0.00511556003759538;
  } else {
    rtDW.PrevY_as = rtb_Saturation7;
  }

  /* End of RateLimiter: '<S3>/Rate Limiter' */

  /* Saturate: '<S3>/Saturation1' */
  if (rtb_u[5] > 0.52359877559829882) {
    rtb_Saturation7 = 0.52359877559829882;
  } else if (rtb_u[5] < -0.52359877559829882) {
    rtb_Saturation7 = -0.52359877559829882;
  } else {
    rtb_Saturation7 = rtb_u[5];
  }

  /* End of Saturate: '<S3>/Saturation1' */

  /* RateLimiter: '<S3>/Rate Limiter1' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_l;
  if (x_idx_0 > 0.00511556003759538) {
    rtDW.PrevY_l += 0.00511556003759538;
  } else if (x_idx_0 < -0.00511556003759538) {
    rtDW.PrevY_l -= 0.00511556003759538;
  } else {
    rtDW.PrevY_l = rtb_Saturation7;
  }

  /* End of RateLimiter: '<S3>/Rate Limiter1' */

  /* Saturate: '<S3>/Saturation8' */
  if (rtb_u[6] > 0.52359877559829882) {
    rtb_Saturation7 = 0.52359877559829882;
  } else if (rtb_u[6] < -0.52359877559829882) {
    rtb_Saturation7 = -0.52359877559829882;
  } else {
    rtb_Saturation7 = rtb_u[6];
  }

  /* End of Saturate: '<S3>/Saturation8' */

  /* RateLimiter: '<S3>/Rate Limiter7' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_j;
  if (x_idx_0 > 0.00511556003759538) {
    rtDW.PrevY_j += 0.00511556003759538;
  } else if (x_idx_0 < -0.00511556003759538) {
    rtDW.PrevY_j -= 0.00511556003759538;
  } else {
    rtDW.PrevY_j = rtb_Saturation7;
  }

  /* End of RateLimiter: '<S3>/Rate Limiter7' */

  /* Saturate: '<S3>/Saturation7' */
  if (rtb_u[7] > 1780.0) {
    rtb_Saturation7 = 1780.0;
  } else {
    rtb_Saturation7 = rtb_u[7];
  }

  /* End of Saturate: '<S3>/Saturation7' */

  /* RateLimiter: '<S3>/Rate Limiter6' */
  x_idx_0 = rtb_Saturation7 - rtDW.PrevY_d;
  if (x_idx_0 > 307.755) {
    rtb_Saturation7 = rtDW.PrevY_d + 307.755;
  } else if (x_idx_0 < -307.755) {
    rtb_Saturation7 = rtDW.PrevY_d - 307.755;
  }

  rtDW.PrevY_d = rtb_Saturation7;

  /* End of RateLimiter: '<S3>/Rate Limiter6' */

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  MATLAB Function: '<Root>/state'
   */
  H_tmp[0] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7];
  H_tmp[3] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
  H_tmp[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9];
  H_tmp[9] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8];
  H_tmp[1] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8];
  H_tmp[4] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9];
  H_tmp[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
  H_tmp[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7];
  H_tmp[2] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9];
  H_tmp[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8];
  H_tmp[8] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7];
  H_tmp[11] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
  H_p_tmp[0] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  H_p_tmp[3] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  H_p_tmp[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  H_p_tmp[9] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  H_p_tmp[1] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  H_p_tmp[4] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  H_p_tmp[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  H_p_tmp[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  H_p_tmp[2] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  H_p_tmp[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  H_p_tmp[8] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  H_p_tmp[11] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  x_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE[6] *
    rtDW.DiscreteTimeIntegrator_DSTATE[6];
  S_tmp_1 = x_idx_0 * 2.0 - 1.0;
  S_tmp_5 = rtDW.DiscreteTimeIntegrator_DSTATE[7] *
    rtDW.DiscreteTimeIntegrator_DSTATE[7];
  S[0] = S_tmp_5 * 2.0 + S_tmp_1;
  S_tmp = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7] *
    rtDW.DiscreteTimeIntegrator_DSTATE[8];
  S_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] *
    rtDW.DiscreteTimeIntegrator_DSTATE[9];
  S[3] = S_tmp - S_tmp_0;
  S_tmp_2 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7] *
    rtDW.DiscreteTimeIntegrator_DSTATE[9];
  S_tmp_3 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] *
    rtDW.DiscreteTimeIntegrator_DSTATE[8];
  S[6] = S_tmp_2 + S_tmp_3;
  S[1] = S_tmp + S_tmp_0;
  S_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[8] *
    rtDW.DiscreteTimeIntegrator_DSTATE[8];
  S[4] = S_tmp * 2.0 + S_tmp_1;
  S_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8] *
    rtDW.DiscreteTimeIntegrator_DSTATE[9];
  S_tmp_4 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] *
    rtDW.DiscreteTimeIntegrator_DSTATE[7];
  S[7] = S_tmp_0 - S_tmp_4;
  S[2] = S_tmp_2 - S_tmp_3;
  S[5] = S_tmp_0 + S_tmp_4;
  S_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[9] *
    rtDW.DiscreteTimeIntegrator_DSTATE[9];
  S[8] = S_tmp_0 * 2.0 + S_tmp_1;
  S_tmp_1 = rtDW.DiscreteTimeIntegrator_DSTATE[20] *
    rtDW.DiscreteTimeIntegrator_DSTATE[20];
  S_p_tmp = S_tmp_1 * 2.0 - 1.0;
  S_tmp_2 = rtDW.DiscreteTimeIntegrator_DSTATE[21] *
    rtDW.DiscreteTimeIntegrator_DSTATE[21];
  S_p[0] = S_tmp_2 * 2.0 + S_p_tmp;
  S_tmp_3 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21] *
    rtDW.DiscreteTimeIntegrator_DSTATE[22];
  S_tmp_4 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20] *
    rtDW.DiscreteTimeIntegrator_DSTATE[23];
  S_p[3] = S_tmp_3 - S_tmp_4;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21] *
    rtDW.DiscreteTimeIntegrator_DSTATE[23];
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20] *
    rtDW.DiscreteTimeIntegrator_DSTATE[22];
  S_p[6] = S_p_tmp_0 + S_p_tmp_1;
  S_p[1] = S_tmp_3 + S_tmp_4;
  S_tmp_3 = rtDW.DiscreteTimeIntegrator_DSTATE[22] *
    rtDW.DiscreteTimeIntegrator_DSTATE[22];
  S_p[4] = S_tmp_3 * 2.0 + S_p_tmp;
  S_tmp_4 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22] *
    rtDW.DiscreteTimeIntegrator_DSTATE[23];
  S_p_tmp_2 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20] *
    rtDW.DiscreteTimeIntegrator_DSTATE[21];
  S_p[7] = S_tmp_4 - S_p_tmp_2;
  S_p[2] = S_p_tmp_0 - S_p_tmp_1;
  S_p[5] = S_tmp_4 + S_p_tmp_2;
  S_tmp_4 = rtDW.DiscreteTimeIntegrator_DSTATE[23] *
    rtDW.DiscreteTimeIntegrator_DSTATE[23];
  S_p[8] = S_tmp_4 * 2.0 + S_p_tmp;
  L2[0] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9];
  L2[3] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8];
  L2[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7];
  L2[9] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
  L2[1] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
  L2[4] = 0.0;
  L2[7] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8];
  L2[10] = 0.0;
  L2[2] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7];
  L2[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6];
  L2[8] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9];
  L2[11] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8];
  L2_dot[0] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13];
  L2_dot[3] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12];
  L2_dot[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11];
  L2_dot[9] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10];
  L2_dot[1] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10];
  L2_dot[4] = 0.0;
  L2_dot[7] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12];
  L2_dot[10] = 0.0;
  L2_dot[2] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11];
  L2_dot[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10];
  L2_dot[8] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13];
  L2_dot[11] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12];
  L1_p[0] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  L1_p[3] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  L1_p[6] = 0.0;
  L1_p[9] = 0.0;
  L1_p[1] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  L1_p[4] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  L1_p[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  L1_p[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  L1_p[2] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  L1_p[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  L1_p[8] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  L1_p[11] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  L3_p[0] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  L3_p[3] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  L3_p[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  L3_p[9] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  L3_p[1] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21];
  L3_p[4] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  L3_p[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  L3_p[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22];
  L3_p[2] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20];
  L3_p[5] = 0.0;
  L3_p[8] = 0.0;
  L3_p[11] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23];
  L1_p_dot[0] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
  L1_p_dot[3] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25];
  L1_p_dot[6] = 0.0;
  L1_p_dot[9] = 0.0;
  L1_p_dot[1] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27];
  L1_p_dot[4] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26];
  L1_p_dot[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25];
  L1_p_dot[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
  L1_p_dot[2] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26];
  L1_p_dot[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27];
  L1_p_dot[8] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
  L1_p_dot[11] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25];
  L3_p_dot[0] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26];
  L3_p_dot[3] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27];
  L3_p_dot[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
  L3_p_dot[9] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25];
  L3_p_dot[1] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25];
  L3_p_dot[4] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
  L3_p_dot[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27];
  L3_p_dot[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26];
  L3_p_dot[2] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
  L3_p_dot[5] = 0.0;
  L3_p_dot[8] = 0.0;
  L3_p_dot[11] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27];
  M_tmp_1[0] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] * 0.0;
  M_tmp_1[3] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7] * 0.0;
  M_tmp_1[6] = 0.0;
  M_tmp_1[9] = 0.0;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9] * 0.0;
  M_tmp_1[1] = S_p_tmp_0;
  M_tmp_1[4] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8] * 0.0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7] * 0.0;
  M_tmp_1[7] = S_p_tmp_1;
  M_tmp_1[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] * 0.0;
  M_tmp_1[2] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8] * 0.0;
  M_tmp_1[5] = S_p_tmp_0;
  M_tmp_1[8] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] * 0.0;
  M_tmp_1[11] = S_p_tmp_1;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[8] * -0.026;
  tmp[0] = S_p_tmp_0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9] * -0.026;
  tmp[3] = S_p_tmp_1;
  tmp[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] * -0.026;
  tmp[9] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7] * -0.026;
  tmp[1] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[7] * -0.026;
  tmp[4] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] * -0.026;
  tmp[7] = S_p_tmp_1;
  tmp[10] = S_p_tmp_0;
  tmp[2] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[6] * -0.026;
  tmp[5] = -0.0;
  tmp[8] = -0.0;
  tmp[11] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[9] * -0.026;
  M_tmp[0] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23] * 0.0;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22] * 0.0;
  M_tmp[3] = S_p_tmp_0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[21] * 0.0;
  M_tmp[6] = S_p_tmp_1;
  M_tmp[9] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20] * 0.0;
  M_tmp[1] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20] * 0.0;
  M_tmp[4] = 0.0;
  M_tmp[7] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[22] * 0.0;
  M_tmp[10] = 0.0;
  M_tmp[2] = S_p_tmp_1;
  M_tmp[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[20] * 0.0;
  M_tmp[8] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[23] * 0.0;
  M_tmp[11] = S_p_tmp_0;
  for (i = 0; i <= 10; i += 2) {
    /* MATLAB Function: '<Root>/Drone' */
    tmp_9 = _mm_loadu_pd(&L2[i]);
    tmp_a = _mm_loadu_pd(&M_tmp_1[i]);
    tmp_b = _mm_loadu_pd(&tmp[i]);
    _mm_storeu_pd(&LAB[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.0),
      tmp_9), tmp_a), tmp_b));
    tmp_9 = _mm_loadu_pd(&L1_p[i]);
    tmp_a = _mm_loadu_pd(&M_tmp[i]);
    tmp_b = _mm_loadu_pd(&L3_p[i]);
    _mm_storeu_pd(&LAP[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.0782),
      tmp_9), tmp_a), _mm_mul_pd(_mm_set1_pd(-0.1556), tmp_b)));
  }

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  SignalConversion generated from: '<S1>/ SFunction '
   */
  memset(&M[0], 0, 196U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    M[14 * i] = varargin_1[3 * i];
    M[14 * i + 1] = varargin_1[3 * i + 1];
    M[14 * i + 2] = varargin_1[3 * i + 2];
    M_tmp_tmp = i << 2;
    M_tmp[M_tmp_tmp] = H_tmp[i];
    M_tmp[M_tmp_tmp + 1] = H_tmp[i + 3];
    M_tmp[M_tmp_tmp + 2] = H_tmp[i + 6];
    M_tmp[M_tmp_tmp + 3] = H_tmp[i + 9];
  }

  for (i = 0; i < 4; i++) {
    S_p_tmp = M_tmp[i + 4];
    S_p_tmp_0 = M_tmp[i];
    S_p_tmp_1 = M_tmp[i + 8];
    for (vcol = 0; vcol < 3; vcol++) {
      M_tmp_0[i + (vcol << 2)] = (y[3 * vcol + 1] * S_p_tmp + y[3 * vcol] *
        S_p_tmp_0) + y[3 * vcol + 2] * S_p_tmp_1;
    }

    S_p_tmp = M_tmp_0[i + 4];
    S_p_tmp_0 = M_tmp_0[i];
    S_p_tmp_1 = M_tmp_0[i + 8];
    for (vcol = 0; vcol < 4; vcol++) {
      M[(i + 14 * (vcol + 3)) + 3] = (H_tmp[3 * vcol + 1] * S_p_tmp + H_tmp[3 *
        vcol] * S_p_tmp_0) + H_tmp[3 * vcol + 2] * S_p_tmp_1;
    }
  }

  for (i = 0; i < 3; i++) {
    vcol = (i + 7) * 14;
    M[vcol + 7] = varargin_3[3 * i];
    M[vcol + 8] = varargin_3[3 * i + 1];
    M[vcol + 9] = varargin_3[3 * i + 2];
    M_tmp_tmp = i << 2;
    M_tmp_0[M_tmp_tmp] = H_p_tmp[i];
    M_tmp_0[M_tmp_tmp + 1] = H_p_tmp[i + 3];
    M_tmp_0[M_tmp_tmp + 2] = H_p_tmp[i + 6];
    M_tmp_0[M_tmp_tmp + 3] = H_p_tmp[i + 9];
  }

  for (i = 0; i < 4; i++) {
    S_p_tmp = M_tmp_0[i + 4];
    S_p_tmp_0 = M_tmp_0[i];
    S_p_tmp_1 = M_tmp_0[i + 8];
    for (vcol = 0; vcol < 3; vcol++) {
      M_tmp_1[i + (vcol << 2)] = (b_y[3 * vcol + 1] * S_p_tmp + b_y[3 * vcol] *
        S_p_tmp_0) + b_y[3 * vcol + 2] * S_p_tmp_1;
    }

    S_p_tmp = M_tmp_1[i + 4];
    S_p_tmp_0 = M_tmp_1[i];
    S_p_tmp_1 = M_tmp_1[i + 8];
    for (vcol = 0; vcol < 4; vcol++) {
      M[(i + 14 * (vcol + 10)) + 10] = (H_p_tmp[3 * vcol + 1] * S_p_tmp +
        H_p_tmp[3 * vcol] * S_p_tmp_0) + H_p_tmp[3 * vcol + 2] * S_p_tmp_1;
    }
  }

  rtb_RateLimiter2 *= rtb_RateLimiter2;
  F_motor_UR[0] = rtb_RateLimiter2 * 1.3277417013759314E-6;
  F_motor_UR[1] = 0.0;
  F_motor_UR[2] = rtb_RateLimiter2 * -4.95519948886292E-6;
  rtb_RateLimiter3 *= rtb_RateLimiter3;
  F_motor_BR[0] = rtb_RateLimiter3 * -1.3277417013759314E-6;
  F_motor_BR[1] = 0.0;
  F_motor_BR[2] = rtb_RateLimiter3 * -4.95519948886292E-6;
  rtb_RateLimiter4 *= rtb_RateLimiter4;
  F_motor_BL[0] = rtb_RateLimiter4 * -1.3277417013759314E-6;
  F_motor_BL[1] = 0.0;
  F_motor_BL[2] = rtb_RateLimiter4 * -4.95519948886292E-6;
  rtb_RateLimiter5 *= rtb_RateLimiter5;
  F_motor_UL[0] = rtb_RateLimiter5 * 1.3277417013759314E-6;
  F_motor_UL[1] = 0.0;
  F_motor_UL[2] = rtb_RateLimiter5 * -4.95519948886292E-6;
  rtb_RateLimiter5 = rtb_Saturation7 * rtb_Saturation7 * -5.13E-6;
  for (i = 0; i < 3; i++) {
    b[i] = (S_p[3 * i + 1] * 0.0 + S_p[3 * i] * 0.0) + S_p[3 * i + 2];
    F_motor_UR_0[i] = ((F_motor_UR[i] + F_motor_BR[i]) + F_motor_BL[i]) +
      F_motor_UL[i];
  }

  S_p_tmp_0 = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11] * -2.0;
  M_tmp_1[0] = S_p_tmp_0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * -2.0;
  M_tmp_1[1] = S_p_tmp_1;
  M_tmp_1[2] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13] * -2.0;
  rtb_Saturation7 = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12] * -2.0;
  M_tmp_1[3] = rtb_Saturation7;
  M_tmp_1[4] = rtb_Saturation7;
  rtb_Saturation7 = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13] * -2.0;
  M_tmp_1[5] = rtb_Saturation7;
  M_tmp_1[6] = S_p_tmp_1;
  M_tmp_1[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11] * -2.0;
  M_tmp_1[8] = rtb_Saturation7;
  M_tmp_1[9] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12] * -2.0;
  M_tmp_1[10] = S_p_tmp_0;
  M_tmp_1[11] = S_p_tmp_1;
  for (i = 0; i < 4; i++) {
    S_p_tmp_0 = M_tmp_1[i + 4];
    S_p_tmp_1 = M_tmp_1[i];
    rtb_Saturation7 = M_tmp_1[i + 8];
    for (vcol = 0; vcol < 3; vcol++) {
      tmp[i + (vcol << 2)] = (y[3 * vcol + 1] * S_p_tmp_0 + y[3 * vcol] *
        S_p_tmp_1) + y[3 * vcol + 2] * rtb_Saturation7;
    }

    S_p_tmp_0 = tmp[i + 4];
    S_p_tmp_1 = tmp[i];
    rtb_Saturation7 = tmp[i + 8];
    for (vcol = 0; vcol < 4; vcol++) {
      tmp_0[i + (vcol << 2)] = (H_tmp[3 * vcol + 1] * S_p_tmp_0 + H_tmp[3 * vcol]
        * S_p_tmp_1) + H_tmp[3 * vcol + 2] * rtb_Saturation7;
    }
  }

  H_p_tmp_0[0] = ((0.24 * F_motor_UR[2] + 0.24 * F_motor_BR[2]) + -0.24 *
                  F_motor_BL[2]) + -0.24 * F_motor_UL[2];
  H_p_tmp_0[1] = (((0.011 * F_motor_UR[0] - -0.084 * F_motor_UR[2]) + (0.011 *
    F_motor_BR[0] - 0.084 * F_motor_BR[2])) + (0.011 * F_motor_BL[0] - 0.084 *
    F_motor_BL[2])) + (0.011 * F_motor_UL[0] - -0.084 * F_motor_UL[2]);
  H_p_tmp_0[2] = (((-0.0 - 0.24 * F_motor_UR[0]) + (0.0 - 0.24 * F_motor_BR[0]))
                  + (0.0 - -0.24 * F_motor_BL[0])) + (-0.0 - -0.24 * F_motor_UL
    [0]);
  S_p_tmp_0 = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25] * -2.0;
  M_tmp_1[0] = S_p_tmp_0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24] * -2.0;
  M_tmp_1[1] = S_p_tmp_1;
  M_tmp_1[2] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27] * -2.0;
  rtb_Saturation7 = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26] * -2.0;
  M_tmp_1[3] = rtb_Saturation7;
  M_tmp_1[4] = rtb_Saturation7;
  rtb_Saturation7 = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27] * -2.0;
  M_tmp_1[5] = rtb_Saturation7;
  M_tmp_1[6] = S_p_tmp_1;
  M_tmp_1[7] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25] * -2.0;
  M_tmp_1[8] = rtb_Saturation7;
  M_tmp_1[9] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26] * -2.0;
  M_tmp_1[10] = S_p_tmp_0;
  M_tmp_1[11] = S_p_tmp_1;
  for (i = 0; i < 4; i++) {
    A_tmp[i] = ((tmp_0[i + 4] * rtDW.DiscreteTimeIntegrator_DSTATE[11] + tmp_0[i]
                 * rtDW.DiscreteTimeIntegrator_DSTATE[10]) + tmp_0[i + 8] *
                rtDW.DiscreteTimeIntegrator_DSTATE[12]) + tmp_0[i + 12] *
      rtDW.DiscreteTimeIntegrator_DSTATE[13];
    S_p_tmp = 0.0;
    S_p_tmp_0 = M_tmp_1[i + 4];
    S_p_tmp_1 = M_tmp_1[i];
    rtb_Saturation7 = M_tmp_1[i + 8];
    for (vcol = 0; vcol < 3; vcol++) {
      M_tmp_tmp = (vcol << 2) + i;
      S_p_tmp += M_tmp[M_tmp_tmp] * H_p_tmp_0[vcol];
      tmp[M_tmp_tmp] = (b_y[3 * vcol + 1] * S_p_tmp_0 + b_y[3 * vcol] *
                        S_p_tmp_1) + b_y[3 * vcol + 2] * rtb_Saturation7;
    }

    M_tmp_2[i] = S_p_tmp;
    S_p_tmp_0 = tmp[i + 4];
    S_p_tmp_1 = tmp[i];
    rtb_Saturation7 = tmp[i + 8];
    for (vcol = 0; vcol < 4; vcol++) {
      tmp_1[i + (vcol << 2)] = (H_p_tmp[3 * vcol + 1] * S_p_tmp_0 + H_p_tmp[3 *
        vcol] * S_p_tmp_1) + H_p_tmp[3 * vcol + 2] * rtb_Saturation7;
    }
  }

  rtb_Saturation7 = -0.562315086 * b[1] - -0.0 * b[0];
  rtb_RateLimiter4 = (-0.0 * b[2] - 1.118877588 * b[1]) + 0.0 * rtb_RateLimiter5;
  rtb_RateLimiter3 = (1.118877588 * b[0] - -0.562315086 * b[2]) + (0.0 - -0.3062
    * rtb_RateLimiter5);
  S_p_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[25];
  S_p_tmp_1 = rtDW.DiscreteTimeIntegrator_DSTATE[24];
  rtb_RateLimiter2 = rtDW.DiscreteTimeIntegrator_DSTATE[26];
  S_p_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[27];
  for (i = 0; i <= 2; i += 2) {
    /* MATLAB Function: '<Root>/Drone' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    tmp_9 = _mm_loadu_pd(&tmp_1[i + 4]);
    tmp_a = _mm_loadu_pd(&tmp_1[i]);
    tmp_b = _mm_loadu_pd(&tmp_1[i + 8]);
    tmp_8 = _mm_loadu_pd(&tmp_1[i + 12]);
    _mm_storeu_pd(&tmp_2[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_9,
      _mm_set1_pd(S_p_tmp_0)), _mm_mul_pd(tmp_a, _mm_set1_pd(S_p_tmp_1))),
      _mm_mul_pd(tmp_b, _mm_set1_pd(rtb_RateLimiter2))), _mm_mul_pd(tmp_8,
      _mm_set1_pd(S_p_tmp))));
    tmp_9 = _mm_loadu_pd(&M_tmp_0[i + 4]);
    tmp_a = _mm_loadu_pd(&M_tmp_0[i]);
    tmp_b = _mm_loadu_pd(&M_tmp_0[i + 8]);
    _mm_storeu_pd(&M_tmp_3[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_9,
      _mm_set1_pd(rtb_RateLimiter3)), _mm_mul_pd(tmp_a, _mm_set1_pd
      (rtb_RateLimiter4))), _mm_mul_pd(tmp_b, _mm_set1_pd(rtb_Saturation7))));
  }

  /* MATLAB Function: '<Root>/Drone' */
  F_motor_UR_3 = F_motor_UR_0[1];
  F_motor_UR_4 = F_motor_UR_0[0];
  F_motor_UR_5 = F_motor_UR_0[2];
  for (i = 0; i <= 0; i += 2) {
    /* MATLAB Function: '<Root>/Drone' */
    tmp_9 = _mm_loadu_pd(&S_p[i + 3]);
    tmp_a = _mm_set1_pd(0.0);
    tmp_b = _mm_loadu_pd(&S_p[i]);
    tmp_8 = _mm_loadu_pd(&S_p[i + 6]);
    _mm_storeu_pd(&F_motor_BR[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (tmp_9, tmp_a), _mm_mul_pd(tmp_b, tmp_a)), _mm_mul_pd(tmp_8, _mm_set1_pd
      (rtb_RateLimiter5))), _mm_loadu_pd(&e[i])));
    tmp_9 = _mm_loadu_pd(&S[i + 3]);
    tmp_a = _mm_loadu_pd(&S[i]);
    tmp_b = _mm_loadu_pd(&S[i + 6]);
    _mm_storeu_pd(&Q[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(tmp_9,
      _mm_set1_pd(F_motor_UR_3)), _mm_mul_pd(tmp_a, _mm_set1_pd(F_motor_UR_4))),
      _mm_mul_pd(tmp_b, _mm_set1_pd(F_motor_UR_5))), _mm_loadu_pd(&d[i])));
  }

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  for (i = 2; i < 3; i++) {
    F_motor_BR[i] = ((S_p[i + 3] * 0.0 + S_p[i] * 0.0) + S_p[i + 6] *
                     rtb_RateLimiter5) + e_0[i];
    Q[i] = ((S[i + 3] * F_motor_UR_3 + S[i] * F_motor_UR_4) + S[i + 6] *
            F_motor_UR_5) + d_0[i];
  }

  Q[3] = A_tmp[0] + M_tmp_2[0];
  Q[4] = A_tmp[1] + M_tmp_2[1];
  Q[5] = A_tmp[2] + M_tmp_2[2];
  Q[6] = A_tmp[3] + M_tmp_2[3];
  Q[7] = F_motor_BR[0];
  Q[8] = F_motor_BR[1];
  Q[9] = F_motor_BR[2];
  rtb_RateLimiter5 = S_p[6];
  rtb_Saturation7 = S_p[0];
  S_p_tmp_0 = S_p[7];
  S_p_tmp_1 = S_p[1];
  F_motor_UR_3 = S_p[2];
  F_motor_UR_4 = S_p[8];
  rtb_RateLimiter4 = S[4];
  rtb_RateLimiter3 = S[3];
  S_p_tmp_2 = S[5];
  for (i = 0; i < 4; i++) {
    Q[i + 10] = tmp_2[i] + M_tmp_3[i];
    L2_0 = L2[3 * i];
    rtb_RateLimiter2 = rtb_RateLimiter5 * L2_0;
    S_p_tmp = rtb_Saturation7 * L2_0;
    vcol = 3 * i + 1;
    L2_0 = L2[vcol];
    rtb_RateLimiter2 += S_p_tmp_0 * L2_0;
    S_p_tmp += S_p_tmp_1 * L2_0;
    M_tmp_tmp = 3 * i + 2;
    L2_0 = L2[M_tmp_tmp];
    A_tmp_1[i] = (L1_p[3 * i] * rtb_RateLimiter3 + L1_p[vcol] * rtb_RateLimiter4)
      + L1_p[M_tmp_tmp] * S_p_tmp_2;
    A_tmp_0[i] = F_motor_UR_3 * L2_0 + S_p_tmp;
    M_tmp_2[i] = (L3_p[3 * i] * rtb_RateLimiter3 + L3_p[vcol] * rtb_RateLimiter4)
      + L3_p[M_tmp_tmp] * S_p_tmp_2;
    A_tmp[i] = F_motor_UR_4 * L2_0 + rtb_RateLimiter2;
  }

  A[0] = 0.0;
  A[8] = 0.0;
  A[16] = 0.0;
  A[24] = rtDW.DiscreteTimeIntegrator_DSTATE[6];
  A[32] = rtDW.DiscreteTimeIntegrator_DSTATE[7];
  A[40] = rtDW.DiscreteTimeIntegrator_DSTATE[8];
  A[48] = rtDW.DiscreteTimeIntegrator_DSTATE[9];
  A[56] = 0.0;
  A[64] = 0.0;
  A[72] = 0.0;
  A[80] = 0.0;
  A[88] = 0.0;
  A[96] = 0.0;
  A[104] = 0.0;
  A[1] = 0.0;
  A[9] = 0.0;
  A[17] = 0.0;
  A[25] = 0.0;
  A[33] = 0.0;
  A[41] = 0.0;
  A[49] = 0.0;
  A[57] = 0.0;
  A[65] = 0.0;
  A[73] = 0.0;
  A[81] = rtDW.DiscreteTimeIntegrator_DSTATE[20];
  A[89] = rtDW.DiscreteTimeIntegrator_DSTATE[21];
  A[97] = rtDW.DiscreteTimeIntegrator_DSTATE[22];
  A[105] = rtDW.DiscreteTimeIntegrator_DSTATE[23];
  A[2] = 0.0;
  A[10] = 0.0;
  A[18] = 0.0;
  A[26] = A_tmp[0];
  A[34] = A_tmp[1];
  A[42] = A_tmp[2];
  A[50] = A_tmp[3];
  A[58] = 0.0;
  A[66] = 0.0;
  A[74] = 0.0;
  A[82] = M_tmp_2[0];
  A[90] = M_tmp_2[1];
  A[98] = M_tmp_2[2];
  A[106] = M_tmp_2[3];
  A[3] = 0.0;
  A[11] = 0.0;
  A[19] = 0.0;
  A[27] = A_tmp_0[0];
  A[35] = A_tmp_0[1];
  A[43] = A_tmp_0[2];
  A[51] = A_tmp_0[3];
  A[59] = 0.0;
  A[67] = 0.0;
  A[75] = 0.0;
  A[83] = A_tmp_1[0];
  A[91] = A_tmp_1[1];
  A[99] = A_tmp_1[2];
  A[107] = A_tmp_1[3];
  for (i = 0; i < 3; i++) {
    vcol = i << 3;
    A[vcol + 4] = f[3 * i];
    A[vcol + 5] = f[3 * i + 1];
    A[vcol + 6] = f[3 * i + 2];
  }

  for (i = 0; i < 4; i++) {
    vcol = (i + 3) << 3;
    A[vcol + 4] = LAB[3 * i];
    A[vcol + 5] = LAB[3 * i + 1];
    A[vcol + 6] = LAB[3 * i + 2];
  }

  for (i = 0; i < 3; i++) {
    vcol = (i + 7) << 3;
    A[vcol + 4] = g[3 * i];
    A[vcol + 5] = g[3 * i + 1];
    A[vcol + 6] = g[3 * i + 2];
  }

  for (i = 0; i < 4; i++) {
    vcol = (i + 10) << 3;
    A[vcol + 4] = -LAP[3 * i];
    A[vcol + 5] = -LAP[3 * i + 1];
    A[vcol + 6] = -LAP[3 * i + 2];
  }

  for (i = 0; i < 14; i++) {
    A[(i << 3) + 7] = M[14 * i + 12];
  }

  for (i = 0; i < 8; i++) {
    for (vcol = 0; vcol < 14; vcol++) {
      b_A[vcol + 14 * i] = A[(vcol << 3) + i];
    }
  }

  p = true;
  for (i = 0; i < 112; i++) {
    X_1[i] = 0.0;
    if (p) {
      rtb_RateLimiter5 = b_A[i];
      if (rtIsInf(rtb_RateLimiter5) || rtIsNaN(rtb_RateLimiter5)) {
        p = false;
      }
    }
  }

  if (!p) {
    for (i = 0; i < 112; i++) {
      X_1[i] = (rtNaN);
    }
  } else {
    svd(b_A, U, rtb_u, V);
    rtb_RateLimiter2 = 14.0 * eps(rtb_u[0]);
    i = -1;
    vcol = 0;
    while ((vcol < 8) && (rtb_u[vcol] > rtb_RateLimiter2)) {
      i++;
      vcol++;
    }

    if (i + 1 > 0) {
      vcol = 1;
      for (M_tmp_tmp = 0; M_tmp_tmp <= i; M_tmp_tmp++) {
        rtb_RateLimiter2 = 1.0 / rtb_u[M_tmp_tmp];
        scalarLB_tmp = 8 + vcol;
        vectorUB = vcol + 6;
        for (ar = vcol; ar <= vectorUB; ar += 2) {
          tmp_9 = _mm_loadu_pd(&V[ar - 1]);
          _mm_storeu_pd(&V[ar - 1], _mm_mul_pd(tmp_9, _mm_set1_pd
            (rtb_RateLimiter2)));
        }

        for (ar = scalarLB_tmp; ar <= vcol + 7; ar++) {
          V[ar - 1] *= rtb_RateLimiter2;
        }

        vcol += 8;
      }

      vcol = 0;
      for (M_tmp_tmp = 0; M_tmp_tmp <= 104; M_tmp_tmp += 8) {
        memset(&X_1[M_tmp_tmp], 0, sizeof(real_T) << 3U);
      }

      for (M_tmp_tmp = 0; M_tmp_tmp <= 104; M_tmp_tmp += 8) {
        ar = -1;
        vcol++;
        scalarLB_tmp = 14 * i + vcol;
        for (ib = vcol; ib <= scalarLB_tmp; ib += 14) {
          scalarLB = M_tmp_tmp + 9;
          vectorUB = M_tmp_tmp + 7;
          for (b_ic = M_tmp_tmp + 1; b_ic <= vectorUB; b_ic += 2) {
            tmp_9 = _mm_loadu_pd(&V[(ar + b_ic) - M_tmp_tmp]);
            tmp_a = _mm_loadu_pd(&X_1[b_ic - 1]);
            _mm_storeu_pd(&X_1[b_ic - 1], _mm_add_pd(_mm_mul_pd(tmp_9,
              _mm_set1_pd(U[ib - 1])), tmp_a));
          }

          for (b_ic = scalarLB; b_ic <= M_tmp_tmp + 8; b_ic++) {
            X_1[b_ic - 1] += V[(ar + b_ic) - M_tmp_tmp] * U[ib - 1];
          }

          ar += 8;
        }
      }
    }
  }

  memset(&b_I[0], 0, 196U * sizeof(int8_T));
  for (i = 0; i < 14; i++) {
    b_I[i + 14 * i] = 1;
  }

  S_p_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[10];
  S_p_tmp_1 = rtDW.DiscreteTimeIntegrator_DSTATE[11];
  rtb_RateLimiter2 = rtDW.DiscreteTimeIntegrator_DSTATE[12];
  S_p_tmp = rtDW.DiscreteTimeIntegrator_DSTATE[13];
  for (i = 0; i <= 0; i += 2) {
    /* MATLAB Function: '<Root>/Drone' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    tmp_9 = _mm_loadu_pd(&L2[i]);
    tmp_a = _mm_loadu_pd(&L2[i + 3]);
    tmp_b = _mm_loadu_pd(&L2[i + 6]);
    tmp_8 = _mm_loadu_pd(&L2[i + 9]);
    _mm_storeu_pd(&F_motor_UR[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_set1_pd(2.0 * S_p_tmp_0), tmp_9), _mm_mul_pd(_mm_set1_pd(2.0 *
      S_p_tmp_1), tmp_a)), _mm_mul_pd(_mm_set1_pd(2.0 * rtb_RateLimiter2), tmp_b)),
      _mm_mul_pd(_mm_set1_pd(2.0 * S_p_tmp), tmp_8)));
    tmp_9 = _mm_loadu_pd(&S[i + 3]);
    _mm_storeu_pd(&F_motor_BR[i], _mm_mul_pd(tmp_9, _mm_set1_pd(8.0)));
  }

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  for (i = 2; i < 3; i++) {
    F_motor_UR[i] = ((2.0 * S_p_tmp_0 * L2[i] + 2.0 * S_p_tmp_1 * L2[i + 3]) +
                     2.0 * rtb_RateLimiter2 * L2[i + 6]) + 2.0 * S_p_tmp * L2[i
      + 9];
    F_motor_BR[i] = S[i + 3] * 8.0;
  }

  for (i = 0; i < 14; i++) {
    for (vcol = 0; vcol < 14; vcol++) {
      S_p_tmp_0 = 0.0;
      for (M_tmp_tmp = 0; M_tmp_tmp < 8; M_tmp_tmp++) {
        S_p_tmp_0 += X_1[(i << 3) + M_tmp_tmp] * A[(vcol << 3) + M_tmp_tmp];
      }

      M_tmp_tmp = 14 * vcol + i;
      b_I_0[M_tmp_tmp] = (real_T)b_I[M_tmp_tmp] - S_p_tmp_0;
    }

    for (vcol = 0; vcol < 14; vcol++) {
      rtb_RateLimiter5 = 0.0;
      for (M_tmp_tmp = 0; M_tmp_tmp < 14; M_tmp_tmp++) {
        rtb_RateLimiter5 += b_I_0[14 * M_tmp_tmp + i] * M[14 * vcol + M_tmp_tmp];
      }

      b_I_1[i + 14 * vcol] = rtb_RateLimiter5;
    }
  }

  for (i = 0; i < 14; i++) {
    memcpy(&b_I_2[i * 22], &b_I_1[i * 14], 14U * sizeof(real_T));
    memcpy(&b_I_2[i * 22 + 14], &A[i << 3], sizeof(real_T) << 3U);
  }

  pinv(b_I_2, tmp_3);
  S_p_idx_0_0 = -S_p[6];
  S_p_idx_1_0 = -S_p[7];
  S_p_idx_2_0 = -S_p[8];
  rtb_RateLimiter5 = 0.0;
  rtb_RateLimiter4 = 0.0;
  F_motor_UR_3 = 0.0;
  rtb_RateLimiter2 = 0.0;
  S_p_tmp = 0.0;
  S_p_idx_0 = -S_p[0];
  S_p_idx_1 = -S_p[1];
  S_p_idx_2 = -S_p[2];
  rtb_Saturation7 = 0.0;
  rtb_RateLimiter3 = 0.0;
  F_motor_UR_4 = 0.0;
  L2_0 = 0.0;
  A_tmp_2 = 0.0;
  S_p_tmp_2 = S[4];
  S_0 = S[3];
  S_1 = S[5];
  F_motor_UR_5 = F_motor_UR[1];
  F_motor_UR_1 = F_motor_UR[0];
  F_motor_UR_2 = F_motor_UR[2];
  for (i = 0; i < 4; i++) {
    vcol = 3 * i + 1;
    L2_dot_0 = L2_dot[vcol];
    L2_dot_1 = L2_dot[3 * i];
    M_tmp_tmp = 3 * i + 2;
    L2_dot_2 = L2_dot[M_tmp_tmp];
    S_p_tmp_0 = rtDW.DiscreteTimeIntegrator_DSTATE[i + 10];
    rtb_RateLimiter5 += ((L2_dot_0 * S_p_idx_1_0 + L2_dot_1 * S_p_idx_0_0) +
                         L2_dot_2 * S_p_idx_2_0) * S_p_tmp_0;
    S_p_tmp_1 = rtDW.DiscreteTimeIntegrator_DSTATE[i + 24];
    rtb_RateLimiter4 += ((L3_p_dot[3 * i] * S_0 + L3_p_dot[vcol] * S_p_tmp_2) +
                         L3_p_dot[M_tmp_tmp] * S_1) * S_p_tmp_1;
    F_motor_UR_3 += ((L3_p[3 * i] * F_motor_UR_1 + L3_p[vcol] * F_motor_UR_5) +
                     L3_p[M_tmp_tmp] * F_motor_UR_2) * S_p_tmp_1;
    rtb_RateLimiter2 += S_p_tmp_0 * A_tmp[i];
    S_p_tmp += M_tmp_2[i] * S_p_tmp_1;
    rtb_Saturation7 += ((L2_dot_0 * S_p_idx_1 + L2_dot_1 * S_p_idx_0) + L2_dot_2
                        * S_p_idx_2) * S_p_tmp_0;
    rtb_RateLimiter3 += ((L1_p_dot[3 * i] * S_0 + L1_p_dot[vcol] * S_p_tmp_2) +
                         L1_p_dot[M_tmp_tmp] * S_1) * S_p_tmp_1;
    F_motor_UR_4 += ((L1_p[3 * i] * F_motor_UR_1 + L1_p[vcol] * F_motor_UR_5) +
                     L1_p[M_tmp_tmp] * F_motor_UR_2) * S_p_tmp_1;
    L2_0 += S_p_tmp_0 * A_tmp_0[i];
    A_tmp_2 += A_tmp_1[i] * S_p_tmp_1;
  }

  M_tmp_1[0] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27] * 0.0;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26] * 0.0;
  M_tmp_1[3] = S_p_tmp_0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[25] * 0.0;
  M_tmp_1[6] = S_p_tmp_1;
  M_tmp_1[9] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24] * 0.0;
  M_tmp_1[1] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24] * 0.0;
  M_tmp_1[4] = 0.0;
  M_tmp_1[7] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[26] * 0.0;
  M_tmp_1[10] = 0.0;
  M_tmp_1[2] = S_p_tmp_1;
  M_tmp_1[5] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[24] * 0.0;
  M_tmp_1[8] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[27] * 0.0;
  M_tmp_1[11] = S_p_tmp_0;
  M_tmp[0] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * 0.0;
  M_tmp[3] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11] * 0.0;
  M_tmp[6] = 0.0;
  M_tmp[9] = 0.0;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13] * 0.0;
  M_tmp[1] = S_p_tmp_0;
  M_tmp[4] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12] * 0.0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11] * 0.0;
  M_tmp[7] = S_p_tmp_1;
  M_tmp[10] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * 0.0;
  M_tmp[2] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12] * 0.0;
  M_tmp[5] = S_p_tmp_0;
  M_tmp[8] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * 0.0;
  M_tmp[11] = S_p_tmp_1;
  S_p_tmp_0 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[12] * -0.026;
  L2[0] = S_p_tmp_0;
  S_p_tmp_1 = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13] * -0.026;
  L2[3] = S_p_tmp_1;
  L2[6] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * -0.026;
  L2[9] = 2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11] * -0.026;
  L2[1] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[11] * -0.026;
  L2[4] = -2.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * -0.026;
  L2[7] = S_p_tmp_1;
  L2[10] = S_p_tmp_0;
  L2[2] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[10] * -0.026;
  L2[5] = -0.0;
  L2[8] = -0.0;
  L2[11] = 4.0 * rtDW.DiscreteTimeIntegrator_DSTATE[13] * -0.026;
  for (i = 0; i <= 10; i += 2) {
    /* MATLAB Function: '<Root>/Drone' */
    tmp_9 = _mm_loadu_pd(&L1_p_dot[i]);
    tmp_a = _mm_loadu_pd(&M_tmp_1[i]);
    tmp_b = _mm_loadu_pd(&L3_p_dot[i]);
    _mm_storeu_pd(&tmp[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.0782),
      tmp_9), tmp_a), _mm_mul_pd(_mm_set1_pd(-0.1556), tmp_b)));
    tmp_9 = _mm_loadu_pd(&L2_dot[i]);
    tmp_a = _mm_loadu_pd(&M_tmp[i]);
    tmp_b = _mm_loadu_pd(&L2[i]);
    _mm_storeu_pd(&L1_p[i], _mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.0),
      tmp_9), tmp_a), tmp_b));
  }

  for (i = 0; i <= 0; i += 2) {
    /* MATLAB Function: '<Root>/Drone' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    tmp_9 = _mm_loadu_pd(&LAP[i + 3]);
    tmp_a = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[25]);
    tmp_b = _mm_loadu_pd(&LAP[i]);
    tmp_8 = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[24]);
    tmp_4 = _mm_loadu_pd(&LAP[i + 6]);
    tmp_c = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[26]);
    tmp_5 = _mm_loadu_pd(&LAP[i + 9]);
    tmp_d = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[27]);
    _mm_storeu_pd(&F_motor_UL[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (tmp_9, tmp_a), _mm_mul_pd(tmp_b, tmp_8)), _mm_mul_pd(tmp_4, tmp_c)),
      _mm_mul_pd(tmp_5, tmp_d)));
    tmp_9 = _mm_loadu_pd(&L1_p[i + 3]);
    tmp_b = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[11]);
    tmp_4 = _mm_loadu_pd(&L1_p[i]);
    tmp_5 = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[10]);
    tmp_6 = _mm_loadu_pd(&L1_p[i + 6]);
    tmp_e = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[12]);
    tmp_7 = _mm_loadu_pd(&L1_p[i + 9]);
    tmp_f = _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[13]);
    _mm_storeu_pd(&F_motor_UR[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (tmp_9, tmp_b), _mm_mul_pd(tmp_4, tmp_5)), _mm_mul_pd(tmp_6, tmp_e)),
      _mm_mul_pd(tmp_7, tmp_f)));
    tmp_9 = _mm_loadu_pd(&tmp[i + 3]);
    tmp_4 = _mm_loadu_pd(&tmp[i]);
    tmp_6 = _mm_loadu_pd(&tmp[i + 6]);
    tmp_7 = _mm_loadu_pd(&tmp[i + 9]);
    _mm_storeu_pd(&H_p_tmp_0[i], _mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd
      (tmp_9, tmp_a), _mm_mul_pd(tmp_4, tmp_8)), _mm_mul_pd(tmp_6, tmp_c)),
      _mm_mul_pd(tmp_7, tmp_d)));
    tmp_9 = _mm_loadu_pd(&LAB[i + 3]);
    tmp_a = _mm_loadu_pd(&LAB[i]);
    tmp_8 = _mm_loadu_pd(&LAB[i + 6]);
    tmp_4 = _mm_loadu_pd(&LAB[i + 9]);
    tmp_c = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i + 3]);
    tmp_d = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i + 17]);
    _mm_storeu_pd(&F_motor_BL[i], _mm_sub_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_mul_pd(tmp_9, tmp_b), _mm_mul_pd(tmp_a, tmp_5)),
       _mm_mul_pd(tmp_8, tmp_e)), _mm_mul_pd(tmp_4, tmp_f)), tmp_c), tmp_d));
  }

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  for (i = 2; i < 3; i++) {
    F_motor_UL[i] = ((LAP[i + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[25] +
                      LAP[i] * rtDW.DiscreteTimeIntegrator_DSTATE[24]) + LAP[i +
                     6] * rtDW.DiscreteTimeIntegrator_DSTATE[26]) + LAP[i + 9] *
      rtDW.DiscreteTimeIntegrator_DSTATE[27];
    F_motor_UR[i] = ((L1_p[i + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[11] +
                      L1_p[i] * rtDW.DiscreteTimeIntegrator_DSTATE[10]) + L1_p[i
                     + 6] * rtDW.DiscreteTimeIntegrator_DSTATE[12]) + L1_p[i + 9]
      * rtDW.DiscreteTimeIntegrator_DSTATE[13];
    H_p_tmp_0[i] = ((tmp[i + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[25] + tmp[i]
                     * rtDW.DiscreteTimeIntegrator_DSTATE[24]) + tmp[i + 6] *
                    rtDW.DiscreteTimeIntegrator_DSTATE[26]) + tmp[i + 9] *
      rtDW.DiscreteTimeIntegrator_DSTATE[27];
    F_motor_BL[i] = ((((LAB[i + 3] * rtDW.DiscreteTimeIntegrator_DSTATE[11] +
                        LAB[i] * rtDW.DiscreteTimeIntegrator_DSTATE[10]) + LAB[i
                       + 6] * rtDW.DiscreteTimeIntegrator_DSTATE[12]) + LAB[i +
                      9] * rtDW.DiscreteTimeIntegrator_DSTATE[13]) +
                     rtDW.DiscreteTimeIntegrator_DSTATE[i + 3]) -
      rtDW.DiscreteTimeIntegrator_DSTATE[i + 17];
  }

  memcpy(&Q_0[0], &Q[0], 14U * sizeof(real_T));
  Q_0[14] = ((((-0.5 * rtDW.DiscreteTimeIntegrator_DSTATE[6] *
                rtDW.DiscreteTimeIntegrator_DSTATE[10] + -0.5 *
                rtDW.DiscreteTimeIntegrator_DSTATE[7] *
                rtDW.DiscreteTimeIntegrator_DSTATE[11]) + -0.5 *
               rtDW.DiscreteTimeIntegrator_DSTATE[8] *
               rtDW.DiscreteTimeIntegrator_DSTATE[12]) + -0.5 *
              rtDW.DiscreteTimeIntegrator_DSTATE[9] *
              rtDW.DiscreteTimeIntegrator_DSTATE[13]) - ((((x_idx_0 + S_tmp_5) +
    S_tmp) + S_tmp_0) - 1.0) * 4.0) - (((rtDW.DiscreteTimeIntegrator_DSTATE[10] *
    rtDW.DiscreteTimeIntegrator_DSTATE[10] + rtDW.DiscreteTimeIntegrator_DSTATE
    [11] * rtDW.DiscreteTimeIntegrator_DSTATE[11]) +
    rtDW.DiscreteTimeIntegrator_DSTATE[12] * rtDW.DiscreteTimeIntegrator_DSTATE
    [12]) + rtDW.DiscreteTimeIntegrator_DSTATE[13] *
    rtDW.DiscreteTimeIntegrator_DSTATE[13]);
  Q_0[15] = ((((-0.5 * rtDW.DiscreteTimeIntegrator_DSTATE[20] *
                rtDW.DiscreteTimeIntegrator_DSTATE[24] + -0.5 *
                rtDW.DiscreteTimeIntegrator_DSTATE[21] *
                rtDW.DiscreteTimeIntegrator_DSTATE[25]) + -0.5 *
               rtDW.DiscreteTimeIntegrator_DSTATE[22] *
               rtDW.DiscreteTimeIntegrator_DSTATE[26]) + -0.5 *
              rtDW.DiscreteTimeIntegrator_DSTATE[23] *
              rtDW.DiscreteTimeIntegrator_DSTATE[27]) - ((((S_tmp_1 + S_tmp_2) +
    S_tmp_3) + S_tmp_4) - 1.0) * 4.0) - (((rtDW.DiscreteTimeIntegrator_DSTATE[24]
    * rtDW.DiscreteTimeIntegrator_DSTATE[24] +
    rtDW.DiscreteTimeIntegrator_DSTATE[25] * rtDW.DiscreteTimeIntegrator_DSTATE
    [25]) + rtDW.DiscreteTimeIntegrator_DSTATE[26] *
    rtDW.DiscreteTimeIntegrator_DSTATE[26]) +
    rtDW.DiscreteTimeIntegrator_DSTATE[27] * rtDW.DiscreteTimeIntegrator_DSTATE
    [27]);
  Q_0[16] = (((rtb_RateLimiter5 - rtb_RateLimiter4) - F_motor_UR_3) -
             (rtb_RateLimiter2 + S_p_tmp) * 0.5) - ((F_motor_BR[0] * S_p[6] +
    F_motor_BR[1] * S_p[7]) + F_motor_BR[2] * S_p[8]);
  Q_0[17] = (((rtb_Saturation7 - rtb_RateLimiter3) - F_motor_UR_4) - (L2_0 +
              A_tmp_2) * 0.5) - ((F_motor_BR[0] * S_p[0] + F_motor_BR[1] * S_p[1])
    + F_motor_BR[2] * S_p[2]);
  Q_0[18] = ((H_p_tmp_0[0] - F_motor_UR[0]) - (F_motor_BL[0] - F_motor_UL[0]) *
             0.5) - (rtDW.DiscreteTimeIntegrator_DSTATE[0] -
                     (rtDW.DiscreteTimeIntegrator_DSTATE[14] + 0.0782)) * 8.0;
  Q_0[19] = ((H_p_tmp_0[1] - F_motor_UR[1]) - (F_motor_BL[1] - F_motor_UL[1]) *
             0.5) - (rtDW.DiscreteTimeIntegrator_DSTATE[1] -
                     rtDW.DiscreteTimeIntegrator_DSTATE[15]) * 8.0;
  Q_0[20] = ((H_p_tmp_0[2] - F_motor_UR[2]) - (F_motor_BL[2] - F_motor_UL[2]) *
             0.5) - ((rtDW.DiscreteTimeIntegrator_DSTATE[2] - 0.026) -
                     (rtDW.DiscreteTimeIntegrator_DSTATE[16] - 0.1556)) * 8.0;
  Q_0[21] = Q[12];
  for (i = 0; i < 14; i++) {
    x_idx_0 = 0.0;
    for (vcol = 0; vcol < 22; vcol++) {
      x_idx_0 += tmp_3[14 * vcol + i] * Q_0[vcol];
    }

    Q[i] = x_idx_0;
  }

  rtb_dxdt[0] = rtDW.DiscreteTimeIntegrator_DSTATE[3];
  rtb_dxdt[3] = Q[0];
  rtb_dxdt[1] = rtDW.DiscreteTimeIntegrator_DSTATE[4];
  rtb_dxdt[4] = Q[1];
  rtb_dxdt[2] = rtDW.DiscreteTimeIntegrator_DSTATE[5];
  rtb_dxdt[5] = Q[2];
  rtb_dxdt[6] = rtDW.DiscreteTimeIntegrator_DSTATE[10];
  rtb_dxdt[10] = Q[3];
  rtb_dxdt[7] = rtDW.DiscreteTimeIntegrator_DSTATE[11];
  rtb_dxdt[11] = Q[4];
  rtb_dxdt[8] = rtDW.DiscreteTimeIntegrator_DSTATE[12];
  rtb_dxdt[12] = Q[5];
  rtb_dxdt[9] = rtDW.DiscreteTimeIntegrator_DSTATE[13];
  rtb_dxdt[13] = Q[6];
  rtb_dxdt[14] = rtDW.DiscreteTimeIntegrator_DSTATE[17];
  rtb_dxdt[17] = Q[7];
  rtb_dxdt[15] = rtDW.DiscreteTimeIntegrator_DSTATE[18];
  rtb_dxdt[18] = Q[8];
  rtb_dxdt[16] = rtDW.DiscreteTimeIntegrator_DSTATE[19];
  rtb_dxdt[19] = Q[9];

  /* MATLAB Function: '<Root>/state' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  q2e(&rtDW.DiscreteTimeIntegrator_DSTATE[6], &rtb_RateLimiter3,
      &rtb_RateLimiter2, &rtb_RateLimiter4);
  q2e(&rtDW.DiscreteTimeIntegrator_DSTATE[20], &rtb_RateLimiter3,
      &rtb_RateLimiter4, &rtb_RateLimiter5);

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtb_dxdt[20] = rtDW.DiscreteTimeIntegrator_DSTATE[24];
  rtb_dxdt[24] = Q[10];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.q[0] = rtDW.DiscreteTimeIntegrator_DSTATE[6];

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtb_dxdt[21] = rtDW.DiscreteTimeIntegrator_DSTATE[25];
  rtb_dxdt[25] = Q[11];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.q[1] = rtDW.DiscreteTimeIntegrator_DSTATE[7];

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtb_dxdt[22] = rtDW.DiscreteTimeIntegrator_DSTATE[26];
  rtb_dxdt[26] = Q[12];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.q[2] = rtDW.DiscreteTimeIntegrator_DSTATE[8];

  /* MATLAB Function: '<Root>/Drone' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtb_dxdt[23] = rtDW.DiscreteTimeIntegrator_DSTATE[27];
  rtb_dxdt[27] = Q[13];

  /* Outport: '<Root>/q' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  rtY.q[3] = rtDW.DiscreteTimeIntegrator_DSTATE[9];

  /* Outport: '<Root>/pendulum_angle' incorporates:
   *  MATLAB Function: '<Root>/state'
   */
  rtY.pendulum_angle = -(rtb_RateLimiter2 - rtb_RateLimiter4);
  for (i = 0; i <= 0; i += 2) {
    /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
     *  Outport: '<Root>/p'
     */
    tmp_9 = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i]);

    /* Outport: '<Root>/p' */
    _mm_storeu_pd(&rtY.p[i], tmp_9);

    /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
     *  Outport: '<Root>/p'
     */
    tmp_9 = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i + 3]);

    /* Outport: '<Root>/v' incorporates:
     *  Outport: '<Root>/p'
     */
    _mm_storeu_pd(&rtY.v[i], tmp_9);

    /* MATLAB Function: '<Root>/state' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  Outport: '<Root>/p'
     */
    tmp_9 = _mm_loadu_pd(&rtb_dxdt[i + 3]);

    /* Outport: '<Root>/accel' incorporates:
     *  Outport: '<Root>/p'
     */
    _mm_storeu_pd(&rtY.accel[i], tmp_9);

    /* MATLAB Function: '<Root>/state' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  Outport: '<Root>/p'
     */
    tmp_9 = _mm_loadu_pd(&H_tmp[i]);
    tmp_a = _mm_loadu_pd(&H_p_tmp[i]);
    tmp_b = _mm_loadu_pd(&H_tmp[i + 3]);
    tmp_8 = _mm_loadu_pd(&H_p_tmp[i + 3]);
    tmp_4 = _mm_loadu_pd(&H_tmp[i + 6]);
    tmp_c = _mm_loadu_pd(&H_p_tmp[i + 6]);
    tmp_5 = _mm_loadu_pd(&H_p_tmp[i + 9]);
    _mm_storeu_pd(&H_p_tmp_0[i], _mm_add_pd(_mm_mul_pd(tmp_5, _mm_set1_pd
      (rtb_dxdt[27])), _mm_add_pd(_mm_mul_pd(tmp_c, _mm_set1_pd(rtb_dxdt[26])),
      _mm_add_pd(_mm_mul_pd(tmp_8, _mm_set1_pd(rtb_dxdt[25])), _mm_mul_pd(tmp_a,
      _mm_set1_pd(rtb_dxdt[24]))))));
    _mm_storeu_pd(&F_motor_BR[i], _mm_add_pd(_mm_mul_pd(tmp_5, _mm_set1_pd
      (rtDW.DiscreteTimeIntegrator_DSTATE[27])), _mm_add_pd(_mm_mul_pd(tmp_c,
      _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[26])), _mm_add_pd
      (_mm_mul_pd(tmp_8, _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[25])),
       _mm_mul_pd(tmp_a, _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[24]))))));
    tmp_a = _mm_loadu_pd(&H_tmp[i + 9]);

    /* Outport: '<Root>/rotaccel' incorporates:
     *  MATLAB Function: '<Root>/state'
     *  Outport: '<Root>/p'
     */
    _mm_storeu_pd(&rtY.rotaccel[i], _mm_add_pd(_mm_mul_pd(tmp_a, _mm_set1_pd
      (rtb_dxdt[13])), _mm_add_pd(_mm_mul_pd(tmp_4, _mm_set1_pd(rtb_dxdt[12])),
      _mm_add_pd(_mm_mul_pd(tmp_b, _mm_set1_pd(rtb_dxdt[11])), _mm_mul_pd(tmp_9,
      _mm_set1_pd(rtb_dxdt[10]))))));

    /* Outport: '<Root>/omega' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  MATLAB Function: '<Root>/state'
     *  Outport: '<Root>/p'
     */
    _mm_storeu_pd(&rtY.omega[i], _mm_add_pd(_mm_mul_pd(tmp_a, _mm_set1_pd
      (rtDW.DiscreteTimeIntegrator_DSTATE[13])), _mm_add_pd(_mm_mul_pd(tmp_4,
      _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[12])), _mm_add_pd
      (_mm_mul_pd(tmp_b, _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[11])),
       _mm_mul_pd(tmp_9, _mm_set1_pd(rtDW.DiscreteTimeIntegrator_DSTATE[10]))))));
  }

  for (i = 2; i < 3; i++) {
    /* Outport: '<Root>/p' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    rtY.p[i] = rtDW.DiscreteTimeIntegrator_DSTATE[i];

    /* Outport: '<Root>/v' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    rtY.v[i] = rtDW.DiscreteTimeIntegrator_DSTATE[i + 3];

    /* Outport: '<Root>/accel' incorporates:
     *  MATLAB Function: '<Root>/state'
     */
    rtY.accel[i] = rtb_dxdt[i + 3];

    /* MATLAB Function: '<Root>/state' */
    x_idx_0 = H_tmp[i];

    /* Outport: '<Root>/omega' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  MATLAB Function: '<Root>/state'
     */
    S_tmp_5 = x_idx_0 * rtDW.DiscreteTimeIntegrator_DSTATE[10];

    /* Outport: '<Root>/rotaccel' incorporates:
     *  MATLAB Function: '<Root>/state'
     */
    S_tmp = x_idx_0 * rtb_dxdt[10];

    /* MATLAB Function: '<Root>/state' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    S_tmp_0 = H_p_tmp[i];
    S_tmp_1 = S_tmp_0 * rtDW.DiscreteTimeIntegrator_DSTATE[24];
    S_tmp_2 = S_tmp_0 * rtb_dxdt[24];
    x_idx_0 = H_tmp[i + 3];

    /* Outport: '<Root>/omega' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  MATLAB Function: '<Root>/state'
     */
    S_tmp_5 += x_idx_0 * rtDW.DiscreteTimeIntegrator_DSTATE[11];

    /* Outport: '<Root>/rotaccel' incorporates:
     *  MATLAB Function: '<Root>/state'
     */
    S_tmp += x_idx_0 * rtb_dxdt[11];

    /* MATLAB Function: '<Root>/state' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    S_tmp_0 = H_p_tmp[i + 3];
    S_tmp_1 += S_tmp_0 * rtDW.DiscreteTimeIntegrator_DSTATE[25];
    S_tmp_2 += S_tmp_0 * rtb_dxdt[25];
    x_idx_0 = H_tmp[i + 6];

    /* Outport: '<Root>/omega' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  MATLAB Function: '<Root>/state'
     */
    S_tmp_5 += x_idx_0 * rtDW.DiscreteTimeIntegrator_DSTATE[12];

    /* Outport: '<Root>/rotaccel' incorporates:
     *  MATLAB Function: '<Root>/state'
     */
    S_tmp += x_idx_0 * rtb_dxdt[12];

    /* MATLAB Function: '<Root>/state' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     */
    S_tmp_0 = H_p_tmp[i + 6];
    S_tmp_1 += S_tmp_0 * rtDW.DiscreteTimeIntegrator_DSTATE[26];
    S_tmp_2 += S_tmp_0 * rtb_dxdt[26];
    x_idx_0 = H_tmp[i + 9];
    S_tmp_0 = H_p_tmp[i + 9];
    H_p_tmp_0[i] = S_tmp_0 * rtb_dxdt[27] + S_tmp_2;
    F_motor_BR[i] = S_tmp_0 * rtDW.DiscreteTimeIntegrator_DSTATE[27] + S_tmp_1;

    /* Outport: '<Root>/rotaccel' incorporates:
     *  MATLAB Function: '<Root>/state'
     */
    rtY.rotaccel[i] = x_idx_0 * rtb_dxdt[13] + S_tmp;

    /* Outport: '<Root>/omega' incorporates:
     *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
     *  MATLAB Function: '<Root>/state'
     */
    rtY.omega[i] = x_idx_0 * rtDW.DiscreteTimeIntegrator_DSTATE[13] + S_tmp_5;
  }

  /* Outport: '<Root>/pendulum_ang_speed' incorporates:
   *  MATLAB Function: '<Root>/state'
   */
  rtY.pendulum_ang_speed = F_motor_BR[1];

  /* Outport: '<Root>/pendulum_ang_acc' incorporates:
   *  MATLAB Function: '<Root>/state'
   */
  rtY.pendulum_ang_acc = H_p_tmp_0[1];
  for (i = 0; i <= 26; i += 2) {
    /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
    tmp_9 = _mm_loadu_pd(&rtb_dxdt[i]);
    tmp_a = _mm_loadu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i]);
    _mm_storeu_pd(&rtDW.DiscreteTimeIntegrator_DSTATE[i], _mm_add_pd(_mm_mul_pd
      (_mm_set1_pd(0.000977), tmp_9), tmp_a));
  }
}

/* Model initialize function */
void colibri_v1_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  memcpy(&rtDW.DiscreteTimeIntegrator_DSTATE[0],
         &rtConstP.DiscreteTimeIntegrator_IC[0], 28U * sizeof(real_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
