#include "matrixlib.h"
#include <math.h>
#include <string.h>

/*
 * Function: rt_MatMultRR_Dbl
 * Abstract:
 *      2-input matrix multiply function
 *      Input 1: Real, double-precision
 *      Input 2: Real, double-precision
 */
void MatMultRR_Dbl(real_T       *y, 
                   const real_T *A,
                   const real_T *B, 
                   const int_T    dims[3])
{
  int_T k;
  for(k=dims[2]; k-- > 0; ) {
    const real_T *A1 = A;
    int_T i;
    for(i=dims[0]; i-- > 0; ) {
      const real_T *A2 = A1;
      const real_T *B1 = B;
      real_T acc = (real_T)0.0;
      int_T j;
      A1++;
      for(j=dims[1]; j-- > 0; ) {
        acc += *A2 * *B1;
        B1++;
        A2 += dims[0];
      }
      *y++ = acc;
    }
    B += dims[1];
  }
}

/* Function: rt_BackwardSubstitutionRR_Dbl =====================================
 * Abstract: Backward substitution: Solving Ux=b 
 *           U: real, double
 *           b: real, double
 *           U is an upper (or unit upper) triangular full matrix.
 *           The entries in the lower triangle are ignored.
 *           U is a NxN matrix
 *           X is a NxP matrix
 *           B is a NxP matrix
 */
void BackwardSubstitutionRR_Dbl(real_T          *pU,
                                   const real_T    *pb,
                                   real_T          *x,
                                   int_T            N,
                                   int_T            P,
                                   boolean_T        unit_upper)
{
  int_T i,k;

  for(k=P; k>0; k--) {
    real_T *pUcol = pU;
    for(i=0; i<N; i++) {
      real_T *xj = x + k*N-1;
      real_T s = 0.0;
      real_T *pUrow = pUcol--;          /* access current row of U */

      {
        int_T j = i;
        while(j-- > 0) {
          s += *pUrow * *xj--;
          pUrow -= N;
        }
      }

      if (unit_upper) {
        *xj = *pb-- - s;
      } else {
        *xj = (*pb-- - s) / *pUrow;
      }
    }
  }
}

/* Function: rt_ForwardSubstitutionRR_Dbl ======================================
 * Abstract: Forward substitution: solving Lx=b 
 *           L: Real, double
 *           b: Real, double
 *           L is a lower (or unit lower) triangular full matrix.
 *           The entries in the upper triangle are ignored.
 *           L is a NxN matrix
 *           X is a NxP matrix
 *           B is a NxP matrix
 */
void ForwardSubstitutionRR_Dbl(real_T        *pL,
                                  const real_T  *pb,
                                  real_T        *x,
                                  int_T          N,
                                  int_T          P,
                                  const int32_T *piv,
                                  boolean_T      unit_lower)
{  
  /* Real inputs: */
  int_T i, k;
  for(k=0; k<P; k++) {
    real_T *pLcol = pL;
    for(i=0; i<N; i++) {
      real_T *xj = x + k*N;
      real_T s = 0.0;
      real_T *pLrow = pLcol++;          /* access current row of L */

      {
        int_T j = i;
        while(j-- > 0) {
          s += *pLrow * *xj++;
          pLrow += N;
        }
      }

      if (unit_lower) {
        *xj = pb[piv[i]] - s;
      } else {
        *xj = (pb[piv[i]] - s) / *pLrow;
      }
    }
    pb += N;
  }
}

/* Function: rt_lu_real  =======================================================
 * Abstract: A is real.
 *
 */
void lu_real(real_T *A,             /* in and out                         */
                const int_T n,         /* number or rows = number of columns */
                int32_T *piv)          /* pivote vector                      */
{
  int_T k;

  /* initialize row-pivot indices: */
  for (k = 0; k < n; k++) {
    piv[k] = k;
  }

  /* Loop over each column: */
  for (k = 0; k < n; k++) {
    const int_T kn = k*n;
    int_T p = k;

    /* Scan the lower triangular part of this column only
     * Record row of largest value
     */
    {
      int_T i;
      real_T Amax = fabs(A[p+kn]);     /* assume diag is max */
      for (i = k+1; i < n; i++) {
        real_T q = fabs(A[i+kn]);
        if (q > Amax) {
          p = i;
          Amax = q;
        }
      }
    }

    /* swap rows if required */
    if (p != k) {
      int_T j;
      int32_T t1;
      for (j = 0; j < n; j++) {
        real_T t;
        const int_T j_n = j*n;
        t = A[p+j_n];
        A[p+j_n] = A[k+j_n];
        A[k+j_n] = t;
      }

      /* swap pivot row indices */
      t1 = piv[p];
      piv[p] = piv[k];
      piv[k] = t1;
    }

    /* column reduction */
    {
      real_T Adiag = A[k+kn];
      int_T i,j;
      if (Adiag != 0.0) {              /* non-zero diagonal entry */
        /* divide lower triangular part of column by max */
        Adiag = 1.0/Adiag;
        for (i = k+1; i < n; i++) {
          A[i+kn] *= Adiag;
        }

        /* subtract multiple of column from remaining columns */
        for (j = k+1; j < n; j++) {
          int_T j_n = j*n;
          for (i = k+1; i < n; i++) {
            A[i+j_n] -= A[i+kn]*A[k+j_n];
          }
        }
      }
    }
  }
}

/*
 * Function: rt_MatDivRR_Dbl
 * Abstract:
 *      2-real double input matrix division function
 */
void MatDivRR_Dbl(real_T *Out,
                     const real_T *In1,
                     const real_T *In2,
                     real_T *lu,
                     int32_T *piv,
                     real_T *x,
                     const int_T dims[3])
{
  int_T N = dims[0];
  int_T N2 = N * N;
  int_T P = dims[2];
  int_T NP = N * P;
  const boolean_T unit_upper = 0;
  const boolean_T unit_lower = 1;
  (void)memcpy(lu, In1, N2*sizeof(real_T));
  lu_real(lu, N, piv);
  ForwardSubstitutionRR_Dbl(lu, In2, x, N, P, piv, unit_lower);
  BackwardSubstitutionRR_Dbl(lu + N2 -1, x + NP -1, Out, N, P, unit_upper);
}
