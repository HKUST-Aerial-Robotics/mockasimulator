#include "gp/gp.hpp"

#include <stdio.h>
#include <string.h>

#define MAX_LINE_LENGTH 256
#define DEBUG 0
#define PRINT_GRDLAG 0
#define PRINT_HESVAL 0
#define OBJSCAL 1.0
#define DUMPHESSIAN 0

#if DEBUG
#include <assert.h>
#endif

void MSKAPI
printcb(void* handle, const char str[])
{
  printf("%s", str);
}

typedef struct
{
  /*
   * Data structure for storing
   * data about the nonlinear
   * function in the objective.
   */

  MSKtask_t task;

  MSKint32t n; /* Number of variables.  */
  MSKint32t t; /* Number of terms in
                  the objective of the primal problem. */
  MSKint32t* p;
  MSKint32t  numhesnz; /* Number of non-zeros in
                          the Hessian.          */
} nlhandt_dgopt;

typedef nlhandt_dgopt* nlhand_dgopt_t;

static int MSKAPI
printnldata(nlhand_dgopt_t nlh)
{
  MSKidxt i;

  printf("* Begin: dgo nl data debug. *\n");

  printf("n = %d, t = %d\n", nlh->n, nlh->t);

  for (i = 0; i < nlh->t + 1; ++i)
    printf("p[%d] = %d\n", i, nlh->p[i]);

  printf("* End: dgo nl data debug. *\n");

  return 0;
} /* printnldata */

static int MSKAPI
dgostruc(void* nlhandle, MSKint32t* numgrdobjnz, MSKint32t* grdobjsub,
         MSKint32t i, int* convali, MSKint32t* grdconinz, MSKint32t* grdconisub,
         MSKint32t yo, MSKint32t numycnz, const MSKint32t* ycsub,
         MSKint32t maxnumhesnz, MSKint32t* numhesnz, MSKint32t* hessubi,
         MSKint32t* hessubj)
/* Purpose: Provide information to MOSEK about the problem structure
            and sparsity.
 */
{
  MSKint32t      j, k, l;
  nlhand_dgopt_t nlh;

  nlh = (nlhand_dgopt_t)nlhandle;

  MSK_checkmemtask(nlh->task, __FILE__, __LINE__);

  if (numgrdobjnz)
  {
    /* All the variables appear nonlinearly
     * in the objective.
     */

    numgrdobjnz[0] = 0;

    for (k = 0; k < 1; ++k)
    {
      for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
      {
        if (grdobjsub)
          grdobjsub[numgrdobjnz[0]] = j;

        ++numgrdobjnz[0];
      }
    }

    for (k = 1; k < nlh->t; ++k)
    {
      if (nlh->p[k + 1] - nlh->p[k] > 1)
      {
        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
        {
          if (grdobjsub)
            grdobjsub[numgrdobjnz[0]] = j;

          ++numgrdobjnz[0];
        }
      }
    }
  }

  if (convali)
    convali[0] = 0; /* Zero because no nonlinear
                     * expression in the constraints.
                     */

  if (grdconinz)
    grdconinz[0] = 0; /* Zero because no nonlinear
                       * expression in the constraints.
                       */

  if (numhesnz)
  {
    if (yo)
      numhesnz[0] = nlh->numhesnz;
    else
      numhesnz[0] = 0;
  }

  if (maxnumhesnz)
  {
    /* Should return information about the Hessian too. */

    if (maxnumhesnz < numhesnz[0])
    {
      /* Not enough space have been allocated for
       * storing the Hessian.
       */

      return (1);
    }
    else
    {
      if (yo)
      {
        if (hessubi && hessubj)
        {
          /*
           * Compute and store the sparsity pattern of the
           * Hessian of the Lagrangian.
           */

          l = 0;
          for (j = nlh->p[0]; j < nlh->p[1]; ++j)
          {
            hessubi[l] = j;
            hessubj[l] = j;
            ++l;
          }

          for (k = 1; k < nlh->t; ++k)
          {
            for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
            {
              for (i = j; i < nlh->p[k + 1]; ++i)
              {
                if (nlh->p[k + 1] - nlh->p[k] > 1)
                {
                  hessubi[l] = i;
                  hessubj[l] = j;
                  ++l;
                }
              }
            }
          }
        }
      }
    }
  }

  return (0);
} /* dgostruc */

static int MSKAPI
dgoeval(void* nlhandle, const double* xx, double yo, const double* yc,
        double* objval, MSKint32t* numgrdobjnz, MSKint32t* grdobjsub,
        double* grdobjval, MSKint32t numi, const MSKidxt* subi, double* conval,
        const MSKintt* grdconptrb, const MSKintt* grdconptre,
        const MSKidxt* grdconsub, double* grdconval, double* grdlag,
        MSKint32t maxnumhesnz, MSKint32t* numhesnz, MSKint32t* hessubi,
        MSKint32t* hessubj, double* hesval)
/* Purpose: To evaluate the nonlinear function and return the
            requested information to MOSEK.
 */
{
  double         rtemp;
  MSKint32t      i, j, k, l, itemp;
  nlhand_dgopt_t nlh;

  nlh = (nlhand_dgopt_t)nlhandle;

#if 0
  MSK_checkmemtask(nlh->task,__FILE__,__LINE__);
#endif

  if (objval)
  {
    /* f(x) is computed and stored in objval[0]. */
    objval[0] = 0.0;

    for (k = 0; k < nlh->t; ++k)
    {
      if (nlh->p[k + 1] - nlh->p[k] > 1)
      {
        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
        {
          if (xx[j] <= 0.0)
          {
            return (1);
          }
        }
      }
    }

    for (k = 0; k < 1; ++k)
    {
      for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
      {
#if DEBUG
        printf("(%d) xx = %p, k = %d, j = %d, nlh = %p, p[0] = %d\n", __LINE__,
               xx, k, j, nlh, nlh->p[0]);
        if (xx[j] <= 0.0)
          printf("Zero xx[%d]: %e", j, xx[j]);

        assert(xx[j] > 0.0);
#endif

        objval[0] -= xx[j] * log(xx[j]);
      }
    }

    for (k = 1; k < nlh->t; ++k)
    {
      if (nlh->p[k + 1] - nlh->p[k] > 1)
      {
        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
        {
#if DEBUG
          if (xx[j] <= 0.0)
            printf("Zero xx[%d]: %e", j, xx[j]);

          assert(xx[j] > 0);
#endif

          objval[0] -= xx[j] * log(xx[j]);
        }
        rtemp = 0.0;

        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
          rtemp += xx[j];

        if (rtemp <= 0.0)
          return (1);

#if DEBUG
        assert(rtemp > 0);
#endif

        objval[0] += rtemp * log(rtemp);
      }
    }

    objval[0] *= OBJSCAL;

#if DEBUG
    printf("objval = %e\n", objval[0]);
#endif
  }

  if (numgrdobjnz)
  {
    /* Compute and store the gradient of the f. */

    itemp = 0;

    for (k = 0; k < 1; ++k)
    {
      for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
      {
        grdobjsub[itemp] = j;
#if DEBUG
        assert(xx[j] > 0);
#endif
        grdobjval[itemp] = -log(xx[j]) - 1.0;
        ++itemp;
      }
    }

    for (k = 1; k < nlh->t; ++k)
    {
      if (nlh->p[k + 1] - nlh->p[k] > 1)
      {
        rtemp = 0.0;
        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
          rtemp += xx[j];

        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
        {
          grdobjsub[itemp] = j;
#if DEBUG
          assert(xx[j] > 0);
#endif
          grdobjval[itemp] = log(rtemp / xx[j]);
          ++itemp;
        }
      }
    }

    numgrdobjnz[0] = itemp;

    for (k = 0; k < numgrdobjnz[0]; ++k)
      grdobjval[k] *= OBJSCAL;
  }

  if (conval)
    for (k      = 0; k < numi; ++k)
      conval[k] = 0.0;

  if (grdlag)
  {
    /* Compute and store the gradient of the Lagrangian.
     * Note that it is stored as a dense vector.
     */

    for (j      = 0; j < nlh->n; ++j)
      grdlag[j] = 0.0;

    for (k = 0; k < 1; ++k)
    {
      for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
      {
        grdlag[j] = yo * (-log(xx[j]) - 1.0);
      }
    }

    for (k = 1; k < nlh->t; ++k)
    {
      if (nlh->p[k + 1] - nlh->p[k] > 1)
      {
        rtemp = 0.0;
        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
          rtemp += xx[j];

        for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
        {
          grdlag[j] = yo * log(rtemp / xx[j]);
        }
      }
    }

    for (j = 0; j < nlh->n; ++j)
      grdlag[j] *= OBJSCAL;

#if DEBUG && PRINT_GRDLAG
    for (j = 0; j < nlh->n; ++j)
      printf("grdlag[%d] = %e\n", j, grdlag[j]);
#endif
  }

  if (maxnumhesnz)
  {
    /* Compute and store the Hessian of the Lagrangian
     * which in this case is identical to the Hessian
     * of f times yo.
     */

    if (yo == 0.0)
    {
      if (numhesnz)
        numhesnz[0] = 0;
    }
    else
    {
      if (numhesnz)
      {
        numhesnz[0] = nlh->numhesnz;

        if (maxnumhesnz < nlh->numhesnz)
          return (1);

        /* The diagonal element. */
        l = 0;
        for (j = nlh->p[0]; j < nlh->p[1]; ++j)
        {
          hessubi[l] = j;
          hessubj[l] = j;
          hesval[l]  = -yo / xx[j];
          ++l;
        }

        for (k = 1; k < nlh->t; ++k)
        {
          if (nlh->p[k + 1] - nlh->p[k] > 1)
          {
            double invrtemp;

            rtemp = 0.0;
            for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
            {
              rtemp += xx[j];
            }

            invrtemp = 1.0 / rtemp;

            /* The diagonal element. */
            for (j = nlh->p[k]; j < nlh->p[k + 1]; ++j)
            {
              hessubi[l] = j;
              hessubj[l] = j;
              /* equivalent to hesval[l]   = yo*(invrtemp - 1.0/xx[j]); */
              hesval[l] = yo * (xx[j] - rtemp) / (rtemp * xx[j]);
              ++l;

              /* The off diagonal elements. */
              for (i = j + 1; i < nlh->p[k + 1]; ++i)
              {
                hessubi[l] = i;
                hessubj[l] = j;
                hesval[l]  = yo * invrtemp;
                ++l;
              }
            }
          }
        }

        for (k = 0; k < numhesnz[0]; ++k)
          hesval[k] *= OBJSCAL;

#if DUMPHESSIAN
        {
          FILE* f;

          f = fopen("hessian.txt", "wt");
          for (k = 0; k < numhesnz[0]; ++k)
            fprintf(f, "%d %d %24.16e\n", hessubi[k], hessubj[k], hesval[k]);

          fclose(f);
        }
#endif

#if DEBUG && PRINT_HESVAL
        for (k = 0; k < numhesnz[0]; ++k)
          printf("hesval[%d] = %e\n", k, hesval[k]);
#endif
      }
    }
  }
  MSK_checkmemtask(nlh->task, __FILE__, __LINE__);

  return (0);
} /* dgoeval */

MSKrescodee
MSK_dgoread(MSKtask_t task, const char* nldatafile,
            MSKint32t*  numvar, /* numterms in primal */
            MSKint32t*  numcon, /* numvar in primal */
            MSKint32t*  t,      /* number of constraints in primal*/
            double**    v,      /* coiefients for terms in primal*/
            MSKint32t** p       /* corresponds to number of terms
                                   in each constraint in the
                                   primal */
            )
{
  MSKrescodee r = MSK_RES_OK;
  MSKenv_t    env;
  char        buf[MAX_LINE_LENGTH];
  FILE*       f;
  MSKint32t   i;

  MSK_getenv(task, &env);
  v[0] = NULL;
  p[0] = NULL;

  f = fopen(nldatafile, "rt");

  if (f)
  {
    fgets(buf, sizeof(buf), f);
    t[0] = (int)atol(buf);
  }
  else
  {
    printf("Could not open file '%s'\n", nldatafile);
    r = MSK_RES_ERR_FILE_OPEN;
  }

  if (r == MSK_RES_OK)
    r = MSK_getnumvar(task, numvar);

  if (r == MSK_RES_OK)
    r = MSK_getnumcon(task, numcon);

  if (r == MSK_RES_OK)
  {
    p[0] = (int*)MSK_calloctask(task, t[0], sizeof(int));
    if (p[0] == NULL)
      r = MSK_RES_ERR_SPACE;
  }

  if (r == MSK_RES_OK)
  {
    v[0] = (double*)MSK_calloctask(task, numvar[0], sizeof(double));
    if (v[0] == NULL)
      r = MSK_RES_ERR_SPACE;
  }

  if (r == MSK_RES_OK)
  {
    for (i = 0; i < numvar[0]; ++i)
    {
      fgets(buf, sizeof(buf), f);
      v[0][i] = atof(buf);
    }

    for (i = 0; i < t[0]; ++i)
    {
      fgets(buf, sizeof(buf), f);
      p[0][i] = (int)atol(buf);
    }
  }

  return (r);
}

MSKrescodee
MSK_dgosetup(MSKtask_t task, MSKint32t numvar, MSKint32t numcon, MSKint32t t,
             double* v, MSKint32t* p, dgohand_t* dgoh)
{

  MSKint32t       j, k;
  MSKrescodee     r   = MSK_RES_OK;
  nlhand_dgopt_t* nlh = (nlhand_dgopt_t*)dgoh;

  nlh[0] = (nlhand_dgopt_t)MSK_calloctask(task, 1, sizeof(nlhandt));
  if (nlh[0] != NULL)
  {
    /* set up nonlinear part */

    nlh[0]->p    = NULL;
    nlh[0]->n    = numvar;
    nlh[0]->t    = t;
    nlh[0]->task = task;

    nlh[0]->p = (MSKint32t*)MSK_calloctask(task, nlh[0]->t + 1, sizeof(int));
    if (nlh[0]->p != NULL)
    {
      nlh[0]->p[0] = 0;
      for (k = 0; k < nlh[0]->t; ++k)
      {
        nlh[0]->p[k + 1] = nlh[0]->p[k] + p[k];
      }

      for (k = 0; k < nlh[0]->t; ++k)
      {
        for (j = nlh[0]->p[k]; j < nlh[0]->p[k + 1]; ++j)
        {
          r = MSK_putcj(task, j, OBJSCAL * log(v[j]));
        }
      }

      if (nlh[0]->p[nlh[0]->t] == nlh[0]->n)
      {
        /*
         * The problem is now defined
         * and the setup can proceed.
         * Next, the number of Hessian non-zeros
         * is computed.
         */

        nlh[0]->numhesnz = nlh[0]->p[1] - nlh[0]->p[0];
        for (k = 1; k < nlh[0]->t; ++k)
        {
          if ((nlh[0]->p[k + 1] - nlh[0]->p[k]) > 1)
          {
            /* If only one term in primal constraint,
               the corresponding value in H is zero.
             */
            nlh[0]->numhesnz += ((nlh[0]->p[k + 1] - nlh[0]->p[k]) *
                                 (1 + nlh[0]->p[k + 1] - nlh[0]->p[k])) /
                                2;
          }
        }
        printf("Number of Hessian non-zeros: %d\n", nlh[0]->numhesnz);

        MSK_putnlfunc(task, nlh[0], dgostruc, dgoeval);
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);
      }
      else
      {
        printf("Incorrect function definition.\n");
        printf("n gathered from the task file: %d\n", nlh[0]->n);
        printf("n computed based on p        : %d\n", nlh[0]->p[nlh[0]->t]);
        r = MSK_RES_ERR_UNKNOWN;
      }
    }
    else
      r = MSK_RES_ERR_SPACE;
  }
  else
    r = MSK_RES_ERR_SPACE;

  return (r);
} /* dgosetup */

MSKrescodee
MSK_freedgo(MSKtask_t task, dgohand_t* dgoh)
{
  nlhand_dgopt_t* nlh = (nlhand_dgopt_t*)dgoh;

  if (nlh[0])
  {
    /* Free allocated data. */

    MSK_freetask(task, nlh[0]->p);
    MSK_freetask(task, nlh[0]);
    nlh[0] = NULL;
  }

  return (MSK_RES_OK);
}

typedef struct
{
  /*
   * Data structure for storing
   * data about the nonlinear
   * functions.
   */

  int     numcon; /* Number of constraints. */
  int     numvar; /* Number of variables.   */
  int     numopro;
  int*    opro;
  int*    oprjo;
  double* oprfo;
  double* oprgo;
  double* oprho;

  int     numoprc;
  int*    oprc;
  int*    opric;
  int*    oprjc;
  double* oprfc;
  double* oprgc;
  double* oprhc;

  /* */
  int* ptrc;
  int* subc;

  /* Work storage employed when evaluating the functions. */
  int*    ibuf;
  int*    zibuf;
  double* zdbuf;

} nlhandt_scopt;

typedef nlhandt_scopt* nlhand_scopt_t;

static void
scgrdobjstruc(nlhand_scopt_t nlh, int* nz, int* sub)
/* Purpose: Compute number of nonzeros and sparsity
            pattern of the gradient of the objective function.
 */
{
  int j, k, *zibuf;

  zibuf = nlh->zibuf;

#if DEBUG > 2
  printf("scgrdobjstruc: begin\n");
#endif

  if (nz)
  {
    nz[0] = 0;
    for (k = 0; k < nlh->numopro; ++k)
    {
      j = nlh->oprjo[k];

      if (!zibuf[j])
      {
        /* A new nonzero in the gradient of the objective has been located. */

        if (sub)
          sub[nz[0]] = j;

        ++nz[0];
        zibuf[j] = 1;
      }
    }

    /* Zero zibuf again. */
    for (k = 0; k < nlh->numopro; ++k)
    {
      j        = nlh->oprjo[k];
      zibuf[j] = 0;
    }
  }

#if DEBUG > 5
  printf("grdnz: %d\n", nz[0]);
#endif
#if DEBUG > 2
  printf("scgrdobjstruc: end\n");
#endif

} /* scgrdobjstruc */

static void
scgrdconistruc(nlhand_scopt_t nlh, int i, int* nz, int* sub)
{
  int j, k, *zibuf;

#if DEBUG > 2
  printf("scgrdconistruc: begin\n");
#endif

  zibuf = nlh->zibuf;

  nz[0] = 0;
  if (nlh->ptrc)
  {
    for (k = nlh->ptrc[i]; k < nlh->ptrc[i + 1]; ++k)
    {
      j = nlh->oprjc[nlh->subc[k]];

      if (!zibuf[j])
      {
        /* A new nonzero in the gradient of the ith
           constraint has been located. */

        if (sub)
          sub[nz[0]] = j;

        ++nz[0];
        zibuf[j] = 1;
      }
    }

    /* Zero zibuf again. */
    for (k = nlh->ptrc[i]; k < nlh->ptrc[i + 1]; ++k)
    {
      j        = nlh->oprjc[nlh->subc[k]];
      zibuf[j] = 0;
    }
  }

#if DEBUG > 5
  printf("i: %d nz: %d\n", i, nz[0]);
#endif
#if DEBUG > 2
  printf("scgrdconistruc: end\n");
#endif
} /* scgrdconistruc */

static int
schesstruc(nlhand_scopt_t nlh, int yo, int numycnz, const int* ycsub, int* nz,
           int* sub)
/* Computes the number nonzeros the lower triangular part of
   the Hessian of the Lagrange function and sparsity pattern.

   nz: Number of nonzeros in the Hessian of the Lagrange function.
   sub: List of nonzero diagonal elements in the Hessian.
        The separable structure is exploited.
 */
{
  int i, j, k, p, *zibuf;

#if DEBUG > 2
  printf("schesstruc: begin\n");
#endif

  zibuf = nlh->zibuf;

  nz[0] = 0;

  if (yo)
  {
    /* Information about the objective function should be computed. */
    for (k = 0; k < nlh->numopro; ++k)
    {
      j = nlh->oprjo[k];

      if (!zibuf[j])
      {
        /* A new nonzero in the gradient has been located. */

        if (sub)
          sub[nz[0]] = j;

        ++nz[0];
        zibuf[j] = 1;
      }
    }
  }

  if (nlh->ptrc)
  {
    /* Evaluate the sparsity of the Hessian. Only constraints specified
       by ycsub should be included.
     */
    for (p = 0; p < numycnz; ++p)
    {
      i = ycsub[p]; /* Constraint index. */
      for (k = nlh->ptrc[i]; k < nlh->ptrc[i + 1]; ++k)
      {
        j = nlh->oprjc[nlh->subc[k]];

        if (!zibuf[j])
        {
          /* A new nonzero diagonal element in the Hessian has been located. */
          if (sub)
            sub[nz[0]] = j;

          ++nz[0];
          zibuf[j] = 1;
        }
      }
    }
  }

  /*
   * Zero work vectors.
   */

  if (yo)
  {
    for (k = 0; k < nlh->numopro; ++k)
    {
      j        = nlh->oprjo[k];
      zibuf[j] = 0;
    }
  }

  if (nlh->ptrc)
  {
    for (p = 0; p < numycnz; ++p)
    {
      i = ycsub[p];
      for (k = nlh->ptrc[i]; k < nlh->ptrc[i + 1]; ++k)
      {
        j        = nlh->oprjc[nlh->subc[k]];
        zibuf[j] = 0;
      }
    }
  }

#if DEBUG > 5
  printf("Hessian size: %d\n", nz[0]);
#endif
#if DEBUG > 2
  printf("schesstruc: end\n");
#endif

  return (MSK_RES_OK);
} /* schesstruc */

static int MSKAPI
scstruc(void* nlhandle, int* numgrdobjnz, int* grdobjsub, int i, int* convali,
        int* grdconinz, int* grdconisub, int yo, int numycnz, const int* ycsub,
        int maxnumhesnz, int* numhesnz, int* hessubi, int* hessubj)
/* Purpose: Provide information to MOSEK about the
            problem structure and sparsity.
 */
{
  int            k, itemp;
  nlhand_scopt_t nlh;

#if DEBUG > 2
  printf("scstruc: begin\n");
#endif

  nlh = (nlhand_scopt_t)nlhandle;

  if (numgrdobjnz)
    scgrdobjstruc(nlh, numgrdobjnz, grdobjsub);

  if (convali || grdconinz)
  {
    scgrdconistruc(nlh, i, &itemp, grdconisub);

    if (convali)
      convali[0] = itemp > 0;

    if (grdconinz)
      grdconinz[0] = itemp;
  }

  if (numhesnz)
  {
#if DEBUG > 2
    printf("Evaluate Hessian structure yo: %d\n", yo);
#endif

    schesstruc(nlh, yo, numycnz, ycsub, numhesnz, hessubi);

    if (numhesnz[0] > maxnumhesnz && hessubi)
    {
      printf("%s(%d): Hessian size error. %d %d\n", __FILE__, __LINE__,
             numhesnz[0], maxnumhesnz);
      exit(0);
    }

    if (hessubi)
    {
      /* In this case the Hessian is diagonal matrix. */

      for (k       = 0; k < numhesnz[0]; ++k)
        hessubj[k] = hessubi[k];
    }
  }

#if DEBUG > 5
  if (numhesnz)
  {
    printf("Number of Hessian nonzeros: %d\n", numhesnz[0]);
  }
#endif

#if DEBUG > 2
  printf("scstruc: end\n");
#endif

  return (MSK_RES_OK);
} /* scstruc */

static int
evalopr(int opr, double f, double g, double h, int j, double xj, double* fxj,
        double* grdfxj, double* hesfxj)
/* Purpose: Evaluates an operator and its derivatives.
     fxj:    Is the function value
     grdfxj: Is the first derivative.
     hexfxj: Is the second derivative.
   Return: A nonzero value if a function could not evaluated.
 */
{
  double rtemp;

  switch (opr)
  {
    case MSK_OPR_ENT:
      if (xj <= 0.0)
        return (1);

      if (fxj)
        fxj[0] = f * xj * log(xj);

      if (grdfxj)
        grdfxj[0] = f * (1.0 + log(xj));

      if (hesfxj)
        hesfxj[0] = f / xj;
      break;
    case MSK_OPR_EXP:
      rtemp = exp(g * xj + h);

      if (fxj)
        fxj[0] = f * rtemp;

      if (grdfxj)
        grdfxj[0] = f * g * rtemp;

      if (hesfxj)
        hesfxj[0] = f * g * g * rtemp;
      break;
    case MSK_OPR_LOG:
      rtemp = g * xj + h;
      if (rtemp <= 0.0)
        return (1);

      if (fxj)
        fxj[0] = f * log(rtemp);

      if (grdfxj)
        grdfxj[0] = (g * f) / (rtemp);

      if (hesfxj)
        hesfxj[0] = -(f * g * g) / (rtemp * rtemp);
      break;
    case MSK_OPR_POW:
      if (g == 1.0)
      {
        /* This is the linear case.
           We strongly recommend not to put
           linear terms among the nonlinearities. */

        if (fxj)
          fxj[0] = f * (xj + h);

        if (grdfxj)
          grdfxj[0] = f;

        if (hesfxj)
          hesfxj[0] = 0.0;
      }
      else
      {
        if (fxj)
          fxj[0] = f * pow(xj + h, g);

        if (grdfxj)
        {
          if (xj == 0.0 && g < 1.0)
          {
#if DEBUG
            printf("%s(%d): Hessian evaluation error for variable x%d: "
                   "%e*(%e+%e)^%e\n",
                   __FILE__, __LINE__, j, f, xj, h, g);
#endif
            return (1);
          }

          grdfxj[0] = f * g * pow(xj + h, g - 1.0);
        }

        if (hesfxj)
        {
          if ((xj + h) == 0.0 && g < 2.0)
          {
/* The second order derivative is not defined because 1.0/0.0 has to be
 * evaluated. */

#if DEBUG
            printf("%s(%d): Hessian evaluation error for variable x%d: "
                   "%e*(%e+%e)^%e\n",
                   __FILE__, __LINE__, j, f, xj, h, g);
#endif
            return (1);
          }
          hesfxj[0] = f * g * (g - 1.0) * pow(xj + h, g - 2.0);
        }
      }
      break;
    case MSK_OPR_SQRT: /* handle operator f * sqrt(gx + h) */

      rtemp = g * xj + h;
      if (rtemp < 0.0 || (rtemp <= 0.0 && (grdfxj || hesfxj)))
      {
#if DEBUG
        printf("%s(%d): Function evaluation error\n", __FILE__, __LINE__);
#endif

        return (1);
      }

      if (fxj)
        fxj[0] = f * sqrt(rtemp); /* The function value. */

      if (grdfxj)
        grdfxj[0] = 0.5 * f * g / sqrt(rtemp); /* The gradient. */

      if (hesfxj)
        hesfxj[0] = -0.25 * f * g * g * pow(rtemp, -1.5);
      break;
    default:
      printf("scopt.c: Unknown operator %d\n", opr);
      exit(0);
  }

  return (0);
} /* evalopr */

static int
scobjeval(nlhand_scopt_t nlh, const double* x, double* objval, int* grdnz,
          int* grdsub, double* grdval)
/* Purpose: Compute number objective value and the gradient. */
{
  int    j, k, *zibuf;
  int    r = 0;
  double fx, grdfx, *zdbuf;

#if DEBUG > 2
  printf("scobjeval: begin\n");
#endif

  zibuf = nlh->zibuf;
  zdbuf = nlh->zdbuf;

  if (objval)
    objval[0] = 0.0;

  if (grdnz)
    grdnz[0] = 0;

  for (k = 0; k < nlh->numopro && r == MSK_RES_OK; ++k)
  {
    j = nlh->oprjo[k];

    r = evalopr(nlh->opro[k], nlh->oprfo[k], nlh->oprgo[k], nlh->oprho[k], j,
                x[j], &fx, &grdfx, NULL);
    if (r)
    {
#if DEBUG
      printf("Failure for variable j: %d\n", j);
#endif
    }
    else
    {
      if (objval)
        objval[0] += fx;

      if (grdnz)
      {
        zdbuf[j] += grdfx;

        if (!zibuf[j])
        {
          /* A new nonzero in the gradient has been located. */
          grdsub[grdnz[0]] = j;
          zibuf[j]         = 1;
          ++grdnz[0];
        }
      }
    }
  }

  if (grdnz != NULL)
  {
    /* Buffers should be zeroed. */
    for (k = 0; k < grdnz[0]; ++k)
    {
      j = grdsub[k];

      if (grdval)
        grdval[k] = zdbuf[j];

      zibuf[j] = 0;
      zdbuf[j] = 0.0;
    }
  }

#if DEBUG > 5
  if (objval != NULL)
    printf("objval: %e\n", objval[0]);

  if (grdnz != NULL)
    printf("grdnz: %d\n", grdnz[0]);

  if (grdsub && grdval)
  {
    printf("grdobj:");
    for (k = 0; k < grdnz[0]; ++k)
      printf(" %e[%d]", grdval[k], grdsub[k]);
    printf("\n");
  }
#endif

#if DEBUG > 2
  printf("scobjeval: end\n");
#endif

  return (r);
} /* scobjeval */

static int
scgrdconeval(nlhand_scopt_t nlh, int i, const double* x, double* fval,
             int grdnz, const int* grdsub, double* grdval)
/* Purpose: Compute number value and the gradient of constraint i.
            grdsub[0,...,grdnz-1] tells which values are needed in gradient
            that is required. */
{
  int    r = 0, j, k, p, gnz, *ibuf, *zibuf;
  double fx, grdfx, *zdbuf;

#if DEBUG > 2
  printf("scgrdconeval: begin\n");
#endif

  ibuf  = nlh->ibuf;
  zibuf = nlh->zibuf;
  zdbuf = nlh->zdbuf;

  if (fval)
    fval[0] = 0.0;

  if (nlh->ptrc)
  {
    gnz = 0;
    for (p = nlh->ptrc[i]; p < nlh->ptrc[i + 1] && !r; ++p)
    {
      k = nlh->subc[p];
      j = nlh->oprjc[k];

      r = evalopr(nlh->oprc[k], nlh->oprfc[k], nlh->oprgc[k], nlh->oprhc[k], j,
                  x[j], &fx, &grdfx, NULL);

      if (r)
      {
#if DEBUG
        printf("Failure for variable j: %d\n", j);
#endif
      }
      else
      {
        if (fval)
          fval[0] += fx;

        if (grdnz > 0)
        {
          zdbuf[j] += grdfx;

          if (!zibuf[j])
          {
            /* A new nonzero in the gradient has been located. */

            ibuf[gnz] = j;
            zibuf[j]  = 1;
            ++gnz;
          }
        }
      }
    }

    if (grdval != NULL)
    {
      /* Setup gradient. */
      for (k = 0; k < grdnz; ++k)
      {
        j         = grdsub[k];
        grdval[k] = zdbuf[j];
      }
    }

    for (k = 0; k < gnz; ++k)
    {
      j        = ibuf[k];
      zibuf[j] = 0;
      zdbuf[j] = 0.0;
    }
  }
  else if (grdval)
  {
    for (k      = 0; k < grdnz; ++k)
      grdval[k] = 0.0;
  }

#if DEBUG > 5
  printf("i: %d\n", i);
  if (fval != NULL)
    printf("fval: %e\n", fval[0]);

  if (grdnz)
  {
    printf("grdnz: %d\n", grdnz);

    if (grdsub && grdval)
    {
      printf("grd:");
      for (k = 0; k < grdnz; ++k)
        printf(" %e[%d]", grdval[k], grdsub[k]);
      printf("\n");
    }
  }
#endif

#if DEBUG > 2
  printf("scgrdconeval: end\n");
#endif

  return (r);
} /* scgrdconeval */

static int MSKAPI
sceval(void* nlhandle, const double* xx, double yo, const double* yc,
       double* objval, int* numgrdobjnz, int* grdobjsub, double* grdobjval,
       int numi, const int* subi, double* conval, const int* grdconptrb,
       const int* grdconptre, const int* grdconsub, double* grdconval,
       double* grdlag, int maxnumhesnz, int* numhesnz, int* hessubi,
       int* hessubj, double* hesval)
/* Purpose: Evalute the nonlinear function and return the
            requested information to MOSEK.
 */
{
  nlhand_scopt_t  nlh    = (nlhand_scopt_t)nlhandle;
  const MSKint32t numcon = nlh->numcon, numvar = nlh->numvar;
  double          fx, grdfx, hesfx;
  int             r = 0;
  int             i, j, k, l, p, *zibuf;

#if DEBUG
  printf("sceval: begin\n");
#endif

  if (numhesnz)
    numhesnz[0] = 0;

  r = scobjeval(nlh, xx, objval, numgrdobjnz, grdobjsub, grdobjval);

  for (k = 0; k < numi && !r; ++k)
  {
    i = subi[k];
    r = scgrdconeval(nlh, i, xx, conval == NULL ? NULL : conval + k,
                     grdconsub == NULL ? 0 : grdconptre[k] - grdconptrb[k],
                     grdconsub == NULL ? NULL : grdconsub + grdconptrb[k],
                     grdconval == NULL ? NULL : grdconval + grdconptrb[k]);
  }

#if DEBUG
  if (r)
    printf("%s(%d): r=%d\n", __FILE__, __LINE__, r);
#endif

  if (grdlag && !r)
  {
    /* Compute and store the gradient of the Lagrangian.
     * Note it is stored as a dense vector.
     */

    for (j      = 0; j < numvar; ++j)
      grdlag[j] = 0.0;

    if (yo != 0.0)
    {
      for (k = 0; k < nlh->numopro && !r; ++k)
      {
        j = nlh->oprjo[k];
        r = evalopr(nlh->opro[k], nlh->oprfo[k], nlh->oprgo[k], nlh->oprho[k],
                    j, xx[j], NULL, &grdfx, NULL);
        if (r)
        {
#if DEBUG
          printf(
            "%s(%d): Function evaluation error for variable j: %d xx: %e\n",
            __FILE__, __LINE__, j, xx[j]);
#endif
        }
        else
          grdlag[j] += yo * grdfx;
      }
    }

#if DEBUG
    if (r)
      printf("%s(%d): r=%d\n", __FILE__, __LINE__, r);
#endif

    if (nlh->ptrc)
    {
      for (l = 0; l < numi && !r; ++l)
      {
        i = subi[l];
        for (p = nlh->ptrc[i]; p < nlh->ptrc[i + 1] && !r; ++p)
        {
          k = nlh->subc[p];
          j = nlh->oprjc[k];
          r = evalopr(nlh->oprc[k], nlh->oprfc[k], nlh->oprgc[k], nlh->oprhc[k],
                      j, xx[j], NULL, &grdfx, NULL);
          if (r)
          {
#if DEBUG
            printf(
              "%s(%d): Function evaluation error for variable j: %d xx: %e\n",
              __FILE__, __LINE__, j, xx[j]);
#endif
          }
          else
            grdlag[j] -= yc[i] * grdfx;
        }
      }
    }
  }

#if DEBUG
  if (r)
    printf("%s(%d): r=%d\n", __FILE__, __LINE__, r);
#endif

  if (maxnumhesnz && !r)
  {
/* Compute and store the Hessian of the Lagrange function. */

#if DEBUG & 0
    printf("x: \n");
    for (j = 0; j < numvar; ++j)
      printf(" %e\n", xx[j]);

    printf("yc: \n");
    for (i = 0; i < numcon; ++i)
      printf(" %e\n", yc[i]);
#endif

    zibuf       = nlh->zibuf;
    numhesnz[0] = 0;
    if (yo != 0.0)
    {
      for (k = 0; k < nlh->numopro && !r; ++k)
      {
        j = nlh->oprjo[k];
        r = evalopr(nlh->opro[k], nlh->oprfo[k], nlh->oprgo[k], nlh->oprho[k],
                    j, xx[j], NULL, NULL, &hesfx);
        if (r)
        {
#if DEBUG
          printf(
            "%s(%d): Function evaluation error for variable j: %d xx: %e\n",
            __FILE__, __LINE__, j, xx[j]);
#endif
        }
        else
        {
          if (!zibuf[j])
          {
            ++numhesnz[0];
            zibuf[j]              = numhesnz[0];
            hessubi[zibuf[j] - 1] = j;
            hesval[zibuf[j] - 1]  = 0.0;
          }
          hesval[zibuf[j] - 1] += yo * hesfx;
        }
      }
    }

    if (nlh->ptrc)
    {
      for (l = 0; l < numi && !r; ++l)
      {
        i = subi[l];
        for (p = nlh->ptrc[i]; p < nlh->ptrc[i + 1] && !r; ++p)
        {
          k = nlh->subc[p];
          j = nlh->oprjc[k];
          r = evalopr(nlh->oprc[k], nlh->oprfc[k], nlh->oprgc[k], nlh->oprhc[k],
                      j, xx[j], NULL, NULL, &hesfx);
          if (r)
          {
#if DEBUG
            printf(
              "%s(%d): Function evaluation error for variable j: %d xx: %e\n",
              __FILE__, __LINE__, j, xx[j]);
#endif
          }
          else
          {
            if (!zibuf[j])
            {
              ++numhesnz[0];
              zibuf[j]              = numhesnz[0];
              hesval[zibuf[j] - 1]  = 0.0;
              hessubi[zibuf[j] - 1] = j;
            }
            hesval[zibuf[j] - 1] -= yc[i] * hesfx;
          }
        }
      }
    }

    if (numhesnz[0] > maxnumhesnz)
    {
      printf("Hessian evalauation error\n");
      exit(0);
    }

    for (k = 0; k < numhesnz[0]; ++k)
    {
      j          = hessubi[k];
      hessubj[k] = j;
      zibuf[j]   = 0;
    }
  }

#if DEBUG > 5
  if (conval != NULL)
  {
    printf("conval:");
    for (k = 0; k < numi; ++k)
      printf(" %e[%d]", conval[k], subi[k]);
    printf("\n");
  }
  if (grdlag != NULL)
  {
    printf("grdlag:");
    for (j = 0; j < numvar; ++j)
      printf(" %e", grdlag[j]);
    printf("\n");
  }

  if (numhesnz != NULL)
  {
    printf("Hessian: ");
    for (k = 0; k < numhesnz[0]; ++k)
      printf(" %e[%d,%d]", hesval[k], hessubi[k], hessubj[k]);
    printf("\n");
  }
#endif

#if DEBUG
  printf("sceval: end\n");
#endif

  return (r);
} /* sceval */

MSKrescodee
MSK_scbegin(MSKtask_t task, int numopro, int* opro, int* oprjo, double* oprfo,
            double* oprgo, double* oprho, int numoprc, int* oprc, int* opric,
            int* oprjc, double* oprfc, double* oprgc, double* oprhc,
            schand_t* sch)
{
  int            itemp, k, p, sum;
  MSKrescodee    r = MSK_RES_OK;
  nlhand_scopt_t nlh;

#if DEBUG
  printf("MSK_scbegin: begin\n");
#endif

  nlh = (nlhand_scopt_t)MSK_calloctask(task, 1, sizeof(nlhandt_scopt));
  if (nlh)
  {
    sch[0] = (void*)nlh;

    MSK_getnumcon(task, &nlh->numcon);
    MSK_getnumvar(task, &nlh->numvar);

    nlh->numopro = numopro;
    nlh->opro    = (int*)MSK_calloctask(task, numopro, sizeof(int));
    nlh->oprjo   = (int*)MSK_calloctask(task, numopro, sizeof(int));
    nlh->oprfo   = (double*)MSK_calloctask(task, numopro, sizeof(double));
    nlh->oprgo   = (double*)MSK_calloctask(task, numopro, sizeof(double));
    nlh->oprho   = (double*)MSK_calloctask(task, numopro, sizeof(double));

    nlh->numoprc = numoprc;
    nlh->oprc    = (int*)MSK_calloctask(task, numoprc, sizeof(int));
    nlh->opric   = (int*)MSK_calloctask(task, numoprc, sizeof(int));
    nlh->oprjc   = (int*)MSK_calloctask(task, numoprc, sizeof(int));
    nlh->oprfc   = (double*)MSK_calloctask(task, numoprc, sizeof(double));
    nlh->oprgc   = (double*)MSK_calloctask(task, numoprc, sizeof(double));
    nlh->oprhc   = (double*)MSK_calloctask(task, numoprc, sizeof(double));

    if ((!numopro ||
         (nlh->opro && nlh->oprjo && nlh->oprfo && nlh->oprgo && nlh->oprho)) &&
        (!numoprc || (nlh->oprc && nlh->opric && nlh->oprjc && nlh->oprfc &&
                      nlh->oprgc && nlh->oprhc)))
    {
      p = 0;
      for (k = 0; k < numopro; ++k)
      {
        nlh->opro[p]  = opro[k];
        nlh->oprjo[p] = oprjo[k];
        nlh->oprfo[p] = oprfo[k];
        nlh->oprgo[p] = oprgo[k];
        nlh->oprho[p] = oprho[k];
        ++p;
      }

      nlh->numopro = p;

      for (k = 0; k < numoprc; ++k)
      {
        nlh->oprc[k]  = oprc[k];
        nlh->opric[k] = opric[k];
        nlh->oprjc[k] = oprjc[k];
        nlh->oprfc[k] = oprfc[k];
        nlh->oprgc[k] = oprgc[k];
        nlh->oprhc[k] = oprhc[k];
      }

#if DEBUG
      /*
       * Check if data is valid.
       */

      for (k = 0; k < numopro; ++k)
      {
        if (oprjo[k] < 0 || oprjo[k] >= nlh->numvar)
        {
          printf("oprjo[%d]=%d is invalid.\n", k, oprjo[k]);
          exit(0);
        }
      }

      for (k = 0; k < numoprc; ++k)
      {
        if (opric[k] < 0 || opric[k] >= nlh->numcon)
        {
          printf("opric[%d]=%d is invalid. numcon: %d\n", k, opric[k],
                 nlh->numcon);
          exit(0);
        }

        if (oprjc[k] < 0 || oprjc[k] >= nlh->numvar)
        {
          printf("oprjc[%d]=%d is invalid.\n", k, oprjc[k]);
          exit(0);
        }
      }
#endif

      /*
       * Allocate work vectors.
       */

      nlh->ibuf  = (int*)MSK_calloctask(task, nlh->numvar, sizeof(int));
      nlh->zibuf = (int*)MSK_calloctask(task, nlh->numvar, sizeof(int));
      nlh->zdbuf = (double*)MSK_calloctask(task, nlh->numvar, sizeof(double));
      if (numoprc)
      {
        nlh->ptrc = (int*)MSK_calloctask(task, nlh->numcon + 1, sizeof(int));
        nlh->subc = (int*)MSK_calloctask(task, numoprc, sizeof(int));
      }

      if ((!nlh->numvar || (nlh->ibuf && nlh->zibuf && nlh->zdbuf)) &&
          (!numoprc || (nlh->ptrc && nlh->subc)))
      {
        if (nlh->numcon && numoprc > 0)
        {
          /*
             Setup of ptrc and sub. Afte the setup then
             1) ptrc[i+1]-ptrc[i]: Is the number of nonlinear terms in the ith
             constraint.
             2) subc[ptrc[i],...,ptrc[i+1]-1]: List the nonlinear terms in the
             ith constrint.
           */

          for (k = 0; k < numoprc; ++k)
            ++nlh->ptrc[opric[k]];

          sum = 0;
          for (k = 0; k <= nlh->numcon; ++k)
          {
            itemp        = nlh->ptrc[k];
            nlh->ptrc[k] = sum;
            sum += itemp;
          }

          for (k = 0; k < numoprc; ++k)
          {
            nlh->subc[nlh->ptrc[opric[k]]] = k;
            ++nlh->ptrc[opric[k]];
          }

          for (k         = nlh->numcon; k; --k)
            nlh->ptrc[k] = nlh->ptrc[k - 1];

          nlh->ptrc[0] = 0;

#if DEBUG
          {
            int p;
            for (k = 0; k < nlh->numcon; ++k)
            {
              printf("ptrc[%d]: %d subc: \n", k, nlh->ptrc[k]);
              for (p = nlh->ptrc[k]; p < nlh->ptrc[k + 1]; ++p)
                printf(" %d", nlh->subc[p]);
              printf("\n");
            }
          }
#endif
        }
        r = MSK_putnlfunc(task, (void*)nlh, scstruc, sceval);
      }
      else
        r = MSK_RES_ERR_SPACE;
    }
    else
      r = MSK_RES_ERR_SPACE;
  }
  else
    r = MSK_RES_ERR_SPACE;

#if DEBUG
  printf("MSC_begin: end\n");
#endif

  return (r);
} /* MSK_scbegin */

MSKrescodee
MSK_scwrite(MSKtask_t task, schand_t sch, char filename[])
{
  char*          fn;
  int            k;
  FILE*          f;
  MSKrescodee    r = MSK_RES_OK;
  nlhand_scopt_t nlh;
  size_t         l;

  nlh = (nlhand_scopt_t)sch;
  l   = strlen(filename);
  fn  = (char*)MSK_calloctask(task, l + 5, sizeof(char));
  if (fn)
  {
    for (l  = 0; filename[l] && filename[l] != '.'; ++l)
      fn[l] = filename[l];

    strcpy(fn + l, ".mps");

    r = MSK_writedata(task, fn);
    if (r == MSK_RES_OK)
    {
      strcpy(fn + l, ".sco");

      f = fopen(fn, "wt");
      if (f)
      {
        printf("Writing: %s\n", fn);

        fprintf(f, "%d\n", nlh->numopro);

        for (k = 0; k < nlh->numopro; ++k)
          fprintf(f, "%-8d %-8d %-24.16e %-24.16e %-24.16e\n", nlh->opro[k],
                  nlh->oprjo[k], nlh->oprfo[k], nlh->oprgo[k], nlh->oprho[k]);

        fprintf(f, "%d\n", nlh->numoprc);
        for (k = 0; k < nlh->numoprc; ++k)
          fprintf(f, "%-8d %-8d %-8d %-24.16e %-24.16e %-24.16e\n",
                  nlh->oprc[k], nlh->opric[k], nlh->oprjc[k], nlh->oprfc[k],
                  nlh->oprgc[k], nlh->oprhc[k]);
      }
      else
      {
        printf("Could not open file: '%s'\n", filename);
        r = MSK_RES_ERR_FILE_OPEN;
      }
      fclose(f);
    }
  }
  else
    r = MSK_RES_ERR_SPACE;

  MSK_freetask(task, fn);

#if DEBUG
  printf("MSK_scwrite: end\n");
#endif

  return (r);
} /* MSK_scwrite */

MSKrescodee
MSK_scread(MSKtask_t task, schand_t* sch, char filename[])
{
  char    buffer[1024], fbuf[80], hbuf[80], gbuf[80], *fn;
  double *oprfo = NULL, *oprgo = NULL, *oprho = NULL, *oprfc = NULL,
         *oprgc = NULL, *oprhc = NULL;
  int k, p, numopro, numoprc, *opro = NULL, *oprjo = NULL, *oprc = NULL,
                              *opric = NULL, *oprjc = NULL;
  FILE*       f;
  MSKrescodee r;
  size_t      l;

#if DEBUG
  printf("MSK_scread: begin\n");
#endif

  sch[0] = NULL;
  l      = strlen(filename);
  fn     = (char*)MSK_calloctask(task, l + 5, sizeof(char));
  if (fn)
  {
    strcpy(fn, filename);
    for (k = 0; fn[k] && fn[k] != '.'; ++k)
      ;

    strcpy(fn + k, ".mps");

    {
      r = MSK_readdata(task, fn);
      if (r == MSK_RES_OK)
      {
        strcpy(fn + k, ".sco");

        printf("Opening: %s\n", fn);

        f = fopen(fn, "rt");
        if (f)
        {
          printf("Reading.\n");

          fgets(buffer, sizeof(buffer), f);
          sscanf(buffer, "%d", &numopro);

          if (numopro)
          {
            opro  = (int*)MSK_calloctask(task, numopro, sizeof(int));
            oprjo = (int*)MSK_calloctask(task, numopro, sizeof(int));
            oprfo = (double*)MSK_calloctask(task, numopro, sizeof(double));
            oprgo = (double*)MSK_calloctask(task, numopro, sizeof(double));
            oprho = (double*)MSK_calloctask(task, numopro, sizeof(double));

            if (opro && oprjo && oprfo && oprgo && oprho)
            {
              for (k = 0; k < numopro; ++k)
              {
                fgets(buffer, sizeof(buffer), f);

                for (p = 0; buffer[p]; ++p)
                  if (buffer[p] == ' ')
                    buffer[p] = '\n';

                sscanf(buffer, "%d %d %s %s %s", opro + k, oprjo + k, fbuf,
                       gbuf, hbuf);

                oprfo[k] = atof(fbuf);
                oprgo[k] = atof(gbuf);
                oprho[k] = atof(hbuf);
              }
            }
            else
              r = MSK_RES_ERR_SPACE;
          }

          if (r == MSK_RES_OK)
          {
            fgets(buffer, sizeof(buffer), f);
            sscanf(buffer, "%d", &numoprc);

            if (numoprc)
            {
              oprc  = (int*)MSK_calloctask(task, numoprc, sizeof(int));
              opric = (int*)MSK_calloctask(task, numoprc, sizeof(int));
              oprjc = (int*)MSK_calloctask(task, numoprc, sizeof(int));
              oprfc = (double*)MSK_calloctask(task, numoprc, sizeof(double));
              oprgc = (double*)MSK_calloctask(task, numoprc, sizeof(double));
              oprhc = (double*)MSK_calloctask(task, numoprc, sizeof(double));

              if (oprc && oprjc && oprfc && oprgc && oprhc)
              {
                for (k = 0; k < numoprc; ++k)
                {
                  fgets(buffer, sizeof(buffer), f);

                  for (p = 0; buffer[p]; ++p)
                    if (buffer[p] == ' ')
                      buffer[p] = '\n';

                  sscanf(buffer, "%d %d %d %s %s %s", oprc + k, opric + k,
                         oprjc + k, fbuf, gbuf, hbuf);

                  oprfc[k] = atof(fbuf);
                  oprgc[k] = atof(gbuf);
                  oprhc[k] = atof(hbuf);
                }
              }
              else
                r = MSK_RES_ERR_SPACE;
            }
            else
              printf("No nonlinear terms in constraints\n");
          }

          fclose(f);
        }
        else
        {
          printf("Could not open file: '%s'\n", fn);
          r = MSK_RES_ERR_FILE_OPEN;
        }

        if (r == MSK_RES_OK)
          r =
            MSK_scbegin(task, numopro, opro, oprjo, oprfo, oprgo, oprho,
                        numoprc, oprc, opric, oprjc, oprfc, oprgc, oprhc, sch);

        MSK_freetask(task, opro);
        MSK_freetask(task, oprjo);
        MSK_freetask(task, oprfo);
        MSK_freetask(task, oprgo);
        MSK_freetask(task, oprho);

        MSK_freetask(task, oprc);
        MSK_freetask(task, opric);
        MSK_freetask(task, oprjc);
        MSK_freetask(task, oprfc);
        MSK_freetask(task, oprgc);
        MSK_freetask(task, oprhc);
      }
      else
      {
        printf("Could not open file: '%s'\n", filename);
        r = MSK_RES_ERR_FILE_OPEN;
      }
    }
  }
  else
    r = MSK_RES_ERR_SPACE;

  MSK_freetask(task, fn);

#if DEBUG
  printf("MSK_scread: end r: %d\n", r);
#endif

  return (r);
} /* MSK_scread */

MSKrescodee
MSK_scend(MSKtask_t task, schand_t* sch)
/* Purpose: Free all data associated with nlh. */
{
  nlhand_scopt_t nlh;

#if DEBUG
  printf("MSK_scend: begin\n");
#endif

  if (sch[0])
  {
    /* Remove nonlinear function data. */
    MSK_putnlfunc(task, NULL, NULL, NULL);

    nlh    = (nlhand_scopt_t)sch[0];
    sch[0] = nlh;

#if DEBUG
    printf("MSK_scend: deallocate\n");
#endif

    MSK_freetask(task, nlh->opro);
    MSK_freetask(task, nlh->oprjo);
    MSK_freetask(task, nlh->oprfo);
    MSK_freetask(task, nlh->oprgo);
    MSK_freetask(task, nlh->oprho);

    MSK_freetask(task, nlh->oprc);
    MSK_freetask(task, nlh->opric);
    MSK_freetask(task, nlh->oprjc);
    MSK_freetask(task, nlh->oprfc);
    MSK_freetask(task, nlh->oprgc);
    MSK_freetask(task, nlh->oprhc);

    MSK_freetask(task, nlh->ptrc);
    MSK_freetask(task, nlh->subc);
    MSK_freetask(task, nlh->ibuf);
    MSK_freetask(task, nlh->zibuf);
    MSK_freetask(task, nlh->zdbuf);

    MSK_freetask(task, nlh);
  }

#if DEBUG
  printf("MSK_scend: end\n");
#endif

  return (MSK_RES_OK);
} /* MSK_scend */

ExponentialOptimization::ExponentialOptimization()
{
}

void
ExponentialOptimization::solve()
{
  //! @note main example for exponential optimization
  int r = MSK_RES_OK, numcon = 1, numvar = 3, numter = 5;

  int    subi[] = { 0, 0, 0, 1, 1 };
  int    subk[] = { 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4 };
  double c[]    = { 40.0, 20.0, 40.0, 0.333333, 1.333333 };
  int    subj[] = { 0, 1, 2, 0, 2, 0, 1, 2, 0, 1, 1, 2 };
  double akj[]  = { -1,  -0.5, -1.0, 1.0,  1.0, 1.0,
                   1.0, 1.0,  -2.0, -2.0, 0.5, -1.0 };
  int          numanz = 12;
  double       objval;
  double       xx[3];
  double       y[5];
  MSKenv_t     env;
  MSKprostae   prosta;
  MSKsolstae   solsta;
  MSKtask_t    expopttask;
  expopthand_t expopthnd = NULL;
  /* Pointer to data structure that holds nonlinear information */

  if (r == MSK_RES_OK)
    r = MSK_makeenv(&env, NULL);

  if (r == MSK_RES_OK)
    MSK_makeemptytask(env, &expopttask);

  if (r == MSK_RES_OK)
    r = MSK_linkfunctotaskstream(expopttask, MSK_STREAM_LOG, NULL, printcb);

  if (r == MSK_RES_OK)
  {
    /* Initialize expopttask with problem data */
    r = MSK_expoptsetup(expopttask, 1, /* Solve the dual formulation */
                        numcon, numvar, numter, subi, c, subk, subj, akj,
                        numanz, &expopthnd
                        /* Pointer to data structure holding nonlinear data */
                        );
  }

  /* Any parameter can now be changed with standard mosek function calls */
  if (r == MSK_RES_OK)
    r = MSK_putintparam(expopttask, MSK_IPAR_INTPNT_MAX_ITERATIONS, 200);

  /* Optimize, xx holds the primal optimal solution,
   y holds solution to the dual problem if the dual formulation is used
  */

  if (r == MSK_RES_OK)
    r =
      MSK_expoptimize(expopttask, &prosta, &solsta, &objval, xx, y, &expopthnd);

  /* Free data allocated by expoptsetup */
  if (expopthnd)
    MSK_expoptfree(expopttask, &expopthnd);

  MSK_deletetask(&expopttask);
  MSK_deleteenv(&env);
}

int
isemptyline(char* text)
{
  for (; text[0] != '\0'; text++)
  {
    if (text[0] != ' ' && text[0] != '\f' && text[0] != '\n' &&
        text[0] != '\t' && text[0] != '\v')
    {
      if (text[0] == '*')
        return 1;
      else
        return 0;
    }
  }
  return 1;
}

char*
fgets0(char* buf, int s, FILE* f)
{
  size_t numreadtotal = 0, numread = 0;
  int    i = 0;
  char   c;

  while (i < s - 1)
  {
    numread = fread(&c, 1, 1, f);

    if (numread != 1)
      break;

    numreadtotal += numread;

    if (c != '\r')
      buf[i++] = c;

    if (c == '\n')
    {
      break;
    }
  }

  buf[i++] = '\0';

  if (numreadtotal)
    return buf;
  else
    return NULL;
}

char*
getnextline(char* buf, int s, FILE* f, int* line)
{
  char* p;
  int   l = 0;
  do
  {
    p = fgets0(buf, s, f);
    line[0]++;
  } while (p && isemptyline(buf));

  if (p == NULL || isemptyline(p))
    return (NULL);
  else
    return (p);
}

int
parseint(char* buf, int* val)
{
  char* end;

  val[0] = (int)strtol(buf, &end, 10);

  if (isemptyline(end))
    return (0);
  else
    return (1);
}

int
parsedbl(char* buf, double* val)
{
  char* end;

  val[0] = strtod(buf, &end);

  if (isemptyline(end))
    return (0);
  else
    return (1);
}

int
parsetriple(char* buf, int* val1, int* val2, double* val3)
{
  char* end;

  val1[0] = (int)strtol(buf, &end, 10);

  if (end[0] != ' ' && end[0] != '\t' && end[0] != '\v')
  {
    return (1);
  }

  val2[0] = (int)strtol(end, &end, 10);

  if (end[0] != ' ' && end[0] != '\t' && end[0] != '\v')
  {
    return (1);
  }

  val3[0] = strtod(end, &end);

  if (isemptyline(end))
    return (0);
  else
    return (1);
}

MSKrescodee MSK_expoptread(MSKenv_t env, const char* filename,
                           MSKint32t* numcon, MSKint32t* numvar,
                           MSKint32t* numter, MSKint32t** subi, double** c,
                           MSKint32t** subk, MSKint32t** subj, double** akj,
                           MSKint32t* numanz) /* Length of akj */

/* Purpose:
           Read a geometric optimization problem on the exponential
           optimization form from file filename.  The function allocates the
           arrays subi[0],c,subk[0],subj[0] and akj[0] and it is the users
           responsibility to free them with MSK_free after use.
 */
{
  MSKrescodee r = MSK_RES_OK;
  char        buf[MAX_LINE_LENGTH];
  char        buf2[MAX_LINE_LENGTH];
  FILE*       f;
  int         line = 0;
  MSKidxt     i;
  long        fpos;
  MSKintt     itmp, itmp2;
  double      rtmp;

  subi[0] = NULL;
  subj[0] = NULL;
  subk[0] = NULL;
  akj[0]  = NULL;
  c[0]    = NULL;

  f = fopen(filename, "rb");

  if (!f)
  {
    printf("Could not open file '%s'\n", filename);
    r = MSK_RES_ERR_FILE_OPEN;
  }
  else
  {
    if (r == MSK_RES_OK)
      getnextline(buf, sizeof(buf), f, &line);

    if (r == MSK_RES_OK && (parseint(buf, numcon) != 0))
    {
      printf("Syntax error in '%s' line %d.\n", filename, line);
      r = MSK_RES_ERR_FILE_READ;
    }

    if (r == MSK_RES_OK)
      getnextline(buf, sizeof(buf), f, &line);

    if (r == MSK_RES_OK && (parseint(buf, numvar) != 0))
    {
      printf("Syntax error in '%s' line %d.\n", filename, line);
      r = MSK_RES_ERR_FILE_READ;
    }

    if (r == MSK_RES_OK && getnextline(buf, sizeof(buf), f, &line) == NULL)
    {
      printf("Syntax error: Unexpected EOF in '%s' line %d.\n", filename, line);
      r = MSK_RES_ERR_FILE_READ;
    }

    if (r == MSK_RES_OK && (parseint(buf, numter) != 0))
    {
      printf("Syntax error in '%s' line %d.\n", filename, line);
      r = MSK_RES_ERR_FILE_READ;
    }

    if (r == MSK_RES_OK)
    {
      c[0] = (double*)MSK_callocenv(env, numter[0], sizeof(double));
      if (c[0] == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    if (r == MSK_RES_OK)
    {
      subi[0] = (MSKint32t*)MSK_callocenv(env, numter[0], sizeof(double));
      if (subi[0] == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    /* read coef for terms */
    for (i = 0; r == MSK_RES_OK && i < numter[0]; ++i)
    {
      if (getnextline(buf, sizeof(buf), f, &line) == NULL)
      {
        printf("Syntax error: Unexpected EOF in '%s' line %d.\n", filename,
               line);
        r = MSK_RES_ERR_FILE_READ;
      }

      if (r == MSK_RES_OK && parsedbl(buf, c[0] + i) != 0)
      {
        printf("Syntax error in '%s' line %d.\n", filename, line);
        r = MSK_RES_ERR_FILE_READ;
      }

      if (c[0][i] == 0.0)
      {
        printf(
          "In '%s' line %d: Coeficients with value zero is not allaowed.\n",
          filename, line);
        r = MSK_RES_ERR_FILE_READ;
      }
    }

    /* read constraint index of terms */

    for (i = 0; r == MSK_RES_OK && i < numter[0]; ++i)
    {

      if (getnextline(buf, sizeof(buf), f, &line) == NULL)
      {
        printf("Syntax error: Unexpected EOF in '%s' line %d.\n", filename,
               line);
        r = MSK_RES_ERR_FILE_READ;
      }

      if (r == MSK_RES_OK && parseint(buf, subi[0] + i) != 0)
      {
        printf("Syntax error on line %d. Constraint index expected.\n", line);
        r = MSK_RES_ERR_FILE_READ;
      }

      if (subi[0][i] > numcon[0])
      {
        printf("The index subi[%d] = %d is to large (%d) in '%s' line %d\n", i,
               subi[0][i], numcon[0], filename, line);
        r = MSK_RES_ERR_FILE_READ;
      }
    }

    /* Estimate number of nz coefficients */

    fpos = ftell(f);

    numanz[0] = 0;
    itmp2     = line;
    while (r == MSK_RES_OK)
    {
      char* ret;

      ret = getnextline(buf, sizeof(buf), f, &itmp2);

      if (!ret)
        break;

      if (parsetriple(buf, &itmp, &itmp, &rtmp) == 0)
      {
        numanz[0]++;
      }
      else
      {
        printf("Syntax error on line %d.\n", itmp2);
        r = MSK_RES_ERR_FILE_READ;
      }
    }

    fseek(f, fpos, SEEK_SET);

    if (r == MSK_RES_OK)
    {
      akj[0] = (double*)MSK_callocenv(env, numanz[0], sizeof(double));
      if (akj[0] == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    if (r == MSK_RES_OK)
    {
      subk[0] = (MSKint32t*)MSK_callocenv(env, numanz[0], sizeof(int));
      if (subk[0] == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    if (r == MSK_RES_OK)
    {
      subj[0] = (MSKint32t*)MSK_callocenv(env, numanz[0], sizeof(int));
      if (subj[0] == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    /* read coefficients */
    for (i = 0; i < numanz[0] && r == MSK_RES_OK; ++i)
    {
      getnextline(buf, sizeof(buf), f, &line);

      if (parsetriple(buf, subk[0] + i, subj[0] + i, akj[0] + i) != 0)
      {
        printf("Syntax error on line %d.\n", line);
        r = MSK_RES_ERR_FILE_READ;
      }

      if (subk[0][i] >= numter[0])
      {
        printf("The index subk[%d] = %d is to large in line %d\n", i,
               subk[0][i], line);
        r = MSK_RES_ERR_FILE_READ;
      }

      if (subj[0][i] >= numvar[0])
      {
        printf("The index subj[%d] = %d is to large (> %d) in line %d\n", i,
               subj[0][i], numvar[0], line);
        r = MSK_RES_ERR_FILE_READ;
      }
    }
  }
  return (r);
}

MSKrescodee
MSK_expoptwrite(MSKenv_t env, const char* filename, MSKint32t numcon,
                MSKint32t numvar, MSKint32t numter, MSKint32t* subi, double* c,
                MSKint32t* subk, MSKint32t* subj, double* akj, MSKint32t numanz)
{
  MSKrescodee r = MSK_RES_OK;
  FILE*       f;
  MSKint32t   i;

  f = fopen(filename, "wt");

  if (f)
  {
    fprintf(f, "%d\n", numcon);
    fprintf(f, "%d\n", numvar);
    fprintf(f, "%d\n", numter);

    for (i = 0; i < numter; ++i)
      fprintf(f, "%e\n", c[i]);

    for (i = 0; i < numter; ++i)
      fprintf(f, "%d\n", subi[i]);

    for (i = 0; i < numanz; ++i)
      fprintf(f, "%d %d %e\n", subk[i], subj[i], akj[i]);
  }
  else
  {
    printf("Could not open file '%s'\n", filename);
    r = MSK_RES_ERR_FILE_OPEN;
  }

  fclose(f);

  return (r);
}

/*
 *
MSK_expoptsetup¶
MSKrescodee MSK_expoptsetup(
    MSKtask_t     expopttask,
    MSKint32t     solveform,
    MSKint32t     numcon,
    MSKint32t     numvar,
    MSKint32t     numter,
    MSKidxt       *subi,
    double        *c,
    MSKidxt       *subk,
    MSKidxt       *subj,
    double        *akj,
    MSKint32t     numanz,
    expopthand_t  *expopthnd)
Sets up an exponential optimization problem.

Parameters:

expopttask (MSKtask_t) – The optimization task.
solveform (MSKint32t) – If 0 solver is chosen freely, 1: solve dual, -1: solve
primal
numcon (MSKint32t) – Number of constraints
numvar (MSKint32t) – Number of variables
numter (MSKint32t) – Number of exponential terms
subi (MSKint32t*) – The constraint where the term belongs. Zero denotes the
objective.
c (double* ) – The c coefficients of nonlinear terms.
subk (MSKint32t*) – Term indices.
subj (MSKint32t*) – Variable indices.
akj (double* ) – akj[i] is coefficient of variable subj[i] in term subk[i], i.e.
asubk[i],subj[i]=akj[i].
numanz (MSKint32t) – Number of linear terms in the exponents, i.e. the length of
subk, subj and akj.
expopthnd (void**) – Data structure containing the nonlinear information.
Return:

(MSKrescodee) – The function response code.
 * */

MSKrescodee MSK_expoptsetup(
  MSKtask_t expopttask, MSKint32t solveform, MSKint32t numcon____,
  MSKint32t numvar, MSKint32t numter, MSKint32t* subi, double* c,
  MSKint32t* subk, MSKint32t* subj, double* akj, MSKint32t numanz,
  expopthand_t* expopthnd) /* Data structure containing nonlinear information */

/* Purpose: Setup problem in expopttask.  For every call to expoptsetup there
            must be a corresponding call to expoptfree to dealocate data.
  */
{
  MSKrescodee r = MSK_RES_OK;

#if DEBUG > 0
  printf("**numvar = %d\n", numvar);
  printf("**numcon = %d\n", numcon);
  printf("**numter = %d\n", numter);
#endif

  expopthnd[0] = (expopthand_t)MSK_calloctask(expopttask, 1, sizeof(nlhandt));

  if (expopthnd[0])
  {
    MSKint32t  i, k, itmp, itmp2;
    MSKint32t  numobjterm, numconterm, *nter_per_con = NULL;
    MSKint32t *opro = NULL, *oprjo = NULL;
    double *   oprfo = NULL, *oprgo = NULL, *oprho = NULL;
    MSKint32t  numopro, numoprc, *oprc = NULL, *opric = NULL, *oprjc = NULL,
                                *ibuf = NULL;
    double * oprfc = NULL, *oprgc = NULL, *oprhc = NULL, *rbuf = NULL;
    nlhand_t nlh = (nlhand_t)expopthnd[0];

    nlh->solveform = solveform;
    nlh->numvar    = numvar;
    nlh->nl_data   = NULL;

    /* clear expopttask */
    {
      MSKidxt* delsub = NULL;
      MSKintt  delsublen;
      if (r == MSK_RES_OK)
        r = MSK_putnlfunc(expopttask, NULL, NULL, NULL);
      if (r == MSK_RES_OK)
        r = MSK_getnumvar(expopttask, &itmp);

      if (r == MSK_RES_OK)
        r = MSK_getnumcon(expopttask, &itmp2);

      delsublen = itmp < itmp2 ? itmp2 : itmp;

      if (delsublen)
      {
        if (r == MSK_RES_OK)
        {
          delsub =
            (MSKint32t*)MSK_calloctask(expopttask, delsublen, sizeof(MSKidxt));
          if (delsub == NULL)
            r = MSK_RES_ERR_SPACE;
        }

        for (i      = 0; i < delsublen && r == MSK_RES_OK; ++i)
          delsub[i] = i;

        if (r == MSK_RES_OK)
          r = MSK_removevars(expopttask, itmp, delsub);

        if (r == MSK_RES_OK)
          r = MSK_removecons(expopttask, itmp2, delsub);

        MSK_freetask(expopttask, delsub);
      }
    }

    for (i = 0; i < numter && r == MSK_RES_OK; ++i)
    {
      if (subi[i] > numcon____)
      {
        printf("The index subi[%d] = %d is to large\n", i, subi[i]);
        r = MSK_RES_ERR_INDEX_IS_TOO_LARGE;
      }

      if (subi[i] < 0 && r == MSK_RES_OK)
      {
        printf("The index subi[%d] = %d is negative\n", i, subi[i]);
        r = MSK_RES_ERR_INDEX_IS_TOO_SMALL;
      }
    }

    for (i = 0; i < numanz; ++i && r == MSK_RES_OK)
    {
      if (subj[i] >= numvar)
      {
        printf("The index subj[%d] = %d is to large\n", i, subj[i]);
        r = MSK_RES_ERR_INDEX_IS_TOO_LARGE;
      }

      if (subj[i] < 0 && r == MSK_RES_OK)
      {
        printf("The index subj[%d] = %d is negative\n", i, subj[i]);
        r = MSK_RES_ERR_INDEX_IS_TOO_SMALL;
      }
    }

    for (i = 0; i < numter && r == MSK_RES_OK; ++i)
    {
      if (c[i] <= 0.0)
      {
        printf(
          "The coefficient c[%d] <= 0. Only positive coefficients allowed\n",
          i);
        r = MSK_RES_ERR_UNKNOWN;
      }
    }

    numobjterm = 0;
    for (i = 0; i < numter; ++i)
      if (subi[i] == 0)
        ++numobjterm;

    if (r == MSK_RES_OK)
    {
      rbuf = (double*)MSK_calloctask(expopttask, numvar + 1, sizeof(double));
      if (rbuf == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    if (r == MSK_RES_OK)
    {
      ibuf = (MSKint32t*)MSK_calloctask(expopttask, numvar + 1, sizeof(double));
      if (ibuf == NULL)
        r = MSK_RES_ERR_SPACE;
    }

    for (i    = 0; i < numvar && r == MSK_RES_OK; ++i)
      ibuf[i] = 0;

    for (i = 0; i < numanz && r == MSK_RES_OK; ++i)
      if (akj[i] < 0.0)
        ibuf[subj[i]] = 1;

    for (i = 0; i < numvar && r == MSK_RES_OK; ++i)
      if (!ibuf[i])
      {
        printf("Warning: The variable with index '%d' has only positive "
               "coefficients akj.\n The problem is possibly ill-posed.\n.\n",
               i);
      }

    for (i    = 0; i < numvar && r == MSK_RES_OK; ++i)
      ibuf[i] = 0;

    for (i = 0; i < numanz && r == MSK_RES_OK; ++i)
      if (akj[i] > 0.0)
        ibuf[subj[i]] = 1;

    for (i = 0; i < numvar && r == MSK_RES_OK; ++i)
      if (!ibuf[i])
      {
        printf("Warning: The variable with index '%d' has only negative "
               "coefficients akj.\n The problem is possibly ill-posed.\n",
               i);
      }

    MSK_checkmemtask(expopttask, __FILE__, __LINE__);

    /* Sort subk,subj,akj increasingly according to subk */

    if (r == MSK_RES_OK)
    {
      MSKintt* displist    = NULL;
      MSKidxt *subk_sorted = NULL, *subj_sorted = NULL;
      double*  akj_sorted = NULL;

      if (r == MSK_RES_OK)
      {
        displist =
          (MSKintt*)MSK_calloctask(expopttask, numter + 1, sizeof(MSKintt));
        if (displist == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        subk_sorted =
          (MSKidxt*)MSK_calloctask(expopttask, numanz, sizeof(MSKidxt));
        if (subk_sorted == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        subj_sorted =
          (MSKidxt*)MSK_calloctask(expopttask, numanz, sizeof(MSKidxt));
        if (subj_sorted == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        akj_sorted =
          (double*)MSK_calloctask(expopttask, numanz, sizeof(double));
        if (akj_sorted == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        for (i        = 0; i < numter + 1; ++i)
          displist[i] = 0;

        for (i = 0; i < numanz; ++i)
          displist[subk[i] + 1]++;

        for (i            = 0; i < numter - 1; ++i)
          displist[i + 1] = displist[i] + displist[i + 1];

        for (i = 0; i < numanz; ++i)
        {
          int pos          = displist[subk[i]]++;
          subk_sorted[pos] = subk[i];
          subj_sorted[pos] = subj[i];
          akj_sorted[pos]  = akj[i];
        }

        subk = subk_sorted;
        subj = subj_sorted;
        akj  = akj_sorted;
      }

      /* Detect duplicates in subj*/
      itmp = 0;
      for (i    = 0; i < numvar; ++i)
        ibuf[i] = 0;

      while (itmp < numanz && r == MSK_RES_OK)
      {
        MSKidxt curterm = subk[itmp], begin;

        begin = itmp;
        while ((itmp < numanz) && (subk[itmp] == curterm) && r == MSK_RES_OK)
        {
          if (ibuf[subj[itmp]]++)
          {
            printf("Duplicate variable index in term '%d'. For a given term "
                   "only one variable index subj[k] is allowed.\n",
                   curterm);
            r = MSK_RES_ERR_UNKNOWN;
          }
          itmp++;
        }

        itmp = begin;

        while ((itmp < numanz) && (subk[itmp] == curterm) && r == MSK_RES_OK)
        {
          ibuf[subj[itmp]] = 0;
          itmp++;
        }
      }

      MSK_freetask(expopttask, displist);
    }

    if (solveform >= 0) /* If the dual formulation was chosen */
    {
      MSKidxt *p = NULL, *displist = NULL, *pos = NULL;
      double*  v = NULL;
      if (r == MSK_RES_OK)
      {
        p = (MSKint32t*)MSK_calloctask(expopttask, numcon____ + 1,
                                       sizeof(MSKidxt));
        if (p == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        displist = (MSKint32t*)MSK_calloctask(expopttask, numcon____ + 1,
                                              sizeof(MSKidxt));
        if (displist == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        pos = (MSKint32t*)MSK_calloctask(expopttask, numter, sizeof(MSKidxt));
        if (pos == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        v = (double*)MSK_calloctask(expopttask, numter, sizeof(double));
        if (pos == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
        r = MSK_appendvars(expopttask, numter);

      if (r == MSK_RES_OK)
        r = MSK_appendcons(expopttask, numvar + 1);

      if (r == MSK_RES_OK)
      {
        /* Count number of therm in each constraint */
        for (i = 0; i < numcon____ + 1; ++i)
          p[i] = 0;

        for (i = 0; i < numter; ++i)
          p[subi[i]] += 1;

        /* Find order pos of subi sorted increasingly */

        displist[0] = 0;
        for (i        = 1; i < numcon____ + 1 && r == MSK_RES_OK; ++i)
          displist[i] = displist[i - 1] + p[i - 1];

        for (i   = 0; i < numter && r == MSK_RES_OK; ++i)
          pos[i] = displist[subi[i]]++;

        for (i      = 0; i < numter && r == MSK_RES_OK; ++i)
          v[pos[i]] = c[i];

        itmp = 0;
      }

      while (itmp < numanz && r == MSK_RES_OK)
      {
        MSKidxt curterm = subk[itmp];
        MSKintt nz      = 0;
        while ((itmp < numanz) && (subk[itmp] == curterm))
        {
          ibuf[nz] = subj[itmp];
          rbuf[nz] = akj[itmp];
          nz++;
          itmp++;
        }

        if (subi[curterm] == 0 && r == MSK_RES_OK) /* in objective */
        {
          ibuf[nz] = numvar;
          rbuf[nz] = 1.0;
          nz++;
        }

        if (r == MSK_RES_OK)
          r = MSK_putacol(expopttask, pos[curterm], nz, ibuf, rbuf);
      }

      for (i = 0; i < numter && r == MSK_RES_OK; ++i)
        r = MSK_putvarbound(expopttask, i, MSK_BK_LO, 0, MSK_INFINITY);

      for (i = 0; i < numvar && r == MSK_RES_OK; ++i)
        r = MSK_putconbound(expopttask, i, MSK_BK_FX, 0, 0);

      if (r == MSK_RES_OK)
        r = MSK_putconbound(expopttask, numvar, MSK_BK_FX, 1.0, 1.0);

#if DEBUG > 0
      /* write linear part */
      MSK_putintparam(expopttask, MSK_IPAR_WRITE_GENERIC_NAMES, MSK_ON);
      MSK_writedata(expopttask, "lp_part_dual_formulation.lp");
#endif

      if (r == MSK_RES_OK)
        r = MSK_dgosetup(expopttask, numter, numvar, numcon____ + 1, v, p,
                         &(nlh->nl_data));

      MSK_freetask(expopttask, p);
      MSK_freetask(expopttask, displist);
      MSK_freetask(expopttask, pos);
      MSK_freetask(expopttask, v);
    }
    else /* If the primal formulation was chosen */
    {
      if (r == MSK_RES_OK)
      {
        opro =
          (MSKint32t*)MSK_calloctask(expopttask, numobjterm, sizeof(MSKintt));
        if (opro == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        oprjo =
          (MSKint32t*)MSK_calloctask(expopttask, numobjterm, sizeof(double));
        if (oprjo == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        oprfo = (double*)MSK_calloctask(expopttask, numobjterm, sizeof(double));
        if (oprfo == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        oprgo = (double*)MSK_calloctask(expopttask, numobjterm, sizeof(double));
        if (oprgo == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        oprho = (double*)MSK_calloctask(expopttask, numobjterm, sizeof(double));
        if (oprho == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      if (r == MSK_RES_OK)
      {
        nter_per_con = (MSKint32t*)MSK_calloctask(expopttask, numcon____ + 1,
                                                  sizeof(MSKintt));
        if (nter_per_con == NULL)
          r = MSK_RES_ERR_SPACE;
      }

      MSK_checkmemtask(expopttask, __FILE__, __LINE__);

      if (r == MSK_RES_OK)
      {

        for (i            = 0; i < numcon____ + 1; ++i)
          nter_per_con[i] = 0;

        /* Setup nonlinear objective */

        for (i = 0; i < numter; ++i)
          nter_per_con[subi[i]]++;

        for (i = 0, k = 0; i < numter && r == MSK_RES_OK; ++i)
        {
          if (subi[i] == 0)
          {
            oprho[k] = 0;
            oprgo[k] = 1.0;
            oprfo[k] = 1.0;
            oprjo[k] = numvar + i;
            opro[k]  = MSK_OPR_EXP;
            ++k;
          }
        }
      }

      numopro    = numobjterm;
      numconterm = numter - numobjterm;

      if (r == MSK_RES_OK)
      {
        oprc =
          (MSKint32t*)MSK_calloctask(expopttask, numconterm, sizeof(MSKintt));
        opric =
          (MSKint32t*)MSK_calloctask(expopttask, numconterm, sizeof(double));
        oprjc =
          (MSKint32t*)MSK_calloctask(expopttask, numconterm, sizeof(double));
        oprfc = (double*)MSK_calloctask(expopttask, numconterm, sizeof(double));
        oprgc = (double*)MSK_calloctask(expopttask, numconterm, sizeof(double));
        oprhc = (double*)MSK_calloctask(expopttask, numconterm, sizeof(double));
      }

      if (oprc == NULL && opric == NULL && oprjc == NULL && oprfc == NULL &&
          oprgc == NULL && oprhc == NULL)
        r = MSK_RES_ERR_SPACE;

      if (r == MSK_RES_OK)
      {
        for (i = 0, k = 0; i < numter && r == MSK_RES_OK; ++i)
        {
          if ((subi[i] != 0) && (nter_per_con[subi[i]] > 1))
          {
            oprc[k]  = MSK_OPR_EXP;
            opric[k] = subi[i] - 1;
            oprjc[k] = numvar + i;
            oprfc[k] = 1.0;
            oprgc[k] = 1.0;
            oprhc[k] = 0.0;
            k++;
          }
        }
        numoprc = k;
      }

      if (r == MSK_RES_OK)
        r = MSK_appendvars(expopttask, numvar + numter);

      if (r == MSK_RES_OK)
        r = MSK_appendcons(expopttask, numcon____ + numter);

      for (i = 0; i < numcon____ && r == MSK_RES_OK; ++i)
      {
        r = MSK_putconbound(expopttask, i, MSK_BK_UP, -MSK_INFINITY, 1);
      }

      for (i = numcon____; i < numcon____ + numter && r == MSK_RES_OK; ++i)
      {
        if ((nter_per_con[subi[i - numcon____]]) > 1 ||
            (subi[i - numcon____] == 0))
        {
          r = MSK_putconbound(expopttask, i, MSK_BK_FX, -log(c[i - numcon____]),
                              -log(c[i - numcon____]));
        }
        else
        {
          r = MSK_putconbound(expopttask, i, MSK_BK_UP, -MSK_INFINITY,
                              -log(c[i - numcon____]));
        }
      }

      for (i = 0; i < numvar && r == MSK_RES_OK; ++i)
      {
        r = MSK_putvarbound(expopttask, i, MSK_BK_FR, -MSK_INFINITY,
                            MSK_INFINITY);
      }

      for (i = numvar; i < numvar + numter && r == MSK_RES_OK; ++i)
      {
        r = MSK_putvarbound(expopttask, i, MSK_BK_FR, -MSK_INFINITY,
                            MSK_INFINITY);
      }

      MSK_checkmemtask(expopttask, __FILE__, __LINE__);

      itmp = 0;
      {
        MSKidxt termindex;

        for (termindex = 0; termindex < numter; ++termindex)
        {
          if (subk[itmp] != termindex)
          {
            MSKintt nz = 0;
            ibuf[nz]   = numvar + termindex; /* add v_t */
            rbuf[nz]   = -1.0;
            nz++;

            if (r == MSK_RES_OK)
              r =
                MSK_putarow(expopttask, termindex + numcon____, nz, ibuf, rbuf);

            if (r == MSK_RES_OK)
              r = MSK_putconbound(expopttask, termindex + numcon____, MSK_BK_FX,
                                  -log(c[termindex]), -log(c[termindex]));
          }
          else
          {
            MSKintt nz = 0;

            while ((itmp < numanz) && (subk[itmp] == termindex))
            {
              ibuf[nz] = subj[itmp];
              rbuf[nz] = akj[itmp];
              nz++;
              itmp++;
            }

            if ((nter_per_con[subi[termindex]] > 1) || subi[termindex] == 0)
            {
              ibuf[nz] = numvar + termindex; /* add v_t */
              rbuf[nz] = -1.0;
              nz++;

              if (r == MSK_RES_OK)
                r =
                  MSK_putconbound(expopttask, termindex + numcon____, MSK_BK_FX,
                                  -log(c[termindex]), -log(c[termindex]));
            }
            else
            {
              if (r == MSK_RES_OK)
                r =
                  MSK_putconbound(expopttask, termindex + numcon____, MSK_BK_UP,
                                  -MSK_INFINITY, -log(c[termindex]));
            }

            if (r == MSK_RES_OK)
              r =
                MSK_putarow(expopttask, termindex + numcon____, nz, ibuf, rbuf);
          }
        }
      }

      MSK_checkmemtask(expopttask, __FILE__, __LINE__);

      if (r == MSK_RES_OK)
        r = MSK_scbegin(expopttask, numopro, opro, oprjo, oprfo, oprgo, oprho,
                        numoprc, oprc, opric, oprjc, oprfc, oprgc, oprhc,
                        &(nlh->nl_data));

      if (r == MSK_RES_OK)
        r = MSK_putobjsense(expopttask, MSK_OBJECTIVE_SENSE_MINIMIZE);

      MSK_checkmemtask(expopttask, __FILE__, __LINE__);

#if WRITE_AS_SCOPT
      MSK_putintparam(expopttask, MSK_IPAR_WRITE_GENERIC_NAMES, MSK_ON);

      MSK_scwrite(expopttask, nlh->nl_data, "scoptp");
#endif

      MSK_freetask(expopttask, opro);
      MSK_freetask(expopttask, oprjo);
      MSK_freetask(expopttask, oprfo);
      MSK_freetask(expopttask, oprgo);
      MSK_freetask(expopttask, oprho);
      MSK_freetask(expopttask, oprc);
      MSK_freetask(expopttask, oprjc);
      MSK_freetask(expopttask, opric);
      MSK_freetask(expopttask, oprfc);
      MSK_freetask(expopttask, oprgc);
      MSK_freetask(expopttask, oprhc);
    }

    MSK_freetask(expopttask, rbuf);
    MSK_freetask(expopttask, ibuf);
    MSK_freetask(expopttask, subk);
    MSK_freetask(expopttask, subj);
    MSK_freetask(expopttask, akj);
    MSK_freetask(expopttask, nter_per_con);
  }
  else
    r = MSK_RES_ERR_SPACE;

  return (r);
}

MSKrescodee
MSK_expoptimize(MSKtask_t expopttask, MSKprostae* prosta, MSKsolstae* solsta,
                double* objval, double* xx, double* y, expopthand_t* expopthnd)

/* Purpose:
           Solve the problem. The primal solution is returned in xx.
*/
{
  MSKrescodee r = MSK_RES_OK;
  nlhand_t    nlh;
  MSKidxt     i;
  nlh = (nlhand_t)expopthnd[0];

#if DEBUG > 0
  /* write linear part */
  MSK_putintparam(expopttask, MSK_IPAR_WRITE_GENERIC_NAMES, MSK_ON);
  MSK_writedata(expopttask, "lp_part_dual_formulation.lp");
#endif

  if (nlh->solveform == 1)
  {
    MSK_echotask(
      expopttask, MSK_STREAM_MSG,
      "* Solving exponential optimization problem on dual form. *\n");
    MSK_echotask(expopttask, MSK_STREAM_MSG, "* The following log information "
                                             "refers to the solution of the "
                                             "dual problem. *\n");
  }
  else
  {
    MSK_echotask(
      expopttask, MSK_STREAM_MSG,
      "* Solving exponential optimization problem on primal form. *\n");
  }

  if (r == MSK_RES_OK)
  {
    r = MSK_optimize(expopttask);
  }

  if (r == MSK_RES_OK)
    r = MSK_getsolution(expopttask, MSK_SOL_ITR, prosta, solsta, NULL, NULL,
                        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
  if (r == MSK_RES_OK)
    printf("solsta = %d, prosta = %d\n", (int)*solsta, (int)*prosta);

  if (r != MSK_RES_OK)
  {
    MSK_echotask(expopttask, MSK_STREAM_MSG, "Return code from optimize - %d\n",
                 r);
  }

  MSK_solutionsummary(expopttask, MSK_STREAM_MSG);

#if DEBUG > 0
  MSK_solutionsummary(expopttask, MSK_STREAM_MSG);
  MSK_writesolution(expopttask, MSK_SOL_ITR, "expoptsol.itr");
#endif
  if (nlh->solveform == 1)
  {
    int numvar_dual;
    /* transform to primal solution */
    MSK_echotask(expopttask, MSK_STREAM_MSG,
                 "* End solution on dual form. *\n");
    MSK_echotask(expopttask, MSK_STREAM_MSG,
                 "Transforming to primal solution.\n");

    if (r == MSK_RES_OK)
      r = MSK_getsolutionslice(expopttask, MSK_SOL_ITR, MSK_SOL_ITEM_Y, 0,
                               nlh->numvar, xx);

    if (r == MSK_RES_OK)
      r = MSK_getprimalobj(expopttask, MSK_SOL_ITR, objval);

    objval[0] = exp(objval[0]);

    for (i  = 0; i < nlh->numvar; ++i)
      xx[i] = -xx[i];

    /* Get dual sol vars */

    if (r == MSK_RES_OK)
      r = MSK_getnumvar(expopttask, &numvar_dual);

    if (r == MSK_RES_OK)
      r = MSK_getsolutionslice(expopttask, MSK_SOL_ITR, MSK_SOL_ITEM_XX, 0,
                               numvar_dual, y);
  }
  else
  {
    if (r == MSK_RES_OK)
      r = MSK_getsolutionslice(expopttask, MSK_SOL_ITR, MSK_SOL_ITEM_XX, 0,
                               nlh->numvar, xx);

    if (r == MSK_RES_OK)
      r = MSK_getprimalobj(expopttask, MSK_SOL_ITR, objval);
  }

  return (r);
}

MSKrescodee
MSK_expoptfree(MSKtask_t expopttask, expopthand_t* expopthnd)
{
  /* Purpose: Free data allocated by expoptsetup. For every call
              to expoptsetup there must be exactly one call to expoptfree.
   */
  MSKrescodee r   = MSK_RES_OK;
  nlhand_t*   nlh = (nlhand_t*)expopthnd;

  if (nlh[0] != NULL)
  {
    if (nlh[0]->nl_data != NULL)
    {
      if (nlh[0]->solveform < 0)
        r = MSK_scend(expopttask, &(nlh[0]->nl_data));
      else
        r = MSK_freedgo(expopttask, &(nlh[0]->nl_data));
    }

    MSK_freetask(expopttask, nlh[0]);
    nlh[0] = NULL;
  }

  return (r);
}
