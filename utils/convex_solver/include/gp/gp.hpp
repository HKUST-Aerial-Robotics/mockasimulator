#ifndef GP_HPP
#define GP_HPP

#include "mosek/fusion.h"
#include "mosek/mosek.h"

#include <vector>

typedef void* expopthand_t;

//! @todo grabadge

#define MSK_OPR_ENT 0
#define MSK_OPR_EXP 1
#define MSK_OPR_LOG 2
#define MSK_OPR_POW 3
#define MSK_OPR_SQRT 4
/* constant for square root operator:  f * sqrt(gx + h) */

typedef void* schand_t;

MSKrescodee MSK_scbegin(MSKtask_t task, int numopro, int* opro, int* oprjo,
                        double* oprfo, double* oprgo, double* oprho,
                        int numoprc, int* oprc, int* opric, int* oprjc,
                        double* oprfc, double* oprgc, double* oprhc,
                        schand_t* sch);
/* Purpose: The function is used to feed a nonlinear
            separable function to MOSEK. The procedure must
            called before MSK_optimize is invoked.
 */

MSKrescodee MSK_scwrite(MSKtask_t task, schand_t sch, char filename[]);
/* Purpose: Writes two data files which specifies the problem. One named
            filename.mps and the other is filename.sco.
 */

MSKrescodee MSK_scread(MSKtask_t task, schand_t* sch, char filename[]);
/* Purpose: Read the data files created by MSK_scwrite.
 */

MSKrescodee MSK_scend(MSKtask_t task, schand_t* sch);
/* Purpose: When the nonlinear function data is no longer needed or
            should be changed, then this procedure should be called
            to deallocate previous allocated data in MSK_scbegin.
 */

MSKrescodee MSK_expoptread(
  MSKenv_t env, const char* filename, MSKint32t* numcon, MSKint32t* numvar,
  MSKint32t* numter,
  MSKidxt** subi, /* Which constraint a term belongs to or zero for objective */
  double**  c,    /* Coefficients of terms */
  MSKidxt** subk, /* Term index */
  MSKidxt** subj, /* Variable index */
  double**  akj, /* akj[i] is coefficient of variable subj[i] in term subk[i] */
  MSKint32t* numanz); /* Length of akj */
                      /* Purpose:
                                 Read a geometric optimization problem on the exponential
                                 optimization form from file filename.  The function allocates the
                                 arrays subi[0],c,subk[0],subj[0] and akj[0] and it is the users
                                 responsibility to free them with MSK_free after use.
                      */

MSKrescodee MSK_expoptwrite(MSKenv_t env, const char* filename,
                            MSKint32t numcon, MSKint32t numvar,
                            MSKint32t numter, MSKidxt* subi, double* c,
                            MSKidxt* subk, MSKidxt* subj, double* akj,
                            MSKint32t numanz);

/* Purpose:
           Write an exponential optimization problem to a file.
*/

MSKrescodee MSK_expoptsetup(
  MSKtask_t expopttask,
  MSKint32t solveform, /* If 0 solver is chosen freely, 1: solve by dual
                          formulation, -1: solve by primal formulation */
  MSKint32t numcon____, MSKint32t numvar, MSKint32t numter, MSKidxt* subi,
  double* c, MSKidxt* subk, MSKidxt* subj, double* akj, MSKint32t numanz,
  expopthand_t*
    expopthnd); /* Data structure containing nonlinear information */

/* Purpose: Setup problem in expopttask.  For every call to expoptsetup there
            must be a corresponding call to expoptfree to dealocate data.
*/

MSKrescodee MSK_expoptimize(
  MSKtask_t expopttask, MSKprostae* prosta, MSKsolstae* solsta,
  double*       objval, /* Primal solution value */
  double*       xx,     /* Primal solution */
  double*       y, /* Dual solution. Only given when solving on dual form */
  expopthand_t* expopthnd);

/* Purpose:
            Solve the problem. The primal solution is returned in xx.
*/

MSKrescodee MSK_expoptfree(MSKtask_t expopttask, expopthand_t* expopthnd);

/* Purpose:
            Free data allocated by expoptsetup. For every call
            to expoptsetup there must be exactly one call to expoptfree.
*/

/*
   Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.

   File:      expopt.c

   Purpose:   Solve the exponential optimization problem.
*/
#define SCALE 1
#define DEBUG 0
#define WRITE_AS_SCOPT 0

#include <math.h>
#include <stdio.h>

typedef struct
{

  int       solveform;
  MSKint32t numvar;
  void*     nl_data;

} nlhandt;

typedef nlhandt* nlhand_t;

#define MAX_LINE_LENGTH 256 /* max length of a line in data format is 256 */

typedef void* dgohand_t;

MSKrescodee MSK_dgoread(MSKtask_t task, const char* nldatafile,
                        MSKint32t*  numvar, /* numterms in primal */
                        MSKint32t*  numcon, /* numvar in primal */
                        MSKint32t*  t,      /* number of constraints in primal*/
                        double**    v,      /* coiefients for terms in primal*/
                        MSKint32t** p       /* corresponds to number of terms
                                               in each constraint in the
                                               primal */
                        );

MSKrescodee MSK_dgosetup(MSKtask_t task, MSKint32t numvar, MSKint32t numcon,
                         MSKint32t t, double* v, MSKint32t* p, dgohand_t* nlh);

MSKrescodee MSK_freedgo(MSKtask_t task, dgohand_t* nlh);

static int isemptyline(char* text);

static char* fgets0(char* buf, int s, FILE* f);

static char* getnextline(char* buf, int s, FILE* f, int* line);

static int parseint(char* buf, int* val);

static int parsedbl(char* buf, double* val);

static int parsetriple(char* buf, int* val1, int* val2, double* val3);

MSKrescodee MSK_expoptread(
  MSKenv_t env, const char* filename, MSKint32t* numcon, MSKint32t* numvar,
  MSKint32t* numter,
  MSKidxt** subi, /* Which constraint a term belongs to or zero for objective */
  double**  c,    /* Coefficients of terms */
  MSKidxt** subk, /* Term index */
  MSKidxt** subj, /* Variable index */
  double**  akj, /* akj[i] is coefficient of variable subj[i] in term subk[i] */
  MSKint32t* numanz);

MSKrescodee MSK_expoptwrite(MSKenv_t env, const char* filename,
                            MSKint32t numcon, MSKint32t numvar,
                            MSKint32t numter, MSKidxt* subi, double* c,
                            MSKidxt* subk, MSKidxt* subj, double* akj,
                            MSKint32t numanz);

MSKrescodee MSK_expoptsetup(
  MSKtask_t expopttask,
  MSKint32t solveform, /* If 1 solve by dual formulation */
  MSKint32t numcon, MSKint32t numvar, MSKint32t numter, MSKidxt* subi,
  double* c, MSKidxt* subk, MSKidxt* subj, double* akj, MSKint32t numanz,
  expopthand_t* expopthnd); /* MSK_expoptsetup*/

MSKrescodee MSK_expoptimize(MSKtask_t expopttask, MSKprostae* prosta,
                            MSKsolstae* solsta,
                            double*     objval, /* Primal solution value */
                            double*     xx,     /* Primal solution */
                            double* y, /* Dual solution, this is ONLY supplied
                                          when solving on dual form */
                            expopthand_t* expopthnd);

MSKrescodee MSK_expoptfree(MSKtask_t expopttask, expopthand_t* expopthnd);

class SeparabelConvexOptimization
{
public:
  //! @todo for primal solver
  SeparabelConvexOptimization();

}; // SeparabelConvexOptimization

class DualGeometricOptimization
{
public:
  //! @todo for dual solver
  DualGeometricOptimization();

}; // DualGeometricOptimization

class ExponentialOptimization
{
public:
  ExponentialOptimization();

public:
  void solve();

private:
  std::vector<int>    subi;
  std::vector<int>    subk;
  std::vector<double> c;
  std::vector<int>    subj;
  std::vector<double> akj;
  int                 numanz;
  double              objval;
  std::vector<double> xx;
  std::vector<double> y;

  MSKenv_t   env;
  MSKprostae prosta;
  MSKsolstae solsta;
  MSKtask_t  expottask;
}; // ExponentialOptimization

class Monomial
{
public:
  Monomial();

private:
  std::vector<double> coeff;
  std::vector<int>    varIndex;
}; // Monomial

class Posynomial
{
public:
  Posynomial();

private:
  std::vector<Monomial> coeff;
}; // Posynomials

class GeometricProgramming
{
public:
  GeometricProgramming();

public:
  void solve();

public:
  void setObjectiveFunction(const Posynomial& posynomial);
  void addInequalityConstrain(const Posynomial& posynomial);
  void addEqualityConstrian(const Monomial& monomial);

public:
  // getters
  double getValue(const int& index);
  std::vector<double> getValue(const std::vector<int>& indexes);

public:
private:
  void setup();
  void evaluate();

private:
  // problem data
  Posynomial              f0;
  std::vector<Posynomial> fi;
  std::vector<Monomial>   gi;

  int totalVar;

private:
  // solver data
};

#endif // GP_HPP
