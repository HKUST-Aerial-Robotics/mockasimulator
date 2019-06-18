#include "mosek/mosektask.h"
#include "mosek/monty_iterator.h"
#include "mosektask_p.h"

#include <iostream>
#define MSK(name, ...)                                                         \
  if (MSK_RES_OK != MSK_##name(task, __VA_ARGS__))                             \
  throw MosekException(task)

namespace mosek
{

MSKenv_t Task::env;
int      Task::env_refc;
bool     Task::env_release;

std::mutex Task::env_lock;

MSKenv_t
Task::getEnv(bool borrow)
{
  std::lock_guard<std::mutex> guard(env_lock);
  if (env == NULL)
  {
    MSKrescodee r = MSK_makeenv(&env, NULL);
    env_refc      = 0;
    env_release   = false;
    if (r != MSK_RES_OK || env == NULL)
    {
      std::stringstream ss;
      ss << r << ": Failed to create global environment";
      throw MosekException(ss.str());
    }
  }
  if (!borrow)
    ++env_refc;
  return env;
}

MSKenv_t
Task::getEnv()
{
  std::lock_guard<std::mutex> guard(env_lock);
  if (env == NULL)
  {
    MSKrescodee r = MSK_makeenv(&env, NULL);
    env_refc      = 0;
    env_release   = false;
    if (r != MSK_RES_OK || env == NULL)
    {
      std::stringstream ss;
      ss << r << ": Failed to create global environment";
      throw MosekException(ss.str());
    }
  }
  ++env_refc;
  return env;
}

void
Task::releaseEnv()
{
  std::lock_guard<std::mutex> guard(env_lock);
  if (env_refc > 0)
  {
    --env_refc;
    if (env_refc == 0 && env_release)
      MSK_deleteenv(&env);
  }
}

void
Task::releaseGlobalEnv()
{
  std::lock_guard<std::mutex> guard(env_lock);
  env_release = true;
  if (env != NULL && env_refc == 0)
    MSK_deleteenv(&env);
}
void
releaseGlobalEnv()
{
  Task::releaseGlobalEnv();
}

void
Task::msghandler(void* handle, const char* msg)
{
  Task* self = (Task*)handle;
  if (self->msguserfunc)
    self->msguserfunc(msg);
}

int
Task::pgshandler(MSKtask_t task, void* handle, MSKcallbackcodee caller,
                 const double* dinf, const int* iinf, const long long* linf)
{
  Task* self = (Task*)handle;

  /*
  if (self->pgsuserfunc)
      if (0 != self->pgsuserfunc(self->pgshandle,caller))
          self->breakflag = true;
  */
  /*
  if (! self->breakflag &&
      self->infuserfunc)
      if (0 != self->infuserfunc(self->infhandle,caller,dinf,iinf,linf))
          self->breakflag = true;
  */
  if (self->datacbuserfunc)
    if (self->datacbuserfunc(caller, dinf, iinf, (int64_t*)linf))
      self->breakflag = 1;

  if (self->cbuserfunc)
    if (self->cbuserfunc(caller))
      self->breakflag = 1;

  return self->breakflag ? 1 : 0;
}

static std::string
extract_task_error(MSKtask_t task)
{
  MSKrescodee lastres;
  MSKint32t   len = 0;
  MSK_getlasterror(task, &lastres, 0, &len, NULL);
  std::vector<char> buf(len + 1);
  MSK_getlasterror(task, &lastres, len, NULL, &buf[0]);
  return std::string(&buf[0]);
}

// mosektask.h
MosekException::MosekException(MSKtask_t task)
  : std::runtime_error(extract_task_error(task).c_str())
{
}

Task::Task()
  : breakflag(false)
  , task(nullptr)
  , msguserfunc(nullptr) /*,
     infuserfunc(nullptr),
     pgsuserfunc(nullptr),
     pgshandle(nullptr),
     infhandle(nullptr)
     */
{
  MSKrescodee r = MSK_maketask(getEnv(), 0, 0, &task);
  if (r != MSK_RES_OK)
  {
    std::stringstream ss;
    ss << r << ": "
       << "Failed to create native task object";
    throw MosekException(ss.str());
  }
  MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, this, msghandler);
  MSK_putcallbackfunc(task, pgshandler, this);
}

Task::Task(Task& other)
  : breakflag(false)
  , task(nullptr)
  , msguserfunc(nullptr) /*,
     infuserfunc(nullptr),
     pgsuserfunc(nullptr),
     pgshandle(nullptr),
     infhandle(nullptr)
     */
{
  MSKrescodee r = MSK_clonetask(other.task, &task);
  if (r != MSK_RES_OK)
  {
    std::stringstream ss;
    ss << r << ": "
       << "Failed to create native task object";
    throw MosekException(ss.str());
  }
  MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, this, msghandler);
  MSK_putcallbackfunc(task, pgshandler, this);
}

Task::~Task()
{
  dispose();
}

void
Task::dispose()
{
  if (task)
  {
    MSK_deletetask(&task);
    releaseEnv();
    task = NULL;
  }
}

MSKtask_t
Task::clone() const
{
  MSKrescodee r = MSK_RES_OK;
  MSKtask_t   t2;
  r = MSK_clonetask(task, &t2);
  if (r != MSK_RES_OK)
  {
    std::stringstream ss;
    ss << r << ": "
       << "Failed to create native task object";
    throw MosekException(ss.str());
  }
  return t2;
}

int
Task::getnumvar() const
{
  int num;
  MSK(getnumvar, &num);
  return num;
}
int
Task::getnumcon() const
{
  int num;
  MSK(getnumcon, &num);
  return num;
}
int
Task::getnumcone() const
{
  int num;
  MSK(getnumcone, &num);
  return num;
}
int
Task::getnumbarvar() const
{
  int num;
  MSK(getnumbarvar, &num);
  return num;
}

int
Task::getbarvardim(int j) const
{
  int dim;
  MSK(getdimbarvarj, j, &dim);
  return dim;
}

int
Task::appendvars(int num)
{
  int idx = getnumvar();
  MSK(appendvars, num);
  return idx;
}
int
Task::appendcons(int num)
{
  int idx = getnumcon();
  MSK(appendcons, num);
  return idx;
}
int
Task::appendbarvar(int dim)
{
  int idx = getnumbarvar();
  MSK(appendbarvars, 1, &dim);
  return idx;
}
int
Task::appendconesseq(MSKconetypee ct, int numcone, int conesize, int firstvar)
{
  const std::vector<MSKconetypee> cts(numcone, ct);
  const std::vector<double>       cps(numcone, 0.0);
  const std::vector<int>          szs(numcone, conesize);
  int                             idx = getnumcone();

  MSK(appendconesseq, numcone, &cts[0], &cps[0], &szs[0], firstvar);
  return idx;
}
int
Task::appendcones(MSKconetypee ct, int numcone, int conesize, int firstvar,
                  int d0, int d1)
{
  int idx = getnumcone();

  std::vector<int> subj(conesize);
  for (int k0 = 0; k0 < d0; ++k0)
  {
    for (int i = 0; i < conesize; i++)
      subj[i]  = d1 * (i + k0 * conesize) + firstvar;
    for (int k1 = 0; k1 < d1; ++k1)
    {
      MSK(appendcone, ct, 0.0, conesize, subj.data());
      for (int j = 0; j < conesize; ++j)
        subj[j] += d1;
    }
  }

  return idx;
}

void
Task::bound(MSKaccmodee acc, int idx, MSKboundkeye bk, double lb, double ub)
{
  MSK(putbound, acc, idx, bk, lb, ub);
}
void
Task::boundslice(MSKaccmodee acc, int first, int last, const MSKboundkeye* bk,
                 const double* lb, const double* ub)
{
  MSK(putboundslice, acc, first, last, bk, lb, ub);
}
void
Task::boundlist(MSKaccmodee acc, size_t num, const int* sub,
                const MSKboundkeye* bk, const double* lb, const double* ub)
{
  MSK(putboundlist, acc, num, sub, bk, lb, ub);
}

void
Task::varname(int idx, const std::string& name)
{
  MSK(putvarname, idx, name.c_str());
}
void
Task::conname(int idx, const std::string& name)
{
  MSK(putconname, idx, name.c_str());
}
void
Task::conename(int idx, const std::string& name)
{
  MSK(putconename, idx, name.c_str());
}
void
Task::barvarname(int idx, const std::string& name)
{
  MSK(putbarvarname, idx, name.c_str());
}
void
Task::objname(const std::string& name)
{
  MSK(putobjname, name.c_str());
}
void
Task::taskname(const std::string& name)
{
  MSK(puttaskname, name.c_str());
}

void
Task::putarowslice(int first, int last, const long long* ptrb, const int* subj,
                   const double* cof)
{
  size_t num = last > first ? last - first : 0;
  // std::cout << __LINE__ << ": putarowslice: range : " << first << " : " <<
  // last << std::endl;
  // std::cout << "\tptrb = ";
  // for (int i = 0; i < last-first+1; ++i)
  //  std::cout << ", " << ptrb[i];
  // std::cout << std::endl;

  // if (num > 0)
  //  MSK_putarowslice64(task,first,first+num, ptrb,ptrb+1,subj,cof);
  MSK(putarowslice64, first, first + num, ptrb, ptrb + 1, subj, cof);
}
void
Task::putbaraij(int i, int j, size_t num, const long long* sub,
                const double* alpha)
{
  MSK(putbaraij, i, j, num, sub, alpha);
}

void
Task::putaijlist(const int* subi, const int* subj, const double* cof,
                 long long num)
{
  if (num > 0) // bug1989
    MSK(putaijlist64, num, subi, subj, cof);
}

void
Task::putobjsense(MSKobjsensee sense)
{
  MSK(putobjsense, sense);
}
void
Task::putclist(size_t num, const int* subj, const double* c)
{
  MSK(putclist, (int)num, &subj[0], &c[0]);
}
void
Task::putcfix(double cfix)
{
  MSK(putcfix, cfix);
}
void
Task::putbarcj(int j, size_t num, const long long* sub, const double* alpha)
{
  MSK(putbarcj, j, num, sub, alpha);
}

long long
Task::appendsparsesymmat(int dim, size_t num, const int* subi, const int* subj,
                         const double* valij)
{
  long long idx;
  MSK(appendsparsesymmat, dim, num, subi, subj, valij, &idx);
  return idx;
}

void
Task::breakOptimize()
{
  breakflag = true;
}

MSKrescodee
Task::optimize()
{
  MSKrescodee r = MSK_RES_OK;
  breakflag     = false;
  MSK(optimizetrm, &r);
  return r;
}

void
Task::solutionsummary(MSKstreamtypee strm)
{
  MSK(solutionsummary, strm);
}

bool
Task::solutiondef(MSKsoltypee sol)
{
  int soldef = 0;
  MSK(solutiondef, sol, &soldef);
  return soldef != 0;
}

void
Task::getsolution(MSKsoltypee whichsol, MSKprostae& prosta, MSKsolstae& solsta,
                  MSKstakeye* skc, MSKstakeye* skx, MSKstakeye* skn, double* xc,
                  double* xx, double* y, double* slc, double* suc, double* slx,
                  double* sux, double* snx)
{
  MSK(getsolution, whichsol, &prosta, &solsta, skc, skx, skn, xc, xx, y, slc,
      suc, slx, sux, snx);
}

void
Task::getbarxj(MSKsoltypee sol, int j, double* barxj)
{
  MSK(getbarxj, sol, j, barxj);
}
void
Task::getbarsj(MSKsoltypee sol, int j, double* barxj)
{
  MSK(getbarsj, sol, j, barxj);
}

double
Task::getprimalobj(MSKsoltypee sol)
{
  double obj;
  MSK(getprimalobj, sol, &obj);
  return obj;
}

double
Task::getdualobj(MSKsoltypee sol)
{
  double obj;
  MSK(getdualobj, sol, &obj);
  return obj;
}

void
Task::putparam(const std::string& name, double value)
{
  MSK(putnadouparam, name.c_str(), value);
}
void
Task::putparam(const std::string& name, int value)
{
  MSK(putnaintparam, name.c_str(), value);
}
void
Task::putparam(const std::string& name, const std::string& value)
{
  MSK(putnastrparam, name.c_str(), value.c_str());
}
double
Task::getdinfitem(MSKdinfiteme key)
{
  double val;
  MSK(getdouinf, key, &val);
  return val;
}
int
Task::getiinfitem(MSKiinfiteme key)
{
  int val;
  MSK(getintinf, key, &val);
  return val;
}
long long
Task::getliinfitem(MSKliinfiteme key)
{
  long long val;
  MSK(getlintinf, key, &val);
  return val;
}

void
Task::putintlist(size_t num, const int* subj)
{
  for (int j = 0; j < num; ++j)
    MSK(putvartype, subj[j], MSK_VAR_TYPE_INT);
}
void
Task::putcontlist(size_t num, const int* subj)
{
  for (int j = 0; j < num; ++j)
    MSK(putvartype, subj[j], MSK_VAR_TYPE_CONT);
}
void
Task::write(const std::string& filename)
{
  MSK(writedata, filename.c_str());
}

void
Task::putxxslice(MSKsoltypee which, int first, int last, double* xx)
{
  MSK(putxxslice, which, first, last, xx);
}

void
Task::fixvars(int numvar)
{
  int tasknumvar = getnumvar();
  if (numvar < tasknumvar)
  {
    std::vector<MSKboundkeye> bk(numvar - tasknumvar);
    std::vector<double>       bnd(numvar - tasknumvar);
    MSK(putvarboundslice, numvar, tasknumvar, &bk[0], &bnd[0], &bnd[0]);
  }
}

void
Task::removevar(int idx)
{
  MSK(removevars, 1, &idx);
}

void
putlicensecode(int* code)
{
  MSKrescodee r = MSK_putlicensecode(Task::getEnv(true), code);
  if (r != MSK_RES_OK)
    throw MosekException("Failed to put license code");
}

void
putlicensepath(const std::string& path)
{
  MSKrescodee r = MSK_putlicensepath(Task::getEnv(true), path.c_str());
  if (r != MSK_RES_OK)
    throw MosekException("Failed to put license path");
}

void
putlicensewait(int wait)
{
  MSK_putlicensewait(Task::getEnv(true), wait);
}

void
Task::revert(int numvar, int numcon, int numcone, int numbarvar)
{
  int tasknumvar    = getnumvar();
  int tasknumcon    = getnumcon();
  int tasknumcone   = getnumcone();
  int tasknumbarvar = getnumbarvar();

  if (tasknumvar > numvar)
  {
    std::vector<int> subj(monty::range<int>(numvar, tasknumvar).reverse_begin(),
                          monty::range<int>(numvar, tasknumvar).reverse_end());
    MSK(removevars, tasknumvar - numvar, &subj[0]);
  }
  if (tasknumcone > numcone)
  {
    std::vector<int> subj(
      monty::range<int>(numcone, tasknumcone).reverse_begin(),
      monty::range<int>(numcone, tasknumcone).reverse_end());
    MSK(removecones, tasknumcone - numcone, &subj[0]);
  }

  if (tasknumbarvar > numbarvar)
  {
    std::vector<int> subj(
      monty::range<int>(numbarvar, tasknumbarvar).reverse_begin(),
      monty::range<int>(numbarvar, tasknumbarvar).reverse_end());
    MSK(removebarvars, tasknumbarvar - numbarvar, &subj[0]);
  }
  if (tasknumcon > numcon)
  {
    std::vector<int> subj(monty::range<int>(numcon, tasknumcon).reverse_begin(),
                          monty::range<int>(numcon, tasknumcon).reverse_end());
    MSK(removecons, tasknumcon - numcon, &subj[0]);
  }
}
}

void
mosek::LinAlg::axpy(int n, double                               alpha,
                    std::shared_ptr<monty::ndarray<double, 1> > x,
                    std::shared_ptr<monty::ndarray<double, 1> > y)
{
  if (n > y->size() || n > x->size())
    throw ArgumentError("Mismatching argument lengths");
  MSKrescodee r = MSK_axpy(Task::getEnv(true), n, alpha, x->raw(), y->raw());
  if (r != MSK_RES_OK)
    throw MosekException("Axpy() failed");
}

void
mosek::LinAlg::dot(int n, std::shared_ptr<monty::ndarray<double, 1> > x,
                   std::shared_ptr<monty::ndarray<double, 1> > y, double& xty)
{
  if (n != y->size() || n != x->size())
    throw ArgumentError("Mismatching argument lengths");
  MSKrescodee r = MSK_dot(Task::getEnv(true), n, x->raw(), y->raw(), &xty);
  if (r != MSK_RES_OK)
    throw MosekException("dot() failed");
}

void
mosek::LinAlg::gemv(bool transa, int m, int n, double alpha,
                    std::shared_ptr<monty::ndarray<double, 1> > a,
                    std::shared_ptr<monty::ndarray<double, 1> > x, double beta,
                    std::shared_ptr<monty::ndarray<double, 1> > y)
{
  if (m * n != a->size() || m > (transa ? x->size() : y->size()) ||
      n > (transa ? y->size() : x->size()))
    throw ArgumentError("Mismatching argument lengths");
  MSKrescodee r =
    MSK_gemv(Task::getEnv(true), transa ? MSK_TRANSPOSE_YES : MSK_TRANSPOSE_NO,
             m, n, alpha, a->raw(), x->raw(), beta, y->raw());
  if (r != MSK_RES_OK)
    throw MosekException("gemv() failed");
}

void
mosek::LinAlg::gemm(bool transa, bool transb, int m, int n, int k, double alpha,
                    std::shared_ptr<monty::ndarray<double, 1> > a,
                    std::shared_ptr<monty::ndarray<double, 1> > b, double beta,
                    std::shared_ptr<monty::ndarray<double, 1> > c)
{
  if (m * n != c->size() || n * k != b->size() || m * k != a->size())
    throw ArgumentError("Mismatching argument lengths");

  MSKrescodee r =
    MSK_gemm(Task::getEnv(true), transa ? MSK_TRANSPOSE_YES : MSK_TRANSPOSE_NO,
             transb ? MSK_TRANSPOSE_YES : MSK_TRANSPOSE_NO, m, n, k, alpha,
             a->raw(), b->raw(), beta, c->raw());
  if (r != MSK_RES_OK)
    throw MosekException("gemm() failed");
}

void
mosek::LinAlg::syrk(MSKuploe uplo, bool transa, int n, int k, double alpha,
                    std::shared_ptr<monty::ndarray<double, 1> > a, double beta,
                    std::shared_ptr<monty::ndarray<double, 1> > c)
{
  if (n * k != a->size() || n * n != c->size())
    throw ArgumentError("Mismatching argument lengths");

  MSKrescodee r = MSK_syrk(Task::getEnv(true), uplo,
                           transa ? MSK_TRANSPOSE_YES : MSK_TRANSPOSE_NO, n, k,
                           alpha, a->raw(), beta, c->raw());
  if (r != MSK_RES_OK)
    throw MosekException("syrk() failed");
}

void
mosek::LinAlg::syeig(MSKuploe uplo, int                          n,
                     std::shared_ptr<monty::ndarray<double, 1> > a,
                     std::shared_ptr<monty::ndarray<double, 1> > w)
{
  if (n * n != a->size() || n != w->size())
    throw ArgumentError("Mismatching argument lengths");

  MSKrescodee r = MSK_syeig(Task::getEnv(true), uplo, n, a->raw(), w->raw());
  if (r != MSK_RES_OK)
    throw MosekException("syeig() failed");
}

void
mosek::LinAlg::syevd(MSKuploe uplo, int                          n,
                     std::shared_ptr<monty::ndarray<double, 1> > a,
                     std::shared_ptr<monty::ndarray<double, 1> > w)
{
  if (n != a->size() || n != w->size())
    throw ArgumentError("Mismatching argument lengths");

  MSKrescodee r = MSK_syeig(Task::getEnv(true), uplo, n, a->raw(), w->raw());
  if (r != MSK_RES_OK)
    throw MosekException("syeig() failed");
}

void
mosek::LinAlg::potrf(MSKuploe uplo, int                          n,
                     std::shared_ptr<monty::ndarray<double, 1> > a)
{
  if (n != a->size())
    throw ArgumentError("Mismatching argument lengths");

  MSKrescodee r = MSK_potrf(Task::getEnv(true), uplo, n, a->raw());
  if (r != MSK_RES_OK)
    throw MosekException("syeig() failed");
}
