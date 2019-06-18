#ifndef _MOSEKTASK_P_H_
#define _MOSEKTASK_P_H_

#include "mosek/mosek.h"
#include <memory>
#include <mutex>
#include <vector>

#include "mosek/mosektask.h"

namespace mosek
{
class Task
{
public:
  typedef std::function<void(const std::string&)> msghandler_t;
  /*
  typedef std::function<int(void * handle, MSKcallbackcodee)> pgshandler_t;
  typedef std::function<int(void * handle, MSKcallbackcodee,const double*,const
  int*,const long long*)> infhandler_t;
  */
private:
  bool      breakflag;
  MSKtask_t task;

  msghandler_t msguserfunc;
  // infhandler_t      infuserfunc;
  // pgshandler_t      pgsuserfunc;
  datacbhandler_t datacbuserfunc;
  cbhandler_t     cbuserfunc;
  /*void            * pgshandle;*/
  // void            * infhandle;

  static void msghandler(void* handle, const char* msg);
  static int pgshandler(MSKtask_t task, void* handle, MSKcallbackcodee caller,
                        const double* dinf, const int* iinf,
                        const long long* linf);

  //---------------------------------
  static MSKenv_t   env;
  static int        env_refc;
  static bool       env_release;
  static std::mutex env_lock;

  //---------------------------------
  static MSKenv_t getEnv();
  static void     releaseEnv();

public:
  static MSKenv_t getEnv(bool borrow);

  Task();
  Task(Task& other);
  ~Task();
  void      dispose();
  MSKtask_t clone() const;

  void setDataCallbackFunc(datacbhandler_t userfunc)
  {
    datacbuserfunc = userfunc;
  }
  void setCallbackFunc(cbhandler_t userfunc)
  {
    cbuserfunc = userfunc;
  }
  /*
  void setProgressFunc(pgshandler_t userfunc, void * handle = nullptr ) {
  pgsuserfunc = userfunc; pgshandle = handle; }
  void setInfoFunc    (infhandler_t userfunc, void * handle = nullptr ) {
  infuserfunc = userfunc; infhandle = handle; }
  */
  void setStreamFunc(msghandler_t userfunc)
  {
    msguserfunc = userfunc;
  }
  /*
  void removeProgressFunc()                   { pgsuserfunc = nullptr; }
  void removeInfoFunc()                       { infuserfunc = nullptr; }
  */
  void removeStreamFunc()
  {
    msguserfunc = nullptr;
  }

  int getnumvar() const;
  int getnumcon() const;
  int getnumcone() const;
  int getnumbarvar() const;

  int getbarvardim(int idx) const;

  int appendvars(int num);
  int appendcons(int num);
  int appendbarvar(int dim);
  int appendconesseq(MSKconetypee ct, int numcone, int conesize, int firstvar);
  int appendcones(MSKconetypee ct, int numcone, int conesize, int firstvar,
                  int d0, int d1);

  // revert the task to this size
  void removevar(int idx);
  void revert(int numvar, int numcon, int numcone, int numbarvar);
  void fixvars(int numvar);

  void bound(MSKaccmodee acc, int idx, MSKboundkeye bk, double lb, double ub);
  void boundslice(MSKaccmodee acc, int first, int last, const MSKboundkeye* bk,
                  const double* lb, const double* ub);
  void boundlist(MSKaccmodee acc, size_t num, const int* sub,
                 const MSKboundkeye* bk, const double* lb, const double* ub);

  void varname(int idx, const std::string& name);
  void conname(int idx, const std::string& name);
  void conename(int idx, const std::string& name);
  void barvarname(int idx, const std::string& name);
  void objname(const std::string& name);
  void taskname(const std::string& name);

  void putarowslice(int first, int last, const long long* ptrb, const int* subj,
                    const double* cof);
  // putacolslice
  void putaijlist(const int* subi, const int* subj, const double* cof,
                  long long num);

  void putbaraij(int i, int j, size_t num, const long long* sub,
                 const double* alpha);

  void putobjsense(MSKobjsensee sense);
  void putclist(size_t num, const int* subj, const double* c);
  void putcfix(double cfix);
  void putbarcj(int j, size_t num, const long long* sub, const double* alpha);

  //--------------------------

  long long appendsparsesymmat(int dim, size_t num, const int* subi,
                               const int* subj, const double* valij);

  //--------------------------

  void        breakOptimize();
  MSKrescodee optimize();

  void solutionsummary(MSKstreamtypee strm);
  bool solutiondef(MSKsoltypee sol);
  void getsolution(MSKsoltypee whichsol, MSKprostae& prosta, MSKsolstae& solsta,
                   MSKstakeye* skc, MSKstakeye* skx, MSKstakeye* skn,
                   double* xc, double* xx, double* y, double* slc, double* suc,
                   double* slx, double* sux, double* snx);
  void getbarxj(MSKsoltypee sol, int j, double* barxj);
  void getbarsj(MSKsoltypee sol, int j, double* barxj);

  double getprimalobj(MSKsoltypee sol);
  double getdualobj(MSKsoltypee sol);

  //--------------------------

  void putxxslice(MSKsoltypee which, int first, int last, double* xx);

  //--------------------------

  void putintlist(size_t num, const int* subj);
  void putcontlist(size_t num, const int* subj);

  //--------------------------

  void write(const std::string& filename);

  //--------------------------

  void putparam(const std::string& name, double value);
  void putparam(const std::string& name, int value);
  void putparam(const std::string& name, const std::string& value);

  double getdinfitem(MSKdinfiteme key);
  int getiinfitem(MSKiinfiteme key);
  long long getliinfitem(MSKliinfiteme key);

  static void env_syeig(int n, const double* a, double* w);
  // static void env_potrf (int n, double * a);
  static void env_syevd(int n, double* a, double* w);

  static void releaseGlobalEnv();
};

void putlicensecode(int* code);
void putlicensepath(const std::string& path);
void putlicensewait(int wait);
}
#endif
