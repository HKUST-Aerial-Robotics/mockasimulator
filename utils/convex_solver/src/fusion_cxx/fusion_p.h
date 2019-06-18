#ifndef __FUSION_P_H__
#define __FUSION_P_H__
#include "list"
#include "mosek/fusion.h"
#include "mosek/monty.h"
#include "mosektask_p.h"
#include "unordered_map"
#include "vector"
namespace mosek
{
namespace fusion
{
// mosek.fusion.BaseModel from file 'src/fusion/cxx/BaseModel_p.h'
// namespace mosek::fusion
struct p_BaseModel
{
  p_BaseModel(BaseModel* _pubthis);

  void _initialize(monty::rc_ptr<BaseModel> m);
  void _initialize(const std::string& name, const std::string& licpath);

  virtual ~p_BaseModel()
  { /* std::cout << "~p_BaseModel()" << std::endl;*/
  }

  static p_BaseModel* _get_impl(Model* _inst)
  {
    return _inst->_impl;
  }

  //----------------------

  bool        synched;
  std::string taskname;

  monty::rc_ptr<SolutionStruct> sol_itr;
  monty::rc_ptr<SolutionStruct> sol_itg;
  monty::rc_ptr<SolutionStruct> sol_bas;

  //---------------------

  std::unique_ptr<Task> task;

  //---------------------
  void task_setLogHandler(const msghandler_t& handler);
  void task_setDataCallbackHandler(const datacbhandler_t& handler);
  void task_setCallbackHandler(const cbhandler_t& handler);

  int alloc_rangedvar(const std::string& name, double lb, double ub);
  int alloc_linearvar(const std::string&         name,
                      mosek::fusion::RelationKey relkey, double bound);
  int task_append_barvar(int size, int num);

  void task_var_name(int index, const std::string& name);
  void task_con_name(int index, const std::string& name);
  void task_cone_name(int index, const std::string& name);
  void task_barvar_name(int index, const std::string& name);
  void task_objectivename(const std::string& name);

  void task_break_solve();

  //--------------------------

  int task_numvar();
  int task_numcon();
  int task_numcone();
  int task_numbarvar();

  //--------------------------

  void task_put_param(const std::string& name, const std::string& value);
  void task_put_param(const std::string& name, int value);
  void task_put_param(const std::string& name, double value);

  double task_get_dinf(const std::string& name);
  int task_get_iinf(const std::string& name);
  long long task_get_liinf(const std::string& name);

  //--------------------------
  void task_con_putboundslice_fr(int first, int last);
  void task_con_putboundslice_lo(
    int first, int                                     last,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_con_putboundslice_up(
    int first, int                                     last,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_con_putboundslice_ra(
    int first, int last, const std::shared_ptr<monty::ndarray<double, 1> >& lb,
    const std::shared_ptr<monty::ndarray<double, 1> >& ub);
  void task_con_putboundslice_fx(
    int first, int                                     last,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);

  void task_con_putboundlist_lo(
    const std::shared_ptr<monty::ndarray<int, 1> >     idxs,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_con_putboundlist_up(
    const std::shared_ptr<monty::ndarray<int, 1> >     idxs,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_con_putboundlist_fx(
    const std::shared_ptr<monty::ndarray<int, 1> >     idxs,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_con_putboundlist_ra(
    const std::shared_ptr<monty::ndarray<int, 1> >     idxs,
    const std::shared_ptr<monty::ndarray<double, 1> >& lb,
    const std::shared_ptr<monty::ndarray<double, 1> >& ub);
  void task_var_putboundslice_fr(int first, int last);
  void task_var_putboundslice_lo(
    int first, int                                     last,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_var_putboundslice_up(
    int first, int                                     last,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_var_putboundslice_ra(
    int first, int last, const std::shared_ptr<monty::ndarray<double, 1> >& lb,
    const std::shared_ptr<monty::ndarray<double, 1> >& ub);
  void task_var_putboundslice_fx(
    int first, int                                     last,
    const std::shared_ptr<monty::ndarray<double, 1> >& rhs);
  void task_var_putintlist(
    const std::shared_ptr<monty::ndarray<int, 1> >& idxs);
  void task_var_putcontlist(
    const std::shared_ptr<monty::ndarray<int, 1> >& idxs);

  //--------------------------

  int task_append_barmatrix(
    int dim, const std::shared_ptr<monty::ndarray<int, 1> >& subi,
    const std::shared_ptr<monty::ndarray<int, 1> >&    subj,
    const std::shared_ptr<monty::ndarray<double, 1> >& cof);
  int task_barvar_dim(int j);
  void task_putbaraij(int i, int j, int k);
  void task_putbaraij(int i, int                                      j,
                      const std::shared_ptr<monty::ndarray<int, 1> >& k);
  void task_putbarcj(int j, int k);
  void task_putbarcj(int j, const std::shared_ptr<monty::ndarray<int, 1> >& k);
  int task_barvardim(int index);

  int task_append_var(int num);
  int task_append_con(int num);
  int task_append_quadcone(int conesize, int first, int num, int d0, int d1);
  int task_append_rquadcone(int conesize, int first, int num, int d0, int d1);

  void task_putarowslice(
    int first, int                                        last,
    const std::shared_ptr<monty::ndarray<long long, 1> >& ptrb,
    const std::shared_ptr<monty::ndarray<int, 1> >&       subj,
    const std::shared_ptr<monty::ndarray<double, 1> >&    cof);
  void task_putaijlist(const std::shared_ptr<monty::ndarray<int, 1> >&    subi,
                       const std::shared_ptr<monty::ndarray<int, 1> >&    subj,
                       const std::shared_ptr<monty::ndarray<double, 1> >& cof,
                       long long num);

  void task_setnumvar(int num);
  void task_cleanup(int oldnum, int oldnumcon, int oldnumcone,
                    int oldnumbarvar);
  void task_solve();

  void task_putobjective(bool maximize,
                         const std::shared_ptr<monty::ndarray<int, 1> >& subj,
                         const std::shared_ptr<monty::ndarray<double, 1> >& cof,
                         double cfix);

  void task_putobjectivename(const std::string& name);

  void task_write(const std::string& filename);
  void task_dump(const std::string& filename);

  MSKtask_t task_get();
  void      dispose();

  void task_putxx_slice(SolutionType which, int first, int last,
                        std::shared_ptr<monty::ndarray<double, 1> >& xx);

  static void env_syeig(int n, std::shared_ptr<monty::ndarray<double, 1> >& a,
                        std::shared_ptr<monty::ndarray<double, 1> >& w);
  static void env_potrf(int n, std::shared_ptr<monty::ndarray<double, 1> >& a);
  static void env_syevd(int n, std::shared_ptr<monty::ndarray<double, 1> >& a,
                        std::shared_ptr<monty::ndarray<double, 1> >& w);

  static void env_putlicensecode(std::shared_ptr<monty::ndarray<int, 1> > code);
  static void env_putlicensepath(const std::string& licfile);
  static void env_putlicensewait(int wait);

  void convertSolutionStatus(MSKsoltypee soltype, p_SolutionStruct* sol,
                             MSKsolstae status, MSKprostae prosta);
};

// End of file 'src/fusion/cxx/BaseModel_p.h'
struct p_Model : public ::mosek::fusion::p_BaseModel
{
  Model*                         _pubthis;
  static mosek::fusion::p_Model* _get_impl(mosek::fusion::Model* _inst)
  {
    return static_cast<mosek::fusion::p_Model*>(
      mosek::fusion::p_BaseModel::_get_impl(_inst));
  }
  static mosek::fusion::p_Model* _get_impl(mosek::fusion::Model::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Model(Model* _pubthis);
  virtual ~p_Model(){ /* std::cout << "~p_Model" << std::endl;*/ };
  int                                                  task_vars_used{};
  int                                                  task_vars_allocated{};
  monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap> con_map{};
  int                                                  cons_used{};
  std::shared_ptr<
    monty::ndarray<monty::rc_ptr< ::mosek::fusion::ModelConstraint>, 1> >
      cons{};
  int vars_used{};
  std::shared_ptr<
    monty::ndarray<monty::rc_ptr< ::mosek::fusion::ModelVariable>, 1> >
    vars{};
  std::shared_ptr<monty::ndarray<bool, 1> >   initsol_xx_flag{};
  std::shared_ptr<monty::ndarray<double, 1> > initsol_xx{};
  int natbarvarmap_num{};
  std::shared_ptr<monty::ndarray<int, 1> > natbarvarmap_Var{};
  monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap> var_map{};
  int                                                  natvarmap_num{};
  std::shared_ptr<monty::ndarray<long long, 1> > natvarmap_idx{};
  std::shared_ptr<monty::ndarray<int, 1> >       natvarmap_Var{};
  mosek::fusion::SolutionType      solutionptr{};
  mosek::fusion::AccSolutionStatus acceptable_sol{};
  std::string                      model_name{};
  virtual void                     destroy();
  static Model::t _new_Model(monty::rc_ptr< ::mosek::fusion::Model> _436);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _436);
  static Model::t _new_Model();
  void            _initialize();
  static Model::t _new_Model(const std::string& _443);
  void _initialize(const std::string& _443);
  static void putlicensewait(bool _444);
  static void putlicensepath(const std::string& _445);
  static void putlicensecode(std::shared_ptr<monty::ndarray<int, 1> > _446);
  static void inst(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
        _447,
    int _448, int _449, std::shared_ptr<monty::ndarray<long long, 1> > _450,
    int _451, std::shared_ptr<monty::ndarray<int, 1> > _452,
    std::shared_ptr<monty::ndarray<int, 1> > _453,
    std::shared_ptr<monty::ndarray<int, 1> > _454);
  static void inst(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _475,
    std::shared_ptr<monty::ndarray<long long, 1> > _476,
    std::shared_ptr<monty::ndarray<int, 1> >       _477,
    std::shared_ptr<monty::ndarray<int, 1> >       _478,
    std::shared_ptr<monty::ndarray<int, 1> >       _479);
  virtual void dispose();
  virtual void varname(int _482, const std::string& _483);
  virtual void nativeVarToStr(
    int _484, monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _485);
  virtual int append_linearvar(
    monty::rc_ptr< ::mosek::fusion::ModelVariable> _486, long long _487,
    mosek::fusion::RelationKey _488, double _489);
  virtual int append_rangedvar(
    monty::rc_ptr< ::mosek::fusion::ModelVariable> _491, long long _492,
    double _493, double _494);
  virtual MSKtask_t getTask();
  virtual void      flushNames();
  virtual void writeTask(const std::string& _498);
  virtual long long getSolverLIntInfo(const std::string& _499);
  virtual int getSolverIntInfo(const std::string& _500);
  virtual double getSolverDoubleInfo(const std::string& _501);
  virtual void setCallbackHandler(mosek::cbhandler_t _502);
  virtual void setDataCallbackHandler(mosek::datacbhandler_t _503);
  virtual void setLogHandler(mosek::msghandler_t _504);
  virtual void setSolverParam(const std::string& _505, double _506);
  virtual void setSolverParam(const std::string& _507, int _508);
  virtual void setSolverParam(const std::string& _509, const std::string& _510);
  virtual void breakSolver();
  virtual void solve();
  virtual void flushSolutions();
  virtual void flush_initsol(mosek::fusion::SolutionType _511);
  virtual mosek::fusion::SolutionStatus getDualSolutionStatus();
  virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus();
  virtual double                        dualObjValue();
  virtual double                        primalObjValue();
  virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct> get_sol_cache(
    mosek::fusion::SolutionType _519, bool _520, bool _521);
  virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct> get_sol_cache(
    mosek::fusion::SolutionType _526, bool _527);
  virtual void setSolution_xx(std::shared_ptr<monty::ndarray<int, 1> >    _528,
                              std::shared_ptr<monty::ndarray<double, 1> > _529);
  virtual void ensure_initsol_xx();
  virtual std::shared_ptr<
    monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
  getSolution_bars(mosek::fusion::SolutionType _535);
  virtual std::shared_ptr<
    monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
  getSolution_barx(mosek::fusion::SolutionType _536);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_y(
    mosek::fusion::SolutionType _537);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_xc(
    mosek::fusion::SolutionType _538);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_snx(
    mosek::fusion::SolutionType _539);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_suc(
    mosek::fusion::SolutionType _540);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_slc(
    mosek::fusion::SolutionType _541);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_sux(
    mosek::fusion::SolutionType _542);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_slx(
    mosek::fusion::SolutionType _543);
  virtual std::shared_ptr<monty::ndarray<double, 1> > getSolution_xx(
    mosek::fusion::SolutionType _544);
  virtual void selectedSolution(mosek::fusion::SolutionType _545);
  virtual mosek::fusion::AccSolutionStatus getAcceptedSolutionStatus();
  virtual void acceptedSolutionStatus(mosek::fusion::AccSolutionStatus _546);
  virtual mosek::fusion::ProblemStatus getProblemStatus(
    mosek::fusion::SolutionType _547);
  virtual mosek::fusion::SolutionStatus getDualSolutionStatus(
    mosek::fusion::SolutionType _549);
  virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus(
    mosek::fusion::SolutionType _550);
  virtual mosek::fusion::SolutionStatus getSolutionStatus(
    mosek::fusion::SolutionType _551, bool _552);
  virtual void objective_(const std::string&                          _555,
                          mosek::fusion::ObjectiveSense               _556,
                          monty::rc_ptr< ::mosek::fusion::Expression> _557);
  virtual void objective(double _595);
  virtual void objective(mosek::fusion::ObjectiveSense _596, double _597);
  virtual void objective(mosek::fusion::ObjectiveSense             _598,
                         monty::rc_ptr< ::mosek::fusion::Variable> _599);
  virtual void objective(mosek::fusion::ObjectiveSense               _600,
                         monty::rc_ptr< ::mosek::fusion::Expression> _601);
  virtual void objective(const std::string& _602, double _603);
  virtual void objective(const std::string&            _604,
                         mosek::fusion::ObjectiveSense _605, double _606);
  virtual void objective(const std::string&                        _607,
                         mosek::fusion::ObjectiveSense             _608,
                         monty::rc_ptr< ::mosek::fusion::Variable> _609);
  virtual void objective(const std::string&                          _610,
                         mosek::fusion::ObjectiveSense               _611,
                         monty::rc_ptr< ::mosek::fusion::Expression> _612);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Variable>    _613,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _614);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _615, monty::rc_ptr< ::mosek::fusion::Variable> _616,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _617);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Set>         _618,
    monty::rc_ptr< ::mosek::fusion::Variable>    _619,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _620);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _621, monty::rc_ptr< ::mosek::fusion::Set> _622,
    monty::rc_ptr< ::mosek::fusion::Variable>    _623,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _624);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Variable>    _625,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _626);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _627, monty::rc_ptr< ::mosek::fusion::Variable> _628,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _629);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Set>         _630,
    monty::rc_ptr< ::mosek::fusion::Variable>    _631,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _632);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _633, monty::rc_ptr< ::mosek::fusion::Set> _634,
    monty::rc_ptr< ::mosek::fusion::Variable>    _635,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _636);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Variable>     _637,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _638);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _639, monty::rc_ptr< ::mosek::fusion::Variable> _640,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _641);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Set>          _642,
    monty::rc_ptr< ::mosek::fusion::Variable>     _643,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _644);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _645, monty::rc_ptr< ::mosek::fusion::Set> _646,
    monty::rc_ptr< ::mosek::fusion::Variable>     _647,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _648);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Variable>     _649,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _650);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _651, monty::rc_ptr< ::mosek::fusion::Variable> _652,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _653);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Variable>  _654,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _655);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _656, monty::rc_ptr< ::mosek::fusion::Variable> _657,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _658);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Expression>  _659,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _660);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _661, monty::rc_ptr< ::mosek::fusion::Expression> _662,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _663);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Set>         _664,
    monty::rc_ptr< ::mosek::fusion::Expression>  _665,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _666);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _667, monty::rc_ptr< ::mosek::fusion::Set> _668,
    monty::rc_ptr< ::mosek::fusion::Expression>  _669,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _670);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Expression>  _671,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _672);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _673, monty::rc_ptr< ::mosek::fusion::Expression> _674,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _675);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Set>         _676,
    monty::rc_ptr< ::mosek::fusion::Expression>  _677,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _678);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _679, monty::rc_ptr< ::mosek::fusion::Set> _680,
    monty::rc_ptr< ::mosek::fusion::Expression>  _681,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _682);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Expression>   _683,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _684);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _685, monty::rc_ptr< ::mosek::fusion::Expression> _686,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _687);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Set>          _688,
    monty::rc_ptr< ::mosek::fusion::Expression>   _689,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _690);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _691, monty::rc_ptr< ::mosek::fusion::Set> _692,
    monty::rc_ptr< ::mosek::fusion::Expression>   _693,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _694);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Expression>   _695,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _696);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _697, monty::rc_ptr< ::mosek::fusion::Expression> _698,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _699);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    monty::rc_ptr< ::mosek::fusion::Expression> _700,
    monty::rc_ptr< ::mosek::fusion::PSDDomain>  _701);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint(
    const std::string& _702, monty::rc_ptr< ::mosek::fusion::Expression> _703,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _704);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _705);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _706, int _707, monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _708);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _709, monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _710);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string&                            _711,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _712);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _713, int _714, int _715,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _716);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _717, int _718,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _719);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _720, monty::rc_ptr< ::mosek::fusion::Set> _721,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _722);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _723);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _724, int _725, monty::rc_ptr< ::mosek::fusion::PSDDomain> _726);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _727, monty::rc_ptr< ::mosek::fusion::PSDDomain> _728);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _729, monty::rc_ptr< ::mosek::fusion::PSDDomain> _730);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _731, int _732, int _733,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _734);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _735, int _736,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _737);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _738, monty::rc_ptr< ::mosek::fusion::Set> _739,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _740);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable> variable(
    int _741, monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> _742);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable> variable(
    const std::string& _743, int _744,
    monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> _745);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _746);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _747);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _748);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    std::shared_ptr<monty::ndarray<int, 1> > _749,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _750);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    std::shared_ptr<monty::ndarray<int, 1> > _751,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _752);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::Set>         _753,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _754);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::Set>         _755,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _756);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    monty::rc_ptr< ::mosek::fusion::Set>          _757,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _758);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _759, monty::rc_ptr< ::mosek::fusion::QConeDomain> _760);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _761, monty::rc_ptr< ::mosek::fusion::RangeDomain> _762);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    int _763, monty::rc_ptr< ::mosek::fusion::LinearDomain> _764);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    std::shared_ptr<monty::ndarray<int, 1> > _765);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(int _766);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _767, monty::rc_ptr< ::mosek::fusion::QConeDomain> _768);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _769, monty::rc_ptr< ::mosek::fusion::RangeDomain> _770);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string&                            _771,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _772);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _773, std::shared_ptr<monty::ndarray<int, 1> > _774,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _775);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _776, std::shared_ptr<monty::ndarray<int, 1> > _777,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _778);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _779, monty::rc_ptr< ::mosek::fusion::Set> _780,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _781);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _782, monty::rc_ptr< ::mosek::fusion::Set> _783,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _784);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _785, monty::rc_ptr< ::mosek::fusion::Set> _786,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _787);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _788, int _789,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _790);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _791, int _792,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _793);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _794, int _795,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _796);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _797, std::shared_ptr<monty::ndarray<int, 1> > _798);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _799, int _800);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable(
    const std::string& _801);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> ranged_variable(
    const std::string& _802, int _803,
    monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain> _804);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> ranged_variable(
    const std::string& _823, monty::rc_ptr< ::mosek::fusion::Set> _824,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _825);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable_(
    const std::string& _842, monty::rc_ptr< ::mosek::fusion::Set> _843,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _844);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable> variable_(
    const std::string& _868, int _869,
    monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> _870);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable_(
    const std::string& _888, monty::rc_ptr< ::mosek::fusion::Set> _889,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _890);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> variable_(
    const std::string& _907, monty::rc_ptr< ::mosek::fusion::Set> _908,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _909);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable> variable_(
    const std::string& _917, monty::rc_ptr< ::mosek::fusion::Set> _918,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _919);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint_(
    const std::string& _924, monty::rc_ptr< ::mosek::fusion::Set> _925,
    monty::rc_ptr< ::mosek::fusion::Expression>  _926,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _927);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint_(
    const std::string& _950, monty::rc_ptr< ::mosek::fusion::Set> _951,
    monty::rc_ptr< ::mosek::fusion::Expression>  _952,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _953);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint_(
    const std::string& _988, monty::rc_ptr< ::mosek::fusion::Set> _989,
    monty::rc_ptr< ::mosek::fusion::Expression>   _990,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _991);
  virtual monty::rc_ptr< ::mosek::fusion::ConNZStruct> build_conA(
    std::shared_ptr<monty::ndarray<long long, 1> > _1012, long long _1013,
    std::shared_ptr<monty::ndarray<long long, 1> > _1014,
    std::shared_ptr<monty::ndarray<long long, 1> > _1015,
    std::shared_ptr<monty::ndarray<double, 1> >    _1016,
    std::shared_ptr<monty::ndarray<double, 1> >    _1017,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _1018);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint_(
    const std::string& _1075, monty::rc_ptr< ::mosek::fusion::Expression> _1076,
    monty::rc_ptr< ::mosek::fusion::LinPSDDomain> _1077);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> constraint_(
    const std::string& _1112, monty::rc_ptr< ::mosek::fusion::Expression> _1113,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _1114);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> nonsym_psdconstraint(
    const std::string& _1127, monty::rc_ptr< ::mosek::fusion::Expression> _1128,
    monty::rc_ptr< ::mosek::fusion::PSDDomain> _1129);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> sdptrilcon(
    const std::string& _1190, int _1191, int _1192,
    std::shared_ptr<monty::ndarray<long long, 1> > _1193,
    std::shared_ptr<monty::ndarray<long long, 1> > _1194,
    std::shared_ptr<monty::ndarray<long long, 1> > _1195,
    std::shared_ptr<monty::ndarray<long long, 1> > _1196,
    std::shared_ptr<monty::ndarray<double, 1> >    _1197,
    std::shared_ptr<monty::ndarray<double, 1> >    _1198,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _1199);
  virtual void addConstraint(
    const std::string&                               _1270,
    monty::rc_ptr< ::mosek::fusion::ModelConstraint> _1271);
  virtual void addVariable(
    const std::string&                             _1275,
    monty::rc_ptr< ::mosek::fusion::ModelVariable> _1276);
  virtual long long numConstraints();
  virtual long long numVariables();
  virtual bool hasConstraint(const std::string& _1280);
  virtual bool hasVariable(const std::string& _1281);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> getConstraint(int _1282);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> getConstraint(
    const std::string& _1283);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> getVariable(int _1284);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> getVariable(
    const std::string& _1285);
  virtual std::string                            getName();
  virtual monty::rc_ptr< ::mosek::fusion::Model> clone();
  virtual void natbarvarmap_ensure(int _1286);
  virtual void natvarmap_ensure(int _1291);
  virtual int task_alloc_vars(int _1296);
}; // struct Model;

// mosek.fusion.Debug from file 'src/fusion/cxx/Debug_p.h'
// namespace mosek::fusion
struct p_Debug
{
  Debug* _pubthis;

  p_Debug(Debug* _pubthis)
    : _pubthis(_pubthis)
  {
  }

  static Debug::t o()
  {
    return monty::rc_ptr<Debug>(new Debug());
  }
  Debug::t p(const std::string& val)
  {
    std::cout << val;
    return Debug::t(_pubthis);
  }
  Debug::t p(int val)
  {
    std::cout << val;
    return Debug::t(_pubthis);
  }
  Debug::t p(long long val)
  {
    std::cout << val;
    return Debug::t(_pubthis);
  }
  Debug::t p(double val)
  {
    std::cout << val;
    return Debug::t(_pubthis);
  }
  Debug::t p(bool val)
  {
    std::cout << val;
    return Debug::t(_pubthis);
  }
  Debug::t lf()
  {
    std::cout << std::endl;
    return Debug::t(_pubthis);
  }

  static p_Debug* _get_impl(Debug* _inst)
  {
    return _inst->ptr.get();
  }

  template <typename T>
  Debug::t p(const std::shared_ptr<monty::ndarray<T, 1> >& val)
  {
    if (val->size() > 0)
    {
      std::cout << (*val)[0];
      for (int i = 1; i < val->size(); ++i)
        std::cout << "," << (*val)[i];
    }
    return Debug::t(_pubthis);
  }
};
// End of file 'src/fusion/cxx/Debug_p.h'
struct p_Sort
{
  Sort*                         _pubthis;
  static mosek::fusion::p_Sort* _get_impl(mosek::fusion::Sort* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Sort* _get_impl(mosek::fusion::Sort::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Sort(Sort* _pubthis);
  virtual ~p_Sort(){ /* std::cout << "~p_Sort" << std::endl;*/ };
  virtual void destroy();
  static void  argTransposeSort(
    std::shared_ptr<monty::ndarray<long long, 1> > _143,
    std::shared_ptr<monty::ndarray<long long, 1> > _144, int _145, int _146,
    int _147, std::shared_ptr<monty::ndarray<long long, 1> > _148);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _156,
                      std::shared_ptr<monty::ndarray<long long, 1> > _157);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _158,
                      std::shared_ptr<monty::ndarray<int, 1> >       _159);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _160,
                      std::shared_ptr<monty::ndarray<long long, 1> > _161,
                      std::shared_ptr<monty::ndarray<long long, 1> > _162);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _163,
                      std::shared_ptr<monty::ndarray<int, 1> >       _164,
                      std::shared_ptr<monty::ndarray<int, 1> >       _165);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _166,
                      std::shared_ptr<monty::ndarray<long long, 1> > _167,
                      long long _168, long long _169);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _170,
                      std::shared_ptr<monty::ndarray<int, 1> >       _171,
                      long long _172, long long _173);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _174,
                      std::shared_ptr<monty::ndarray<long long, 1> > _175,
                      std::shared_ptr<monty::ndarray<long long, 1> > _176,
                      long long _177, long long _178);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _179,
                      std::shared_ptr<monty::ndarray<int, 1> >       _180,
                      std::shared_ptr<monty::ndarray<int, 1> >       _181,
                      long long _182, long long _183);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _184,
                      std::shared_ptr<monty::ndarray<long long, 1> > _185,
                      long long _186, long long _187, bool _188);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _191,
                      std::shared_ptr<monty::ndarray<int, 1> >       _192,
                      long long _193, long long _194, bool _195);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _198,
                      std::shared_ptr<monty::ndarray<long long, 1> > _199,
                      std::shared_ptr<monty::ndarray<long long, 1> > _200,
                      long long _201, long long _202, bool _203);
  static void argsort(std::shared_ptr<monty::ndarray<long long, 1> > _206,
                      std::shared_ptr<monty::ndarray<int, 1> >       _207,
                      std::shared_ptr<monty::ndarray<int, 1> >       _208,
                      long long _209, long long _210, bool _211);
  static void argbucketsort(std::shared_ptr<monty::ndarray<long long, 1> > _214,
                            std::shared_ptr<monty::ndarray<long long, 1> > _215,
                            long long _216, long long _217, long long _218,
                            long long _219);
  static void argbucketsort(std::shared_ptr<monty::ndarray<long long, 1> > _220,
                            std::shared_ptr<monty::ndarray<int, 1> >       _221,
                            long long _222, long long _223, int _224, int _225);
  static void getminmax(std::shared_ptr<monty::ndarray<long long, 1> > _226,
                        std::shared_ptr<monty::ndarray<long long, 1> > _227,
                        std::shared_ptr<monty::ndarray<long long, 1> > _228,
                        long long _229, long long                      _230,
                        std::shared_ptr<monty::ndarray<long long, 1> > _231);
  static void getminmax(std::shared_ptr<monty::ndarray<long long, 1> > _234,
                        std::shared_ptr<monty::ndarray<int, 1> >       _235,
                        std::shared_ptr<monty::ndarray<int, 1> >       _236,
                        long long _237, long long                _238,
                        std::shared_ptr<monty::ndarray<int, 1> > _239);
  static bool issorted(std::shared_ptr<monty::ndarray<long long, 1> > _242,
                       std::shared_ptr<monty::ndarray<long long, 1> > _243,
                       long long _244, long long _245, bool _246);
  static bool issorted(std::shared_ptr<monty::ndarray<long long, 1> > _248,
                       std::shared_ptr<monty::ndarray<int, 1> >       _249,
                       long long _250, long long _251, bool _252);
  static bool issorted(std::shared_ptr<monty::ndarray<long long, 1> > _254,
                       std::shared_ptr<monty::ndarray<long long, 1> > _255,
                       std::shared_ptr<monty::ndarray<long long, 1> > _256,
                       long long _257, long long _258, bool _259);
  static bool issorted(std::shared_ptr<monty::ndarray<long long, 1> > _261,
                       std::shared_ptr<monty::ndarray<int, 1> >       _262,
                       std::shared_ptr<monty::ndarray<int, 1> >       _263,
                       long long _264, long long _265, bool _266);
}; // struct Sort;

struct p_IndexCounter
{
  IndexCounter*                         _pubthis;
  static mosek::fusion::p_IndexCounter* _get_impl(
    mosek::fusion::IndexCounter* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_IndexCounter* _get_impl(
    mosek::fusion::IndexCounter::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_IndexCounter(IndexCounter* _pubthis);
  virtual ~p_IndexCounter(){
    /* std::cout << "~p_IndexCounter" << std::endl;*/
  };
  long long start{};
  std::shared_ptr<monty::ndarray<int, 1> >       dims{};
  std::shared_ptr<monty::ndarray<long long, 1> > strides{};
  std::shared_ptr<monty::ndarray<long long, 1> > st{};
  std::shared_ptr<monty::ndarray<int, 1> >       ii{};
  int                    n{};
  virtual void           destroy();
  static IndexCounter::t _new_IndexCounter(
    monty::rc_ptr< ::mosek::fusion::Set> _268);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Set> _268);
  static IndexCounter::t _new_IndexCounter(
    long long _271, std::shared_ptr<monty::ndarray<int, 1> > _272,
    monty::rc_ptr< ::mosek::fusion::Set> _273);
  void _initialize(long long _271,
                   std::shared_ptr<monty::ndarray<int, 1> > _272,
                   monty::rc_ptr< ::mosek::fusion::Set> _273);
  static IndexCounter::t _new_IndexCounter(
    long long _276, std::shared_ptr<monty::ndarray<int, 1> > _277,
    std::shared_ptr<monty::ndarray<long long, 1> > _278);
  void _initialize(long long _276,
                   std::shared_ptr<monty::ndarray<int, 1> >       _277,
                   std::shared_ptr<monty::ndarray<long long, 1> > _278);
  virtual bool atEnd();
  virtual std::shared_ptr<monty::ndarray<int, 1> > getIndex();
  virtual long long next();
  virtual long long get();
  virtual void      inc();
  virtual void      reset();
}; // struct IndexCounter;

struct p_CommonTools
{
  CommonTools*                         _pubthis;
  static mosek::fusion::p_CommonTools* _get_impl(
    mosek::fusion::CommonTools* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_CommonTools* _get_impl(
    mosek::fusion::CommonTools::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_CommonTools(CommonTools* _pubthis);
  virtual ~p_CommonTools(){ /* std::cout << "~p_CommonTools" << std::endl;*/ };
  virtual void destroy();
  static void ndIncr(std::shared_ptr<monty::ndarray<int, 1> > _284,
                     std::shared_ptr<monty::ndarray<int, 1> > _285,
                     std::shared_ptr<monty::ndarray<int, 1> > _286);
  static void transposeTriplets(
    std::shared_ptr<monty::ndarray<int, 1> >    _288,
    std::shared_ptr<monty::ndarray<int, 1> >    _289,
    std::shared_ptr<monty::ndarray<double, 1> > _290,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<long long, 1> >, 1> >
      _291,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<long long, 1> >, 1> >
      _292,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
              _293,
    long long _294, int _295, int _296);
  static void transposeTriplets(
    std::shared_ptr<monty::ndarray<int, 1> >    _309,
    std::shared_ptr<monty::ndarray<int, 1> >    _310,
    std::shared_ptr<monty::ndarray<double, 1> > _311,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<int, 1> >, 1> >
      _312,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<int, 1> >, 1> >
      _313,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
              _314,
    long long _315, int _316, int _317);
  static void tripletSort(
    std::shared_ptr<monty::ndarray<int, 1> >    _330,
    std::shared_ptr<monty::ndarray<int, 1> >    _331,
    std::shared_ptr<monty::ndarray<double, 1> > _332,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<int, 1> >, 1> >
      _333,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<int, 1> >, 1> >
      _334,
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
              _335,
    long long _336, int _337, int _338);
  static void argMSort(std::shared_ptr<monty::ndarray<int, 1> > _364,
                       std::shared_ptr<monty::ndarray<int, 1> > _365);
  static void mergeInto(std::shared_ptr<monty::ndarray<int, 1> > _370,
                        std::shared_ptr<monty::ndarray<int, 1> > _371,
                        std::shared_ptr<monty::ndarray<int, 1> > _372, int _373,
                        int _374, int _375);
  static void argQsort(std::shared_ptr<monty::ndarray<long long, 1> > _381,
                       std::shared_ptr<monty::ndarray<long long, 1> > _382,
                       std::shared_ptr<monty::ndarray<long long, 1> > _383,
                       long long _384, long long _385);
  static void argQsort(std::shared_ptr<monty::ndarray<long long, 1> > _386,
                       std::shared_ptr<monty::ndarray<int, 1> >       _387,
                       std::shared_ptr<monty::ndarray<int, 1> >       _388,
                       long long _389, long long _390);
}; // struct CommonTools;

struct p_SolutionStruct
{
  SolutionStruct*                         _pubthis;
  static mosek::fusion::p_SolutionStruct* _get_impl(
    mosek::fusion::SolutionStruct* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_SolutionStruct* _get_impl(
    mosek::fusion::SolutionStruct::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SolutionStruct(SolutionStruct* _pubthis);
  virtual ~p_SolutionStruct(){
    /* std::cout << "~p_SolutionStruct" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<double, 1> > snx{};
  std::shared_ptr<monty::ndarray<double, 1> > sux{};
  std::shared_ptr<monty::ndarray<double, 1> > slx{};
  std::shared_ptr<
    monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
    bars{};
  std::shared_ptr<
    monty::ndarray<std::shared_ptr<monty::ndarray<double, 1> >, 1> >
    barx{};
  std::shared_ptr<monty::ndarray<double, 1> > y{};
  std::shared_ptr<monty::ndarray<double, 1> > suc{};
  std::shared_ptr<monty::ndarray<double, 1> > slc{};
  std::shared_ptr<monty::ndarray<double, 1> > xx{};
  std::shared_ptr<monty::ndarray<double, 1> > xc{};
  double                        dobj{};
  double                        pobj{};
  mosek::fusion::ProblemStatus  probstatus{};
  mosek::fusion::SolutionStatus dstatus{};
  mosek::fusion::SolutionStatus pstatus{};
  int                           sol_numbarvar{};
  int                           sol_numcone{};
  int                           sol_numvar{};
  int                           sol_numcon{};
  virtual void                  destroy();
  static SolutionStruct::t _new_SolutionStruct(int _391, int _392, int _393,
                                               int _394);
  void _initialize(int _391, int _392, int _393, int _394);
  static SolutionStruct::t _new_SolutionStruct(
    monty::rc_ptr< ::mosek::fusion::SolutionStruct> _395);
  void _initialize(monty::rc_ptr< ::mosek::fusion::SolutionStruct> _395);
  virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct> clone();
  virtual void resize(int _398, int _399, int _400, int _401);
  virtual bool isDualAcceptable(mosek::fusion::AccSolutionStatus _425);
  virtual bool isPrimalAcceptable(mosek::fusion::AccSolutionStatus _426);
  virtual bool isAcceptable(mosek::fusion::SolutionStatus    _427,
                            mosek::fusion::AccSolutionStatus _428);
}; // struct SolutionStruct;

struct p_ConNZStruct
{
  ConNZStruct*                         _pubthis;
  static mosek::fusion::p_ConNZStruct* _get_impl(
    mosek::fusion::ConNZStruct* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_ConNZStruct* _get_impl(
    mosek::fusion::ConNZStruct::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ConNZStruct(ConNZStruct* _pubthis);
  virtual ~p_ConNZStruct(){ /* std::cout << "~p_ConNZStruct" << std::endl;*/ };
  std::shared_ptr<monty::ndarray<int, 1> >       barmidx{};
  std::shared_ptr<monty::ndarray<int, 1> >       barsubj{};
  std::shared_ptr<monty::ndarray<int, 1> >       barsubi{};
  std::shared_ptr<monty::ndarray<double, 1> >    bfix{};
  std::shared_ptr<monty::ndarray<double, 1> >    cof{};
  std::shared_ptr<monty::ndarray<int, 1> >       subj{};
  std::shared_ptr<monty::ndarray<long long, 1> > ptrb{};
  virtual void          destroy();
  static ConNZStruct::t _new_ConNZStruct(
    std::shared_ptr<monty::ndarray<long long, 1> > _429,
    std::shared_ptr<monty::ndarray<int, 1> >       _430,
    std::shared_ptr<monty::ndarray<double, 1> >    _431,
    std::shared_ptr<monty::ndarray<double, 1> >    _432,
    std::shared_ptr<monty::ndarray<int, 1> >       _433,
    std::shared_ptr<monty::ndarray<int, 1> >       _434,
    std::shared_ptr<monty::ndarray<int, 1> >       _435);
  void _initialize(std::shared_ptr<monty::ndarray<long long, 1> > _429,
                   std::shared_ptr<monty::ndarray<int, 1> >       _430,
                   std::shared_ptr<monty::ndarray<double, 1> >    _431,
                   std::shared_ptr<monty::ndarray<double, 1> >    _432,
                   std::shared_ptr<monty::ndarray<int, 1> >       _433,
                   std::shared_ptr<monty::ndarray<int, 1> >       _434,
                   std::shared_ptr<monty::ndarray<int, 1> >       _435);
}; // struct ConNZStruct;

struct p_BaseVariable : public /*implements*/ ::mosek::fusion::Variable
{
  BaseVariable*                         _pubthis;
  static mosek::fusion::p_BaseVariable* _get_impl(
    mosek::fusion::BaseVariable* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_BaseVariable* _get_impl(
    mosek::fusion::BaseVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_BaseVariable(BaseVariable* _pubthis);
  virtual ~p_BaseVariable(){
    /* std::cout << "~p_BaseVariable" << std::endl;*/
  };
  monty::rc_ptr< ::mosek::fusion::Model> model{};
  monty::rc_ptr< ::mosek::fusion::Set>   shape_p{};
  virtual void                           destroy();
  static BaseVariable::t                 _new_BaseVariable(
    monty::rc_ptr< ::mosek::fusion::BaseVariable> _2661,
    monty::rc_ptr< ::mosek::fusion::Model>        _2662);
  void _initialize(monty::rc_ptr< ::mosek::fusion::BaseVariable> _2661,
                   monty::rc_ptr< ::mosek::fusion::Model>        _2662);
  static BaseVariable::t _new_BaseVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2663,
    monty::rc_ptr< ::mosek::fusion::Set>   _2664);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _2663,
                   monty::rc_ptr< ::mosek::fusion::Set>   _2664);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2665,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2666);
  virtual void elementName(
    long long _2667, monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2668)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual std::string toString();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2674,
                    int _2675, int _2676, long long _2677, long long _2678,
                    std::shared_ptr<monty::ndarray<int, 1> > _2679,
                    std::shared_ptr<monty::ndarray<int, 1> > _2680,
                    std::shared_ptr<monty::ndarray<int, 1> > _2681);
  virtual void inst(long long _2683, long long               _2684,
                    std::shared_ptr<monty::ndarray<int, 1> > _2685,
                    std::shared_ptr<monty::ndarray<int, 1> > _2686,
                    std::shared_ptr<monty::ndarray<int, 1> > _2687)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2688,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2689,
                          bool _2690)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void set_values(long long _2691,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2692,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2693,
                          int _2694,
                          std::shared_ptr<monty::ndarray<double, 1> > _2695,
                          bool _2696)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void values(int _2697,
                      std::shared_ptr<monty::ndarray<double, 1> > _2698,
                      bool _2699);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2704,
                      int _2705,
                      std::shared_ptr<monty::ndarray<double, 1> > _2706,
                      bool _2707)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void values(long long _2708,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2709,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2710,
                      int _2711,
                      std::shared_ptr<monty::ndarray<double, 1> > _2712,
                      bool _2713)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void setLevel(std::shared_ptr<monty::ndarray<double, 1> > _2714);
  virtual monty::rc_ptr< ::mosek::fusion::Model> getModel();
  virtual monty::rc_ptr< ::mosek::fusion::Set>   shape();
  virtual monty::rc_ptr< ::mosek::fusion::Set>   getShape();
  virtual long long                              size();
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual();
  virtual std::shared_ptr<monty::ndarray<double, 1> > level();
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2719)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2720)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void                                      makeContinuous();
  virtual void                                      makeInteger();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> transpose();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(int _2723, int _2724,
                                                          int _2725);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(int _2726, int _2727);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    std::shared_ptr<monty::ndarray<int, 1> > _2728);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(int _2729);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2730,
    std::shared_ptr<monty::ndarray<int, 1> > _2731,
    std::shared_ptr<monty::ndarray<int, 1> > _2732);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2734,
    std::shared_ptr<monty::ndarray<int, 1> > _2735);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 2> > _2737);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2740);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag(int _2742);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag(int _2743);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> general_diag(
    std::shared_ptr<monty::ndarray<int, 1> > _2744,
    std::shared_ptr<monty::ndarray<int, 1> > _2745);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> asExpr();
  virtual monty::rc_ptr< ::mosek::fusion::Variable>   slice(
    std::shared_ptr<monty::ndarray<int, 1> > _2758,
    std::shared_ptr<monty::ndarray<int, 1> > _2759);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(int _2762, int _2763);
}; // struct BaseVariable;

struct p_CompoundVariable : public ::mosek::fusion::p_BaseVariable
{
  CompoundVariable*                         _pubthis;
  static mosek::fusion::p_CompoundVariable* _get_impl(
    mosek::fusion::CompoundVariable* _inst)
  {
    return static_cast<mosek::fusion::p_CompoundVariable*>(
      mosek::fusion::p_BaseVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_CompoundVariable* _get_impl(
    mosek::fusion::CompoundVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_CompoundVariable(CompoundVariable* _pubthis);
  virtual ~p_CompoundVariable(){
    /* std::cout << "~p_CompoundVariable" << std::endl;*/
  };
  int stackdim{};
  std::shared_ptr<monty::ndarray<int, 1> > varsb{};
  std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                             vars{};
  virtual void               destroy();
  static CompoundVariable::t _new_CompoundVariable(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
        _1299,
    int _1300);
  void _initialize(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
        _1299,
    int _1300);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _1306,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1307);
  virtual void elementName(
    long long                                            _1310,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1311);
  virtual void inst(long long _1314, long long               _1315,
                    std::shared_ptr<monty::ndarray<int, 1> > _1316,
                    std::shared_ptr<monty::ndarray<int, 1> > _1317,
                    std::shared_ptr<monty::ndarray<int, 1> > _1318);
  virtual void set_values(long long _1321,
                          std::shared_ptr<monty::ndarray<int, 1> >       _1322,
                          std::shared_ptr<monty::ndarray<long long, 1> > _1323,
                          int _1324,
                          std::shared_ptr<monty::ndarray<double, 1> > _1325,
                          bool _1326);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _1344,
                          std::shared_ptr<monty::ndarray<double, 1> >    _1345,
                          bool _1346);
  virtual void values(long long _1354,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1355,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1356,
                      int _1357,
                      std::shared_ptr<monty::ndarray<double, 1> > _1358,
                      bool _1359);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _1374,
                      int _1375,
                      std::shared_ptr<monty::ndarray<double, 1> > _1376,
                      bool _1377);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _1384);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _1391);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> asExpr();
  virtual monty::rc_ptr< ::mosek::fusion::Variable>   slice(
    std::shared_ptr<monty::ndarray<int, 1> > _1418,
    std::shared_ptr<monty::ndarray<int, 1> > _1419);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(int _1437, int _1438);
  static monty::rc_ptr< ::mosek::fusion::Set> compute_shape(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
        _1446,
    int _1447);
  static monty::rc_ptr< ::mosek::fusion::Model> model_from_var(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _1455);
}; // struct CompoundVariable;

struct p_RepeatVariable : public ::mosek::fusion::p_BaseVariable
{
  RepeatVariable*                         _pubthis;
  static mosek::fusion::p_RepeatVariable* _get_impl(
    mosek::fusion::RepeatVariable* _inst)
  {
    return static_cast<mosek::fusion::p_RepeatVariable*>(
      mosek::fusion::p_BaseVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_RepeatVariable* _get_impl(
    mosek::fusion::RepeatVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_RepeatVariable(RepeatVariable* _pubthis);
  virtual ~p_RepeatVariable(){
    /* std::cout << "~p_RepeatVariable" << std::endl;*/
  };
  long long d2{};
  long long d1{};
  long long d0{};
  int       dim{};
  int       count{};
  long long xsize{};
  std::shared_ptr<monty::ndarray<int, 1> > xdims{};
  monty::rc_ptr< ::mosek::fusion::Variable> x{};
  virtual void                              destroy();
  static RepeatVariable::t                  _new_RepeatVariable(
    monty::rc_ptr< ::mosek::fusion::Variable> _1456, int _1457, int _1458);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Variable> _1456, int _1457,
                   int _1458);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _1465,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1466);
  virtual void elementName(
    long long                                            _1471,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1472);
  virtual void inst(long long _1477, long long               _1478,
                    std::shared_ptr<monty::ndarray<int, 1> > _1479,
                    std::shared_ptr<monty::ndarray<int, 1> > _1480,
                    std::shared_ptr<monty::ndarray<int, 1> > _1481);
  virtual void set_values(long long _1486,
                          std::shared_ptr<monty::ndarray<int, 1> >       _1487,
                          std::shared_ptr<monty::ndarray<long long, 1> > _1488,
                          int _1489,
                          std::shared_ptr<monty::ndarray<double, 1> > _1490,
                          bool _1491);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _1506,
                          std::shared_ptr<monty::ndarray<double, 1> >    _1507,
                          bool _1508);
  virtual void values(long long _1517,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1518,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1519,
                      int _1520,
                      std::shared_ptr<monty::ndarray<double, 1> > _1521,
                      bool _1522);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _1536,
                      int _1537,
                      std::shared_ptr<monty::ndarray<double, 1> > _1538,
                      bool _1539);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _1548);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _1557);
  static monty::rc_ptr< ::mosek::fusion::Set> compute_shape(
    monty::rc_ptr< ::mosek::fusion::Variable> _1566, int _1567, int _1568);
}; // struct RepeatVariable;

struct p_PickVariable : public ::mosek::fusion::p_BaseVariable
{
  PickVariable*                         _pubthis;
  static mosek::fusion::p_PickVariable* _get_impl(
    mosek::fusion::PickVariable* _inst)
  {
    return static_cast<mosek::fusion::p_PickVariable*>(
      mosek::fusion::p_BaseVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_PickVariable* _get_impl(
    mosek::fusion::PickVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_PickVariable(PickVariable* _pubthis);
  virtual ~p_PickVariable(){
    /* std::cout << "~p_PickVariable" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<long long, 1> > indexes{};
  monty::rc_ptr< ::mosek::fusion::Variable> origin{};
  virtual void                              destroy();
  static PickVariable::t                    _new_PickVariable(
    monty::rc_ptr< ::mosek::fusion::Variable> _1577,
    std::shared_ptr<monty::ndarray<long long, 1> > _1578);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Variable> _1577,
                   std::shared_ptr<monty::ndarray<long long, 1> > _1578);
  virtual void inst(long long _1581, long long               _1582,
                    std::shared_ptr<monty::ndarray<int, 1> > _1583,
                    std::shared_ptr<monty::ndarray<int, 1> > _1584,
                    std::shared_ptr<monty::ndarray<int, 1> > _1585);
  virtual void set_values(long long _1586,
                          std::shared_ptr<monty::ndarray<int, 1> >       _1587,
                          std::shared_ptr<monty::ndarray<long long, 1> > _1588,
                          int _1589,
                          std::shared_ptr<monty::ndarray<double, 1> > _1590,
                          bool _1591);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _1594,
                          std::shared_ptr<monty::ndarray<double, 1> >    _1595,
                          bool _1596);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _1598,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1599);
  virtual void elementName(
    long long                                            _1600,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1601);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(int _1602, int _1603);
  virtual void values(long long _1605,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1606,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1607,
                      int _1608,
                      std::shared_ptr<monty::ndarray<double, 1> > _1609,
                      bool _1610);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _1613,
                      int _1614,
                      std::shared_ptr<monty::ndarray<double, 1> > _1615,
                      bool _1616);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _1619);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _1621);
}; // struct PickVariable;

struct p_SliceVariable : public ::mosek::fusion::p_BaseVariable
{
  SliceVariable*                         _pubthis;
  static mosek::fusion::p_SliceVariable* _get_impl(
    mosek::fusion::SliceVariable* _inst)
  {
    return static_cast<mosek::fusion::p_SliceVariable*>(
      mosek::fusion::p_BaseVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_SliceVariable* _get_impl(
    mosek::fusion::SliceVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SliceVariable(SliceVariable* _pubthis);
  virtual ~p_SliceVariable(){
    /* std::cout << "~p_SliceVariable" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<long long, 1> > strides{};
  long long                                 first{};
  monty::rc_ptr< ::mosek::fusion::Variable> origin{};
  virtual void                              destroy();
  static SliceVariable::t                   _new_SliceVariable(
    monty::rc_ptr< ::mosek::fusion::Variable> _1623,
    monty::rc_ptr< ::mosek::fusion::Set> _1624, long long _1625,
    std::shared_ptr<monty::ndarray<long long, 1> >        _1626);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Variable> _1623,
                   monty::rc_ptr< ::mosek::fusion::Set> _1624, long long _1625,
                   std::shared_ptr<monty::ndarray<long long, 1> >        _1626);
  virtual void inst(long long _1627, long long               _1628,
                    std::shared_ptr<monty::ndarray<int, 1> > _1629,
                    std::shared_ptr<monty::ndarray<int, 1> > _1630,
                    std::shared_ptr<monty::ndarray<int, 1> > _1631);
  virtual void set_values(long long _1636,
                          std::shared_ptr<monty::ndarray<int, 1> >       _1637,
                          std::shared_ptr<monty::ndarray<long long, 1> > _1638,
                          int _1639,
                          std::shared_ptr<monty::ndarray<double, 1> > _1640,
                          bool _1641);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _1660,
                          std::shared_ptr<monty::ndarray<double, 1> >    _1661,
                          bool _1662);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _1668,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1669);
  virtual void elementName(
    long long                                            _1674,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1675);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _1680,
    std::shared_ptr<monty::ndarray<int, 1> > _1681);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(int _1685, int _1686);
  virtual void values(long long _1687,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1688,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1689,
                      int _1690,
                      std::shared_ptr<monty::ndarray<double, 1> > _1691,
                      bool _1692);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _1710,
                      int _1711,
                      std::shared_ptr<monty::ndarray<double, 1> > _1712,
                      bool _1713);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _1719);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _1725);
}; // struct SliceVariable;

struct p_BoundInterfaceVariable : public ::mosek::fusion::p_SliceVariable
{
  BoundInterfaceVariable*                         _pubthis;
  static mosek::fusion::p_BoundInterfaceVariable* _get_impl(
    mosek::fusion::BoundInterfaceVariable* _inst)
  {
    return static_cast<mosek::fusion::p_BoundInterfaceVariable*>(
      mosek::fusion::p_SliceVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_BoundInterfaceVariable* _get_impl(
    mosek::fusion::BoundInterfaceVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_BoundInterfaceVariable(BoundInterfaceVariable* _pubthis);
  virtual ~p_BoundInterfaceVariable(){
    /* std::cout << "~p_BoundInterfaceVariable" << std::endl;*/
  };
  monty::rc_ptr< ::mosek::fusion::RangedVariable> originvar{};
  bool                                            islower{};
  virtual void                                    destroy();
  static BoundInterfaceVariable::t                _new_BoundInterfaceVariable(
    monty::rc_ptr< ::mosek::fusion::RangedVariable> _2499,
    monty::rc_ptr< ::mosek::fusion::Set> _2500, long long _2501,
    std::shared_ptr<monty::ndarray<long long, 1> > _2502, bool _2503);
  void _initialize(monty::rc_ptr< ::mosek::fusion::RangedVariable> _2499,
                   monty::rc_ptr< ::mosek::fusion::Set> _2500, long long _2501,
                   std::shared_ptr<monty::ndarray<long long, 1> >        _2502,
                   bool _2503);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice_(
    monty::rc_ptr< ::mosek::fusion::Set> _2504, long long _2505,
    std::shared_ptr<monty::ndarray<long long, 1> >        _2506);
  virtual void dual_values(long long _2507,
                           std::shared_ptr<monty::ndarray<int, 1> >       _2508,
                           std::shared_ptr<monty::ndarray<long long, 1> > _2509,
                           int _2510,
                           std::shared_ptr<monty::ndarray<double, 1> > _2511);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _2512,
                           int _2513,
                           std::shared_ptr<monty::ndarray<double, 1> > _2514);
}; // struct BoundInterfaceVariable;

struct p_ModelVariable : public ::mosek::fusion::p_BaseVariable
{
  ModelVariable*                         _pubthis;
  static mosek::fusion::p_ModelVariable* _get_impl(
    mosek::fusion::ModelVariable* _inst)
  {
    return static_cast<mosek::fusion::p_ModelVariable*>(
      mosek::fusion::p_BaseVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_ModelVariable* _get_impl(
    mosek::fusion::ModelVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ModelVariable(ModelVariable* _pubthis);
  virtual ~p_ModelVariable(){
    /* std::cout << "~p_ModelVariable" << std::endl;*/
  };
  long long               varid{};
  std::string             name{};
  virtual void            destroy();
  static ModelVariable::t _new_ModelVariable(
    monty::rc_ptr< ::mosek::fusion::ModelVariable> _2595,
    monty::rc_ptr< ::mosek::fusion::Model>         _2596);
  void _initialize(monty::rc_ptr< ::mosek::fusion::ModelVariable> _2595,
                   monty::rc_ptr< ::mosek::fusion::Model>         _2596);
  static ModelVariable::t _new_ModelVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2597, const std::string& _2598,
    monty::rc_ptr< ::mosek::fusion::Set> _2599, long long _2600);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _2597,
                   const std::string&                     _2598,
                   monty::rc_ptr< ::mosek::fusion::Set> _2599, long long _2600);
  virtual void flushNames()
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void elementName(
    long long                                            _2601,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2602);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _2603,
    std::shared_ptr<monty::ndarray<int, 1> > _2604);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(int _2610, int _2611);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2613)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
}; // struct ModelVariable;

struct p_SymRangedVariable
  : public ::mosek::fusion::p_ModelVariable,
    public /*implements*/ ::mosek::fusion::SymmetricVariable
{
  SymRangedVariable*                         _pubthis;
  static mosek::fusion::p_SymRangedVariable* _get_impl(
    mosek::fusion::SymRangedVariable* _inst)
  {
    return static_cast<mosek::fusion::p_SymRangedVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_SymRangedVariable* _get_impl(
    mosek::fusion::SymRangedVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SymRangedVariable(SymRangedVariable* _pubthis);
  virtual ~p_SymRangedVariable(){
    /* std::cout << "~p_SymRangedVariable" << std::endl;*/
  };
  int  dim{};
  bool names_flushed{};
  std::shared_ptr<monty::ndarray<int, 1> > nativeidxs{};
  monty::rc_ptr< ::mosek::fusion::RangeDomain>   dom{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  virtual void                                   destroy();
  static SymRangedVariable::t                    _new_SymRangedVariable(
    monty::rc_ptr< ::mosek::fusion::SymRangedVariable> _1731,
    monty::rc_ptr< ::mosek::fusion::Model>             _1732);
  void _initialize(monty::rc_ptr< ::mosek::fusion::SymRangedVariable> _1731,
                   monty::rc_ptr< ::mosek::fusion::Model>             _1732);
  static SymRangedVariable::t _new_SymRangedVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _1734, const std::string& _1735,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _1736, int _1737,
    std::shared_ptr<monty::ndarray<int, 1> > _1738, long long _1739);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>       _1734,
                   const std::string&                           _1735,
                   monty::rc_ptr< ::mosek::fusion::RangeDomain> _1736,
                   int _1737, std::shared_ptr<monty::ndarray<int, 1> > _1738,
                   long long _1739);
  virtual std::string toString();
  virtual void        flushNames();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _1746,
                    int _1747, int _1748, long long _1749, long long _1750,
                    std::shared_ptr<monty::ndarray<int, 1> > _1751,
                    std::shared_ptr<monty::ndarray<int, 1> > _1752,
                    std::shared_ptr<monty::ndarray<int, 1> > _1753);
  virtual void inst(long long _1759, long long               _1760,
                    std::shared_ptr<monty::ndarray<int, 1> > _1761,
                    std::shared_ptr<monty::ndarray<int, 1> > _1762,
                    std::shared_ptr<monty::ndarray<int, 1> > _1763);
  virtual void dual_u(long long _1768,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1769,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1770,
                      int _1771,
                      std::shared_ptr<monty::ndarray<double, 1> > _1772);
  virtual void dual_u(std::shared_ptr<monty::ndarray<long long, 1> > _1784,
                      int _1785,
                      std::shared_ptr<monty::ndarray<double, 1> > _1786);
  virtual void dual_l(long long _1794,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1795,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1796,
                      int _1797,
                      std::shared_ptr<monty::ndarray<double, 1> > _1798);
  virtual void dual_l(std::shared_ptr<monty::ndarray<long long, 1> > _1810,
                      int _1811,
                      std::shared_ptr<monty::ndarray<double, 1> > _1812);
  virtual void dual_values(long long _1819,
                           std::shared_ptr<monty::ndarray<int, 1> >       _1820,
                           std::shared_ptr<monty::ndarray<long long, 1> > _1821,
                           int _1822,
                           std::shared_ptr<monty::ndarray<double, 1> > _1823);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _1835,
                           int _1836,
                           std::shared_ptr<monty::ndarray<double, 1> > _1837);
  virtual void set_values(long long _1845,
                          std::shared_ptr<monty::ndarray<int, 1> >       _1846,
                          std::shared_ptr<monty::ndarray<long long, 1> > _1847,
                          int _1848,
                          std::shared_ptr<monty::ndarray<double, 1> > _1849,
                          bool _1850);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _1864,
                          std::shared_ptr<monty::ndarray<double, 1> >    _1865,
                          bool _1866);
  virtual void values(long long _1876,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1877,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1878,
                      int _1879,
                      std::shared_ptr<monty::ndarray<double, 1> > _1880,
                      bool _1881);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _1890,
                      int _1891,
                      std::shared_ptr<monty::ndarray<double, 1> > _1892,
                      bool _1893);
  virtual long long tril_idx(long long _1900);
  virtual long long tril_lin_idx(long long _1903);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _1906);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _1909);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _1912);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> asExpr() /*override*/
  {
    return mosek::fusion::p_BaseVariable::asExpr();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    int _2610, int _2611) /*override*/
  {
    return mosek::fusion::p_ModelVariable::slice(_2610, _2611);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 2> > _2737) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2737);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2740) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2740);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag();
  }
  virtual void makeContinuous() /*override*/
  {
    mosek::fusion::p_BaseVariable::makeContinuous();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> shape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::shape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2730,
    std::shared_ptr<monty::ndarray<int, 1> > _2731,
    std::shared_ptr<monty::ndarray<int, 1> > _2732) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2730, _2731, _2732);
  }
  virtual void elementName(
    long long                                            _2601,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2602) /*override*/
  {
    mosek::fusion::p_ModelVariable::elementName(_2601, _2602);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2729) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2729);
  }
  virtual void makeInteger() /*override*/
  {
    mosek::fusion::p_BaseVariable::makeInteger();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2723, int _2724, int _2725) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2723, _2724, _2725);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2726, int _2727) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2726, _2727);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> getShape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getShape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> transpose() /*override*/
  {
    return mosek::fusion::p_BaseVariable::transpose();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    std::shared_ptr<monty::ndarray<int, 1> > _2728) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2728);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2734,
    std::shared_ptr<monty::ndarray<int, 1> > _2735) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2734, _2735);
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > level() /*override*/
  {
    return mosek::fusion::p_BaseVariable::level();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Model> getModel() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getModel();
  }
  virtual void setLevel(
    std::shared_ptr<monty::ndarray<double, 1> > _2714) /*override*/
  {
    mosek::fusion::p_BaseVariable::setLevel(_2714);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag(int _2743) /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag(_2743);
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual() /*override*/
  {
    return mosek::fusion::p_BaseVariable::dual();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _2603,
    std::shared_ptr<monty::ndarray<int, 1> > _2604) /*override*/
  {
    return mosek::fusion::p_ModelVariable::slice(_2603, _2604);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2665,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2666) /*override*/
  {
    return mosek::fusion::p_BaseVariable::elementDesc(_2665, _2666);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag(
    int _2742) /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag(_2742);
  }
  virtual long long size() /*override*/
  {
    return mosek::fusion::p_BaseVariable::size();
  }
  virtual void values(int _2697,
                      std::shared_ptr<monty::ndarray<double, 1> > _2698,
                      bool _2699) /*override*/
  {
    mosek::fusion::p_BaseVariable::values(_2697, _2698, _2699);
  }
}; // struct SymRangedVariable;

struct p_RangedVariable : public ::mosek::fusion::p_ModelVariable
{
  RangedVariable*                         _pubthis;
  static mosek::fusion::p_RangedVariable* _get_impl(
    mosek::fusion::RangedVariable* _inst)
  {
    return static_cast<mosek::fusion::p_RangedVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_RangedVariable* _get_impl(
    mosek::fusion::RangedVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_RangedVariable(RangedVariable* _pubthis);
  virtual ~p_RangedVariable(){
    /* std::cout << "~p_RangedVariable" << std::endl;*/
  };
  bool names_flushed{};
  std::shared_ptr<monty::ndarray<int, 1> > nativeidxs{};
  monty::rc_ptr< ::mosek::fusion::RangeDomain>   dom{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  virtual void                                   destroy();
  static RangedVariable::t                       _new_RangedVariable(
    monty::rc_ptr< ::mosek::fusion::RangedVariable> _1913,
    monty::rc_ptr< ::mosek::fusion::Model>          _1914);
  void _initialize(monty::rc_ptr< ::mosek::fusion::RangedVariable> _1913,
                   monty::rc_ptr< ::mosek::fusion::Model>          _1914);
  static RangedVariable::t _new_RangedVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _1916, const std::string& _1917,
    monty::rc_ptr< ::mosek::fusion::Set>         _1918,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _1919,
    std::shared_ptr<monty::ndarray<int, 1> > _1920, long long _1921);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>       _1916,
                   const std::string&                           _1917,
                   monty::rc_ptr< ::mosek::fusion::Set>         _1918,
                   monty::rc_ptr< ::mosek::fusion::RangeDomain> _1919,
                   std::shared_ptr<monty::ndarray<int, 1> > _1920,
                   long long _1921);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _1922,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _1923);
  virtual void flushNames();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _1928,
                    int _1929, int _1930, long long _1931, long long _1932,
                    std::shared_ptr<monty::ndarray<int, 1> > _1933,
                    std::shared_ptr<monty::ndarray<int, 1> > _1934,
                    std::shared_ptr<monty::ndarray<int, 1> > _1935);
  virtual void inst(long long _1939, long long               _1940,
                    std::shared_ptr<monty::ndarray<int, 1> > _1941,
                    std::shared_ptr<monty::ndarray<int, 1> > _1942,
                    std::shared_ptr<monty::ndarray<int, 1> > _1943);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> upperBoundVar();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> lowerBoundVar();
  virtual void dual_u(long long _1950,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1951,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1952,
                      int _1953,
                      std::shared_ptr<monty::ndarray<double, 1> > _1954);
  virtual void dual_u(std::shared_ptr<monty::ndarray<long long, 1> > _1965,
                      int _1966,
                      std::shared_ptr<monty::ndarray<double, 1> > _1967);
  virtual void dual_l(long long _1974,
                      std::shared_ptr<monty::ndarray<int, 1> >       _1975,
                      std::shared_ptr<monty::ndarray<long long, 1> > _1976,
                      int _1977,
                      std::shared_ptr<monty::ndarray<double, 1> > _1978);
  virtual void dual_l(std::shared_ptr<monty::ndarray<long long, 1> > _1989,
                      int _1990,
                      std::shared_ptr<monty::ndarray<double, 1> > _1991);
  virtual void dual_values(long long _1998,
                           std::shared_ptr<monty::ndarray<int, 1> >       _1999,
                           std::shared_ptr<monty::ndarray<long long, 1> > _2000,
                           int _2001,
                           std::shared_ptr<monty::ndarray<double, 1> > _2002);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _2014,
                           int _2015,
                           std::shared_ptr<monty::ndarray<double, 1> > _2016);
  virtual void set_values(long long _2024,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2025,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2026,
                          int _2027,
                          std::shared_ptr<monty::ndarray<double, 1> > _2028,
                          bool _2029);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2043,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2044,
                          bool _2045);
  virtual void values(long long _2055,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2056,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2057,
                      int _2058,
                      std::shared_ptr<monty::ndarray<double, 1> > _2059,
                      bool _2060);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2069,
                      int _2070,
                      std::shared_ptr<monty::ndarray<double, 1> > _2071,
                      bool _2072);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2077);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2080);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2083);
}; // struct RangedVariable;

struct p_LinearPSDVariable : public ::mosek::fusion::p_ModelVariable
{
  LinearPSDVariable*                         _pubthis;
  static mosek::fusion::p_LinearPSDVariable* _get_impl(
    mosek::fusion::LinearPSDVariable* _inst)
  {
    return static_cast<mosek::fusion::p_LinearPSDVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_LinearPSDVariable* _get_impl(
    mosek::fusion::LinearPSDVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_LinearPSDVariable(LinearPSDVariable* _pubthis);
  virtual ~p_LinearPSDVariable(){
    /* std::cout << "~p_LinearPSDVariable" << std::endl;*/
  };
  int                         numcones{};
  int                         coneidx{};
  int                         conesize{};
  int                         sdpvardim{};
  int                         blocksize{};
  virtual void                destroy();
  static LinearPSDVariable::t _new_LinearPSDVariable(
    monty::rc_ptr< ::mosek::fusion::LinearPSDVariable> _2084,
    monty::rc_ptr< ::mosek::fusion::Model>             _2085);
  void _initialize(monty::rc_ptr< ::mosek::fusion::LinearPSDVariable> _2084,
                   monty::rc_ptr< ::mosek::fusion::Model>             _2085);
  static LinearPSDVariable::t _new_LinearPSDVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2086, const std::string& _2087,
    int _2088, monty::rc_ptr< ::mosek::fusion::Set> _2089, int _2090,
    long long _2091);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _2086,
                   const std::string& _2087, int _2088,
                   monty::rc_ptr< ::mosek::fusion::Set> _2089, int _2090,
                   long long _2091);
  virtual void        flushNames();
  virtual std::string toString();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2095,
                    int _2096, int _2097, long long _2098, long long _2099,
                    std::shared_ptr<monty::ndarray<int, 1> > _2100,
                    std::shared_ptr<monty::ndarray<int, 1> > _2101,
                    std::shared_ptr<monty::ndarray<int, 1> > _2102);
  virtual void inst(long long _2112, long long               _2113,
                    std::shared_ptr<monty::ndarray<int, 1> > _2114,
                    std::shared_ptr<monty::ndarray<int, 1> > _2115,
                    std::shared_ptr<monty::ndarray<int, 1> > _2116);
  virtual void set_values(long long _2122,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2123,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2124,
                          int _2125,
                          std::shared_ptr<monty::ndarray<double, 1> > _2126,
                          bool _2127);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2128,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2129,
                          bool _2130);
  virtual void values(long long _2131,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2132,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2133,
                      int _2134,
                      std::shared_ptr<monty::ndarray<double, 1> > _2135,
                      bool _2136);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2145,
                      int _2146,
                      std::shared_ptr<monty::ndarray<double, 1> > _2147,
                      bool _2148);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2154);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2155);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2156);
}; // struct LinearPSDVariable;

struct p_PSDVariable : public ::mosek::fusion::p_ModelVariable,
                       public /*implements*/ ::mosek::fusion::SymmetricVariable
{
  PSDVariable*                         _pubthis;
  static mosek::fusion::p_PSDVariable* _get_impl(
    mosek::fusion::PSDVariable* _inst)
  {
    return static_cast<mosek::fusion::p_PSDVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_PSDVariable* _get_impl(
    mosek::fusion::PSDVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_PSDVariable(PSDVariable* _pubthis);
  virtual ~p_PSDVariable(){ /* std::cout << "~p_PSDVariable" << std::endl;*/ };
  int                   numcones{};
  int                   coneidx{};
  int                   conesize{};
  virtual void          destroy();
  static PSDVariable::t _new_PSDVariable(
    monty::rc_ptr< ::mosek::fusion::PSDVariable> _2157,
    monty::rc_ptr< ::mosek::fusion::Model>       _2158);
  void _initialize(monty::rc_ptr< ::mosek::fusion::PSDVariable> _2157,
                   monty::rc_ptr< ::mosek::fusion::Model>       _2158);
  static PSDVariable::t _new_PSDVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2159, const std::string& _2160,
    int _2161, int _2162, int _2163, long long _2164);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _2159,
                   const std::string& _2160, int _2161, int _2162, int _2163,
                   long long _2164);
  virtual void                                                 flushNames();
  virtual std::string                                          toString();
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2167,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2168);
  virtual void elementName(
    long long                                            _2174,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2175);
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2181,
                    int _2182, int _2183, long long _2184, long long _2185,
                    std::shared_ptr<monty::ndarray<int, 1> > _2186,
                    std::shared_ptr<monty::ndarray<int, 1> > _2187,
                    std::shared_ptr<monty::ndarray<int, 1> > _2188);
  virtual void inst(long long _2197, long long               _2198,
                    std::shared_ptr<monty::ndarray<int, 1> > _2199,
                    std::shared_ptr<monty::ndarray<int, 1> > _2200,
                    std::shared_ptr<monty::ndarray<int, 1> > _2201);
  virtual void set_values(long long _2206,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2207,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2208,
                          int _2209,
                          std::shared_ptr<monty::ndarray<double, 1> > _2210,
                          bool _2211);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2212,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2213,
                          bool _2214);
  virtual void values(long long _2215,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2216,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2217,
                      int _2218,
                      std::shared_ptr<monty::ndarray<double, 1> > _2219,
                      bool _2220);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2232,
                      int _2233,
                      std::shared_ptr<monty::ndarray<double, 1> > _2234,
                      bool _2235);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2243);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2244);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2245);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> asExpr() /*override*/
  {
    return mosek::fusion::p_BaseVariable::asExpr();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    int _2610, int _2611) /*override*/
  {
    return mosek::fusion::p_ModelVariable::slice(_2610, _2611);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 2> > _2737) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2737);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2740) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2740);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag();
  }
  virtual void makeContinuous() /*override*/
  {
    mosek::fusion::p_BaseVariable::makeContinuous();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> shape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::shape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2730,
    std::shared_ptr<monty::ndarray<int, 1> > _2731,
    std::shared_ptr<monty::ndarray<int, 1> > _2732) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2730, _2731, _2732);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2729) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2729);
  }
  virtual void makeInteger() /*override*/
  {
    mosek::fusion::p_BaseVariable::makeInteger();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2723, int _2724, int _2725) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2723, _2724, _2725);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2726, int _2727) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2726, _2727);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> getShape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getShape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> transpose() /*override*/
  {
    return mosek::fusion::p_BaseVariable::transpose();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    std::shared_ptr<monty::ndarray<int, 1> > _2728) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2728);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2734,
    std::shared_ptr<monty::ndarray<int, 1> > _2735) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2734, _2735);
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > level() /*override*/
  {
    return mosek::fusion::p_BaseVariable::level();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Model> getModel() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getModel();
  }
  virtual void setLevel(
    std::shared_ptr<monty::ndarray<double, 1> > _2714) /*override*/
  {
    mosek::fusion::p_BaseVariable::setLevel(_2714);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag(int _2743) /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag(_2743);
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual() /*override*/
  {
    return mosek::fusion::p_BaseVariable::dual();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _2603,
    std::shared_ptr<monty::ndarray<int, 1> > _2604) /*override*/
  {
    return mosek::fusion::p_ModelVariable::slice(_2603, _2604);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag(
    int _2742) /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag(_2742);
  }
  virtual long long size() /*override*/
  {
    return mosek::fusion::p_BaseVariable::size();
  }
  virtual void values(int _2697,
                      std::shared_ptr<monty::ndarray<double, 1> > _2698,
                      bool _2699) /*override*/
  {
    mosek::fusion::p_BaseVariable::values(_2697, _2698, _2699);
  }
}; // struct PSDVariable;

struct p_SymLinearVariable
  : public ::mosek::fusion::p_ModelVariable,
    public /*implements*/ ::mosek::fusion::SymmetricVariable
{
  SymLinearVariable*                         _pubthis;
  static mosek::fusion::p_SymLinearVariable* _get_impl(
    mosek::fusion::SymLinearVariable* _inst)
  {
    return static_cast<mosek::fusion::p_SymLinearVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_SymLinearVariable* _get_impl(
    mosek::fusion::SymLinearVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SymLinearVariable(SymLinearVariable* _pubthis);
  virtual ~p_SymLinearVariable(){
    /* std::cout << "~p_SymLinearVariable" << std::endl;*/
  };
  int                                                    dim{};
  bool                                                   names_flushed{};
  monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> dom{};
  std::shared_ptr<monty::ndarray<int, 1> > nativeidxs{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  virtual void                                   destroy();
  static SymLinearVariable::t                    _new_SymLinearVariable(
    monty::rc_ptr< ::mosek::fusion::SymLinearVariable> _2246,
    monty::rc_ptr< ::mosek::fusion::Model>             _2247);
  void _initialize(monty::rc_ptr< ::mosek::fusion::SymLinearVariable> _2246,
                   monty::rc_ptr< ::mosek::fusion::Model>             _2247);
  static SymLinearVariable::t _new_SymLinearVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2249, const std::string& _2250,
    monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> _2251, int _2252,
    std::shared_ptr<monty::ndarray<int, 1> > _2253, long long _2254);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>                 _2249,
                   const std::string&                                     _2250,
                   monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> _2251,
                   int _2252, std::shared_ptr<monty::ndarray<int, 1> > _2253,
                   long long _2254);
  virtual std::string toString();
  virtual void        flushNames();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2264,
                    int _2265, int _2266, long long _2267, long long _2268,
                    std::shared_ptr<monty::ndarray<int, 1> > _2269,
                    std::shared_ptr<monty::ndarray<int, 1> > _2270,
                    std::shared_ptr<monty::ndarray<int, 1> > _2271);
  virtual void inst(long long _2277, long long               _2278,
                    std::shared_ptr<monty::ndarray<int, 1> > _2279,
                    std::shared_ptr<monty::ndarray<int, 1> > _2280,
                    std::shared_ptr<monty::ndarray<int, 1> > _2281);
  virtual void dual_values(long long _2285,
                           std::shared_ptr<monty::ndarray<int, 1> >       _2286,
                           std::shared_ptr<monty::ndarray<long long, 1> > _2287,
                           int _2288,
                           std::shared_ptr<monty::ndarray<double, 1> > _2289);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _2301,
                           int _2302,
                           std::shared_ptr<monty::ndarray<double, 1> > _2303);
  virtual void set_values(long long _2311,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2312,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2313,
                          int _2314,
                          std::shared_ptr<monty::ndarray<double, 1> > _2315,
                          bool _2316);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2330,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2331,
                          bool _2332);
  virtual void values(long long _2340,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2341,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2342,
                      int _2343,
                      std::shared_ptr<monty::ndarray<double, 1> > _2344,
                      bool _2345);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2355,
                      int _2356,
                      std::shared_ptr<monty::ndarray<double, 1> > _2357,
                      bool _2358);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2366);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2369);
  virtual long long tril_idx(long long _2372);
  virtual long long tril_lin_idx(long long _2375);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2378);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> asExpr() /*override*/
  {
    return mosek::fusion::p_BaseVariable::asExpr();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    int _2610, int _2611) /*override*/
  {
    return mosek::fusion::p_ModelVariable::slice(_2610, _2611);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 2> > _2737) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2737);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2740) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2740);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag();
  }
  virtual void makeContinuous() /*override*/
  {
    mosek::fusion::p_BaseVariable::makeContinuous();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> shape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::shape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2730,
    std::shared_ptr<monty::ndarray<int, 1> > _2731,
    std::shared_ptr<monty::ndarray<int, 1> > _2732) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2730, _2731, _2732);
  }
  virtual void elementName(
    long long                                            _2601,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2602) /*override*/
  {
    mosek::fusion::p_ModelVariable::elementName(_2601, _2602);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2729) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2729);
  }
  virtual void makeInteger() /*override*/
  {
    mosek::fusion::p_BaseVariable::makeInteger();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2723, int _2724, int _2725) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2723, _2724, _2725);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2726, int _2727) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2726, _2727);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> getShape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getShape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> transpose() /*override*/
  {
    return mosek::fusion::p_BaseVariable::transpose();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    std::shared_ptr<monty::ndarray<int, 1> > _2728) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2728);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2734,
    std::shared_ptr<monty::ndarray<int, 1> > _2735) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2734, _2735);
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > level() /*override*/
  {
    return mosek::fusion::p_BaseVariable::level();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Model> getModel() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getModel();
  }
  virtual void setLevel(
    std::shared_ptr<monty::ndarray<double, 1> > _2714) /*override*/
  {
    mosek::fusion::p_BaseVariable::setLevel(_2714);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag(int _2743) /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag(_2743);
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual() /*override*/
  {
    return mosek::fusion::p_BaseVariable::dual();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _2603,
    std::shared_ptr<monty::ndarray<int, 1> > _2604) /*override*/
  {
    return mosek::fusion::p_ModelVariable::slice(_2603, _2604);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2665,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2666) /*override*/
  {
    return mosek::fusion::p_BaseVariable::elementDesc(_2665, _2666);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag(
    int _2742) /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag(_2742);
  }
  virtual long long size() /*override*/
  {
    return mosek::fusion::p_BaseVariable::size();
  }
  virtual void values(int _2697,
                      std::shared_ptr<monty::ndarray<double, 1> > _2698,
                      bool _2699) /*override*/
  {
    mosek::fusion::p_BaseVariable::values(_2697, _2698, _2699);
  }
}; // struct SymLinearVariable;

struct p_LinearVariable : public ::mosek::fusion::p_ModelVariable
{
  LinearVariable*                         _pubthis;
  static mosek::fusion::p_LinearVariable* _get_impl(
    mosek::fusion::LinearVariable* _inst)
  {
    return static_cast<mosek::fusion::p_LinearVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_LinearVariable* _get_impl(
    mosek::fusion::LinearVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_LinearVariable(LinearVariable* _pubthis);
  virtual ~p_LinearVariable(){
    /* std::cout << "~p_LinearVariable" << std::endl;*/
  };
  bool                                          names_flushed{};
  monty::rc_ptr< ::mosek::fusion::LinearDomain> dom{};
  std::shared_ptr<monty::ndarray<int, 1> > nativeidxs{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  virtual void                                   destroy();
  static LinearVariable::t                       _new_LinearVariable(
    monty::rc_ptr< ::mosek::fusion::LinearVariable> _2379,
    monty::rc_ptr< ::mosek::fusion::Model>          _2380);
  void _initialize(monty::rc_ptr< ::mosek::fusion::LinearVariable> _2379,
                   monty::rc_ptr< ::mosek::fusion::Model>          _2380);
  static LinearVariable::t _new_LinearVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2382, const std::string& _2383,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _2384,
    monty::rc_ptr< ::mosek::fusion::Set>          _2385,
    std::shared_ptr<monty::ndarray<int, 1> > _2386, long long _2387);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>        _2382,
                   const std::string&                            _2383,
                   monty::rc_ptr< ::mosek::fusion::LinearDomain> _2384,
                   monty::rc_ptr< ::mosek::fusion::Set>          _2385,
                   std::shared_ptr<monty::ndarray<int, 1> > _2386,
                   long long _2387);
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2388,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2389);
  virtual void elementName(
    long long                                            _2390,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2391);
  virtual void flushNames();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2396,
                    int _2397, int _2398, long long _2399, long long _2400,
                    std::shared_ptr<monty::ndarray<int, 1> > _2401,
                    std::shared_ptr<monty::ndarray<int, 1> > _2402,
                    std::shared_ptr<monty::ndarray<int, 1> > _2403);
  virtual void inst(long long _2407, long long               _2408,
                    std::shared_ptr<monty::ndarray<int, 1> > _2409,
                    std::shared_ptr<monty::ndarray<int, 1> > _2410,
                    std::shared_ptr<monty::ndarray<int, 1> > _2411);
  virtual void dual_values(long long _2413,
                           std::shared_ptr<monty::ndarray<int, 1> >       _2414,
                           std::shared_ptr<monty::ndarray<long long, 1> > _2415,
                           int _2416,
                           std::shared_ptr<monty::ndarray<double, 1> > _2417);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _2429,
                           int _2430,
                           std::shared_ptr<monty::ndarray<double, 1> > _2431);
  virtual void set_values(long long _2439,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2440,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2441,
                          int _2442,
                          std::shared_ptr<monty::ndarray<double, 1> > _2443,
                          bool _2444);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2458,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2459,
                          bool _2460);
  virtual void values(long long _2470,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2471,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2472,
                      int _2473,
                      std::shared_ptr<monty::ndarray<double, 1> > _2474,
                      bool _2475);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2484);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2487);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2490,
                      int _2491,
                      std::shared_ptr<monty::ndarray<double, 1> > _2492,
                      bool _2493);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2498);
}; // struct LinearVariable;

struct p_ConicVariable : public ::mosek::fusion::p_ModelVariable
{
  ConicVariable*                         _pubthis;
  static mosek::fusion::p_ConicVariable* _get_impl(
    mosek::fusion::ConicVariable* _inst)
  {
    return static_cast<mosek::fusion::p_ConicVariable*>(
      mosek::fusion::p_ModelVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_ConicVariable* _get_impl(
    mosek::fusion::ConicVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ConicVariable(ConicVariable* _pubthis);
  virtual ~p_ConicVariable(){
    /* std::cout << "~p_ConicVariable" << std::endl;*/
  };
  bool names_flushed{};
  std::shared_ptr<monty::ndarray<int, 1> > nativeidxs{};
  monty::rc_ptr< ::mosek::fusion::QConeDomain> dom{};
  int                                          numcone{};
  int                                          conesize{};
  int                                          coneidx{};
  virtual void                                 destroy();
  static ConicVariable::t                      _new_ConicVariable(
    monty::rc_ptr< ::mosek::fusion::ConicVariable> _2515,
    monty::rc_ptr< ::mosek::fusion::Model>         _2516);
  void _initialize(monty::rc_ptr< ::mosek::fusion::ConicVariable> _2515,
                   monty::rc_ptr< ::mosek::fusion::Model>         _2516);
  static ConicVariable::t _new_ConicVariable(
    monty::rc_ptr< ::mosek::fusion::Model> _2518, const std::string& _2519,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _2520,
    monty::rc_ptr< ::mosek::fusion::Set>         _2521,
    std::shared_ptr<monty::ndarray<int, 1> > _2522, int _2523, int _2524,
    int _2525, long long _2526);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>       _2518,
                   const std::string&                           _2519,
                   monty::rc_ptr< ::mosek::fusion::QConeDomain> _2520,
                   monty::rc_ptr< ::mosek::fusion::Set>         _2521,
                   std::shared_ptr<monty::ndarray<int, 1> > _2522, int _2523,
                   int _2524, int _2525, long long _2526);
  virtual std::string                                          toString();
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2529,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2530);
  virtual void elementName(
    long long                                            _2531,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2532);
  virtual void flushNames();
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2534,
                    int _2535, int _2536, long long _2537, long long _2538,
                    std::shared_ptr<monty::ndarray<int, 1> > _2539,
                    std::shared_ptr<monty::ndarray<int, 1> > _2540,
                    std::shared_ptr<monty::ndarray<int, 1> > _2541);
  virtual void inst(long long _2543, long long               _2544,
                    std::shared_ptr<monty::ndarray<int, 1> > _2545,
                    std::shared_ptr<monty::ndarray<int, 1> > _2546,
                    std::shared_ptr<monty::ndarray<int, 1> > _2547);
  virtual void set_values(long long _2548,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2549,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2550,
                          int _2551,
                          std::shared_ptr<monty::ndarray<double, 1> > _2552,
                          bool _2553);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2563,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2564,
                          bool _2565);
  virtual void values(long long _2571,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2572,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2573,
                      int _2574,
                      std::shared_ptr<monty::ndarray<double, 1> > _2575,
                      bool _2576);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2583,
                      int _2584,
                      std::shared_ptr<monty::ndarray<double, 1> > _2585,
                      bool _2586);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2589);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2591);
  virtual int get_variable_index(int _2593);
  virtual monty::rc_ptr< ::mosek::fusion::ModelVariable> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _2594);
}; // struct ConicVariable;

struct p_NilVariable : public ::mosek::fusion::p_BaseVariable,
                       public /*implements*/ ::mosek::fusion::SymmetricVariable
{
  NilVariable*                         _pubthis;
  static mosek::fusion::p_NilVariable* _get_impl(
    mosek::fusion::NilVariable* _inst)
  {
    return static_cast<mosek::fusion::p_NilVariable*>(
      mosek::fusion::p_BaseVariable::_get_impl(_inst));
  }
  static mosek::fusion::p_NilVariable* _get_impl(
    mosek::fusion::NilVariable::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_NilVariable(NilVariable* _pubthis);
  virtual ~p_NilVariable(){ /* std::cout << "~p_NilVariable" << std::endl;*/ };
  virtual void          destroy();
  static NilVariable::t _new_NilVariable();
  void                  _initialize();
  virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> elementDesc(
    long long                                            _2614,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2615);
  virtual void elementName(
    long long                                            _2616,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2617);
  virtual void inst(std::shared_ptr<monty::ndarray<long long, 1> > _2618,
                    int _2619, int _2620, long long _2621, long long _2622,
                    std::shared_ptr<monty::ndarray<int, 1> > _2623,
                    std::shared_ptr<monty::ndarray<int, 1> > _2624,
                    std::shared_ptr<monty::ndarray<int, 1> > _2625);
  virtual void inst(long long _2626, long long               _2627,
                    std::shared_ptr<monty::ndarray<int, 1> > _2628,
                    std::shared_ptr<monty::ndarray<int, 1> > _2629,
                    std::shared_ptr<monty::ndarray<int, 1> > _2630);
  virtual void set_values(std::shared_ptr<monty::ndarray<long long, 1> > _2631,
                          std::shared_ptr<monty::ndarray<double, 1> >    _2632,
                          bool _2633);
  virtual void set_values(long long _2634,
                          std::shared_ptr<monty::ndarray<int, 1> >       _2635,
                          std::shared_ptr<monty::ndarray<long long, 1> > _2636,
                          int _2637,
                          std::shared_ptr<monty::ndarray<double, 1> > _2638,
                          bool _2639);
  virtual void values(std::shared_ptr<monty::ndarray<long long, 1> > _2640,
                      int _2641,
                      std::shared_ptr<monty::ndarray<double, 1> > _2642,
                      bool _2643);
  virtual void values(long long _2644,
                      std::shared_ptr<monty::ndarray<int, 1> >       _2645,
                      std::shared_ptr<monty::ndarray<long long, 1> > _2646,
                      int _2647,
                      std::shared_ptr<monty::ndarray<double, 1> > _2648,
                      bool _2649);
  virtual void make_continuous(
    std::shared_ptr<monty::ndarray<long long, 1> > _2650);
  virtual void make_integer(
    std::shared_ptr<monty::ndarray<long long, 1> > _2651);
  virtual void        makeContinuous();
  virtual void        makeInteger();
  virtual std::string toString();
  virtual long long   size();
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual();
  virtual std::shared_ptr<monty::ndarray<double, 1> > level();
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    std::shared_ptr<monty::ndarray<int, 1> > _2654);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(int _2655);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _2656,
    std::shared_ptr<monty::ndarray<int, 1> > _2657);
  virtual monty::rc_ptr< ::mosek::fusion::Variable> slice(int _2659, int _2660);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> asExpr() /*override*/
  {
    return mosek::fusion::p_BaseVariable::asExpr();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 2> > _2737) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2737);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2740) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2740);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> shape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::shape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2730,
    std::shared_ptr<monty::ndarray<int, 1> > _2731,
    std::shared_ptr<monty::ndarray<int, 1> > _2732) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2730, _2731, _2732);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2723, int _2724, int _2725) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2723, _2724, _2725);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag() /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> index(
    int _2726, int _2727) /*override*/
  {
    return mosek::fusion::p_BaseVariable::index(_2726, _2727);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> getShape() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getShape();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> transpose() /*override*/
  {
    return mosek::fusion::p_BaseVariable::transpose();
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _2734,
    std::shared_ptr<monty::ndarray<int, 1> > _2735) /*override*/
  {
    return mosek::fusion::p_BaseVariable::pick(_2734, _2735);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Model> getModel() /*override*/
  {
    return mosek::fusion::p_BaseVariable::getModel();
  }
  virtual void setLevel(
    std::shared_ptr<monty::ndarray<double, 1> > _2714) /*override*/
  {
    mosek::fusion::p_BaseVariable::setLevel(_2714);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> diag(int _2743) /*override*/
  {
    return mosek::fusion::p_BaseVariable::diag(_2743);
  }
  virtual monty::rc_ptr< ::mosek::fusion::Variable> antidiag(
    int _2742) /*override*/
  {
    return mosek::fusion::p_BaseVariable::antidiag(_2742);
  }
  virtual void values(int _2697,
                      std::shared_ptr<monty::ndarray<double, 1> > _2698,
                      bool _2699) /*override*/
  {
    mosek::fusion::p_BaseVariable::values(_2697, _2698, _2699);
  }
}; // struct NilVariable;

struct p_Var
{
  Var*                         _pubthis;
  static mosek::fusion::p_Var* _get_impl(mosek::fusion::Var* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Var* _get_impl(mosek::fusion::Var::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Var(Var* _pubthis);
  virtual ~p_Var(){ /* std::cout << "~p_Var" << std::endl;*/ };
  virtual void                                     destroy();
  static monty::rc_ptr< ::mosek::fusion::Variable> compress(
    monty::rc_ptr< ::mosek::fusion::Variable> _2826);
  static monty::rc_ptr< ::mosek::fusion::Variable> reshape(
    monty::rc_ptr< ::mosek::fusion::Variable> _2831, int _2832);
  static monty::rc_ptr< ::mosek::fusion::Variable> reshape(
    monty::rc_ptr< ::mosek::fusion::Variable> _2833, int _2834, int _2835);
  static monty::rc_ptr< ::mosek::fusion::Variable> flatten(
    monty::rc_ptr< ::mosek::fusion::Variable> _2836);
  static monty::rc_ptr< ::mosek::fusion::Variable> reshape(
    monty::rc_ptr< ::mosek::fusion::Variable> _2837,
    std::shared_ptr<monty::ndarray<int, 1> > _2838);
  static monty::rc_ptr< ::mosek::fusion::Variable> reshape(
    monty::rc_ptr< ::mosek::fusion::Variable> _2840,
    monty::rc_ptr< ::mosek::fusion::Set>      _2841);
  static monty::rc_ptr< ::mosek::fusion::Variable> reshape_(
    monty::rc_ptr< ::mosek::fusion::Variable> _2843,
    monty::rc_ptr< ::mosek::fusion::Set>      _2844);
  static monty::rc_ptr< ::mosek::fusion::Variable> index_flip_(
    monty::rc_ptr< ::mosek::fusion::Variable> _2847,
    std::shared_ptr<monty::ndarray<int, 1> > _2848);
  static monty::rc_ptr< ::mosek::fusion::Variable> index_permute_(
    monty::rc_ptr< ::mosek::fusion::Variable> _2855,
    std::shared_ptr<monty::ndarray<int, 1> > _2856);
  static monty::rc_ptr< ::mosek::fusion::Variable> hrepeat(
    monty::rc_ptr< ::mosek::fusion::Variable> _2861, int _2862);
  static monty::rc_ptr< ::mosek::fusion::Variable> vrepeat(
    monty::rc_ptr< ::mosek::fusion::Variable> _2863, int _2864);
  static monty::rc_ptr< ::mosek::fusion::Variable> repeat(
    monty::rc_ptr< ::mosek::fusion::Variable> _2865, int _2866);
  static monty::rc_ptr< ::mosek::fusion::Variable> repeat(
    monty::rc_ptr< ::mosek::fusion::Variable> _2867, int _2868, int _2869);
  static monty::rc_ptr< ::mosek::fusion::Variable> drepeat(
    monty::rc_ptr< ::mosek::fusion::Variable> _2870, int _2871, int _2872);
  static monty::rc_ptr< ::mosek::fusion::Variable> stack(
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<
                       monty::rc_ptr< ::mosek::fusion::Variable>, 1> >,
                     1> >
      _2873);
  static monty::rc_ptr< ::mosek::fusion::Variable> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _2889,
    monty::rc_ptr< ::mosek::fusion::Variable> _2890,
    monty::rc_ptr< ::mosek::fusion::Variable> _2891);
  static monty::rc_ptr< ::mosek::fusion::Variable> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _2892,
    monty::rc_ptr< ::mosek::fusion::Variable> _2893);
  static monty::rc_ptr< ::mosek::fusion::Variable> vstack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _2894);
  static monty::rc_ptr< ::mosek::fusion::Variable> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _2895,
    monty::rc_ptr< ::mosek::fusion::Variable> _2896,
    monty::rc_ptr< ::mosek::fusion::Variable> _2897);
  static monty::rc_ptr< ::mosek::fusion::Variable> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _2898,
    monty::rc_ptr< ::mosek::fusion::Variable> _2899);
  static monty::rc_ptr< ::mosek::fusion::Variable> hstack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _2900);
  static monty::rc_ptr< ::mosek::fusion::Variable> stack(
    monty::rc_ptr< ::mosek::fusion::Variable> _2901,
    monty::rc_ptr< ::mosek::fusion::Variable> _2902,
    monty::rc_ptr< ::mosek::fusion::Variable> _2903, int _2904);
  static monty::rc_ptr< ::mosek::fusion::Variable> stack(
    monty::rc_ptr< ::mosek::fusion::Variable> _2905,
    monty::rc_ptr< ::mosek::fusion::Variable> _2906, int _2907);
  static monty::rc_ptr< ::mosek::fusion::Variable> stack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
        _2908,
    int _2909);
  static monty::rc_ptr< ::mosek::fusion::Variable> dstack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
        _2910,
    int _2911);
}; // struct Var;

struct p_ConstraintCache
{
  ConstraintCache*                         _pubthis;
  static mosek::fusion::p_ConstraintCache* _get_impl(
    mosek::fusion::ConstraintCache* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_ConstraintCache* _get_impl(
    mosek::fusion::ConstraintCache::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ConstraintCache(ConstraintCache* _pubthis);
  virtual ~p_ConstraintCache(){
    /* std::cout << "~p_ConstraintCache" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<int, 1> > barmatidx{};
  std::shared_ptr<monty::ndarray<int, 1> > barsubj{};
  std::shared_ptr<monty::ndarray<int, 1> > barsubi{};
  long long nbarnz{};
  long long nunordered{};
  std::shared_ptr<monty::ndarray<int, 1> >    buffer_subi{};
  std::shared_ptr<monty::ndarray<int, 1> >    buffer_subj{};
  std::shared_ptr<monty::ndarray<double, 1> > buffer_cof{};
  std::shared_ptr<monty::ndarray<double, 1> > bfix{};
  std::shared_ptr<monty::ndarray<double, 1> > cof{};
  std::shared_ptr<monty::ndarray<int, 1> >    subi{};
  std::shared_ptr<monty::ndarray<int, 1> >    subj{};
  long long                 nnz{};
  int                       nrows{};
  virtual void              destroy();
  static ConstraintCache::t _new_ConstraintCache(
    monty::rc_ptr< ::mosek::fusion::ConstraintCache> _3176);
  void _initialize(monty::rc_ptr< ::mosek::fusion::ConstraintCache> _3176);
  static ConstraintCache::t _new_ConstraintCache(
    std::shared_ptr<monty::ndarray<long long, 1> > _3177,
    std::shared_ptr<monty::ndarray<double, 1> >    _3178,
    std::shared_ptr<monty::ndarray<int, 1> >       _3179,
    std::shared_ptr<monty::ndarray<double, 1> >    _3180,
    std::shared_ptr<monty::ndarray<int, 1> >       _3181,
    std::shared_ptr<monty::ndarray<int, 1> >       _3182,
    std::shared_ptr<monty::ndarray<int, 1> >       _3183);
  void _initialize(std::shared_ptr<monty::ndarray<long long, 1> > _3177,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3178,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3179,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3180,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3181,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3182,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3183);
  virtual void unchecked_add_fx(
    std::shared_ptr<monty::ndarray<double, 1> > _3186);
  virtual long long order_barentries();
  virtual void add_bar(std::shared_ptr<monty::ndarray<int, 1> > _3196,
                       std::shared_ptr<monty::ndarray<int, 1> > _3197,
                       std::shared_ptr<monty::ndarray<int, 1> > _3198);
  virtual void unchecked_add_l(
    std::shared_ptr<monty::ndarray<long long, 1> > _3204,
    std::shared_ptr<monty::ndarray<int, 1> >       _3205,
    std::shared_ptr<monty::ndarray<double, 1> >    _3206,
    std::shared_ptr<monty::ndarray<double, 1> >    _3207);
  virtual void add(std::shared_ptr<monty::ndarray<long long, 1> > _3216,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3217,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3218,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3219);
  virtual long long flush(std::shared_ptr<monty::ndarray<int, 1> >    _3220,
                          std::shared_ptr<monty::ndarray<int, 1> >    _3221,
                          std::shared_ptr<monty::ndarray<double, 1> > _3222,
                          std::shared_ptr<monty::ndarray<double, 1> > _3223);
  virtual long long                                        numUnsorted();
  virtual monty::rc_ptr< ::mosek::fusion::ConstraintCache> clone();
}; // struct ConstraintCache;

struct p_Constraint
{
  Constraint*                         _pubthis;
  static mosek::fusion::p_Constraint* _get_impl(
    mosek::fusion::Constraint* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Constraint* _get_impl(
    mosek::fusion::Constraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Constraint(Constraint* _pubthis);
  virtual ~p_Constraint(){ /* std::cout << "~p_Constraint" << std::endl;*/ };
  monty::rc_ptr< ::mosek::fusion::Set>   shape_p{};
  monty::rc_ptr< ::mosek::fusion::Model> model{};
  virtual void                           destroy();
  static Constraint::t                   _new_Constraint(
    monty::rc_ptr< ::mosek::fusion::Constraint> _3896,
    monty::rc_ptr< ::mosek::fusion::Model>      _3897);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Constraint> _3896,
                   monty::rc_ptr< ::mosek::fusion::Model>      _3897);
  static Constraint::t _new_Constraint(
    monty::rc_ptr< ::mosek::fusion::Model> _3898,
    monty::rc_ptr< ::mosek::fusion::Set>   _3899);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _3898,
                   monty::rc_ptr< ::mosek::fusion::Set>   _3899);
  virtual std::string toString();
  virtual void        toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _3905, long long _3906,
    std::shared_ptr<monty::ndarray<std::string, 1> > _3907)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> add(double _3908);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> add(
    std::shared_ptr<monty::ndarray<double, 1> > _3913);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> add(
    monty::rc_ptr< ::mosek::fusion::Variable> _3917);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _3926);
  static void inst(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _3933,
    std::shared_ptr<monty::ndarray<long long, 1> > _3934,
    std::shared_ptr<monty::ndarray<int, 1> >       _3935,
    std::shared_ptr<monty::ndarray<int, 1> >       _3936,
    std::shared_ptr<monty::ndarray<int, 1> >       _3937);
  virtual void add_l(std::shared_ptr<monty::ndarray<long long, 1> > _3945,
                     std::shared_ptr<monty::ndarray<long long, 1> > _3946,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3947,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3948,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3949,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3950,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3951,
                     long long _3952, int _3953, int _3954)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual(
    std::shared_ptr<monty::ndarray<int, 1> > _3955,
    std::shared_ptr<monty::ndarray<int, 1> > _3956);
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual(int _3964,
                                                           int _3965);
  virtual std::shared_ptr<monty::ndarray<double, 1> > dual();
  virtual void dual_values(int _3969,
                           std::shared_ptr<monty::ndarray<double, 1> > _3970);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3975,
                           int _3976,
                           std::shared_ptr<monty::ndarray<double, 1> > _3977)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void dual_values(long long _3978,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3979,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3980,
                           int _3981,
                           std::shared_ptr<monty::ndarray<double, 1> > _3982)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > level();
  virtual double level(int _3984);
  virtual std::shared_ptr<monty::ndarray<double, 1> > level(
    std::shared_ptr<monty::ndarray<int, 1> > _3987,
    std::shared_ptr<monty::ndarray<int, 1> > _3988);
  virtual std::shared_ptr<monty::ndarray<double, 1> > level(int _3998,
                                                            int _3999);
  virtual void level_values(int _4004,
                            std::shared_ptr<monty::ndarray<double, 1> > _4005);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _4010, int _4011,
    std::shared_ptr<monty::ndarray<double, 1> > _4012)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void level_values(
    long long _4013, std::shared_ptr<monty::ndarray<int, 1> > _4014,
    std::shared_ptr<monty::ndarray<long long, 1> > _4015, int _4016,
    std::shared_ptr<monty::ndarray<double, 1> > _4017)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Model>     get_model();
  virtual int                                        get_nd();
  virtual long long                                  size();
  static monty::rc_ptr< ::mosek::fusion::Constraint> stack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
      _4018);
  static monty::rc_ptr< ::mosek::fusion::Constraint> stack(
    monty::rc_ptr< ::mosek::fusion::Constraint> _4019,
    monty::rc_ptr< ::mosek::fusion::Constraint> _4020,
    monty::rc_ptr< ::mosek::fusion::Constraint> _4021);
  static monty::rc_ptr< ::mosek::fusion::Constraint> stack(
    monty::rc_ptr< ::mosek::fusion::Constraint> _4023,
    monty::rc_ptr< ::mosek::fusion::Constraint> _4024);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> index(
    std::shared_ptr<monty::ndarray<int, 1> > _4026);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> index(int _4028);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _4029,
    std::shared_ptr<monty::ndarray<int, 1> > _4030)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(int _4031,
                                                            int _4032)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> reduceDims();
  virtual monty::rc_ptr< ::mosek::fusion::Set>        shape();
}; // struct Constraint;

struct p_CompoundConstraint : public ::mosek::fusion::p_Constraint
{
  CompoundConstraint*                         _pubthis;
  static mosek::fusion::p_CompoundConstraint* _get_impl(
    mosek::fusion::CompoundConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_CompoundConstraint*>(
      mosek::fusion::p_Constraint::_get_impl(_inst));
  }
  static mosek::fusion::p_CompoundConstraint* _get_impl(
    mosek::fusion::CompoundConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_CompoundConstraint(CompoundConstraint* _pubthis);
  virtual ~p_CompoundConstraint(){
    /* std::cout << "~p_CompoundConstraint" << std::endl;*/
  };
  int stackdim{};
  std::shared_ptr<monty::ndarray<int, 1> > consb{};
  std::shared_ptr<
    monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
                               cons{};
  virtual void                 destroy();
  static CompoundConstraint::t _new_CompoundConstraint(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
      _3246);
  void _initialize(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
      _3246);
  virtual void toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _3253, long long _3254,
    std::shared_ptr<monty::ndarray<std::string, 1> > _3255);
  virtual void add_l(std::shared_ptr<monty::ndarray<long long, 1> > _3256,
                     std::shared_ptr<monty::ndarray<long long, 1> > _3257,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3258,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3259,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3260,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3261,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3262,
                     long long _3263, int _3264, int _3265);
  virtual void dual_values(long long _3290,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3291,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3292,
                           int _3293,
                           std::shared_ptr<monty::ndarray<double, 1> > _3294);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3311,
                           int _3312,
                           std::shared_ptr<monty::ndarray<double, 1> > _3313);
  virtual void level_values(
    long long _3320, std::shared_ptr<monty::ndarray<int, 1> > _3321,
    std::shared_ptr<monty::ndarray<long long, 1> > _3322, int _3323,
    std::shared_ptr<monty::ndarray<double, 1> > _3324);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3341, int _3342,
    std::shared_ptr<monty::ndarray<double, 1> > _3343);
  virtual void add(std::shared_ptr<monty::ndarray<long long, 1> > _3350,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3351,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3352,
                   std::shared_ptr<monty::ndarray<double, 1> > _3353, int _3354,
                   std::shared_ptr<monty::ndarray<int, 1> > _3355, int _3356);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _3357,
    std::shared_ptr<monty::ndarray<int, 1> > _3358);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(int _3359,
                                                            int _3360);
  static monty::rc_ptr< ::mosek::fusion::Set> compute_shape(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
        _3361,
    int _3362);
  static int count_numcon(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
      _3369);
  static monty::rc_ptr< ::mosek::fusion::Model> model_from_con(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Constraint>, 1> >
      _3375);
}; // struct CompoundConstraint;

struct p_SliceConstraint : public ::mosek::fusion::p_Constraint
{
  SliceConstraint*                         _pubthis;
  static mosek::fusion::p_SliceConstraint* _get_impl(
    mosek::fusion::SliceConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_SliceConstraint*>(
      mosek::fusion::p_Constraint::_get_impl(_inst));
  }
  static mosek::fusion::p_SliceConstraint* _get_impl(
    mosek::fusion::SliceConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SliceConstraint(SliceConstraint* _pubthis);
  virtual ~p_SliceConstraint(){
    /* std::cout << "~p_SliceConstraint" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<long long, 1> > strides{};
  long long                                        first{};
  monty::rc_ptr< ::mosek::fusion::ModelConstraint> origin{};
  virtual void                                     destroy();
  static SliceConstraint::t                        _new_SliceConstraint(
    monty::rc_ptr< ::mosek::fusion::ModelConstraint> _3392,
    monty::rc_ptr< ::mosek::fusion::Set> _3393, long long _3394,
    std::shared_ptr<monty::ndarray<long long, 1> >        _3395);
  void _initialize(monty::rc_ptr< ::mosek::fusion::ModelConstraint> _3392,
                   monty::rc_ptr< ::mosek::fusion::Set> _3393, long long _3394,
                   std::shared_ptr<monty::ndarray<long long, 1> >        _3395);
  virtual void add_l(std::shared_ptr<monty::ndarray<long long, 1> > _3396,
                     std::shared_ptr<monty::ndarray<long long, 1> > _3397,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3398,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3399,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3400,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3401,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3402,
                     long long _3403, int _3404, int _3405);
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _3416,
                      std::shared_ptr<monty::ndarray<double, 1> >    _3417,
                      long long _3418, int _3419, int _3420);
  virtual void dual_values(long long _3427,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3428,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3429,
                           int _3430,
                           std::shared_ptr<monty::ndarray<double, 1> > _3431);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3454,
                           int _3455,
                           std::shared_ptr<monty::ndarray<double, 1> > _3456);
  virtual void level_values(
    long long _3462, std::shared_ptr<monty::ndarray<int, 1> > _3463,
    std::shared_ptr<monty::ndarray<long long, 1> > _3464, int _3465,
    std::shared_ptr<monty::ndarray<double, 1> > _3466);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3488, int _3489,
    std::shared_ptr<monty::ndarray<double, 1> > _3490);
  virtual void toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _3496, long long _3497,
    std::shared_ptr<monty::ndarray<std::string, 1> > _3498);
  virtual long long                                   size();
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _3504,
    std::shared_ptr<monty::ndarray<int, 1> > _3505);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(int _3509,
                                                            int _3510);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice_(
    monty::rc_ptr< ::mosek::fusion::Set> _3512, long long _3513,
    std::shared_ptr<monty::ndarray<long long, 1> >        _3514);
}; // struct SliceConstraint;

struct p_BoundInterfaceConstraint : public ::mosek::fusion::p_SliceConstraint
{
  BoundInterfaceConstraint*                         _pubthis;
  static mosek::fusion::p_BoundInterfaceConstraint* _get_impl(
    mosek::fusion::BoundInterfaceConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_BoundInterfaceConstraint*>(
      mosek::fusion::p_SliceConstraint::_get_impl(_inst));
  }
  static mosek::fusion::p_BoundInterfaceConstraint* _get_impl(
    mosek::fusion::BoundInterfaceConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_BoundInterfaceConstraint(BoundInterfaceConstraint* _pubthis);
  virtual ~p_BoundInterfaceConstraint(){
    /* std::cout << "~p_BoundInterfaceConstraint" << std::endl;*/
  };
  monty::rc_ptr< ::mosek::fusion::RangedConstraint> origincon{};
  bool                                              islower{};
  virtual void                                      destroy();
  static BoundInterfaceConstraint::t _new_BoundInterfaceConstraint(
    monty::rc_ptr< ::mosek::fusion::RangedConstraint> _3376,
    monty::rc_ptr< ::mosek::fusion::Set> _3377, long long _3378,
    std::shared_ptr<monty::ndarray<long long, 1> > _3379, bool _3380);
  void _initialize(monty::rc_ptr< ::mosek::fusion::RangedConstraint> _3376,
                   monty::rc_ptr< ::mosek::fusion::Set> _3377, long long _3378,
                   std::shared_ptr<monty::ndarray<long long, 1> >        _3379,
                   bool _3380);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice_(
    monty::rc_ptr< ::mosek::fusion::Set> _3381, long long _3382,
    std::shared_ptr<monty::ndarray<long long, 1> >        _3383);
  virtual void dual_values(long long _3384,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3385,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3386,
                           int _3387,
                           std::shared_ptr<monty::ndarray<double, 1> > _3388);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3389,
                           int _3390,
                           std::shared_ptr<monty::ndarray<double, 1> > _3391);
}; // struct BoundInterfaceConstraint;

struct p_ModelConstraint : public ::mosek::fusion::p_Constraint
{
  ModelConstraint*                         _pubthis;
  static mosek::fusion::p_ModelConstraint* _get_impl(
    mosek::fusion::ModelConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_ModelConstraint*>(
      mosek::fusion::p_Constraint::_get_impl(_inst));
  }
  static mosek::fusion::p_ModelConstraint* _get_impl(
    mosek::fusion::ModelConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ModelConstraint(ModelConstraint* _pubthis);
  virtual ~p_ModelConstraint(){
    /* std::cout << "~p_ModelConstraint" << std::endl;*/
  };
  bool names_flushed{};
  std::shared_ptr<monty::ndarray<int, 1> > nativeindexes{};
  std::string name{};
  std::shared_ptr<monty::ndarray<double, 1> > cache_bfix{};
  monty::rc_ptr< ::mosek::fusion::ConstraintCache> cache{};
  virtual void                                     destroy();
  static ModelConstraint::t                        _new_ModelConstraint(
    monty::rc_ptr< ::mosek::fusion::ModelConstraint> _3782,
    monty::rc_ptr< ::mosek::fusion::Model>           _3783);
  void _initialize(monty::rc_ptr< ::mosek::fusion::ModelConstraint> _3782,
                   monty::rc_ptr< ::mosek::fusion::Model>           _3783);
  static ModelConstraint::t _new_ModelConstraint(
    monty::rc_ptr< ::mosek::fusion::Model> _3786, const std::string& _3787,
    monty::rc_ptr< ::mosek::fusion::Set> _3788,
    std::shared_ptr<monty::ndarray<int, 1> >       _3789,
    std::shared_ptr<monty::ndarray<long long, 1> > _3790,
    std::shared_ptr<monty::ndarray<int, 1> >       _3791,
    std::shared_ptr<monty::ndarray<double, 1> >    _3792,
    std::shared_ptr<monty::ndarray<double, 1> >    _3793,
    std::shared_ptr<monty::ndarray<int, 1> >       _3794,
    std::shared_ptr<monty::ndarray<int, 1> >       _3795,
    std::shared_ptr<monty::ndarray<int, 1> >       _3796);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _3786,
                   const std::string&                     _3787,
                   monty::rc_ptr< ::mosek::fusion::Set>   _3788,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3789,
                   std::shared_ptr<monty::ndarray<long long, 1> > _3790,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3791,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3792,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3793,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3794,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3795,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3796);
  virtual void        flushNames();
  virtual std::string toString();
  virtual void        toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _3803, long long _3804,
    std::shared_ptr<monty::ndarray<std::string, 1> > _3805);
  virtual void domainToString(
    long long _3821, monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _3822)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void add_l(std::shared_ptr<monty::ndarray<long long, 1> > _3823,
                     std::shared_ptr<monty::ndarray<long long, 1> > _3824,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3825,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3826,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3827,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3828,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3829,
                     long long _3830, int _3831, int _3832);
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _3880,
                      std::shared_ptr<monty::ndarray<double, 1> >    _3881,
                      long long _3882, int _3883, int _3884)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _3885,
    std::shared_ptr<monty::ndarray<int, 1> > _3886);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint> slice(int _3892,
                                                            int _3893);
  virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _3895)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
}; // struct ModelConstraint;

struct p_LinearPSDConstraint : public ::mosek::fusion::p_ModelConstraint
{
  LinearPSDConstraint*                         _pubthis;
  static mosek::fusion::p_LinearPSDConstraint* _get_impl(
    mosek::fusion::LinearPSDConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_LinearPSDConstraint*>(
      mosek::fusion::p_ModelConstraint::_get_impl(_inst));
  }
  static mosek::fusion::p_LinearPSDConstraint* _get_impl(
    mosek::fusion::LinearPSDConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_LinearPSDConstraint(LinearPSDConstraint* _pubthis);
  virtual ~p_LinearPSDConstraint(){
    /* std::cout << "~p_LinearPSDConstraint" << std::endl;*/
  };
  int                           psdvardim{};
  bool                          names_flushed{};
  int                           numcones{};
  int                           conesize{};
  int                           coneidx{};
  virtual void                  destroy();
  static LinearPSDConstraint::t _new_LinearPSDConstraint(
    monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint> _2925,
    monty::rc_ptr< ::mosek::fusion::Model>               _2926);
  void _initialize(monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint> _2925,
                   monty::rc_ptr< ::mosek::fusion::Model>               _2926);
  static LinearPSDConstraint::t _new_LinearPSDConstraint(
    monty::rc_ptr< ::mosek::fusion::Model> _2927, const std::string& _2928,
    monty::rc_ptr< ::mosek::fusion::Set> _2929,
    std::shared_ptr<monty::ndarray<int, 1> > _2930, int _2931, int _2932,
    int _2933, std::shared_ptr<monty::ndarray<long long, 1> > _2934,
    std::shared_ptr<monty::ndarray<int, 1> >    _2935,
    std::shared_ptr<monty::ndarray<double, 1> > _2936,
    std::shared_ptr<monty::ndarray<double, 1> > _2937,
    std::shared_ptr<monty::ndarray<int, 1> >    _2938,
    std::shared_ptr<monty::ndarray<int, 1> >    _2939,
    std::shared_ptr<monty::ndarray<int, 1> >    _2940);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _2927,
                   const std::string&                     _2928,
                   monty::rc_ptr< ::mosek::fusion::Set>   _2929,
                   std::shared_ptr<monty::ndarray<int, 1> > _2930, int _2931,
                   int _2932, int                                 _2933,
                   std::shared_ptr<monty::ndarray<long long, 1> > _2934,
                   std::shared_ptr<monty::ndarray<int, 1> >       _2935,
                   std::shared_ptr<monty::ndarray<double, 1> >    _2936,
                   std::shared_ptr<monty::ndarray<double, 1> >    _2937,
                   std::shared_ptr<monty::ndarray<int, 1> >       _2938,
                   std::shared_ptr<monty::ndarray<int, 1> >       _2939,
                   std::shared_ptr<monty::ndarray<int, 1> >       _2940);
  virtual void domainToString(
    long long                                            _2943,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _2944);
  virtual std::string toString();
  virtual void        toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _2954, long long _2955,
    std::shared_ptr<monty::ndarray<std::string, 1> > _2956);
  virtual void toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _2957, long long _2958,
    std::shared_ptr<monty::ndarray<std::string, 1> > _2959, bool _2960);
  virtual void flushNames();
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _2988,
                      std::shared_ptr<monty::ndarray<double, 1> >    _2989,
                      long long _2990, int _2991, int _2992);
  virtual void dual_values(long long _2997,
                           std::shared_ptr<monty::ndarray<int, 1> >       _2998,
                           std::shared_ptr<monty::ndarray<long long, 1> > _2999,
                           int _3000,
                           std::shared_ptr<monty::ndarray<double, 1> > _3001);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3010,
                           int _3011,
                           std::shared_ptr<monty::ndarray<double, 1> > _3012);
  virtual void level_values(
    long long _3018, std::shared_ptr<monty::ndarray<int, 1> > _3019,
    std::shared_ptr<monty::ndarray<long long, 1> > _3020, int _3021,
    std::shared_ptr<monty::ndarray<double, 1> > _3022);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3031, int _3032,
    std::shared_ptr<monty::ndarray<double, 1> > _3033);
  virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _3039);
}; // struct LinearPSDConstraint;

struct p_PSDConstraint : public ::mosek::fusion::p_ModelConstraint
{
  PSDConstraint*                         _pubthis;
  static mosek::fusion::p_PSDConstraint* _get_impl(
    mosek::fusion::PSDConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_PSDConstraint*>(
      mosek::fusion::p_ModelConstraint::_get_impl(_inst));
  }
  static mosek::fusion::p_PSDConstraint* _get_impl(
    mosek::fusion::PSDConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_PSDConstraint(PSDConstraint* _pubthis);
  virtual ~p_PSDConstraint(){
    /* std::cout << "~p_PSDConstraint" << std::endl;*/
  };
  bool                    names_flushed{};
  int                     numcones{};
  int                     conesize{};
  int                     coneidx{};
  virtual void            destroy();
  static PSDConstraint::t _new_PSDConstraint(
    monty::rc_ptr< ::mosek::fusion::PSDConstraint> _3040,
    monty::rc_ptr< ::mosek::fusion::Model>         _3041);
  void _initialize(monty::rc_ptr< ::mosek::fusion::PSDConstraint> _3040,
                   monty::rc_ptr< ::mosek::fusion::Model>         _3041);
  static PSDConstraint::t _new_PSDConstraint(
    monty::rc_ptr< ::mosek::fusion::Model> _3042, const std::string& _3043,
    monty::rc_ptr< ::mosek::fusion::Set> _3044,
    std::shared_ptr<monty::ndarray<int, 1> > _3045, int _3046, int _3047,
    int _3048, std::shared_ptr<monty::ndarray<long long, 1> > _3049,
    std::shared_ptr<monty::ndarray<int, 1> >    _3050,
    std::shared_ptr<monty::ndarray<double, 1> > _3051,
    std::shared_ptr<monty::ndarray<double, 1> > _3052,
    std::shared_ptr<monty::ndarray<int, 1> >    _3053,
    std::shared_ptr<monty::ndarray<int, 1> >    _3054,
    std::shared_ptr<monty::ndarray<int, 1> >    _3055);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model> _3042,
                   const std::string&                     _3043,
                   monty::rc_ptr< ::mosek::fusion::Set>   _3044,
                   std::shared_ptr<monty::ndarray<int, 1> > _3045, int _3046,
                   int _3047, int                                 _3048,
                   std::shared_ptr<monty::ndarray<long long, 1> > _3049,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3050,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3051,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3052,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3053,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3054,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3055);
  virtual void domainToString(
    long long                                            _3056,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _3057);
  virtual std::string toString();
  virtual void        toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _3067, long long _3068,
    std::shared_ptr<monty::ndarray<std::string, 1> > _3069);
  virtual void toStringArray(
    std::shared_ptr<monty::ndarray<long long, 1> > _3070, long long _3071,
    std::shared_ptr<monty::ndarray<std::string, 1> > _3072, bool _3073);
  virtual void flushNames();
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _3106,
                      std::shared_ptr<monty::ndarray<double, 1> >    _3107,
                      long long _3108, int _3109, int _3110);
  virtual void dual_values(long long _3115,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3116,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3117,
                           int _3118,
                           std::shared_ptr<monty::ndarray<double, 1> > _3119);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3135,
                           int _3136,
                           std::shared_ptr<monty::ndarray<double, 1> > _3137);
  virtual void level_values(
    long long _3145, std::shared_ptr<monty::ndarray<int, 1> > _3146,
    std::shared_ptr<monty::ndarray<long long, 1> > _3147, int _3148,
    std::shared_ptr<monty::ndarray<double, 1> > _3149);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3165, int _3166,
    std::shared_ptr<monty::ndarray<double, 1> > _3167);
  virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _3175);
}; // struct PSDConstraint;

struct p_RangedConstraint : public ::mosek::fusion::p_ModelConstraint
{
  RangedConstraint*                         _pubthis;
  static mosek::fusion::p_RangedConstraint* _get_impl(
    mosek::fusion::RangedConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_RangedConstraint*>(
      mosek::fusion::p_ModelConstraint::_get_impl(_inst));
  }
  static mosek::fusion::p_RangedConstraint* _get_impl(
    mosek::fusion::RangedConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_RangedConstraint(RangedConstraint* _pubthis);
  virtual ~p_RangedConstraint(){
    /* std::cout << "~p_RangedConstraint" << std::endl;*/
  };
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  monty::rc_ptr< ::mosek::fusion::RangeDomain>   dom{};
  virtual void                                   destroy();
  static RangedConstraint::t                     _new_RangedConstraint(
    monty::rc_ptr< ::mosek::fusion::RangedConstraint> _3515,
    monty::rc_ptr< ::mosek::fusion::Model>            _3516);
  void _initialize(monty::rc_ptr< ::mosek::fusion::RangedConstraint> _3515,
                   monty::rc_ptr< ::mosek::fusion::Model>            _3516);
  static RangedConstraint::t _new_RangedConstraint(
    monty::rc_ptr< ::mosek::fusion::Model> _3517, const std::string& _3518,
    monty::rc_ptr< ::mosek::fusion::Set>         _3519,
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _3520,
    std::shared_ptr<monty::ndarray<int, 1> >       _3521,
    std::shared_ptr<monty::ndarray<long long, 1> > _3522,
    std::shared_ptr<monty::ndarray<int, 1> >       _3523,
    std::shared_ptr<monty::ndarray<double, 1> >    _3524,
    std::shared_ptr<monty::ndarray<double, 1> >    _3525,
    std::shared_ptr<monty::ndarray<int, 1> >       _3526,
    std::shared_ptr<monty::ndarray<int, 1> >       _3527,
    std::shared_ptr<monty::ndarray<int, 1> >       _3528);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>       _3517,
                   const std::string&                           _3518,
                   monty::rc_ptr< ::mosek::fusion::Set>         _3519,
                   monty::rc_ptr< ::mosek::fusion::RangeDomain> _3520,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3521,
                   std::shared_ptr<monty::ndarray<long long, 1> > _3522,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3523,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3524,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3525,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3526,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3527,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3528);
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _3529,
                      std::shared_ptr<monty::ndarray<double, 1> >    _3530,
                      long long _3531, int _3532, int _3533);
  virtual void dual_u(long long _3539,
                      std::shared_ptr<monty::ndarray<int, 1> >       _3540,
                      std::shared_ptr<monty::ndarray<long long, 1> > _3541,
                      int _3542,
                      std::shared_ptr<monty::ndarray<double, 1> > _3543);
  virtual void dual_u(std::shared_ptr<monty::ndarray<long long, 1> > _3551,
                      int _3552,
                      std::shared_ptr<monty::ndarray<double, 1> > _3553);
  virtual void dual_l(long long _3557,
                      std::shared_ptr<monty::ndarray<int, 1> >       _3558,
                      std::shared_ptr<monty::ndarray<long long, 1> > _3559,
                      int _3560,
                      std::shared_ptr<monty::ndarray<double, 1> > _3561);
  virtual void dual_l(std::shared_ptr<monty::ndarray<long long, 1> > _3569,
                      int _3570,
                      std::shared_ptr<monty::ndarray<double, 1> > _3571);
  virtual void dual_values(long long _3575,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3576,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3577,
                           int _3578,
                           std::shared_ptr<monty::ndarray<double, 1> > _3579);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3587,
                           int _3588,
                           std::shared_ptr<monty::ndarray<double, 1> > _3589);
  virtual void level_values(
    long long _3593, std::shared_ptr<monty::ndarray<int, 1> > _3594,
    std::shared_ptr<monty::ndarray<long long, 1> > _3595, int _3596,
    std::shared_ptr<monty::ndarray<double, 1> > _3597);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3605, int _3606,
    std::shared_ptr<monty::ndarray<double, 1> > _3607);
  virtual void add_l(std::shared_ptr<monty::ndarray<long long, 1> > _3611,
                     std::shared_ptr<monty::ndarray<long long, 1> > _3612,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3613,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3614,
                     std::shared_ptr<monty::ndarray<int, 1> >       _3615,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3616,
                     std::shared_ptr<monty::ndarray<double, 1> >    _3617,
                     int _3618, int _3619, int _3620);
  virtual void domainToString(
    long long                                            _3626,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _3627);
  virtual monty::rc_ptr< ::mosek::fusion::Constraint>      upperBoundCon();
  virtual monty::rc_ptr< ::mosek::fusion::Constraint>      lowerBoundCon();
  virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _3632);
}; // struct RangedConstraint;

struct p_ConicConstraint : public ::mosek::fusion::p_ModelConstraint
{
  ConicConstraint*                         _pubthis;
  static mosek::fusion::p_ConicConstraint* _get_impl(
    mosek::fusion::ConicConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_ConicConstraint*>(
      mosek::fusion::p_ModelConstraint::_get_impl(_inst));
  }
  static mosek::fusion::p_ConicConstraint* _get_impl(
    mosek::fusion::ConicConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ConicConstraint(ConicConstraint* _pubthis);
  virtual ~p_ConicConstraint(){
    /* std::cout << "~p_ConicConstraint" << std::endl;*/
  };
  bool                                         names_flushed{};
  monty::rc_ptr< ::mosek::fusion::QConeDomain> dom{};
  int                                          conesize{};
  int                                          last{};
  int                                          first{};
  int                                          last_slack{};
  int                                          first_slack{};
  int                                          coneidx{};
  virtual void                                 destroy();
  static ConicConstraint::t                    _new_ConicConstraint(
    monty::rc_ptr< ::mosek::fusion::ConicConstraint> _3633,
    monty::rc_ptr< ::mosek::fusion::Model>           _3634);
  void _initialize(monty::rc_ptr< ::mosek::fusion::ConicConstraint> _3633,
                   monty::rc_ptr< ::mosek::fusion::Model>           _3634);
  static ConicConstraint::t _new_ConicConstraint(
    monty::rc_ptr< ::mosek::fusion::Model> _3635, const std::string& _3636,
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _3637,
    monty::rc_ptr< ::mosek::fusion::Set>         _3638,
    std::shared_ptr<monty::ndarray<int, 1> > _3639, int _3640, int _3641,
    int _3642, int _3643, int _3644,
    std::shared_ptr<monty::ndarray<long long, 1> > _3645,
    std::shared_ptr<monty::ndarray<int, 1> >       _3646,
    std::shared_ptr<monty::ndarray<double, 1> >    _3647,
    std::shared_ptr<monty::ndarray<double, 1> >    _3648,
    std::shared_ptr<monty::ndarray<int, 1> >       _3649,
    std::shared_ptr<monty::ndarray<int, 1> >       _3650,
    std::shared_ptr<monty::ndarray<int, 1> >       _3651);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>       _3635,
                   const std::string&                           _3636,
                   monty::rc_ptr< ::mosek::fusion::QConeDomain> _3637,
                   monty::rc_ptr< ::mosek::fusion::Set>         _3638,
                   std::shared_ptr<monty::ndarray<int, 1> > _3639, int _3640,
                   int _3641, int _3642, int _3643, int _3644,
                   std::shared_ptr<monty::ndarray<long long, 1> > _3645,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3646,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3647,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3648,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3649,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3650,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3651);
  virtual void        flushNames();
  virtual std::string toString();
  virtual void dual_values(long long _3659,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3660,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3661,
                           int _3662,
                           std::shared_ptr<monty::ndarray<double, 1> > _3663);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3670,
                           int _3671,
                           std::shared_ptr<monty::ndarray<double, 1> > _3672);
  virtual void level_values(
    long long _3675, std::shared_ptr<monty::ndarray<int, 1> > _3676,
    std::shared_ptr<monty::ndarray<long long, 1> > _3677, int _3678,
    std::shared_ptr<monty::ndarray<double, 1> > _3679);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3687, int _3688,
    std::shared_ptr<monty::ndarray<double, 1> > _3689);
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _3693,
                      std::shared_ptr<monty::ndarray<double, 1> >    _3694,
                      long long _3695, int _3696, int _3697);
  virtual void dual(std::shared_ptr<monty::ndarray<int, 1> > _3702, int _3703,
                    int _3704, int                              _3705,
                    std::shared_ptr<monty::ndarray<double, 1> > _3706);
  virtual void dual_values(std::shared_ptr<monty::ndarray<int, 1> > _3709,
                           std::shared_ptr<monty::ndarray<int, 1> > _3710,
                           int _3711,
                           std::shared_ptr<monty::ndarray<double, 1> > _3712);
  virtual void domainToString(
    long long                                            _3717,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _3718);
  virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _3719);
}; // struct ConicConstraint;

struct p_LinearConstraint : public ::mosek::fusion::p_ModelConstraint
{
  LinearConstraint*                         _pubthis;
  static mosek::fusion::p_LinearConstraint* _get_impl(
    mosek::fusion::LinearConstraint* _inst)
  {
    return static_cast<mosek::fusion::p_LinearConstraint*>(
      mosek::fusion::p_ModelConstraint::_get_impl(_inst));
  }
  static mosek::fusion::p_LinearConstraint* _get_impl(
    mosek::fusion::LinearConstraint::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_LinearConstraint(LinearConstraint* _pubthis);
  virtual ~p_LinearConstraint(){
    /* std::cout << "~p_LinearConstraint" << std::endl;*/
  };
  monty::rc_ptr< ::mosek::fusion::LinearDomain>  dom{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  virtual void                                   destroy();
  static LinearConstraint::t                     _new_LinearConstraint(
    monty::rc_ptr< ::mosek::fusion::LinearConstraint> _3720,
    monty::rc_ptr< ::mosek::fusion::Model>            _3721);
  void _initialize(monty::rc_ptr< ::mosek::fusion::LinearConstraint> _3720,
                   monty::rc_ptr< ::mosek::fusion::Model>            _3721);
  static LinearConstraint::t _new_LinearConstraint(
    monty::rc_ptr< ::mosek::fusion::Model> _3722, const std::string& _3723,
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _3724,
    monty::rc_ptr< ::mosek::fusion::Set>          _3725,
    std::shared_ptr<monty::ndarray<int, 1> >       _3726,
    std::shared_ptr<monty::ndarray<long long, 1> > _3727,
    std::shared_ptr<monty::ndarray<int, 1> >       _3728,
    std::shared_ptr<monty::ndarray<double, 1> >    _3729,
    std::shared_ptr<monty::ndarray<double, 1> >    _3730,
    std::shared_ptr<monty::ndarray<int, 1> >       _3731,
    std::shared_ptr<monty::ndarray<int, 1> >       _3732,
    std::shared_ptr<monty::ndarray<int, 1> >       _3733);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Model>        _3722,
                   const std::string&                            _3723,
                   monty::rc_ptr< ::mosek::fusion::LinearDomain> _3724,
                   monty::rc_ptr< ::mosek::fusion::Set>          _3725,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3726,
                   std::shared_ptr<monty::ndarray<long long, 1> > _3727,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3728,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3729,
                   std::shared_ptr<monty::ndarray<double, 1> >    _3730,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3731,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3732,
                   std::shared_ptr<monty::ndarray<int, 1> >       _3733);
  virtual void add_fx(std::shared_ptr<monty::ndarray<long long, 1> > _3734,
                      std::shared_ptr<monty::ndarray<double, 1> >    _3735,
                      long long _3736, int _3737, int _3738);
  virtual void dual_values(long long _3743,
                           std::shared_ptr<monty::ndarray<int, 1> >       _3744,
                           std::shared_ptr<monty::ndarray<long long, 1> > _3745,
                           int _3746,
                           std::shared_ptr<monty::ndarray<double, 1> > _3747);
  virtual void dual_values(std::shared_ptr<monty::ndarray<long long, 1> > _3755,
                           int _3756,
                           std::shared_ptr<monty::ndarray<double, 1> > _3757);
  virtual void level_values(
    long long _3761, std::shared_ptr<monty::ndarray<int, 1> > _3762,
    std::shared_ptr<monty::ndarray<long long, 1> > _3763, int _3764,
    std::shared_ptr<monty::ndarray<double, 1> > _3765);
  virtual void level_values(
    std::shared_ptr<monty::ndarray<long long, 1> > _3773, int _3774,
    std::shared_ptr<monty::ndarray<double, 1> > _3775);
  virtual void domainToString(
    long long                                            _3779,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _3780);
  virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint> clone(
    monty::rc_ptr< ::mosek::fusion::Model> _3781);
}; // struct LinearConstraint;

struct p_Set
{
  Set*                         _pubthis;
  static mosek::fusion::p_Set* _get_impl(mosek::fusion::Set* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Set* _get_impl(mosek::fusion::Set::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Set(Set* _pubthis);
  virtual ~p_Set(){ /* std::cout << "~p_Set" << std::endl;*/ };
  long long     size{};
  int           nd_p{};
  int           nd{};
  virtual void  destroy();
  static Set::t _new_Set(int _4124, long long _4125);
  void _initialize(int _4124, long long _4125);
  virtual std::string toString();
  virtual std::string indexToString(long long _4128)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _4129,
    std::shared_ptr<monty::ndarray<int, 1> > _4130)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(int _4131, int _4132)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual long long linearidx(int _4133, int _4134, int _4135);
  virtual long long linearidx(int _4136, int _4137);
  virtual long long linearidx(std::shared_ptr<monty::ndarray<int, 1> > _4138);
  virtual std::shared_ptr<monty::ndarray<int, 1> > idxtokey(long long _4141);
  virtual std::string getname(std::shared_ptr<monty::ndarray<int, 1> > _4145)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual std::string getname(long long _4146)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual long long stride(int _4147)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual int dim(int _4148)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  static monty::rc_ptr< ::mosek::fusion::Set> make(
    std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Set>, 1> >
      _4149);
  static monty::rc_ptr< ::mosek::fusion::Set> make(
    monty::rc_ptr< ::mosek::fusion::Set> _4150,
    monty::rc_ptr< ::mosek::fusion::Set> _4151);
  static monty::rc_ptr< ::mosek::fusion::Set> make(
    std::shared_ptr<monty::ndarray<int, 1> > _4152);
  static monty::rc_ptr< ::mosek::fusion::Set> make(int _4153, int _4154,
                                                   int _4155);
  static monty::rc_ptr< ::mosek::fusion::Set> make(int _4156, int _4157);
  static monty::rc_ptr< ::mosek::fusion::Set> make(int _4158);
  static monty::rc_ptr< ::mosek::fusion::Set> scalar();
  static monty::rc_ptr< ::mosek::fusion::Set> make(
    std::shared_ptr<monty::ndarray<std::string, 1> > _4159);
  virtual int       realnd();
  virtual long long getSize();
  virtual bool compare(monty::rc_ptr< ::mosek::fusion::Set> _4162);
}; // struct Set;

struct p_BaseSet : public ::mosek::fusion::p_Set
{
  BaseSet*                         _pubthis;
  static mosek::fusion::p_BaseSet* _get_impl(mosek::fusion::BaseSet* _inst)
  {
    return static_cast<mosek::fusion::p_BaseSet*>(
      mosek::fusion::p_Set::_get_impl(_inst));
  }
  static mosek::fusion::p_BaseSet* _get_impl(mosek::fusion::BaseSet::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_BaseSet(BaseSet* _pubthis);
  virtual ~p_BaseSet(){ /* std::cout << "~p_BaseSet" << std::endl;*/ };
  virtual void      destroy();
  static BaseSet::t _new_BaseSet(long long _4083);
  void _initialize(long long _4083);
  virtual int dim(int _4084);
}; // struct BaseSet;

struct p_IntSet : public ::mosek::fusion::p_BaseSet
{
  IntSet*                         _pubthis;
  static mosek::fusion::p_IntSet* _get_impl(mosek::fusion::IntSet* _inst)
  {
    return static_cast<mosek::fusion::p_IntSet*>(
      mosek::fusion::p_BaseSet::_get_impl(_inst));
  }
  static mosek::fusion::p_IntSet* _get_impl(mosek::fusion::IntSet::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_IntSet(IntSet* _pubthis);
  virtual ~p_IntSet(){ /* std::cout << "~p_IntSet" << std::endl;*/ };
  int              last{};
  int              first{};
  virtual void     destroy();
  static IntSet::t _new_IntSet(int _4055);
  void _initialize(int _4055);
  static IntSet::t _new_IntSet(int _4056, int _4057);
  void _initialize(int _4056, int _4057);
  virtual std::string indexToString(long long _4058);
  virtual std::string getname(std::shared_ptr<monty::ndarray<int, 1> > _4059);
  virtual std::string getname(long long _4060);
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _4061,
    std::shared_ptr<monty::ndarray<int, 1> > _4062);
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(int _4063, int _4064);
  virtual int getidx(int _4065);
  virtual long long stride(int _4066);
}; // struct IntSet;

struct p_StringSet : public ::mosek::fusion::p_BaseSet
{
  StringSet*                         _pubthis;
  static mosek::fusion::p_StringSet* _get_impl(mosek::fusion::StringSet* _inst)
  {
    return static_cast<mosek::fusion::p_StringSet*>(
      mosek::fusion::p_BaseSet::_get_impl(_inst));
  }
  static mosek::fusion::p_StringSet* _get_impl(
    mosek::fusion::StringSet::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_StringSet(StringSet* _pubthis);
  virtual ~p_StringSet(){ /* std::cout << "~p_StringSet" << std::endl;*/ };
  std::shared_ptr<monty::ndarray<std::string, 1> > keys{};
  virtual void        destroy();
  static StringSet::t _new_StringSet(
    std::shared_ptr<monty::ndarray<std::string, 1> > _4067);
  void _initialize(std::shared_ptr<monty::ndarray<std::string, 1> > _4067);
  virtual std::string indexToString(long long _4068);
  virtual std::string getname(std::shared_ptr<monty::ndarray<int, 1> > _4069);
  virtual std::string getname(long long _4070);
  virtual monty::rc_ptr< ::mosek::fusion::BaseSet> slice_(
    std::shared_ptr<monty::ndarray<int, 1> > _4071,
    std::shared_ptr<monty::ndarray<int, 1> > _4072);
  virtual monty::rc_ptr< ::mosek::fusion::BaseSet> slice_(int _4073, int _4074);
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _4076,
    std::shared_ptr<monty::ndarray<int, 1> > _4077);
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(int _4078, int _4079);
  virtual std::string toString();
  virtual long long stride(int _4082);
}; // struct StringSet;

struct p_NDSet : public ::mosek::fusion::p_Set
{
  NDSet*                         _pubthis;
  static mosek::fusion::p_NDSet* _get_impl(mosek::fusion::NDSet* _inst)
  {
    return static_cast<mosek::fusion::p_NDSet*>(
      mosek::fusion::p_Set::_get_impl(_inst));
  }
  static mosek::fusion::p_NDSet* _get_impl(mosek::fusion::NDSet::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_NDSet(NDSet* _pubthis);
  virtual ~p_NDSet(){ /* std::cout << "~p_NDSet" << std::endl;*/ };
  std::shared_ptr<monty::ndarray<long long, 1> > strides{};
  std::shared_ptr<monty::ndarray<int, 1> >       dimdef{};
  virtual void    destroy();
  static NDSet::t _new_NDSet(int _4085, int _4086, int _4087);
  void _initialize(int _4085, int _4086, int _4087);
  static NDSet::t _new_NDSet(int _4088, int _4089);
  void _initialize(int _4088, int _4089);
  static NDSet::t _new_NDSet(std::shared_ptr<monty::ndarray<int, 1> > _4090);
  void _initialize(std::shared_ptr<monty::ndarray<int, 1> > _4090);
  virtual std::string indexToString(long long _4094);
  virtual std::string getname(std::shared_ptr<monty::ndarray<int, 1> > _4098);
  virtual std::string getname(long long _4102);
  virtual int dim(int _4107);
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _4108,
    std::shared_ptr<monty::ndarray<int, 1> > _4109);
  virtual monty::rc_ptr< ::mosek::fusion::Set> slice(int _4113, int _4114);
  virtual std::shared_ptr<monty::ndarray<int, 1> > selectidxs(
    std::shared_ptr<monty::ndarray<std::string, 1> > _4115);
  virtual int linear_index_in_dim(
    int _4116, std::shared_ptr<monty::ndarray<int, 1> > _4117);
  virtual int linear_index_in_dim(int _4118, int _4119);
  static long long sumdims(std::shared_ptr<monty::ndarray<int, 1> > _4120);
  virtual long long stride(int _4123);
}; // struct NDSet;

struct p_ProductSet : public ::mosek::fusion::p_NDSet
{
  ProductSet*                         _pubthis;
  static mosek::fusion::p_ProductSet* _get_impl(
    mosek::fusion::ProductSet* _inst)
  {
    return static_cast<mosek::fusion::p_ProductSet*>(
      mosek::fusion::p_NDSet::_get_impl(_inst));
  }
  static mosek::fusion::p_ProductSet* _get_impl(
    mosek::fusion::ProductSet::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_ProductSet(ProductSet* _pubthis);
  virtual ~p_ProductSet(){ /* std::cout << "~p_ProductSet" << std::endl;*/ };
  std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Set>, 1> >
                       sets{};
  virtual void         destroy();
  static ProductSet::t _new_ProductSet(
    std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Set>, 1> >
      _4037);
  void _initialize(
    std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Set>, 1> >
      _4037);
  virtual std::string indexToString(long long _4039);
  static std::shared_ptr<monty::ndarray<int, 1> > computedims(
    std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Set>, 1> >
      _4049);
}; // struct ProductSet;

struct p_QConeDomain
{
  QConeDomain*                         _pubthis;
  static mosek::fusion::p_QConeDomain* _get_impl(
    mosek::fusion::QConeDomain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_QConeDomain* _get_impl(
    mosek::fusion::QConeDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_QConeDomain(QConeDomain* _pubthis);
  virtual ~p_QConeDomain(){ /* std::cout << "~p_QConeDomain" << std::endl;*/ };
  monty::rc_ptr< ::mosek::fusion::Set> shape{};
  bool                                 int_flag{};
  int                                  axisidx{};
  mosek::fusion::QConeKey              key{};
  virtual void                         destroy();
  static QConeDomain::t                _new_QConeDomain(
    mosek::fusion::QConeKey _4165,
    std::shared_ptr<monty::ndarray<int, 1> > _4166, int _4167);
  void _initialize(mosek::fusion::QConeKey _4165,
                   std::shared_ptr<monty::ndarray<int, 1> > _4166, int _4167);
  virtual std::string domainToString(
    long long                                            _4168,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _4169);
  virtual bool match_shape(monty::rc_ptr< ::mosek::fusion::Set> _4176);
  virtual monty::rc_ptr< ::mosek::fusion::QConeDomain> integral();
  virtual int                                          getAxis();
  virtual monty::rc_ptr< ::mosek::fusion::QConeDomain> axis(int _4177);
}; // struct QConeDomain;

struct p_LinPSDDomain
{
  LinPSDDomain*                         _pubthis;
  static mosek::fusion::p_LinPSDDomain* _get_impl(
    mosek::fusion::LinPSDDomain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_LinPSDDomain* _get_impl(
    mosek::fusion::LinPSDDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_LinPSDDomain(LinPSDDomain* _pubthis);
  virtual ~p_LinPSDDomain(){
    /* std::cout << "~p_LinPSDDomain" << std::endl;*/
  };
  monty::rc_ptr< ::mosek::fusion::Set> shape{};
  virtual void                         destroy();
  static LinPSDDomain::t               _new_LinPSDDomain(
    monty::rc_ptr< ::mosek::fusion::Set> _4178);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Set> _4178);
}; // struct LinPSDDomain;

struct p_PSDDomain
{
  PSDDomain*                         _pubthis;
  static mosek::fusion::p_PSDDomain* _get_impl(mosek::fusion::PSDDomain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_PSDDomain* _get_impl(
    mosek::fusion::PSDDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_PSDDomain(PSDDomain* _pubthis);
  virtual ~p_PSDDomain(){ /* std::cout << "~p_PSDDomain" << std::endl;*/ };
  mosek::fusion::PSDKey                key{};
  monty::rc_ptr< ::mosek::fusion::Set> shape{};
  virtual void                         destroy();
  static PSDDomain::t                  _new_PSDDomain(
    mosek::fusion::PSDKey _4179, monty::rc_ptr< ::mosek::fusion::Set> _4180);
  void _initialize(mosek::fusion::PSDKey                _4179,
                   monty::rc_ptr< ::mosek::fusion::Set> _4180);
  virtual std::string domainToString(
    long long                                            _4181,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _4182);
}; // struct PSDDomain;

struct p_RangeDomain
{
  RangeDomain*                         _pubthis;
  static mosek::fusion::p_RangeDomain* _get_impl(
    mosek::fusion::RangeDomain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_RangeDomain* _get_impl(
    mosek::fusion::RangeDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_RangeDomain(RangeDomain* _pubthis);
  virtual ~p_RangeDomain(){ /* std::cout << "~p_RangeDomain" << std::endl;*/ };
  bool sparse_flag{};
  bool cardinal_flag{};
  std::shared_ptr<monty::ndarray<double, 1> > ub{};
  std::shared_ptr<monty::ndarray<double, 1> > lb{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> idxmap{};
  monty::rc_ptr< ::mosek::fusion::Set>           shape{};
  virtual void                                   destroy();
  static RangeDomain::t                          _new_RangeDomain(
    std::shared_ptr<monty::ndarray<double, 1> >    _4187,
    std::shared_ptr<monty::ndarray<double, 1> >    _4188,
    std::shared_ptr<monty::ndarray<int, 1> >       _4189,
    std::shared_ptr<monty::ndarray<long long, 1> > _4190);
  void _initialize(std::shared_ptr<monty::ndarray<double, 1> >    _4187,
                   std::shared_ptr<monty::ndarray<double, 1> >    _4188,
                   std::shared_ptr<monty::ndarray<int, 1> >       _4189,
                   std::shared_ptr<monty::ndarray<long long, 1> > _4190);
  static RangeDomain::t _new_RangeDomain(
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _4192);
  void _initialize(monty::rc_ptr< ::mosek::fusion::RangeDomain> _4192);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain> symmetric();
  virtual monty::rc_ptr< ::mosek::fusion::RangeDomain>          sparse();
  virtual monty::rc_ptr< ::mosek::fusion::RangeDomain>          integral();
  virtual std::string                                           domainToString(
    long long                                            _4193,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _4194);
  virtual bool match_shape(monty::rc_ptr< ::mosek::fusion::Set> _4195);
  virtual double get_ub_item(long long _4196);
  virtual double get_lb_item(long long _4197);
}; // struct RangeDomain;

struct p_SymmetricRangeDomain : public ::mosek::fusion::p_RangeDomain
{
  SymmetricRangeDomain*                         _pubthis;
  static mosek::fusion::p_SymmetricRangeDomain* _get_impl(
    mosek::fusion::SymmetricRangeDomain* _inst)
  {
    return static_cast<mosek::fusion::p_SymmetricRangeDomain*>(
      mosek::fusion::p_RangeDomain::_get_impl(_inst));
  }
  static mosek::fusion::p_SymmetricRangeDomain* _get_impl(
    mosek::fusion::SymmetricRangeDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SymmetricRangeDomain(SymmetricRangeDomain* _pubthis);
  virtual ~p_SymmetricRangeDomain(){
    /* std::cout << "~p_SymmetricRangeDomain" << std::endl;*/
  };
  int                            dim{};
  virtual void                   destroy();
  static SymmetricRangeDomain::t _new_SymmetricRangeDomain(
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _4186);
  void _initialize(monty::rc_ptr< ::mosek::fusion::RangeDomain> _4186);
}; // struct SymmetricRangeDomain;

struct p_SymmetricLinearDomain
{
  SymmetricLinearDomain*                         _pubthis;
  static mosek::fusion::p_SymmetricLinearDomain* _get_impl(
    mosek::fusion::SymmetricLinearDomain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_SymmetricLinearDomain* _get_impl(
    mosek::fusion::SymmetricLinearDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SymmetricLinearDomain(SymmetricLinearDomain* _pubthis);
  virtual ~p_SymmetricLinearDomain(){
    /* std::cout << "~p_SymmetricLinearDomain" << std::endl;*/
  };
  bool                                          sparse_flag{};
  bool                                          cardinal_flag{};
  mosek::fusion::RelationKey                    key{};
  monty::rc_ptr< ::mosek::fusion::Set>          shape{};
  monty::rc_ptr< ::mosek::fusion::LinearDomain> dom{};
  int                                           dim{};
  virtual void                                  destroy();
  static SymmetricLinearDomain::t               _new_SymmetricLinearDomain(
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _4198);
  void _initialize(monty::rc_ptr< ::mosek::fusion::LinearDomain> _4198);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> sparse();
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> integral();
  virtual std::string                                            domainToString(
    long long                                            _4199,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _4200);
  virtual bool match_shape(monty::rc_ptr< ::mosek::fusion::Set> _4201);
  virtual double get_rhs_item(long long _4202);
}; // struct SymmetricLinearDomain;

struct p_LinearDomain
{
  LinearDomain*                         _pubthis;
  static mosek::fusion::p_LinearDomain* _get_impl(
    mosek::fusion::LinearDomain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_LinearDomain* _get_impl(
    mosek::fusion::LinearDomain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_LinearDomain(LinearDomain* _pubthis);
  virtual ~p_LinearDomain(){
    /* std::cout << "~p_LinearDomain" << std::endl;*/
  };
  bool                       sparse_flag{};
  bool                       cardinal_flag{};
  mosek::fusion::RelationKey key{};
  std::shared_ptr<monty::ndarray<double, 1> > bnd{};
  monty::rc_ptr< ::mosek::fusion::Utils::IntMap> inst{};
  monty::rc_ptr< ::mosek::fusion::Set>           shape{};
  virtual void                                   destroy();
  static LinearDomain::t                         _new_LinearDomain(
    mosek::fusion::RelationKey _4203,
    std::shared_ptr<monty::ndarray<double, 1> >    _4204,
    std::shared_ptr<monty::ndarray<long long, 1> > _4205,
    std::shared_ptr<monty::ndarray<int, 1> >       _4206);
  void _initialize(mosek::fusion::RelationKey _4203,
                   std::shared_ptr<monty::ndarray<double, 1> >    _4204,
                   std::shared_ptr<monty::ndarray<long long, 1> > _4205,
                   std::shared_ptr<monty::ndarray<int, 1> >       _4206);
  static LinearDomain::t _new_LinearDomain(
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _4208);
  void _initialize(monty::rc_ptr< ::mosek::fusion::LinearDomain> _4208);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> symmetric();
  virtual monty::rc_ptr< ::mosek::fusion::LinearDomain>          sparse();
  virtual monty::rc_ptr< ::mosek::fusion::LinearDomain>          integral();
  virtual std::string                                            domainToString(
    long long                                            _4209,
    monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _4210);
  virtual bool match_shape(monty::rc_ptr< ::mosek::fusion::Set> _4211);
  virtual double get_rhs_item(long long _4212);
  virtual bool scalable();
}; // struct LinearDomain;

struct p_Domain
{
  Domain*                         _pubthis;
  static mosek::fusion::p_Domain* _get_impl(mosek::fusion::Domain* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Domain* _get_impl(mosek::fusion::Domain::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Domain(Domain* _pubthis);
  virtual ~p_Domain(){ /* std::cout << "~p_Domain" << std::endl;*/ };
  virtual void     destroy();
  static long long dimsize(std::shared_ptr<monty::ndarray<int, 1> > _4213);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> mkLinearDomain(
    mosek::fusion::RelationKey              _4216,
    monty::rc_ptr< ::mosek::fusion::Matrix> _4217);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> mkLinearDomain(
    mosek::fusion::RelationKey _4223,
    std::shared_ptr<monty::ndarray<double, 2> > _4224);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> mkLinearDomain(
    mosek::fusion::RelationKey _4227,
    std::shared_ptr<monty::ndarray<double, 1> > _4228,
    std::shared_ptr<monty::ndarray<int, 1> >    _4229);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> mkLinearDomain(
    mosek::fusion::RelationKey _4232,
    std::shared_ptr<monty::ndarray<double, 1> > _4233);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> mkLinearDomain(
    mosek::fusion::RelationKey _4235, double _4236,
    std::shared_ptr<monty::ndarray<int, 1> > _4237);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> mkLinearDomain(
    mosek::fusion::RelationKey _4240, double _4241);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    std::shared_ptr<monty::ndarray<double, 1> > _4242,
    std::shared_ptr<monty::ndarray<double, 1> > _4243,
    std::shared_ptr<monty::ndarray<int, 1> >    _4244);
  static monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain> symmetric(
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _4246);
  static monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain> symmetric(
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _4247);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> sparse(
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _4248);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> sparse(
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _4249);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> integral(
    monty::rc_ptr< ::mosek::fusion::RangeDomain> _4250);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> integral(
    monty::rc_ptr< ::mosek::fusion::LinearDomain> _4251);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> integral(
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _4252);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> axis(
    monty::rc_ptr< ::mosek::fusion::QConeDomain> _4253, int _4254);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inRotatedQCone(
    std::shared_ptr<monty::ndarray<int, 1> > _4255);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inRotatedQCone(int _4257,
                                                                     int _4258);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inRotatedQCone(int _4259);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inRotatedQCone();
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inQCone(
    std::shared_ptr<monty::ndarray<int, 1> > _4260);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inQCone(int _4262,
                                                              int _4263);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain> inQCone(int _4264);
  static monty::rc_ptr< ::mosek::fusion::QConeDomain>  inQCone();
  static monty::rc_ptr< ::mosek::fusion::LinPSDDomain> isLinPSD(int _4265,
                                                                int _4266);
  static monty::rc_ptr< ::mosek::fusion::LinPSDDomain> isLinPSD(int _4267);
  static monty::rc_ptr< ::mosek::fusion::LinPSDDomain> isLinPSD();
  static monty::rc_ptr< ::mosek::fusion::PSDDomain> isTrilPSD(int _4268,
                                                              int _4269);
  static monty::rc_ptr< ::mosek::fusion::PSDDomain> isTrilPSD(int _4270);
  static monty::rc_ptr< ::mosek::fusion::PSDDomain> isTrilPSD();
  static monty::rc_ptr< ::mosek::fusion::PSDDomain> inPSDCone(int _4271,
                                                              int _4272);
  static monty::rc_ptr< ::mosek::fusion::PSDDomain> inPSDCone(int _4273);
  static monty::rc_ptr< ::mosek::fusion::PSDDomain>   inPSDCone();
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> binary();
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> binary(
    std::shared_ptr<monty::ndarray<int, 1> > _4274);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> binary(int _4277,
                                                             int _4278);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> binary(int _4281);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    monty::rc_ptr< ::mosek::fusion::Matrix> _4284,
    monty::rc_ptr< ::mosek::fusion::Matrix> _4285);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    monty::rc_ptr< ::mosek::fusion::Matrix> _4286, double _4287);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    double _4289, monty::rc_ptr< ::mosek::fusion::Matrix> _4290);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    std::shared_ptr<monty::ndarray<double, 1> > _4292,
    std::shared_ptr<monty::ndarray<double, 1> > _4293);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    std::shared_ptr<monty::ndarray<double, 1> > _4294, double _4295);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(
    double _4297, std::shared_ptr<monty::ndarray<double, 1> > _4298);
  static monty::rc_ptr< ::mosek::fusion::RangeDomain> inRange(double _4300,
                                                              double _4301);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(
    monty::rc_ptr< ::mosek::fusion::Matrix> _4302);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(
    std::shared_ptr<monty::ndarray<double, 1> > _4303,
    std::shared_ptr<monty::ndarray<int, 1> >    _4304);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(
    std::shared_ptr<monty::ndarray<double, 2> > _4305);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(
    std::shared_ptr<monty::ndarray<double, 1> > _4306);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(
    double _4307, std::shared_ptr<monty::ndarray<int, 1> > _4308);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(double _4309,
                                                                   int    _4310,
                                                                   int _4311);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(double _4312,
                                                                   int _4313);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> greaterThan(
    double _4314);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(
    monty::rc_ptr< ::mosek::fusion::Matrix> _4315);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(
    std::shared_ptr<monty::ndarray<double, 1> > _4316,
    std::shared_ptr<monty::ndarray<int, 1> >    _4317);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(
    std::shared_ptr<monty::ndarray<double, 2> > _4318);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(
    std::shared_ptr<monty::ndarray<double, 1> > _4319);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(
    double _4320, std::shared_ptr<monty::ndarray<int, 1> > _4321);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(double _4322,
                                                                int    _4323,
                                                                int    _4324);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(double _4325,
                                                                int    _4326);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> lessThan(double _4327);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(
    monty::rc_ptr< ::mosek::fusion::Matrix> _4328);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(
    std::shared_ptr<monty::ndarray<double, 1> > _4329,
    std::shared_ptr<monty::ndarray<int, 1> >    _4330);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(
    std::shared_ptr<monty::ndarray<double, 2> > _4331);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(
    std::shared_ptr<monty::ndarray<double, 1> > _4332);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(
    double _4333, std::shared_ptr<monty::ndarray<int, 1> > _4334);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(double _4335,
                                                                int    _4336,
                                                                int    _4337);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(double _4338,
                                                                int    _4339);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> equalsTo(double _4340);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> unbounded(
    std::shared_ptr<monty::ndarray<int, 1> > _4341);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> unbounded(int _4343,
                                                                 int _4344);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> unbounded(int _4345);
  static monty::rc_ptr< ::mosek::fusion::LinearDomain> unbounded();
}; // struct Domain;

struct p_SymmetricExpr
{
  SymmetricExpr*                         _pubthis;
  static mosek::fusion::p_SymmetricExpr* _get_impl(
    mosek::fusion::SymmetricExpr* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_SymmetricExpr* _get_impl(
    mosek::fusion::SymmetricExpr::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SymmetricExpr(SymmetricExpr* _pubthis);
  virtual ~p_SymmetricExpr(){
    /* std::cout << "~p_SymmetricExpr" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                                   xs{};
  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> b{};
  std::shared_ptr<
    monty::ndarray<monty::rc_ptr< ::mosek::fusion::SymmetricMatrix>, 1> >
                          Ms{};
  int                     n{};
  virtual void            destroy();
  static SymmetricExpr::t _new_SymmetricExpr(
    int _4354,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::SymmetricMatrix>, 1> >
      _4355,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                                     _4356,
    monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> _4357);
  void _initialize(
    int _4354,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::SymmetricMatrix>, 1> >
      _4355,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                                     _4356,
    monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> _4357);
  static monty::rc_ptr< ::mosek::fusion::SymmetricExpr> add(
    monty::rc_ptr< ::mosek::fusion::SymmetricExpr>   _4358,
    monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> _4359);
  static monty::rc_ptr< ::mosek::fusion::SymmetricExpr> mul(
    monty::rc_ptr< ::mosek::fusion::SymmetricExpr> _4360, double _4361);
  static monty::rc_ptr< ::mosek::fusion::SymmetricExpr> add(
    monty::rc_ptr< ::mosek::fusion::SymmetricExpr> _4363,
    monty::rc_ptr< ::mosek::fusion::SymmetricExpr> _4364);
  virtual std::string toString();
}; // struct SymmetricExpr;

struct p_Expr : public /*implements*/ ::mosek::fusion::Expression
{
  Expr*                         _pubthis;
  static mosek::fusion::p_Expr* _get_impl(mosek::fusion::Expr* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Expr* _get_impl(mosek::fusion::Expr::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Expr(Expr* _pubthis);
  virtual ~p_Expr(){ /* std::cout << "~p_Expr" << std::endl;*/ };
  std::shared_ptr<monty::ndarray<long long, 1> > varsb{};
  std::shared_ptr<monty::ndarray<long long, 1> > inst{};
  std::shared_ptr<monty::ndarray<double, 1> >    cof_v{};
  std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
    x{};
  std::shared_ptr<monty::ndarray<long long, 1> > subj{};
  std::shared_ptr<monty::ndarray<long long, 1> > ptrb{};
  std::shared_ptr<monty::ndarray<double, 1> >    bfix{};
  monty::rc_ptr< ::mosek::fusion::Set>   shape_p{};
  monty::rc_ptr< ::mosek::fusion::Model> model{};
  virtual void                           destroy();
  static Expr::t                         _new_Expr(
    std::shared_ptr<monty::ndarray<long long, 1> > _4375,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _4376,
    std::shared_ptr<monty::ndarray<long long, 1> > _4377,
    std::shared_ptr<monty::ndarray<double, 1> >    _4378,
    std::shared_ptr<monty::ndarray<double, 1> >    _4379,
    monty::rc_ptr< ::mosek::fusion::Set> _4380,
    std::shared_ptr<monty::ndarray<long long, 1> > _4381);
  void _initialize(
    std::shared_ptr<monty::ndarray<long long, 1> > _4375,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _4376,
    std::shared_ptr<monty::ndarray<long long, 1> > _4377,
    std::shared_ptr<monty::ndarray<double, 1> >    _4378,
    std::shared_ptr<monty::ndarray<double, 1> >    _4379,
    monty::rc_ptr< ::mosek::fusion::Set> _4380,
    std::shared_ptr<monty::ndarray<long long, 1> > _4381);
  static Expr::t _new_Expr(
    std::shared_ptr<monty::ndarray<long long, 1> > _4385,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _4386,
    std::shared_ptr<monty::ndarray<long long, 1> > _4387,
    std::shared_ptr<monty::ndarray<double, 1> >    _4388,
    std::shared_ptr<monty::ndarray<double, 1> >    _4389,
    monty::rc_ptr< ::mosek::fusion::Set> _4390,
    std::shared_ptr<monty::ndarray<long long, 1> > _4391, int _4392);
  void _initialize(
    std::shared_ptr<monty::ndarray<long long, 1> > _4385,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _4386,
    std::shared_ptr<monty::ndarray<long long, 1> > _4387,
    std::shared_ptr<monty::ndarray<double, 1> >    _4388,
    std::shared_ptr<monty::ndarray<double, 1> >    _4389,
    monty::rc_ptr< ::mosek::fusion::Set> _4390,
    std::shared_ptr<monty::ndarray<long long, 1> > _4391, int _4392);
  static Expr::t _new_Expr(monty::rc_ptr< ::mosek::fusion::Expression> _4394);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Expression> _4394);
  virtual std::string toString();
  virtual void tostr(monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer> _4403,
                     int _4404);
  static std::shared_ptr<
    monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
    varstack(std::shared_ptr<
             monty::ndarray<std::shared_ptr<monty::ndarray<
                              monty::rc_ptr< ::mosek::fusion::Variable>, 1> >,
                            1> >
               _4410);
  static std::shared_ptr<
    monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
    varstack(std::shared_ptr<
               monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
               _4413,
             std::shared_ptr<
               monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
               _4414);
  static monty::rc_ptr< ::mosek::fusion::Expression> flatten(
    monty::rc_ptr< ::mosek::fusion::Expression> _4418);
  static monty::rc_ptr< ::mosek::fusion::Expression> reshape(
    monty::rc_ptr< ::mosek::fusion::Expression> _4419, int _4420, int _4421);
  static monty::rc_ptr< ::mosek::fusion::Expression> reshape(
    monty::rc_ptr< ::mosek::fusion::Expression> _4422, int _4423);
  static monty::rc_ptr< ::mosek::fusion::Expression> reshape(
    monty::rc_ptr< ::mosek::fusion::Expression> _4424,
    monty::rc_ptr< ::mosek::fusion::Set>        _4425);
  virtual long long                                  size();
  virtual monty::rc_ptr< ::mosek::fusion::FlatExpr>  eval();
  static monty::rc_ptr< ::mosek::fusion::Expression> zeros(int _4431);
  static monty::rc_ptr< ::mosek::fusion::Expression> ones(int _4437);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _4442);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(
    monty::rc_ptr< ::mosek::fusion::Matrix> _4450);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(double _4462);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(
    monty::rc_ptr< ::mosek::fusion::Set> _4468, double _4469);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(int    _4475,
                                                               double _4476);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(
    std::shared_ptr<monty::ndarray<double, 2> > _4482);
  static monty::rc_ptr< ::mosek::fusion::Expression> constTerm(
    std::shared_ptr<monty::ndarray<double, 1> > _4492);
  virtual long long                                  numNonzeros();
  static monty::rc_ptr< ::mosek::fusion::Expression> sum_expr(
    monty::rc_ptr< ::mosek::fusion::Expression> _4498, int _4499, int _4500);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum_var(
    monty::rc_ptr< ::mosek::fusion::Variable> _4546, int _4547, int _4548);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum(
    monty::rc_ptr< ::mosek::fusion::Expression> _4578, int _4579, int _4580);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum(
    monty::rc_ptr< ::mosek::fusion::Expression> _4581, int _4582);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum(
    monty::rc_ptr< ::mosek::fusion::Variable> _4583, int _4584, int _4585);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum(
    monty::rc_ptr< ::mosek::fusion::Variable> _4586, int _4587);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum(
    monty::rc_ptr< ::mosek::fusion::Variable> _4588);
  static monty::rc_ptr< ::mosek::fusion::Expression> sum(
    monty::rc_ptr< ::mosek::fusion::Expression> _4589);
  static monty::rc_ptr< ::mosek::fusion::Expression> neg(
    monty::rc_ptr< ::mosek::fusion::Variable> _4597);
  static monty::rc_ptr< ::mosek::fusion::Expression> neg(
    monty::rc_ptr< ::mosek::fusion::Expression> _4602);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul__(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _4608,
    monty::rc_ptr< ::mosek::fusion::Expression> _4609);
  static void sparseMatrixVector(
    std::shared_ptr<monty::ndarray<long long, 1> > _4721,
    std::shared_ptr<monty::ndarray<int, 1> >       _4722,
    std::shared_ptr<monty::ndarray<double, 1> >    _4723,
    std::shared_ptr<monty::ndarray<double, 1> >    _4724,
    std::shared_ptr<monty::ndarray<double, 1> > _4725, int _4726);
  static void sparseMatmul(std::shared_ptr<monty::ndarray<long long, 1> > _4731,
                           std::shared_ptr<monty::ndarray<int, 1> >       _4732,
                           std::shared_ptr<monty::ndarray<double, 1> >    _4733,
                           std::shared_ptr<monty::ndarray<long long, 1> > _4734,
                           std::shared_ptr<monty::ndarray<int, 1> >       _4735,
                           std::shared_ptr<monty::ndarray<double, 1> >    _4736,
                           std::shared_ptr<monty::ndarray<long long, 1> > _4737,
                           std::shared_ptr<monty::ndarray<int, 1> >       _4738,
                           std::shared_ptr<monty::ndarray<double, 1> >    _4739,
                           int _4740, int                           _4741,
                           std::shared_ptr<monty::ndarray<int, 1> > _4742);
  static long long computeNz(
    std::shared_ptr<monty::ndarray<long long, 1> > _4754,
    std::shared_ptr<monty::ndarray<int, 1> >       _4755,
    std::shared_ptr<monty::ndarray<long long, 1> > _4756,
    std::shared_ptr<monty::ndarray<int, 1> > _4757, int _4758, int _4759,
    std::shared_ptr<monty::ndarray<int, 1> >       _4760,
    std::shared_ptr<monty::ndarray<long long, 1> > _4761);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    monty::rc_ptr< ::mosek::fusion::Variable> _4770,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _4771);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _4796,
    monty::rc_ptr< ::mosek::fusion::Variable> _4797);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    monty::rc_ptr< ::mosek::fusion::Expression> _4816,
    monty::rc_ptr< ::mosek::fusion::Matrix>     _4817);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _4900,
    monty::rc_ptr< ::mosek::fusion::Expression> _4901);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    monty::rc_ptr< ::mosek::fusion::Variable> _4966,
    std::shared_ptr<monty::ndarray<double, 2> > _4967);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    std::shared_ptr<monty::ndarray<double, 2> > _4968,
    monty::rc_ptr< ::mosek::fusion::Variable> _4969);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    monty::rc_ptr< ::mosek::fusion::Expression> _4970,
    std::shared_ptr<monty::ndarray<double, 2> > _4971);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulDiag(
    std::shared_ptr<monty::ndarray<double, 2> > _4972,
    monty::rc_ptr< ::mosek::fusion::Expression> _4973);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm_(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _4974,
    monty::rc_ptr< ::mosek::fusion::Expression> _4975);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm_(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _4983,
    monty::rc_ptr< ::mosek::fusion::Variable> _4984);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm_(
    std::shared_ptr<monty::ndarray<double, 1> > _4991,
    monty::rc_ptr< ::mosek::fusion::Variable> _4992);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm_(
    std::shared_ptr<monty::ndarray<double, 1> > _4993,
    monty::rc_ptr< ::mosek::fusion::Expression> _4994);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm_(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _4996,
    monty::rc_ptr< ::mosek::fusion::Expression>    _4997);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm_(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _5002,
    monty::rc_ptr< ::mosek::fusion::Variable>      _5003);
  static monty::rc_ptr< ::mosek::fusion::Expression> dotmul_(
    std::shared_ptr<monty::ndarray<long long, 1> > _5005,
    std::shared_ptr<monty::ndarray<double, 1> >    _5006,
    monty::rc_ptr< ::mosek::fusion::Variable> _5007,
    monty::rc_ptr< ::mosek::fusion::Set>      _5008);
  static monty::rc_ptr< ::mosek::fusion::Expression> dotmul_(
    std::shared_ptr<monty::ndarray<long long, 1> > _5014,
    std::shared_ptr<monty::ndarray<double, 1> >    _5015,
    std::shared_ptr<monty::ndarray<long long, 1> > _5016,
    std::shared_ptr<monty::ndarray<long long, 1> > _5017,
    std::shared_ptr<monty::ndarray<double, 1> >    _5018,
    std::shared_ptr<monty::ndarray<double, 1> >    _5019,
    std::shared_ptr<monty::ndarray<long long, 1> > _5020,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                         _5021,
    monty::rc_ptr< ::mosek::fusion::Set> _5022);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _5043,
    monty::rc_ptr< ::mosek::fusion::Expression> _5044);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Expression> _5057,
    monty::rc_ptr< ::mosek::fusion::Matrix>     _5058);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Expression> _5071,
    std::shared_ptr<monty::ndarray<double, 1> > _5072);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    std::shared_ptr<monty::ndarray<double, 1> > _5080,
    monty::rc_ptr< ::mosek::fusion::Expression> _5081);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    double _5089, monty::rc_ptr< ::mosek::fusion::Expression> _5090);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Expression> _5094, double _5095);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul_SMatrix_2DSExpr(
    std::shared_ptr<monty::ndarray<long long, 1> > _5096,
    std::shared_ptr<monty::ndarray<long long, 1> > _5097,
    std::shared_ptr<monty::ndarray<double, 1> >    _5098,
    std::shared_ptr<monty::ndarray<double, 1> >    _5099,
    std::shared_ptr<monty::ndarray<long long, 1> > _5100,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                         _5101,
    monty::rc_ptr< ::mosek::fusion::Set> _5102, int _5103, int _5104,
    std::shared_ptr<monty::ndarray<int, 1> >    _5105,
    std::shared_ptr<monty::ndarray<int, 1> >    _5106,
    std::shared_ptr<monty::ndarray<double, 1> > _5107, int _5108, int _5109);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul_2DSExpr_SMatrix(
    std::shared_ptr<monty::ndarray<long long, 1> > _5161,
    std::shared_ptr<monty::ndarray<long long, 1> > _5162,
    std::shared_ptr<monty::ndarray<double, 1> >    _5163,
    std::shared_ptr<monty::ndarray<double, 1> >    _5164,
    std::shared_ptr<monty::ndarray<long long, 1> > _5165,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                         _5166,
    monty::rc_ptr< ::mosek::fusion::Set> _5167, int _5168, int _5169,
    std::shared_ptr<monty::ndarray<int, 1> >    _5170,
    std::shared_ptr<monty::ndarray<int, 1> >    _5171,
    std::shared_ptr<monty::ndarray<double, 1> > _5172, int _5173, int _5174);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul_DMatrix_2DDExpr(
    std::shared_ptr<monty::ndarray<long long, 1> > _5222,
    std::shared_ptr<monty::ndarray<long long, 1> > _5223,
    std::shared_ptr<monty::ndarray<double, 1> >    _5224,
    std::shared_ptr<monty::ndarray<double, 1> >    _5225,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                         _5226,
    monty::rc_ptr< ::mosek::fusion::Set> _5227, int _5228, int _5229,
    std::shared_ptr<monty::ndarray<double, 1> > _5230, int _5231, int _5232);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul_2DDExpr_DMatrix(
    std::shared_ptr<monty::ndarray<long long, 1> > _5252,
    std::shared_ptr<monty::ndarray<long long, 1> > _5253,
    std::shared_ptr<monty::ndarray<double, 1> >    _5254,
    std::shared_ptr<monty::ndarray<double, 1> >    _5255,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                         _5256,
    monty::rc_ptr< ::mosek::fusion::Set> _5257, int _5258, int _5259,
    std::shared_ptr<monty::ndarray<double, 1> > _5260, int _5261, int _5262);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul_0DExpr_Matrix(
    std::shared_ptr<monty::ndarray<long long, 1> > _5284,
    std::shared_ptr<monty::ndarray<double, 1> >    _5285,
    std::shared_ptr<monty::ndarray<double, 1> >    _5286,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
                                            _5287,
    monty::rc_ptr< ::mosek::fusion::Matrix> _5288);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Variable> _5306,
    std::shared_ptr<monty::ndarray<double, 2> > _5307);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    std::shared_ptr<monty::ndarray<double, 2> > _5311,
    monty::rc_ptr< ::mosek::fusion::Variable> _5312);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Variable> _5313, double _5314);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    double _5315, monty::rc_ptr< ::mosek::fusion::Variable> _5316);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    std::shared_ptr<monty::ndarray<double, 1> > _5323,
    monty::rc_ptr< ::mosek::fusion::Variable> _5324);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Variable> _5336,
    std::shared_ptr<monty::ndarray<double, 1> > _5337);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Variable> _5355,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _5356);
  static monty::rc_ptr< ::mosek::fusion::Expression> mul(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _5399,
    monty::rc_ptr< ::mosek::fusion::Variable> _5400);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot_(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _5446,
    monty::rc_ptr< ::mosek::fusion::Expression> _5447);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot_(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _5455,
    monty::rc_ptr< ::mosek::fusion::Variable> _5456);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot_(
    std::shared_ptr<monty::ndarray<double, 1> > _5463,
    monty::rc_ptr< ::mosek::fusion::Variable> _5464);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot_(
    std::shared_ptr<monty::ndarray<double, 1> > _5465,
    monty::rc_ptr< ::mosek::fusion::Expression> _5466);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot_(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _5468,
    monty::rc_ptr< ::mosek::fusion::Expression>    _5469);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot_(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _5474,
    monty::rc_ptr< ::mosek::fusion::Variable>      _5475);
  static monty::rc_ptr< ::mosek::fusion::Expression> inner_(
    std::shared_ptr<monty::ndarray<long long, 1> > _5477,
    std::shared_ptr<monty::ndarray<double, 1> >    _5478,
    monty::rc_ptr< ::mosek::fusion::Variable> _5479);
  static monty::rc_ptr< ::mosek::fusion::Expression> inner_(
    std::shared_ptr<monty::ndarray<long long, 1> > _5485,
    std::shared_ptr<monty::ndarray<double, 1> >    _5486,
    std::shared_ptr<monty::ndarray<long long, 1> > _5487,
    std::shared_ptr<monty::ndarray<long long, 1> > _5488,
    std::shared_ptr<monty::ndarray<double, 1> >    _5489,
    std::shared_ptr<monty::ndarray<double, 1> >    _5490,
    std::shared_ptr<monty::ndarray<long long, 1> > _5491,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _5492);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer(
    std::shared_ptr<monty::ndarray<double, 1> > _5509,
    monty::rc_ptr< ::mosek::fusion::Expression> _5510);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer(
    monty::rc_ptr< ::mosek::fusion::Expression> _5513,
    std::shared_ptr<monty::ndarray<double, 1> > _5514);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _5517,
    monty::rc_ptr< ::mosek::fusion::Variable> _5518);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer(
    monty::rc_ptr< ::mosek::fusion::Variable> _5524,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _5525);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer(
    std::shared_ptr<monty::ndarray<double, 1> > _5531,
    monty::rc_ptr< ::mosek::fusion::Variable> _5532);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer(
    monty::rc_ptr< ::mosek::fusion::Variable> _5533,
    std::shared_ptr<monty::ndarray<double, 1> > _5534);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer_(
    int _5535, std::shared_ptr<monty::ndarray<long long, 1> > _5536,
    std::shared_ptr<monty::ndarray<long long, 1> > _5537,
    std::shared_ptr<monty::ndarray<double, 1> >    _5538,
    std::shared_ptr<monty::ndarray<double, 1> >    _5539,
    std::shared_ptr<monty::ndarray<long long, 1> > _5540,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _5541,
    std::shared_ptr<monty::ndarray<double, 1> > _5542,
    std::shared_ptr<monty::ndarray<int, 1> > _5543, int _5544, bool _5545);
  static monty::rc_ptr< ::mosek::fusion::Expression> outer_(
    monty::rc_ptr< ::mosek::fusion::Variable> _5575, int _5576,
    std::shared_ptr<monty::ndarray<double, 1> >          _5577,
    std::shared_ptr<monty::ndarray<int, 1> > _5578, int _5579, bool _5580);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> pick(
    std::shared_ptr<monty::ndarray<int, 2> > _5597);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> pick(
    std::shared_ptr<monty::ndarray<int, 1> > _5603);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> pick_(
    std::shared_ptr<monty::ndarray<long long, 1> > _5606);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<
                       monty::rc_ptr< ::mosek::fusion::Expression>, 1> >,
                     1> >
      _5644);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(double _5650,
                                                            double _5651,
                                                            double _5652);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5653, double _5654,
    monty::rc_ptr< ::mosek::fusion::Variable> _5655);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5656, double _5657,
    monty::rc_ptr< ::mosek::fusion::Expression> _5658);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5659, monty::rc_ptr< ::mosek::fusion::Variable> _5660,
    double _5661);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5662, monty::rc_ptr< ::mosek::fusion::Variable> _5663,
    monty::rc_ptr< ::mosek::fusion::Variable> _5664);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5665, monty::rc_ptr< ::mosek::fusion::Variable> _5666,
    monty::rc_ptr< ::mosek::fusion::Expression> _5667);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5668, monty::rc_ptr< ::mosek::fusion::Expression> _5669,
    double _5670);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5671, monty::rc_ptr< ::mosek::fusion::Expression> _5672,
    monty::rc_ptr< ::mosek::fusion::Variable> _5673);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5674, monty::rc_ptr< ::mosek::fusion::Expression> _5675,
    monty::rc_ptr< ::mosek::fusion::Expression> _5676);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5677, double _5678,
    double _5679);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5680, double _5681,
    monty::rc_ptr< ::mosek::fusion::Variable> _5682);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5683, double _5684,
    monty::rc_ptr< ::mosek::fusion::Expression> _5685);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5686,
    monty::rc_ptr< ::mosek::fusion::Variable> _5687, double _5688);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5689,
    monty::rc_ptr< ::mosek::fusion::Variable> _5690,
    monty::rc_ptr< ::mosek::fusion::Variable> _5691);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5692,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5693,
    monty::rc_ptr< ::mosek::fusion::Expression> _5694);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5695,
    monty::rc_ptr< ::mosek::fusion::Expression> _5696, double _5697);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5698,
    monty::rc_ptr< ::mosek::fusion::Expression> _5699,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5700);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5701,
    monty::rc_ptr< ::mosek::fusion::Expression> _5702,
    monty::rc_ptr< ::mosek::fusion::Expression> _5703);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5704, double _5705,
    double _5706);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5707, double _5708,
    monty::rc_ptr< ::mosek::fusion::Variable> _5709);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5710, double _5711,
    monty::rc_ptr< ::mosek::fusion::Expression> _5712);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5713,
    monty::rc_ptr< ::mosek::fusion::Variable> _5714, double _5715);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5716,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5717,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5718);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5719,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5720,
    monty::rc_ptr< ::mosek::fusion::Expression> _5721);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5722,
    monty::rc_ptr< ::mosek::fusion::Expression> _5723, double _5724);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5725,
    monty::rc_ptr< ::mosek::fusion::Expression> _5726,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5727);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5728,
    monty::rc_ptr< ::mosek::fusion::Expression> _5729,
    monty::rc_ptr< ::mosek::fusion::Expression> _5730);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5731, monty::rc_ptr< ::mosek::fusion::Variable> _5732);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    double _5733, monty::rc_ptr< ::mosek::fusion::Expression> _5734);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5735, double _5736);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5737,
    monty::rc_ptr< ::mosek::fusion::Variable> _5738);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5739,
    monty::rc_ptr< ::mosek::fusion::Expression> _5740);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5741, double _5742);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5743,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5744);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5745,
    monty::rc_ptr< ::mosek::fusion::Expression> _5746);
  static monty::rc_ptr< ::mosek::fusion::Expression> vstack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Expression>, 1> >
      _5747);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5749,
    monty::rc_ptr< ::mosek::fusion::Expression> _5750,
    monty::rc_ptr< ::mosek::fusion::Expression> _5751);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5752,
    monty::rc_ptr< ::mosek::fusion::Expression> _5753,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5754);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5755,
    monty::rc_ptr< ::mosek::fusion::Expression> _5756, double _5757);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5758,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5759,
    monty::rc_ptr< ::mosek::fusion::Expression> _5760);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5761,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5762,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5763);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5764,
    monty::rc_ptr< ::mosek::fusion::Variable> _5765, double _5766);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5767, double _5768,
    monty::rc_ptr< ::mosek::fusion::Expression> _5769);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5770, double _5771,
    monty::rc_ptr< ::mosek::fusion::Variable> _5772);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5773, double _5774,
    double _5775);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5776,
    monty::rc_ptr< ::mosek::fusion::Expression> _5777,
    monty::rc_ptr< ::mosek::fusion::Expression> _5778);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5779,
    monty::rc_ptr< ::mosek::fusion::Expression> _5780,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5781);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5782,
    monty::rc_ptr< ::mosek::fusion::Expression> _5783, double _5784);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5785,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5786,
    monty::rc_ptr< ::mosek::fusion::Expression> _5787);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5788,
    monty::rc_ptr< ::mosek::fusion::Variable> _5789,
    monty::rc_ptr< ::mosek::fusion::Variable> _5790);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5791,
    monty::rc_ptr< ::mosek::fusion::Variable> _5792, double _5793);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5794, double _5795,
    monty::rc_ptr< ::mosek::fusion::Expression> _5796);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5797, double _5798,
    monty::rc_ptr< ::mosek::fusion::Variable> _5799);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5800, double _5801,
    double _5802);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5803, monty::rc_ptr< ::mosek::fusion::Expression> _5804,
    monty::rc_ptr< ::mosek::fusion::Expression> _5805);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5806, monty::rc_ptr< ::mosek::fusion::Expression> _5807,
    monty::rc_ptr< ::mosek::fusion::Variable> _5808);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5809, monty::rc_ptr< ::mosek::fusion::Expression> _5810,
    double _5811);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5812, monty::rc_ptr< ::mosek::fusion::Variable> _5813,
    monty::rc_ptr< ::mosek::fusion::Expression> _5814);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5815, monty::rc_ptr< ::mosek::fusion::Variable> _5816,
    monty::rc_ptr< ::mosek::fusion::Variable> _5817);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5818, monty::rc_ptr< ::mosek::fusion::Variable> _5819,
    double _5820);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5821, double _5822,
    monty::rc_ptr< ::mosek::fusion::Expression> _5823);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5824, double _5825,
    monty::rc_ptr< ::mosek::fusion::Variable> _5826);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable>   _5827,
    monty::rc_ptr< ::mosek::fusion::Expression> _5828);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5829,
    monty::rc_ptr< ::mosek::fusion::Variable> _5830);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Variable> _5831, double _5832);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5833, monty::rc_ptr< ::mosek::fusion::Expression> _5834);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    double _5835, monty::rc_ptr< ::mosek::fusion::Variable> _5836);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5837,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5838);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5839, double _5840);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    monty::rc_ptr< ::mosek::fusion::Expression> _5841,
    monty::rc_ptr< ::mosek::fusion::Expression> _5842);
  static monty::rc_ptr< ::mosek::fusion::Expression> hstack(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Expression>, 1> >
      _5843);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5845, monty::rc_ptr< ::mosek::fusion::Expression> _5846,
    monty::rc_ptr< ::mosek::fusion::Expression> _5847,
    monty::rc_ptr< ::mosek::fusion::Expression> _5848);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5849, monty::rc_ptr< ::mosek::fusion::Expression> _5850,
    monty::rc_ptr< ::mosek::fusion::Expression> _5851,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5852);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5853, monty::rc_ptr< ::mosek::fusion::Expression> _5854,
    monty::rc_ptr< ::mosek::fusion::Expression> _5855, double _5856);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5857, monty::rc_ptr< ::mosek::fusion::Expression> _5858,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5859,
    monty::rc_ptr< ::mosek::fusion::Expression> _5860);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5861, monty::rc_ptr< ::mosek::fusion::Expression> _5862,
    monty::rc_ptr< ::mosek::fusion::Variable> _5863,
    monty::rc_ptr< ::mosek::fusion::Variable> _5864);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5865, monty::rc_ptr< ::mosek::fusion::Expression> _5866,
    monty::rc_ptr< ::mosek::fusion::Variable> _5867, double _5868);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5869, monty::rc_ptr< ::mosek::fusion::Expression> _5870, double _5871,
    monty::rc_ptr< ::mosek::fusion::Expression> _5872);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5873, monty::rc_ptr< ::mosek::fusion::Expression> _5874, double _5875,
    monty::rc_ptr< ::mosek::fusion::Variable> _5876);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5877, monty::rc_ptr< ::mosek::fusion::Expression> _5878, double _5879,
    double _5880);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5881, monty::rc_ptr< ::mosek::fusion::Variable> _5882,
    monty::rc_ptr< ::mosek::fusion::Expression> _5883,
    monty::rc_ptr< ::mosek::fusion::Expression> _5884);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5885, monty::rc_ptr< ::mosek::fusion::Variable> _5886,
    monty::rc_ptr< ::mosek::fusion::Expression> _5887,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5888);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5889, monty::rc_ptr< ::mosek::fusion::Variable> _5890,
    monty::rc_ptr< ::mosek::fusion::Expression> _5891, double _5892);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5893, monty::rc_ptr< ::mosek::fusion::Variable> _5894,
    monty::rc_ptr< ::mosek::fusion::Variable>   _5895,
    monty::rc_ptr< ::mosek::fusion::Expression> _5896);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5897, monty::rc_ptr< ::mosek::fusion::Variable> _5898,
    monty::rc_ptr< ::mosek::fusion::Variable> _5899,
    monty::rc_ptr< ::mosek::fusion::Variable> _5900);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5901, monty::rc_ptr< ::mosek::fusion::Variable> _5902,
    monty::rc_ptr< ::mosek::fusion::Variable> _5903, double _5904);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5905, monty::rc_ptr< ::mosek::fusion::Variable> _5906, double _5907,
    monty::rc_ptr< ::mosek::fusion::Expression> _5908);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5909, monty::rc_ptr< ::mosek::fusion::Variable> _5910, double _5911,
    monty::rc_ptr< ::mosek::fusion::Variable> _5912);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5913, monty::rc_ptr< ::mosek::fusion::Variable> _5914, double _5915,
    double _5916);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5917, double _5918, monty::rc_ptr< ::mosek::fusion::Expression> _5919,
    monty::rc_ptr< ::mosek::fusion::Expression> _5920);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5921, double _5922, monty::rc_ptr< ::mosek::fusion::Expression> _5923,
    monty::rc_ptr< ::mosek::fusion::Variable> _5924);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5925, double _5926, monty::rc_ptr< ::mosek::fusion::Expression> _5927,
    double _5928);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5929, double _5930, monty::rc_ptr< ::mosek::fusion::Variable> _5931,
    monty::rc_ptr< ::mosek::fusion::Expression> _5932);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5933, double _5934, monty::rc_ptr< ::mosek::fusion::Variable> _5935,
    monty::rc_ptr< ::mosek::fusion::Variable> _5936);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5937, double _5938, monty::rc_ptr< ::mosek::fusion::Variable> _5939,
    double _5940);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5941, double _5942, double _5943,
    monty::rc_ptr< ::mosek::fusion::Expression> _5944);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5945, double _5946, double _5947,
    monty::rc_ptr< ::mosek::fusion::Variable> _5948);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5949, monty::rc_ptr< ::mosek::fusion::Variable> _5950,
    monty::rc_ptr< ::mosek::fusion::Expression> _5951);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5952, monty::rc_ptr< ::mosek::fusion::Variable> _5953,
    monty::rc_ptr< ::mosek::fusion::Variable> _5954);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5955, monty::rc_ptr< ::mosek::fusion::Variable> _5956, double _5957);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5958, double _5959, monty::rc_ptr< ::mosek::fusion::Expression> _5960);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5961, double _5962, monty::rc_ptr< ::mosek::fusion::Variable> _5963);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5964, monty::rc_ptr< ::mosek::fusion::Expression> _5965,
    monty::rc_ptr< ::mosek::fusion::Variable> _5966);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5967, monty::rc_ptr< ::mosek::fusion::Expression> _5968, double _5969);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5970, monty::rc_ptr< ::mosek::fusion::Expression> _5971,
    monty::rc_ptr< ::mosek::fusion::Expression> _5972);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack(
    int _5973,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Expression>, 1> >
      _5974);
  static monty::rc_ptr< ::mosek::fusion::Expression> stack_(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Expression>, 1> >
        _5975,
    int _5976);
  static monty::rc_ptr< ::mosek::fusion::Expression> repeat(
    monty::rc_ptr< ::mosek::fusion::Expression> _6064, int _6065, int _6066);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Expression>, 1> >
      _6068);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _6138);
  static monty::rc_ptr< ::mosek::fusion::Expression> add_(
    monty::rc_ptr< ::mosek::fusion::Expression> _6151, double _6152,
    monty::rc_ptr< ::mosek::fusion::Expression> _6153, double _6154);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> transpose();
  virtual monty::rc_ptr< ::mosek::fusion::Expression> slice(
    std::shared_ptr<monty::ndarray<int, 1> > _6254,
    std::shared_ptr<monty::ndarray<int, 1> > _6255);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> index(
    std::shared_ptr<monty::ndarray<int, 1> > _6296);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> index(int _6299);
  virtual monty::rc_ptr< ::mosek::fusion::Expression> slice(int _6300,
                                                            int _6301);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6302,
    monty::rc_ptr< ::mosek::fusion::Expression> _6303);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6304,
    monty::rc_ptr< ::mosek::fusion::Variable> _6305);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6306,
    monty::rc_ptr< ::mosek::fusion::Variable>      _6307);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6308,
    monty::rc_ptr< ::mosek::fusion::Expression>    _6309);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    std::shared_ptr<monty::ndarray<double, 2> > _6310,
    monty::rc_ptr< ::mosek::fusion::Variable> _6311);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    std::shared_ptr<monty::ndarray<double, 2> > _6312,
    monty::rc_ptr< ::mosek::fusion::Expression> _6313);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    std::shared_ptr<monty::ndarray<double, 1> > _6314,
    monty::rc_ptr< ::mosek::fusion::Variable> _6315);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    std::shared_ptr<monty::ndarray<double, 1> > _6316,
    monty::rc_ptr< ::mosek::fusion::Expression> _6317);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Expression> _6318,
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6319);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Expression> _6320,
    std::shared_ptr<monty::ndarray<double, 2> > _6321);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Expression> _6322,
    std::shared_ptr<monty::ndarray<double, 1> > _6323);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Expression>    _6324,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6325);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Variable> _6326,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6327);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Variable>      _6328,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6329);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Variable> _6330,
    std::shared_ptr<monty::ndarray<double, 2> > _6331);
  static monty::rc_ptr< ::mosek::fusion::Expression> mulElm(
    monty::rc_ptr< ::mosek::fusion::Variable> _6332,
    std::shared_ptr<monty::ndarray<double, 1> > _6333);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6334,
    monty::rc_ptr< ::mosek::fusion::Expression> _6335);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6336,
    monty::rc_ptr< ::mosek::fusion::Variable> _6337);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6338,
    monty::rc_ptr< ::mosek::fusion::Variable>      _6339);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6340,
    monty::rc_ptr< ::mosek::fusion::Expression>    _6341);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    std::shared_ptr<monty::ndarray<double, 2> > _6342,
    monty::rc_ptr< ::mosek::fusion::Variable> _6343);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    std::shared_ptr<monty::ndarray<double, 2> > _6344,
    monty::rc_ptr< ::mosek::fusion::Expression> _6345);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    std::shared_ptr<monty::ndarray<double, 1> > _6346,
    monty::rc_ptr< ::mosek::fusion::Variable> _6347);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    std::shared_ptr<monty::ndarray<double, 1> > _6348,
    monty::rc_ptr< ::mosek::fusion::Expression> _6349);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Expression> _6350,
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6351);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Expression> _6352,
    std::shared_ptr<monty::ndarray<double, 2> > _6353);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Expression> _6354,
    std::shared_ptr<monty::ndarray<double, 1> > _6355);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Expression>    _6356,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6357);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Variable>      _6358,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6359);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Variable> _6360,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6361);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Variable> _6362,
    std::shared_ptr<monty::ndarray<double, 2> > _6363);
  static monty::rc_ptr< ::mosek::fusion::Expression> dot(
    monty::rc_ptr< ::mosek::fusion::Variable> _6364,
    std::shared_ptr<monty::ndarray<double, 1> > _6365);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6366,
    monty::rc_ptr< ::mosek::fusion::Variable>      _6367);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable>      _6368,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6369);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6370,
    monty::rc_ptr< ::mosek::fusion::Variable> _6371);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable> _6372,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6373);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    double _6374, monty::rc_ptr< ::mosek::fusion::Variable> _6375);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable> _6376, double _6377);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    std::shared_ptr<monty::ndarray<double, 2> > _6378,
    monty::rc_ptr< ::mosek::fusion::Variable> _6379);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    std::shared_ptr<monty::ndarray<double, 1> > _6380,
    monty::rc_ptr< ::mosek::fusion::Variable> _6381);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable> _6382,
    std::shared_ptr<monty::ndarray<double, 2> > _6383);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable> _6384,
    std::shared_ptr<monty::ndarray<double, 1> > _6385);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable> _6386,
    monty::rc_ptr< ::mosek::fusion::Variable> _6387);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6388,
    monty::rc_ptr< ::mosek::fusion::Expression>    _6389);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression>    _6390,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6391);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6392,
    monty::rc_ptr< ::mosek::fusion::Expression> _6393);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression> _6394,
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6395);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    double _6396, monty::rc_ptr< ::mosek::fusion::Expression> _6397);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression> _6398, double _6399);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    std::shared_ptr<monty::ndarray<double, 2> > _6400,
    monty::rc_ptr< ::mosek::fusion::Expression> _6401);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    std::shared_ptr<monty::ndarray<double, 1> > _6402,
    monty::rc_ptr< ::mosek::fusion::Expression> _6403);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression> _6404,
    std::shared_ptr<monty::ndarray<double, 2> > _6405);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression> _6406,
    std::shared_ptr<monty::ndarray<double, 1> > _6407);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Variable>   _6408,
    monty::rc_ptr< ::mosek::fusion::Expression> _6409);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression> _6410,
    monty::rc_ptr< ::mosek::fusion::Variable>   _6411);
  static monty::rc_ptr< ::mosek::fusion::Expression> sub(
    monty::rc_ptr< ::mosek::fusion::Expression> _6412,
    monty::rc_ptr< ::mosek::fusion::Expression> _6413);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6414,
    monty::rc_ptr< ::mosek::fusion::Variable>      _6415);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable>      _6416,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6417);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6418,
    monty::rc_ptr< ::mosek::fusion::Variable> _6419);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable> _6420,
    monty::rc_ptr< ::mosek::fusion::Matrix>   _6421);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    double _6422, monty::rc_ptr< ::mosek::fusion::Variable> _6423);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable> _6424, double _6425);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    std::shared_ptr<monty::ndarray<double, 2> > _6426,
    monty::rc_ptr< ::mosek::fusion::Variable> _6427);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    std::shared_ptr<monty::ndarray<double, 1> > _6428,
    monty::rc_ptr< ::mosek::fusion::Variable> _6429);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable> _6430,
    std::shared_ptr<monty::ndarray<double, 2> > _6431);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable> _6432,
    std::shared_ptr<monty::ndarray<double, 1> > _6433);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable> _6434,
    monty::rc_ptr< ::mosek::fusion::Variable> _6435);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6436,
    monty::rc_ptr< ::mosek::fusion::Expression>    _6437);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression>    _6438,
    monty::rc_ptr< ::mosek::fusion::NDSparseArray> _6439);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6440,
    monty::rc_ptr< ::mosek::fusion::Expression> _6441);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _6442,
    monty::rc_ptr< ::mosek::fusion::Matrix>     _6443);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    double _6444, monty::rc_ptr< ::mosek::fusion::Expression> _6445);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _6446, double _6447);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    std::shared_ptr<monty::ndarray<double, 2> > _6448,
    monty::rc_ptr< ::mosek::fusion::Expression> _6449);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    std::shared_ptr<monty::ndarray<double, 1> > _6450,
    monty::rc_ptr< ::mosek::fusion::Expression> _6451);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _6452,
    std::shared_ptr<monty::ndarray<double, 2> > _6453);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _6454,
    std::shared_ptr<monty::ndarray<double, 1> > _6455);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Variable>   _6456,
    monty::rc_ptr< ::mosek::fusion::Expression> _6457);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _6458,
    monty::rc_ptr< ::mosek::fusion::Variable>   _6459);
  static monty::rc_ptr< ::mosek::fusion::Expression> add(
    monty::rc_ptr< ::mosek::fusion::Expression> _6460,
    monty::rc_ptr< ::mosek::fusion::Expression> _6461);
  virtual monty::rc_ptr< ::mosek::fusion::Set>   shape();
  virtual monty::rc_ptr< ::mosek::fusion::Set>   getShape();
  virtual monty::rc_ptr< ::mosek::fusion::Model> getModel();
  static void                                    validateData(
    std::shared_ptr<monty::ndarray<long long, 1> > _6462,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _6463,
    std::shared_ptr<monty::ndarray<long long, 1> > _6464,
    std::shared_ptr<monty::ndarray<double, 1> >    _6465,
    std::shared_ptr<monty::ndarray<double, 1> >    _6466,
    monty::rc_ptr< ::mosek::fusion::Set> _6467,
    std::shared_ptr<monty::ndarray<long long, 1> > _6468);
  static monty::rc_ptr< ::mosek::fusion::Model> extractModel(
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _6483);
}; // struct Expr;

struct p_FlatExpr
{
  FlatExpr*                         _pubthis;
  static mosek::fusion::p_FlatExpr* _get_impl(mosek::fusion::FlatExpr* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_FlatExpr* _get_impl(mosek::fusion::FlatExpr::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_FlatExpr(FlatExpr* _pubthis);
  virtual ~p_FlatExpr(){ /* std::cout << "~p_FlatExpr" << std::endl;*/ };
  std::shared_ptr<monty::ndarray<long long, 1> > inst{};
  monty::rc_ptr< ::mosek::fusion::Set> shape{};
  long long                            nnz{};
  std::shared_ptr<monty::ndarray<double, 1> > cof{};
  std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
    x{};
  std::shared_ptr<monty::ndarray<long long, 1> > subj{};
  std::shared_ptr<monty::ndarray<long long, 1> > ptrb{};
  std::shared_ptr<monty::ndarray<double, 1> >    bfix{};
  virtual void       destroy();
  static FlatExpr::t _new_FlatExpr(
    monty::rc_ptr< ::mosek::fusion::FlatExpr> _6484);
  void _initialize(monty::rc_ptr< ::mosek::fusion::FlatExpr> _6484);
  static FlatExpr::t _new_FlatExpr(
    std::shared_ptr<monty::ndarray<double, 1> >    _6485,
    std::shared_ptr<monty::ndarray<long long, 1> > _6486,
    std::shared_ptr<monty::ndarray<long long, 1> > _6487,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _6488,
    std::shared_ptr<monty::ndarray<double, 1> > _6489,
    monty::rc_ptr< ::mosek::fusion::Set> _6490,
    std::shared_ptr<monty::ndarray<long long, 1> > _6491);
  void _initialize(
    std::shared_ptr<monty::ndarray<double, 1> >    _6485,
    std::shared_ptr<monty::ndarray<long long, 1> > _6486,
    std::shared_ptr<monty::ndarray<long long, 1> > _6487,
    std::shared_ptr<
      monty::ndarray<monty::rc_ptr< ::mosek::fusion::Variable>, 1> >
      _6488,
    std::shared_ptr<monty::ndarray<double, 1> > _6489,
    monty::rc_ptr< ::mosek::fusion::Set> _6490,
    std::shared_ptr<monty::ndarray<long long, 1> > _6491);
  virtual std::string toString();
  virtual int         size();
}; // struct FlatExpr;

struct p_SymmetricMatrix
{
  SymmetricMatrix*                         _pubthis;
  static mosek::fusion::p_SymmetricMatrix* _get_impl(
    mosek::fusion::SymmetricMatrix* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_SymmetricMatrix* _get_impl(
    mosek::fusion::SymmetricMatrix::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SymmetricMatrix(SymmetricMatrix* _pubthis);
  virtual ~p_SymmetricMatrix(){
    /* std::cout << "~p_SymmetricMatrix" << std::endl;*/
  };
  int    nnz{};
  double scale{};
  std::shared_ptr<monty::ndarray<double, 1> > vval{};
  std::shared_ptr<monty::ndarray<int, 1> >    vsubj{};
  std::shared_ptr<monty::ndarray<int, 1> >    vsubi{};
  std::shared_ptr<monty::ndarray<double, 1> > uval{};
  std::shared_ptr<monty::ndarray<int, 1> >    usubj{};
  std::shared_ptr<monty::ndarray<int, 1> >    usubi{};
  int                       d1{};
  int                       d0{};
  virtual void              destroy();
  static SymmetricMatrix::t _new_SymmetricMatrix(
    int _6493, int _6494, std::shared_ptr<monty::ndarray<int, 1> > _6495,
    std::shared_ptr<monty::ndarray<int, 1> >    _6496,
    std::shared_ptr<monty::ndarray<double, 1> > _6497,
    std::shared_ptr<monty::ndarray<int, 1> >    _6498,
    std::shared_ptr<monty::ndarray<int, 1> >    _6499,
    std::shared_ptr<monty::ndarray<double, 1> > _6500, double _6501);
  void _initialize(int _6493, int                              _6494,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6495,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6496,
                   std::shared_ptr<monty::ndarray<double, 1> > _6497,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6498,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6499,
                   std::shared_ptr<monty::ndarray<double, 1> > _6500,
                   double _6501);
  static monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> rankOne(
    int _6502, std::shared_ptr<monty::ndarray<int, 1> > _6503,
    std::shared_ptr<monty::ndarray<double, 1> > _6504);
  static monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> rankOne(
    std::shared_ptr<monty::ndarray<double, 1> > _6512);
  static monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> antiDiag(
    std::shared_ptr<monty::ndarray<double, 1> > _6520);
  static monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> diag(
    std::shared_ptr<monty::ndarray<double, 1> > _6527);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> add(
    monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> _6533);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> sub(
    monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> _6553);
  virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix> mul(double _6554);
  virtual int getdim();
}; // struct SymmetricMatrix;

struct p_NDSparseArray
{
  NDSparseArray*                         _pubthis;
  static mosek::fusion::p_NDSparseArray* _get_impl(
    mosek::fusion::NDSparseArray* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_NDSparseArray* _get_impl(
    mosek::fusion::NDSparseArray::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_NDSparseArray(NDSparseArray* _pubthis);
  virtual ~p_NDSparseArray(){
    /* std::cout << "~p_NDSparseArray" << std::endl;*/
  };
  std::shared_ptr<monty::ndarray<double, 1> >    cof{};
  std::shared_ptr<monty::ndarray<long long, 1> > inst{};
  std::shared_ptr<monty::ndarray<int, 1> >       dims{};
  long long               size{};
  virtual void            destroy();
  static NDSparseArray::t _new_NDSparseArray(
    std::shared_ptr<monty::ndarray<int, 1> >    _6555,
    std::shared_ptr<monty::ndarray<int, 2> >    _6556,
    std::shared_ptr<monty::ndarray<double, 1> > _6557);
  void _initialize(std::shared_ptr<monty::ndarray<int, 1> >    _6555,
                   std::shared_ptr<monty::ndarray<int, 2> >    _6556,
                   std::shared_ptr<monty::ndarray<double, 1> > _6557);
  static NDSparseArray::t _new_NDSparseArray(
    std::shared_ptr<monty::ndarray<int, 1> >       _6577,
    std::shared_ptr<monty::ndarray<long long, 1> > _6578,
    std::shared_ptr<monty::ndarray<double, 1> >    _6579);
  void _initialize(std::shared_ptr<monty::ndarray<int, 1> >       _6577,
                   std::shared_ptr<monty::ndarray<long long, 1> > _6578,
                   std::shared_ptr<monty::ndarray<double, 1> >    _6579);
  static NDSparseArray::t _new_NDSparseArray(
    monty::rc_ptr< ::mosek::fusion::Matrix> _6593);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Matrix> _6593);
  static monty::rc_ptr< ::mosek::fusion::NDSparseArray> make(
    monty::rc_ptr< ::mosek::fusion::Matrix> _6601);
  static monty::rc_ptr< ::mosek::fusion::NDSparseArray> make(
    std::shared_ptr<monty::ndarray<int, 1> >       _6602,
    std::shared_ptr<monty::ndarray<long long, 1> > _6603,
    std::shared_ptr<monty::ndarray<double, 1> >    _6604);
  static monty::rc_ptr< ::mosek::fusion::NDSparseArray> make(
    std::shared_ptr<monty::ndarray<int, 1> >    _6605,
    std::shared_ptr<monty::ndarray<int, 2> >    _6606,
    std::shared_ptr<monty::ndarray<double, 1> > _6607);
}; // struct NDSparseArray;

struct p_Matrix
{
  Matrix*                         _pubthis;
  static mosek::fusion::p_Matrix* _get_impl(mosek::fusion::Matrix* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Matrix* _get_impl(mosek::fusion::Matrix::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Matrix(Matrix* _pubthis);
  virtual ~p_Matrix(){ /* std::cout << "~p_Matrix" << std::endl;*/ };
  int              dimj{};
  int              dimi{};
  virtual void     destroy();
  static Matrix::t _new_Matrix(int _6676, int _6677);
  void _initialize(int _6676, int _6677);
  virtual std::string                            toString();
  virtual void                                   switchDims();
  static monty::rc_ptr< ::mosek::fusion::Matrix> diag(
    int _6679, monty::rc_ptr< ::mosek::fusion::Matrix> _6680);
  static monty::rc_ptr< ::mosek::fusion::Matrix> diag(
    std::shared_ptr<monty::ndarray<monty::rc_ptr< ::mosek::fusion::Matrix>, 1> >
      _6682);
  static monty::rc_ptr< ::mosek::fusion::Matrix> antidiag(int    _6700,
                                                          double _6701,
                                                          int    _6702);
  static monty::rc_ptr< ::mosek::fusion::Matrix> antidiag(int    _6703,
                                                          double _6704);
  static monty::rc_ptr< ::mosek::fusion::Matrix> diag(int _6705, double _6706,
                                                      int _6707);
  static monty::rc_ptr< ::mosek::fusion::Matrix> diag(int _6708, double _6709);
  static monty::rc_ptr< ::mosek::fusion::Matrix> antidiag(
    std::shared_ptr<monty::ndarray<double, 1> > _6710, int _6711);
  static monty::rc_ptr< ::mosek::fusion::Matrix> antidiag(
    std::shared_ptr<monty::ndarray<double, 1> > _6721);
  static monty::rc_ptr< ::mosek::fusion::Matrix> diag(
    std::shared_ptr<monty::ndarray<double, 1> > _6722, int _6723);
  static monty::rc_ptr< ::mosek::fusion::Matrix> diag(
    std::shared_ptr<monty::ndarray<double, 1> > _6731);
  static monty::rc_ptr< ::mosek::fusion::Matrix> ones(int _6732, int _6733);
  static monty::rc_ptr< ::mosek::fusion::Matrix> eye(int _6734);
  static monty::rc_ptr< ::mosek::fusion::Matrix> dense(
    monty::rc_ptr< ::mosek::fusion::Matrix> _6736);
  static monty::rc_ptr< ::mosek::fusion::Matrix> dense(int _6737, int _6738,
                                                       double _6739);
  static monty::rc_ptr< ::mosek::fusion::Matrix> dense(
    int _6740, int _6741, std::shared_ptr<monty::ndarray<double, 1> > _6742);
  static monty::rc_ptr< ::mosek::fusion::Matrix> dense(
    std::shared_ptr<monty::ndarray<double, 2> > _6743);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    monty::rc_ptr< ::mosek::fusion::Matrix> _6744);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    std::shared_ptr<
      monty::ndarray<std::shared_ptr<monty::ndarray<
                       monty::rc_ptr< ::mosek::fusion::Matrix>, 1> >,
                     1> >
      _6748);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    std::shared_ptr<monty::ndarray<double, 2> > _6779);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(int _6792, int _6793);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    int _6794, int _6795, std::shared_ptr<monty::ndarray<int, 1> > _6796,
    std::shared_ptr<monty::ndarray<int, 1> > _6797, double _6798);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    std::shared_ptr<monty::ndarray<int, 1> > _6800,
    std::shared_ptr<monty::ndarray<int, 1> > _6801, double _6802);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    std::shared_ptr<monty::ndarray<int, 1> >    _6807,
    std::shared_ptr<monty::ndarray<int, 1> >    _6808,
    std::shared_ptr<monty::ndarray<double, 1> > _6809);
  static monty::rc_ptr< ::mosek::fusion::Matrix> sparse(
    int _6814, int _6815, std::shared_ptr<monty::ndarray<int, 1> > _6816,
    std::shared_ptr<monty::ndarray<int, 1> >    _6817,
    std::shared_ptr<monty::ndarray<double, 1> > _6818);
  virtual double get(int _6823, int _6824)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual bool isSparse()
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual std::shared_ptr<monty::ndarray<double, 1> > getDataAsArray()
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual void getDataAsTriplets(
    std::shared_ptr<monty::ndarray<int, 1> >    _6825,
    std::shared_ptr<monty::ndarray<int, 1> >    _6826,
    std::shared_ptr<monty::ndarray<double, 1> > _6827)
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual monty::rc_ptr< ::mosek::fusion::Matrix> transpose()
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual long long numNonzeros()
  {
    throw monty::AbstractClassError("Call to abstract method");
  }
  virtual int numColumns();
  virtual int numRows();
}; // struct Matrix;

struct p_DenseMatrix : public ::mosek::fusion::p_Matrix
{
  DenseMatrix*                         _pubthis;
  static mosek::fusion::p_DenseMatrix* _get_impl(
    mosek::fusion::DenseMatrix* _inst)
  {
    return static_cast<mosek::fusion::p_DenseMatrix*>(
      mosek::fusion::p_Matrix::_get_impl(_inst));
  }
  static mosek::fusion::p_DenseMatrix* _get_impl(
    mosek::fusion::DenseMatrix::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_DenseMatrix(DenseMatrix* _pubthis);
  virtual ~p_DenseMatrix(){ /* std::cout << "~p_DenseMatrix" << std::endl;*/ };
  long long nnz{};
  std::shared_ptr<monty::ndarray<double, 1> > data{};
  virtual void          destroy();
  static DenseMatrix::t _new_DenseMatrix(
    int _6608, int _6609, std::shared_ptr<monty::ndarray<double, 1> > _6610);
  void _initialize(int _6608, int                              _6609,
                   std::shared_ptr<monty::ndarray<double, 1> > _6610);
  static DenseMatrix::t _new_DenseMatrix(
    monty::rc_ptr< ::mosek::fusion::Matrix> _6611);
  void _initialize(monty::rc_ptr< ::mosek::fusion::Matrix> _6611);
  static DenseMatrix::t _new_DenseMatrix(
    std::shared_ptr<monty::ndarray<double, 2> > _6616);
  void _initialize(std::shared_ptr<monty::ndarray<double, 2> > _6616);
  static DenseMatrix::t _new_DenseMatrix(int _6619, int _6620, double _6621);
  void _initialize(int _6619, int _6620, double _6621);
  virtual std::string                             toString();
  virtual monty::rc_ptr< ::mosek::fusion::Matrix> transpose();
  virtual bool                                    isSparse();
  virtual std::shared_ptr<monty::ndarray<double, 1> > getDataAsArray();
  virtual void getDataAsTriplets(
    std::shared_ptr<monty::ndarray<int, 1> >    _6634,
    std::shared_ptr<monty::ndarray<int, 1> >    _6635,
    std::shared_ptr<monty::ndarray<double, 1> > _6636);
  virtual double get(int _6640, int _6641);
  virtual long long numNonzeros();
}; // struct DenseMatrix;

struct p_SparseMatrix : public ::mosek::fusion::p_Matrix
{
  SparseMatrix*                         _pubthis;
  static mosek::fusion::p_SparseMatrix* _get_impl(
    mosek::fusion::SparseMatrix* _inst)
  {
    return static_cast<mosek::fusion::p_SparseMatrix*>(
      mosek::fusion::p_Matrix::_get_impl(_inst));
  }
  static mosek::fusion::p_SparseMatrix* _get_impl(
    mosek::fusion::SparseMatrix::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_SparseMatrix(SparseMatrix* _pubthis);
  virtual ~p_SparseMatrix(){
    /* std::cout << "~p_SparseMatrix" << std::endl;*/
  };
  long long nnz{};
  std::shared_ptr<monty::ndarray<double, 1> > val{};
  std::shared_ptr<monty::ndarray<int, 1> >    subj{};
  std::shared_ptr<monty::ndarray<int, 1> >    subi{};
  virtual void           destroy();
  static SparseMatrix::t _new_SparseMatrix(
    int _6642, int _6643, std::shared_ptr<monty::ndarray<int, 1> > _6644,
    std::shared_ptr<monty::ndarray<int, 1> >    _6645,
    std::shared_ptr<monty::ndarray<double, 1> > _6646, long long _6647);
  void _initialize(int _6642, int                              _6643,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6644,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6645,
                   std::shared_ptr<monty::ndarray<double, 1> > _6646,
                   long long _6647);
  static SparseMatrix::t _new_SparseMatrix(
    int _6652, int _6653, std::shared_ptr<monty::ndarray<int, 1> > _6654,
    std::shared_ptr<monty::ndarray<int, 1> >    _6655,
    std::shared_ptr<monty::ndarray<double, 1> > _6656);
  void _initialize(int _6652, int                              _6653,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6654,
                   std::shared_ptr<monty::ndarray<int, 1> >    _6655,
                   std::shared_ptr<monty::ndarray<double, 1> > _6656);
  virtual std::shared_ptr<monty::ndarray<long long, 1> > formPtrb();
  virtual std::string                             toString();
  virtual long long                               numNonzeros();
  virtual monty::rc_ptr< ::mosek::fusion::Matrix> transpose();
  virtual bool                                    isSparse();
  virtual std::shared_ptr<monty::ndarray<double, 1> > getDataAsArray();
  virtual void getDataAsTriplets(
    std::shared_ptr<monty::ndarray<int, 1> >    _6668,
    std::shared_ptr<monty::ndarray<int, 1> >    _6669,
    std::shared_ptr<monty::ndarray<double, 1> > _6670);
  virtual double get(int _6671, int _6672);
}; // struct SparseMatrix;

struct p_Parameters
{
  Parameters*                         _pubthis;
  static mosek::fusion::p_Parameters* _get_impl(
    mosek::fusion::Parameters* _inst)
  {
    assert(_inst);
    assert(_inst->_impl);
    return _inst->_impl;
  }
  static mosek::fusion::p_Parameters* _get_impl(
    mosek::fusion::Parameters::t _inst)
  {
    return _get_impl(_inst.get());
  }
  p_Parameters(Parameters* _pubthis);
  virtual ~p_Parameters(){ /* std::cout << "~p_Parameters" << std::endl;*/ };
  virtual void destroy();
  static void setParameter(monty::rc_ptr< ::mosek::fusion::Model> _6851,
                           const std::string& _6852, double _6853);
  static void setParameter(monty::rc_ptr< ::mosek::fusion::Model> _6952,
                           const std::string& _6953, int _6954);
  static void setParameter(monty::rc_ptr< ::mosek::fusion::Model> _7053,
                           const std::string& _7054, const std::string& _7055);
  static int string_to_miocontsoltype_value(const std::string& _7301);
  static int string_to_internal_dinf_value(const std::string& _7302);
  static int string_to_presolvemode_value(const std::string& _7303);
  static int string_to_optimizertype_value(const std::string& _7304);
  static int string_to_stakey_value(const std::string& _7305);
  static int string_to_iinfitem_value(const std::string& _7306);
  static int string_to_simreform_value(const std::string& _7307);
  static int string_to_value_value(const std::string& _7308);
  static int string_to_scalingmethod_value(const std::string& _7309);
  static int string_to_soltype_value(const std::string& _7310);
  static int string_to_startpointtype_value(const std::string& _7311);
  static int string_to_language_value(const std::string& _7312);
  static int string_to_checkconvexitytype_value(const std::string& _7313);
  static int string_to_variabletype_value(const std::string& _7314);
  static int string_to_mpsformat_value(const std::string& _7315);
  static int string_to_nametype_value(const std::string& _7316);
  static int string_to_compresstype_value(const std::string& _7317);
  static int string_to_simdupvec_value(const std::string& _7318);
  static int string_to_dparam_value(const std::string& _7319);
  static int string_to_inftype_value(const std::string& _7320);
  static int string_to_problemtype_value(const std::string& _7321);
  static int string_to_orderingtype_value(const std::string& _7322);
  static int string_to_dataformat_value(const std::string& _7323);
  static int string_to_simdegen_value(const std::string& _7324);
  static int string_to_onoffkey_value(const std::string& _7325);
  static int string_to_transpose_value(const std::string& _7326);
  static int string_to_mionodeseltype_value(const std::string& _7327);
  static int string_to_rescode_value(const std::string& _7328);
  static int string_to_scalingtype_value(const std::string& _7329);
  static int string_to_prosta_value(const std::string& _7330);
  static int string_to_rescodetype_value(const std::string& _7331);
  static int string_to_parametertype_value(const std::string& _7332);
  static int string_to_dinfitem_value(const std::string& _7333);
  static int string_to_miomode_value(const std::string& _7334);
  static int string_to_xmlwriteroutputtype_value(const std::string& _7335);
  static int string_to_simseltype_value(const std::string& _7336);
  static int string_to_internal_liinf_value(const std::string& _7337);
  static int string_to_iomode_value(const std::string& _7338);
  static int string_to_streamtype_value(const std::string& _7339);
  static int string_to_conetype_value(const std::string& _7340);
  static int string_to_mark_value(const std::string& _7341);
  static int string_to_feature_value(const std::string& _7342);
  static int string_to_symmattype_value(const std::string& _7343);
  static int string_to_callbackcode_value(const std::string& _7344);
  static int string_to_simhotstart_value(const std::string& _7345);
  static int string_to_liinfitem_value(const std::string& _7346);
  static int string_to_branchdir_value(const std::string& _7347);
  static int string_to_basindtype_value(const std::string& _7348);
  static int string_to_internal_iinf_value(const std::string& _7349);
  static int string_to_boundkey_value(const std::string& _7350);
  static int string_to_solitem_value(const std::string& _7351);
  static int string_to_objsense_value(const std::string& _7352);
  static int string_to_solsta_value(const std::string& _7353);
  static int string_to_iparam_value(const std::string& _7354);
  static int string_to_sparam_value(const std::string& _7355);
  static int string_to_intpnthotstart_value(const std::string& _7356);
  static int string_to_uplo_value(const std::string& _7357);
  static int string_to_sensitivitytype_value(const std::string& _7358);
  static int string_to_accmode_value(const std::string& _7359);
  static int string_to_problemitem_value(const std::string& _7360);
  static int string_to_solveform_value(const std::string& _7361);
}; // struct Parameters;
}
}
namespace mosek
{
namespace fusion
{
namespace Utils
{
// mosek.fusion.Utils.IntMap from file 'src/fusion/cxx/IntMap_p.h'
struct p_IntMap
{
  IntMap* _pubself;

  static p_IntMap* _get_impl(IntMap* _inst)
  {
    return _inst->_impl.get();
  }

  p_IntMap(IntMap* _pubself)
    : _pubself(_pubself)
  {
  }

  static IntMap::t _new_IntMap()
  {
    return new IntMap();
  }

  ::std::unordered_map<long long, int> m;

  bool hasItem(long long key)
  {
    return m.find(key) != m.end();
  }
  int getItem(long long key)
  {
    return m.find(key)->second;
  } // will probably throw something or crash of no such key
  void setItem(long long key, int val)
  {
    m[key] = val;
  }

  std::shared_ptr<monty::ndarray<long long, 1> > keys()
  {
    size_t size = m.size();
    auto   res  = std::shared_ptr<monty::ndarray<long long, 1> >(
      new monty::ndarray<long long, 1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it  = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->first;

    return res;
  }

  std::shared_ptr<monty::ndarray<int, 1> > values()
  {
    size_t size = m.size();
    auto   res  = std::shared_ptr<monty::ndarray<int, 1> >(
      new monty::ndarray<int, 1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it  = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->second;

    return res;
  }

  IntMap::t clone();
};

struct p_StringIntMap
{
  StringIntMap* _pubself;

  static p_StringIntMap* _get_impl(StringIntMap* _inst)
  {
    return _inst->_impl.get();
  }

  p_StringIntMap(StringIntMap* _pubself)
    : _pubself(_pubself)
  {
  }

  static StringIntMap::t _new_StringIntMap()
  {
    return new StringIntMap();
  }

  ::std::unordered_map<std::string, int> m;

  bool hasItem(const std::string& key)
  {
    return m.find(key) != m.end();
  }
  int getItem(const std::string& key)
  {
    return m.find(key)->second;
  } // will probably throw something or crash of no such key
  void setItem(const std::string& key, int val)
  {
    m[key] = val;
  }

  std::shared_ptr<monty::ndarray<std::string, 1> > keys()
  {
    size_t size = m.size();
    auto   res  = std::shared_ptr<monty::ndarray<std::string, 1> >(
      new monty::ndarray<std::string, 1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it  = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->first;

    return res;
  }

  std::shared_ptr<monty::ndarray<int, 1> > values()
  {
    size_t size = m.size();
    auto   res  = std::shared_ptr<monty::ndarray<int, 1> >(
      new monty::ndarray<int, 1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it  = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->second;

    return res;
  }

  StringIntMap::t clone();
};
// End of file 'src/fusion/cxx/IntMap_p.h'
// mosek.fusion.Utils.StringBuffer from file 'src/fusion/cxx/StringBuffer_p.h'
// namespace mosek::fusion::Utils
struct p_StringBuffer
{
  StringBuffer*     _pubthis;
  std::stringstream ss;

  p_StringBuffer(StringBuffer* _pubthis)
    : _pubthis(_pubthis)
  {
  }

  static p_StringBuffer* _get_impl(StringBuffer::t ptr)
  {
    return ptr->_impl.get();
  }
  static p_StringBuffer* _get_impl(StringBuffer* ptr)
  {
    return ptr->_impl.get();
  }

  static StringBuffer::t _new_StringBuffer()
  {
    return new StringBuffer();
  }

  StringBuffer::t clear();

  template <typename T>
  StringBuffer::t a(const monty::ndarray<T, 1>& val);

  template <typename T>
  StringBuffer::t a(const T& val);

  StringBuffer::t lf();
  std::string     toString() const;
};

template <typename T>
StringBuffer::t
p_StringBuffer::a(const monty::ndarray<T, 1>& val)
{
  if (val.size() > 0)
  {
    ss << val[0];
    for (int i = 1; i < val.size(); ++i)
      ss << "," << val[i];
  }
  return StringBuffer::t(_pubthis);
}

template <typename T>
StringBuffer::t
p_StringBuffer::a(const T& val)
{
  ss << val;
  return _pubthis;
}

// End of file 'src/fusion/cxx/StringBuffer_p.h'
}
}
}
#endif
