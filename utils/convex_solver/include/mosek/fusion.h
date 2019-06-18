#ifndef __FUSION_H__
#define __FUSION_H__
#include "monty.h"
#include "stdlib.h"
#include "cmath"
#include "mosektask.h"
#include "mosek.h"
#include "fusion_fwd.h"
namespace mosek
{
namespace fusion
{
enum class RelationKey
{
EqualsTo,
LessThan,
GreaterThan,
IsFree,
InRange
};
std::ostream & operator<<(std::ostream & os, RelationKey val);
enum class PSDKey
{
IsSymPSD,
IsTrilPSD
};
std::ostream & operator<<(std::ostream & os, PSDKey val);
enum class QConeKey
{
InQCone,
InRotatedQCone
};
std::ostream & operator<<(std::ostream & os, QConeKey val);
enum class ObjectiveSense
{
Undefined,
Minimize,
Maximize
};
std::ostream & operator<<(std::ostream & os, ObjectiveSense val);
enum class SolutionStatus
{
Undefined,
Unknown,
Optimal,
NearOptimal,
Feasible,
NearFeasible,
Certificate,
NearCertificate,
IllposedCert
};
std::ostream & operator<<(std::ostream & os, SolutionStatus val);
enum class AccSolutionStatus
{
Anything,
Optimal,
NearOptimal,
Feasible,
Certificate
};
std::ostream & operator<<(std::ostream & os, AccSolutionStatus val);
enum class ProblemStatus
{
Unknown,
PrimalAndDualFeasible,
PrimalFeasible,
DualFeasible,
PrimalInfeasible,
DualInfeasible,
PrimalAndDualInfeasible,
IllPosed,
PrimalInfeasibleOrUnbounded
};
std::ostream & operator<<(std::ostream & os, ProblemStatus val);
enum class SolutionType
{
Default,
Basic,
Interior,
Integer
};
std::ostream & operator<<(std::ostream & os, SolutionType val);
enum class StatusKey
{
Unknown,
Basic,
SuperBasic,
OnBound,
Infinity
};
std::ostream & operator<<(std::ostream & os, StatusKey val);
}
}
namespace mosek
{
namespace fusion
{
class /*interface*/ Variable : public virtual monty::RefCounted
{
public:
virtual void destroy() = 0;
virtual ~Variable() {};
typedef monty::rc_ptr< Variable > t;
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) = 0;
virtual void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) = 0;
virtual std::string toString() = 0;
virtual void inst(std::shared_ptr< monty::ndarray< long long,1 > > indexes,int first_idx,int last_idx,long long index_offset,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) = 0;
virtual void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > shape() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > getShape() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Model > getModel() = 0;
virtual long long size() = 0;
virtual void setLevel(std::shared_ptr< monty::ndarray< double,1 > > v) = 0;
virtual void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) = 0;
virtual void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) = 0;
virtual std::shared_ptr< monty::ndarray< double,1 > > dual() = 0;
virtual std::shared_ptr< monty::ndarray< double,1 > > level() = 0;
virtual void values(int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) = 0;
virtual void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) = 0;
virtual void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) = 0;
virtual void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) = 0;
virtual void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) = 0;
virtual void makeContinuous() = 0;
virtual void makeInteger() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > transpose() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2,std::shared_ptr< monty::ndarray< int,1 > > i3) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,2 > > midxs) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > idxs) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > antidiag() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > antidiag(int index) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > diag() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > diag(int index) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(std::shared_ptr< monty::ndarray< int,1 > > idx) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(int i1,int i2,int i3) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(int i1,int i2) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(int i1) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > asExpr() = 0;
}; // interface Variable;

class /*interface*/ SymmetricVariable : public virtual ::mosek::fusion::Variable
{
public:
virtual void destroy() = 0;
virtual ~SymmetricVariable() {};
typedef monty::rc_ptr< SymmetricVariable > t;
}; // interface SymmetricVariable;

class /*interface*/ Expression : public virtual monty::RefCounted
{
public:
virtual void destroy() = 0;
virtual ~Expression() {};
typedef monty::rc_ptr< Expression > t;
virtual std::string toString() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > transpose() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > pick(std::shared_ptr< monty::ndarray< int,2 > > indexrows) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > pick(std::shared_ptr< monty::ndarray< int,1 > > indexes) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > index(std::shared_ptr< monty::ndarray< int,1 > > indexes) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > index(int i) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Expression > slice(int first,int last) = 0;
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > eval() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Model > getModel() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > shape() = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > getShape() = 0;
}; // interface Expression;

class FusionException : public ::monty::Exception
{
private:
std::string msg;
protected:
public:
typedef monty::rc_ptr< FusionException > t;

FusionException(const std::string &  msg_);
std::string toString() /* override */;
}; // class FusionException;

class SolutionError : public ::mosek::fusion::FusionException
{
private:
protected:
public:
typedef monty::rc_ptr< SolutionError > t;

SolutionError();
SolutionError(const std::string &  msg);
}; // class SolutionError;

class UnimplementedError : public ::monty::RuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< UnimplementedError > t;

UnimplementedError(const std::string &  msg);
}; // class UnimplementedError;

class FatalError : public ::monty::RuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< FatalError > t;

FatalError(const std::string &  msg);
}; // class FatalError;

class UnexpectedError : public ::monty::RuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< UnexpectedError > t;

UnexpectedError(::mosek::fusion::FusionException e);
UnexpectedError(const std::string &  msg);
}; // class UnexpectedError;

class FusionRuntimeException : public ::monty::RuntimeException
{
private:
std::string msg;
protected:
public:
typedef monty::rc_ptr< FusionRuntimeException > t;

FusionRuntimeException(const std::string &  msg_);
std::string toString() /* override */;
}; // class FusionRuntimeException;

class SparseFormatError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< SparseFormatError > t;

SparseFormatError(const std::string &  msg);
}; // class SparseFormatError;

class SliceError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< SliceError > t;

SliceError();
SliceError(const std::string &  msg);
}; // class SliceError;

class SetDefinitionError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< SetDefinitionError > t;

SetDefinitionError(const std::string &  msg);
}; // class SetDefinitionError;

class OptimizeError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< OptimizeError > t;

OptimizeError(const std::string &  msg);
}; // class OptimizeError;

class NameError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< NameError > t;

NameError(const std::string &  msg);
}; // class NameError;

class ModelError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< ModelError > t;

ModelError(const std::string &  msg);
}; // class ModelError;

class MatrixError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< MatrixError > t;

MatrixError(const std::string &  msg);
}; // class MatrixError;

class DimensionError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< DimensionError > t;

DimensionError(const std::string &  msg);
}; // class DimensionError;

class LengthError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< LengthError > t;

LengthError(const std::string &  msg);
}; // class LengthError;

class RangeError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< RangeError > t;

RangeError(const std::string &  msg);
}; // class RangeError;

class IndexError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< IndexError > t;

IndexError(const std::string &  msg);
}; // class IndexError;

class DomainError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< DomainError > t;

DomainError(const std::string &  msg);
}; // class DomainError;

class ValueConversionError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< ValueConversionError > t;

ValueConversionError(const std::string &  msg);
}; // class ValueConversionError;

class ParameterError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< ParameterError > t;

ParameterError(const std::string &  msg);
}; // class ParameterError;

class ExpressionError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< ExpressionError > t;

ExpressionError(const std::string &  msg);
}; // class ExpressionError;

class IOError : public ::mosek::fusion::FusionRuntimeException
{
private:
protected:
public:
typedef monty::rc_ptr< IOError > t;

IOError(const std::string &  msg);
}; // class IOError;

// mosek.fusion.BaseModel from file 'src/fusion/cxx/BaseModel.h'

class BaseModel : public monty::RefCounted
{
private:
  p_BaseModel * _impl;
protected:
  BaseModel(p_BaseModel * ptr);
public:
  friend class p_BaseModel;

  virtual void dispose();

  ~BaseModel();
};
// End of file 'src/fusion/cxx/BaseModel.h'
class Model : public ::mosek::fusion::BaseModel
{
Model(monty::rc_ptr< ::mosek::fusion::Model > m);
protected: Model(p_Model * _impl);
public:
Model(const Model &) = delete;
const Model & operator=(const Model &) = delete;
friend class p_Model;
virtual ~Model();
virtual void destroy();
typedef monty::rc_ptr< Model > t;

Model();
Model(const std::string &  name);
static  void putlicensewait(bool wait);
static  void putlicensepath(const std::string &  licfile);
static  void putlicensecode(std::shared_ptr< monty::ndarray< int,1 > > code);
void dispose() /* override */;
virtual MSKtask_t getTask();
virtual void flushNames();
virtual void writeTask(const std::string &  filename);
virtual long long getSolverLIntInfo(const std::string &  name);
virtual int getSolverIntInfo(const std::string &  name);
virtual double getSolverDoubleInfo(const std::string &  name);
virtual void setCallbackHandler(mosek::cbhandler_t  h);
virtual void setDataCallbackHandler(mosek::datacbhandler_t  h);
virtual void setLogHandler(mosek::msghandler_t  h);
virtual void setSolverParam(const std::string &  name,double floatval);
virtual void setSolverParam(const std::string &  name,int intval);
virtual void setSolverParam(const std::string &  name,const std::string &  strval);
virtual void breakSolver();
virtual void solve();
virtual void flushSolutions();
virtual mosek::fusion::SolutionStatus getDualSolutionStatus();
virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus();
virtual double dualObjValue();
virtual double primalObjValue();
virtual void selectedSolution(mosek::fusion::SolutionType soltype);
virtual mosek::fusion::AccSolutionStatus getAcceptedSolutionStatus();
virtual void acceptedSolutionStatus(mosek::fusion::AccSolutionStatus what);
virtual mosek::fusion::ProblemStatus getProblemStatus(mosek::fusion::SolutionType which);
virtual mosek::fusion::SolutionStatus getDualSolutionStatus(mosek::fusion::SolutionType which);
virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus(mosek::fusion::SolutionType which);
virtual void objective(double c);
virtual void objective(mosek::fusion::ObjectiveSense sense,double c);
virtual void objective(mosek::fusion::ObjectiveSense sense,monty::rc_ptr< ::mosek::fusion::Variable > v);
virtual void objective(mosek::fusion::ObjectiveSense sense,monty::rc_ptr< ::mosek::fusion::Expression > expr);
virtual void objective(const std::string &  name,double c);
virtual void objective(const std::string &  name,mosek::fusion::ObjectiveSense sense,double c);
virtual void objective(const std::string &  name,mosek::fusion::ObjectiveSense sense,monty::rc_ptr< ::mosek::fusion::Variable > v);
virtual void objective(const std::string &  name,mosek::fusion::ObjectiveSense sense,monty::rc_ptr< ::mosek::fusion::Expression > expr);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shape,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > constraint(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int n,int m,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int n,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int n,int m,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int n,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > lpsddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int n,int m,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int n,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int n,int m,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int n,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::PSDDomain > psddom);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable > variable(int size,monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > symdom);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable > variable(const std::string &  name,int size,monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > symdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(std::shared_ptr< monty::ndarray< int,1 > > sizea,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(std::shared_ptr< monty::ndarray< int,1 > > sizea,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int size,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int size,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int size,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(std::shared_ptr< monty::ndarray< int,1 > > sizea);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(int size);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable();
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,std::shared_ptr< monty::ndarray< int,1 > > sizea,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,std::shared_ptr< monty::ndarray< int,1 > > sizea,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,monty::rc_ptr< ::mosek::fusion::Set > shp,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int size,monty::rc_ptr< ::mosek::fusion::QConeDomain > qdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int size,monty::rc_ptr< ::mosek::fusion::RangeDomain > rdom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int size,monty::rc_ptr< ::mosek::fusion::LinearDomain > ldom);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,std::shared_ptr< monty::ndarray< int,1 > > sizea);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name,int size);
virtual monty::rc_ptr< ::mosek::fusion::Variable > variable(const std::string &  name);
virtual long long numConstraints();
virtual long long numVariables();
virtual bool hasConstraint(const std::string &  name);
virtual bool hasVariable(const std::string &  name);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > getConstraint(int index);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > getConstraint(const std::string &  name);
virtual monty::rc_ptr< ::mosek::fusion::Variable > getVariable(int index);
virtual monty::rc_ptr< ::mosek::fusion::Variable > getVariable(const std::string &  name);
virtual std::string getName();
virtual monty::rc_ptr< ::mosek::fusion::Model > clone();
}; // class Model;

// mosek.fusion.Debug from file 'src/fusion/cxx/Debug.h'
class Debug : public monty::RefCounted
{    
  std::unique_ptr<p_Debug> ptr;
public:
  friend struct p_Debug;
  typedef monty::rc_ptr<Debug> t;

  Debug();

  static t o();
  t p(const std::string & val);
  t p(      int val);
  t p(long long val);
  t p(double    val);
  t p(  bool    val);
  t lf();

  t p(const std::shared_ptr<monty::ndarray<double,1>>    & val);
  t p(const std::shared_ptr<monty::ndarray<int,1>>       & val);
  t p(const std::shared_ptr<monty::ndarray<long long,1>> & val);

};
// End of file 'src/fusion/cxx/Debug.h'
class Sort : public virtual monty::RefCounted
{
public: p_Sort * _impl;
protected: Sort(p_Sort * _impl);
public:
Sort(const Sort &) = delete;
const Sort & operator=(const Sort &) = delete;
friend class p_Sort;
virtual ~Sort();
virtual void destroy();
typedef monty::rc_ptr< Sort > t;

static  void argTransposeSort(std::shared_ptr< monty::ndarray< long long,1 > > perm,std::shared_ptr< monty::ndarray< long long,1 > > ptrb,int m,int n,int p,std::shared_ptr< monty::ndarray< long long,1 > > val);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,std::shared_ptr< monty::ndarray< long long,1 > > vals2);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,std::shared_ptr< monty::ndarray< int,1 > > vals2);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,long long first,long long last);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,long long first,long long last);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,std::shared_ptr< monty::ndarray< long long,1 > > vals2,long long first,long long last);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,std::shared_ptr< monty::ndarray< int,1 > > vals2,long long first,long long last);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,long long first,long long last,bool check);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,long long first,long long last,bool check);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,std::shared_ptr< monty::ndarray< long long,1 > > vals2,long long first,long long last,bool check);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,std::shared_ptr< monty::ndarray< int,1 > > vals2,long long first,long long last,bool check);
static  void argbucketsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals,long long first,long long last,long long minv,long long maxv);
static  void argbucketsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals,long long first,long long last,int minv,int maxv);
static  void getminmax(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,std::shared_ptr< monty::ndarray< long long,1 > > vals2,long long first,long long last,std::shared_ptr< monty::ndarray< long long,1 > > res);
static  void getminmax(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,std::shared_ptr< monty::ndarray< int,1 > > vals2,long long first,long long last,std::shared_ptr< monty::ndarray< int,1 > > res);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,long long first,long long last,bool check);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,long long first,long long last,bool check);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,std::shared_ptr< monty::ndarray< long long,1 > > vals2,long long first,long long last,bool check);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,std::shared_ptr< monty::ndarray< int,1 > > vals2,long long first,long long last,bool check);
}; // class Sort;

class IndexCounter : public virtual monty::RefCounted
{
public: p_IndexCounter * _impl;
protected: IndexCounter(p_IndexCounter * _impl);
public:
IndexCounter(const IndexCounter &) = delete;
const IndexCounter & operator=(const IndexCounter &) = delete;
friend class p_IndexCounter;
virtual ~IndexCounter();
virtual void destroy();
typedef monty::rc_ptr< IndexCounter > t;

IndexCounter(monty::rc_ptr< ::mosek::fusion::Set > shape);
IndexCounter(long long start_,std::shared_ptr< monty::ndarray< int,1 > > dims_,monty::rc_ptr< ::mosek::fusion::Set > shape);
IndexCounter(long long start_,std::shared_ptr< monty::ndarray< int,1 > > dims_,std::shared_ptr< monty::ndarray< long long,1 > > strides_);
virtual bool atEnd();
virtual std::shared_ptr< monty::ndarray< int,1 > > getIndex();
virtual long long next();
virtual long long get();
virtual void inc();
virtual void reset();
}; // class IndexCounter;

class CommonTools : public virtual monty::RefCounted
{
public: p_CommonTools * _impl;
protected: CommonTools(p_CommonTools * _impl);
public:
CommonTools(const CommonTools &) = delete;
const CommonTools & operator=(const CommonTools &) = delete;
friend class p_CommonTools;
virtual ~CommonTools();
virtual void destroy();
typedef monty::rc_ptr< CommonTools > t;

static  void ndIncr(std::shared_ptr< monty::ndarray< int,1 > > ndidx,std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last);
static  void transposeTriplets(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > val,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< long long,1 > >,1 > > tsubi_,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< long long,1 > >,1 > > tsubj_,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > tval_,long long nelm,int dimi,int dimj);
static  void transposeTriplets(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > val,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > tsubi_,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > tsubj_,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > tval_,long long nelm,int dimi,int dimj);
static  void tripletSort(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > val,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > tsubi_,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > tsubj_,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > tval_,long long nelm,int dimi,int dimj);
static  void argMSort(std::shared_ptr< monty::ndarray< int,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals);
static  void argQsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< long long,1 > > vals1,std::shared_ptr< monty::ndarray< long long,1 > > vals2,long long first,long long last);
static  void argQsort(std::shared_ptr< monty::ndarray< long long,1 > > idx,std::shared_ptr< monty::ndarray< int,1 > > vals1,std::shared_ptr< monty::ndarray< int,1 > > vals2,long long first,long long last);
}; // class CommonTools;

class SolutionStruct : public virtual monty::RefCounted
{
public: p_SolutionStruct * _impl;
protected: SolutionStruct(p_SolutionStruct * _impl);
public:
SolutionStruct(const SolutionStruct &) = delete;
const SolutionStruct & operator=(const SolutionStruct &) = delete;
friend class p_SolutionStruct;
virtual ~SolutionStruct();
virtual void destroy();
typedef monty::rc_ptr< SolutionStruct > t;
std::shared_ptr< monty::ndarray< double,1 > > get_snx();
void set_snx(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_sux();
void set_sux(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_slx();
void set_slx(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > get_bars();
void set_bars(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > val);
std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > get_barx();
void set_barx(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_y();
void set_y(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_suc();
void set_suc(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_slc();
void set_slc(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_xx();
void set_xx(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_xc();
void set_xc(std::shared_ptr< monty::ndarray< double,1 > > val);
double get_dobj();
void set_dobj(double val);
double get_pobj();
void set_pobj(double val);
mosek::fusion::ProblemStatus get_probstatus();
void set_probstatus(mosek::fusion::ProblemStatus val);
mosek::fusion::SolutionStatus get_dstatus();
void set_dstatus(mosek::fusion::SolutionStatus val);
mosek::fusion::SolutionStatus get_pstatus();
void set_pstatus(mosek::fusion::SolutionStatus val);
int get_sol_numbarvar();
void set_sol_numbarvar(int val);
int get_sol_numcone();
void set_sol_numcone(int val);
int get_sol_numvar();
void set_sol_numvar(int val);
int get_sol_numcon();
void set_sol_numcon(int val);

SolutionStruct(int numvar,int numcon,int numcone,int numbarvar);
SolutionStruct(monty::rc_ptr< ::mosek::fusion::SolutionStruct > that);
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > clone();
virtual void resize(int numvar,int numcon,int numcone,int numbarvar);
virtual bool isDualAcceptable(mosek::fusion::AccSolutionStatus acceptable_sol);
virtual bool isPrimalAcceptable(mosek::fusion::AccSolutionStatus acceptable_sol);
}; // class SolutionStruct;

class ConNZStruct : public virtual monty::RefCounted
{
public: p_ConNZStruct * _impl;
protected: ConNZStruct(p_ConNZStruct * _impl);
public:
ConNZStruct(const ConNZStruct &) = delete;
const ConNZStruct & operator=(const ConNZStruct &) = delete;
friend class p_ConNZStruct;
virtual ~ConNZStruct();
virtual void destroy();
typedef monty::rc_ptr< ConNZStruct > t;
std::shared_ptr< monty::ndarray< int,1 > > get_barmidx();
void set_barmidx(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_barsubj();
void set_barsubj(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_barsubi();
void set_barsubi(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_bfix();
void set_bfix(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_cof();
void set_cof(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_subj();
void set_subj(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< long long,1 > > get_ptrb();
void set_ptrb(std::shared_ptr< monty::ndarray< long long,1 > > val);

ConNZStruct(std::shared_ptr< monty::ndarray< long long,1 > > ptrb_,std::shared_ptr< monty::ndarray< int,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > cof_,std::shared_ptr< monty::ndarray< double,1 > > bfix_,std::shared_ptr< monty::ndarray< int,1 > > barsubi_,std::shared_ptr< monty::ndarray< int,1 > > barsubj_,std::shared_ptr< monty::ndarray< int,1 > > barmidx_);
}; // class ConNZStruct;

class BaseVariable : public virtual ::mosek::fusion::Variable
{
public: p_BaseVariable * _impl;
protected: BaseVariable(p_BaseVariable * _impl);
public:
BaseVariable(const BaseVariable &) = delete;
const BaseVariable & operator=(const BaseVariable &) = delete;
friend class p_BaseVariable;
virtual ~BaseVariable();
virtual void destroy();
typedef monty::rc_ptr< BaseVariable > t;

BaseVariable(monty::rc_ptr< ::mosek::fusion::Model > m,monty::rc_ptr< ::mosek::fusion::Set > shape);
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb);
virtual void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) = 0;
std::string toString() /* override */;
virtual void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj);
virtual void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) = 0;
virtual void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) = 0;
virtual void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) = 0;
virtual void values(int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal);
virtual void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) = 0;
virtual void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) = 0;
virtual void setLevel(std::shared_ptr< monty::ndarray< double,1 > > v);
virtual monty::rc_ptr< ::mosek::fusion::Model > getModel();
virtual monty::rc_ptr< ::mosek::fusion::Set > shape();
virtual monty::rc_ptr< ::mosek::fusion::Set > getShape();
virtual long long size();
virtual std::shared_ptr< monty::ndarray< double,1 > > dual();
virtual std::shared_ptr< monty::ndarray< double,1 > > level();
virtual void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) = 0;
virtual void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) = 0;
virtual void makeContinuous();
virtual void makeInteger();
virtual monty::rc_ptr< ::mosek::fusion::Variable > transpose();
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1,int i2);
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1);
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(std::shared_ptr< monty::ndarray< int,1 > > index);
virtual monty::rc_ptr< ::mosek::fusion::Variable > index(int index);
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2);
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1);
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,2 > > midxs);
virtual monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > idxs);
virtual monty::rc_ptr< ::mosek::fusion::Variable > antidiag(int index);
virtual monty::rc_ptr< ::mosek::fusion::Variable > antidiag();
virtual monty::rc_ptr< ::mosek::fusion::Variable > diag(int index);
virtual monty::rc_ptr< ::mosek::fusion::Variable > diag();
virtual monty::rc_ptr< ::mosek::fusion::Expression > asExpr();
virtual monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last);
virtual monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last);
public:
}; // class BaseVariable;

class CompoundVariable : public ::mosek::fusion::BaseVariable
{
CompoundVariable(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > v,int dim);
protected: CompoundVariable(p_CompoundVariable * _impl);
public:
CompoundVariable(const CompoundVariable &) = delete;
const CompoundVariable & operator=(const CompoundVariable &) = delete;
friend class p_CompoundVariable;
virtual ~CompoundVariable();
virtual void destroy();
typedef monty::rc_ptr< CompoundVariable > t;

monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
monty::rc_ptr< ::mosek::fusion::Expression > asExpr() /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last) /* override */;
}; // class CompoundVariable;

class RepeatVariable : public ::mosek::fusion::BaseVariable
{
protected: RepeatVariable(p_RepeatVariable * _impl);
public:
RepeatVariable(const RepeatVariable &) = delete;
const RepeatVariable & operator=(const RepeatVariable &) = delete;
friend class p_RepeatVariable;
virtual ~RepeatVariable();
virtual void destroy();
typedef monty::rc_ptr< RepeatVariable > t;

RepeatVariable(monty::rc_ptr< ::mosek::fusion::Variable > x,int dim,int count);
monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
}; // class RepeatVariable;

class PickVariable : public ::mosek::fusion::BaseVariable
{
PickVariable(monty::rc_ptr< ::mosek::fusion::Variable > origin,std::shared_ptr< monty::ndarray< long long,1 > > idxs);
protected: PickVariable(p_PickVariable * _impl);
public:
PickVariable(const PickVariable &) = delete;
const PickVariable & operator=(const PickVariable &) = delete;
friend class p_PickVariable;
virtual ~PickVariable();
virtual void destroy();
typedef monty::rc_ptr< PickVariable > t;

void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
}; // class PickVariable;

class SliceVariable : public ::mosek::fusion::BaseVariable
{
SliceVariable(monty::rc_ptr< ::mosek::fusion::Variable > origin,monty::rc_ptr< ::mosek::fusion::Set > shape,long long first,std::shared_ptr< monty::ndarray< long long,1 > > strides);
protected: SliceVariable(p_SliceVariable * _impl);
public:
SliceVariable(const SliceVariable &) = delete;
const SliceVariable & operator=(const SliceVariable &) = delete;
friend class p_SliceVariable;
virtual ~SliceVariable();
virtual void destroy();
typedef monty::rc_ptr< SliceVariable > t;

void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides_,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > firstidx,std::shared_ptr< monty::ndarray< int,1 > > lastidx) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(int firstidx,int lastidx) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides_,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
}; // class SliceVariable;

class BoundInterfaceVariable : public ::mosek::fusion::SliceVariable
{
BoundInterfaceVariable(monty::rc_ptr< ::mosek::fusion::RangedVariable > origin_,monty::rc_ptr< ::mosek::fusion::Set > shape_,long long first_,std::shared_ptr< monty::ndarray< long long,1 > > strides_,bool islower_);
protected: BoundInterfaceVariable(p_BoundInterfaceVariable * _impl);
public:
BoundInterfaceVariable(const BoundInterfaceVariable &) = delete;
const BoundInterfaceVariable & operator=(const BoundInterfaceVariable &) = delete;
friend class p_BoundInterfaceVariable;
virtual ~BoundInterfaceVariable();
virtual void destroy();
typedef monty::rc_ptr< BoundInterfaceVariable > t;

}; // class BoundInterfaceVariable;

class ModelVariable : public ::mosek::fusion::BaseVariable
{
protected: ModelVariable(p_ModelVariable * _impl);
public:
ModelVariable(const ModelVariable &) = delete;
const ModelVariable & operator=(const ModelVariable &) = delete;
friend class p_ModelVariable;
virtual ~ModelVariable();
virtual void destroy();
typedef monty::rc_ptr< ModelVariable > t;

void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last) /* override */;
}; // class ModelVariable;

class SymRangedVariable : public ::mosek::fusion::ModelVariable, public virtual ::mosek::fusion::SymmetricVariable
{
SymRangedVariable(monty::rc_ptr< ::mosek::fusion::SymRangedVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
SymRangedVariable(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::RangeDomain > dom_,int dim,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,long long varid_);
protected: SymRangedVariable(p_SymRangedVariable * _impl);
public:
SymRangedVariable(const SymRangedVariable &) = delete;
const SymRangedVariable & operator=(const SymRangedVariable &) = delete;
friend class p_SymRangedVariable;
virtual ~SymRangedVariable();
virtual void destroy();
typedef monty::rc_ptr< SymRangedVariable > t;

std::string toString() /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index_,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index_,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
public:
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Expression > asExpr();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,2 > > midxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > idxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag();
virtual /* override */ void makeContinuous();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > shape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2);
virtual /* override */ void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int index);
virtual /* override */ void makeInteger();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1,int i2);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > getShape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > transpose();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(std::shared_ptr< monty::ndarray< int,1 > > index);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > level();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Model > getModel();
virtual /* override */ void setLevel(std::shared_ptr< monty::ndarray< double,1 > > v);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag(int index);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > dual();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag(int index);
virtual /* override */ long long size();
virtual /* override */ void values(int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal);
}; // class SymRangedVariable;

class RangedVariable : public ::mosek::fusion::ModelVariable
{
RangedVariable(monty::rc_ptr< ::mosek::fusion::RangedVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
RangedVariable(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::Set > shape_p,monty::rc_ptr< ::mosek::fusion::RangeDomain > dom_,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,long long varid_);
protected: RangedVariable(p_RangedVariable * _impl);
public:
RangedVariable(const RangedVariable &) = delete;
const RangedVariable & operator=(const RangedVariable &) = delete;
friend class p_RangedVariable;
virtual ~RangedVariable();
virtual void destroy();
typedef monty::rc_ptr< RangedVariable > t;

monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
virtual monty::rc_ptr< ::mosek::fusion::Variable > upperBoundVar();
virtual monty::rc_ptr< ::mosek::fusion::Variable > lowerBoundVar();
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
}; // class RangedVariable;

class LinearPSDVariable : public ::mosek::fusion::ModelVariable
{
LinearPSDVariable(monty::rc_ptr< ::mosek::fusion::LinearPSDVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
LinearPSDVariable(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,int n,monty::rc_ptr< ::mosek::fusion::Set > shp_,int coneidx_,long long varid_);
protected: LinearPSDVariable(p_LinearPSDVariable * _impl);
public:
LinearPSDVariable(const LinearPSDVariable &) = delete;
const LinearPSDVariable & operator=(const LinearPSDVariable &) = delete;
friend class p_LinearPSDVariable;
virtual ~LinearPSDVariable();
virtual void destroy();
typedef monty::rc_ptr< LinearPSDVariable > t;

std::string toString() /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
}; // class LinearPSDVariable;

class PSDVariable : public ::mosek::fusion::ModelVariable, public virtual ::mosek::fusion::SymmetricVariable
{
PSDVariable(monty::rc_ptr< ::mosek::fusion::PSDVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
PSDVariable(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,int conesize_,int coneidx_,int num_,long long varid_);
protected: PSDVariable(p_PSDVariable * _impl);
public:
PSDVariable(const PSDVariable &) = delete;
const PSDVariable & operator=(const PSDVariable &) = delete;
friend class p_PSDVariable;
virtual ~PSDVariable();
virtual void destroy();
typedef monty::rc_ptr< PSDVariable > t;

std::string toString() /* override */;
monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
public:
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Expression > asExpr();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,2 > > midxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > idxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag();
virtual /* override */ void makeContinuous();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > shape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int index);
virtual /* override */ void makeInteger();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1,int i2);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > getShape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > transpose();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(std::shared_ptr< monty::ndarray< int,1 > > index);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > level();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Model > getModel();
virtual /* override */ void setLevel(std::shared_ptr< monty::ndarray< double,1 > > v);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag(int index);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > dual();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag(int index);
virtual /* override */ long long size();
virtual /* override */ void values(int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal);
}; // class PSDVariable;

class SymLinearVariable : public ::mosek::fusion::ModelVariable, public virtual ::mosek::fusion::SymmetricVariable
{
SymLinearVariable(monty::rc_ptr< ::mosek::fusion::SymLinearVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
SymLinearVariable(monty::rc_ptr< ::mosek::fusion::Model > model,const std::string &  name,monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > dom,int dim,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs,long long varid);
protected: SymLinearVariable(p_SymLinearVariable * _impl);
public:
SymLinearVariable(const SymLinearVariable &) = delete;
const SymLinearVariable & operator=(const SymLinearVariable &) = delete;
friend class p_SymLinearVariable;
virtual ~SymLinearVariable();
virtual void destroy();
typedef monty::rc_ptr< SymLinearVariable > t;

std::string toString() /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index_,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
public:
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Expression > asExpr();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,2 > > midxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > idxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag();
virtual /* override */ void makeContinuous();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > shape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2);
virtual /* override */ void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int index);
virtual /* override */ void makeInteger();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1,int i2);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > getShape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > transpose();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(std::shared_ptr< monty::ndarray< int,1 > > index);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > level();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Model > getModel();
virtual /* override */ void setLevel(std::shared_ptr< monty::ndarray< double,1 > > v);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag(int index);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > dual();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag(int index);
virtual /* override */ long long size();
virtual /* override */ void values(int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal);
}; // class SymLinearVariable;

class LinearVariable : public ::mosek::fusion::ModelVariable
{
LinearVariable(monty::rc_ptr< ::mosek::fusion::LinearVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
LinearVariable(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::LinearDomain > dom_,monty::rc_ptr< ::mosek::fusion::Set > shape_p,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,long long varid_);
protected: LinearVariable(p_LinearVariable * _impl);
public:
LinearVariable(const LinearVariable &) = delete;
const LinearVariable & operator=(const LinearVariable &) = delete;
friend class p_LinearVariable;
virtual ~LinearVariable();
virtual void destroy();
typedef monty::rc_ptr< LinearVariable > t;

monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
}; // class LinearVariable;

class ConicVariable : public ::mosek::fusion::ModelVariable
{
ConicVariable(monty::rc_ptr< ::mosek::fusion::ConicVariable > v,monty::rc_ptr< ::mosek::fusion::Model > m);
ConicVariable(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::QConeDomain > dom_,monty::rc_ptr< ::mosek::fusion::Set > shape_p,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,int conesize_,int firstcone_,int numcone_,long long varid_);
protected: ConicVariable(p_ConicVariable * _impl);
public:
ConicVariable(const ConicVariable &) = delete;
const ConicVariable & operator=(const ConicVariable &) = delete;
friend class p_ConicVariable;
virtual ~ConicVariable();
virtual void destroy();
typedef monty::rc_ptr< ConicVariable > t;

std::string toString() /* override */;
monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > val,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
}; // class ConicVariable;

class NilVariable : public ::mosek::fusion::BaseVariable, public virtual ::mosek::fusion::SymmetricVariable
{
NilVariable();
protected: NilVariable(p_NilVariable * _impl);
public:
NilVariable(const NilVariable &) = delete;
const NilVariable & operator=(const NilVariable &) = delete;
friend class p_NilVariable;
virtual ~NilVariable();
virtual void destroy();
typedef monty::rc_ptr< NilVariable > t;

monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > elementDesc(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void elementName(long long index,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > sb) /* override */;
void inst(std::shared_ptr< monty::ndarray< long long,1 > > index,int first_idx,int last_idx,long long index_offset,long long dst_offset,std::shared_ptr< monty::ndarray< int,1 > > dst_nindex,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubi,std::shared_ptr< monty::ndarray< int,1 > > dst_nsubj) /* override */;
void inst(long long index,long long offset,std::shared_ptr< monty::ndarray< int,1 > > nindex,std::shared_ptr< monty::ndarray< int,1 > > nsubi,std::shared_ptr< monty::ndarray< int,1 > > nsubj) /* override */;
void set_values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void set_values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(std::shared_ptr< monty::ndarray< long long,1 > > idxs,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void values(long long start,std::shared_ptr< monty::ndarray< int,1 > > nsize,std::shared_ptr< monty::ndarray< long long,1 > > strides,int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal) /* override */;
void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > idxs) /* override */;
void makeContinuous() /* override */;
void makeInteger() /* override */;
std::string toString() /* override */;
long long size() /* override */;
std::shared_ptr< monty::ndarray< double,1 > > dual() /* override */;
std::shared_ptr< monty::ndarray< double,1 > > level() /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > index(std::shared_ptr< monty::ndarray< int,1 > > first) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > index(int first) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last) /* override */;
monty::rc_ptr< ::mosek::fusion::Variable > slice(int first,int last) /* override */;
public:
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Expression > asExpr();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,2 > > midxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > idxs);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > shape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1,std::shared_ptr< monty::ndarray< int,1 > > i2);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1,int i2);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > index(int i0,int i1);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Set > getShape();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > transpose();
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > pick(std::shared_ptr< monty::ndarray< int,1 > > i0,std::shared_ptr< monty::ndarray< int,1 > > i1);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Model > getModel();
virtual /* override */ void setLevel(std::shared_ptr< monty::ndarray< double,1 > > v);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > diag(int index);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > antidiag(int index);
virtual /* override */ void values(int offset,std::shared_ptr< monty::ndarray< double,1 > > target,bool primal);
}; // class NilVariable;

class Var : public virtual monty::RefCounted
{
public: p_Var * _impl;
protected: Var(p_Var * _impl);
public:
Var(const Var &) = delete;
const Var & operator=(const Var &) = delete;
friend class p_Var;
virtual ~Var();
virtual void destroy();
typedef monty::rc_ptr< Var > t;

static  monty::rc_ptr< ::mosek::fusion::Variable > compress(monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > v,int d1);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > v,int d1,int d2);
static  monty::rc_ptr< ::mosek::fusion::Variable > flatten(monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::Set > s);
static  monty::rc_ptr< ::mosek::fusion::Variable > hrepeat(monty::rc_ptr< ::mosek::fusion::Variable > v,int n);
static  monty::rc_ptr< ::mosek::fusion::Variable > vrepeat(monty::rc_ptr< ::mosek::fusion::Variable > v,int n);
static  monty::rc_ptr< ::mosek::fusion::Variable > repeat(monty::rc_ptr< ::mosek::fusion::Variable > v,int n);
static  monty::rc_ptr< ::mosek::fusion::Variable > repeat(monty::rc_ptr< ::mosek::fusion::Variable > v,int dim,int n);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > >,1 > > vlist);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > v);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > v);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3,int dim);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,int dim);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > v,int dim);
}; // class Var;

class ConstraintCache : public virtual monty::RefCounted
{
public: p_ConstraintCache * _impl;
ConstraintCache(monty::rc_ptr< ::mosek::fusion::ConstraintCache > cc);
protected: ConstraintCache(p_ConstraintCache * _impl);
public:
ConstraintCache(const ConstraintCache &) = delete;
const ConstraintCache & operator=(const ConstraintCache &) = delete;
friend class p_ConstraintCache;
virtual ~ConstraintCache();
virtual void destroy();
typedef monty::rc_ptr< ConstraintCache > t;
std::shared_ptr< monty::ndarray< int,1 > > get_barmatidx();
void set_barmatidx(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_barsubj();
void set_barsubj(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_barsubi();
void set_barsubi(std::shared_ptr< monty::ndarray< int,1 > > val);
long long get_nbarnz();
void set_nbarnz(long long val);
long long get_nunordered();
void set_nunordered(long long val);
std::shared_ptr< monty::ndarray< int,1 > > get_buffer_subi();
void set_buffer_subi(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_buffer_subj();
void set_buffer_subj(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_buffer_cof();
void set_buffer_cof(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_bfix();
void set_bfix(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_cof();
void set_cof(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_subi();
void set_subi(std::shared_ptr< monty::ndarray< int,1 > > val);
std::shared_ptr< monty::ndarray< int,1 > > get_subj();
void set_subj(std::shared_ptr< monty::ndarray< int,1 > > val);
long long get_nnz();
void set_nnz(long long val);
int get_nrows();
void set_nrows(int val);

ConstraintCache(std::shared_ptr< monty::ndarray< long long,1 > > ptrb_,std::shared_ptr< monty::ndarray< double,1 > > cof_,std::shared_ptr< monty::ndarray< int,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > bfix_,std::shared_ptr< monty::ndarray< int,1 > > barsubi_,std::shared_ptr< monty::ndarray< int,1 > > barsubj_,std::shared_ptr< monty::ndarray< int,1 > > barmatidx_);
virtual void add(std::shared_ptr< monty::ndarray< long long,1 > > ptrb_,std::shared_ptr< monty::ndarray< int,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > cof_,std::shared_ptr< monty::ndarray< double,1 > > bfix_);
virtual long long flush(std::shared_ptr< monty::ndarray< int,1 > > ressubi,std::shared_ptr< monty::ndarray< int,1 > > ressubj,std::shared_ptr< monty::ndarray< double,1 > > rescof,std::shared_ptr< monty::ndarray< double,1 > > resbfix);
virtual long long numUnsorted();
}; // class ConstraintCache;

class Constraint : public virtual monty::RefCounted
{
public: p_Constraint * _impl;
protected: Constraint(p_Constraint * _impl);
public:
Constraint(const Constraint &) = delete;
const Constraint & operator=(const Constraint &) = delete;
friend class p_Constraint;
virtual ~Constraint();
virtual void destroy();
typedef monty::rc_ptr< Constraint > t;

Constraint(monty::rc_ptr< ::mosek::fusion::Constraint > c,monty::rc_ptr< ::mosek::fusion::Model > m);
Constraint(monty::rc_ptr< ::mosek::fusion::Model > model_,monty::rc_ptr< ::mosek::fusion::Set > shape_);
std::string toString() /* override */;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > add(double c);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > add(std::shared_ptr< monty::ndarray< double,1 > > cs);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > add(monty::rc_ptr< ::mosek::fusion::Variable > v);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > add(monty::rc_ptr< ::mosek::fusion::Expression > expr);
virtual std::shared_ptr< monty::ndarray< double,1 > > dual(std::shared_ptr< monty::ndarray< int,1 > > firstidxa,std::shared_ptr< monty::ndarray< int,1 > > lastidxa);
virtual std::shared_ptr< monty::ndarray< double,1 > > dual(int firstidx,int lastidx);
virtual std::shared_ptr< monty::ndarray< double,1 > > dual();
virtual std::shared_ptr< monty::ndarray< double,1 > > level();
virtual monty::rc_ptr< ::mosek::fusion::Model > get_model();
virtual int get_nd();
virtual long long size();
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > clist);
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(monty::rc_ptr< ::mosek::fusion::Constraint > v1,monty::rc_ptr< ::mosek::fusion::Constraint > v2,monty::rc_ptr< ::mosek::fusion::Constraint > v3);
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(monty::rc_ptr< ::mosek::fusion::Constraint > v1,monty::rc_ptr< ::mosek::fusion::Constraint > v2);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > index(std::shared_ptr< monty::ndarray< int,1 > > idxa);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > index(int idx);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > slice(int first,int last) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > shape();
}; // class Constraint;

class CompoundConstraint : public ::mosek::fusion::Constraint
{
protected: CompoundConstraint(p_CompoundConstraint * _impl);
public:
CompoundConstraint(const CompoundConstraint &) = delete;
const CompoundConstraint & operator=(const CompoundConstraint &) = delete;
friend class p_CompoundConstraint;
virtual ~CompoundConstraint();
virtual void destroy();
typedef monty::rc_ptr< CompoundConstraint > t;

CompoundConstraint(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > c);
monty::rc_ptr< ::mosek::fusion::Constraint > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) /* override */;
monty::rc_ptr< ::mosek::fusion::Constraint > slice(int first,int last) /* override */;
}; // class CompoundConstraint;

class SliceConstraint : public ::mosek::fusion::Constraint
{
SliceConstraint(monty::rc_ptr< ::mosek::fusion::ModelConstraint > origin_,monty::rc_ptr< ::mosek::fusion::Set > shape_,long long first_,std::shared_ptr< monty::ndarray< long long,1 > > strides_);
protected: SliceConstraint(p_SliceConstraint * _impl);
public:
SliceConstraint(const SliceConstraint &) = delete;
const SliceConstraint & operator=(const SliceConstraint &) = delete;
friend class p_SliceConstraint;
virtual ~SliceConstraint();
virtual void destroy();
typedef monty::rc_ptr< SliceConstraint > t;

long long size() /* override */;
monty::rc_ptr< ::mosek::fusion::Constraint > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) /* override */;
monty::rc_ptr< ::mosek::fusion::Constraint > slice(int first,int last) /* override */;
}; // class SliceConstraint;

class BoundInterfaceConstraint : public ::mosek::fusion::SliceConstraint
{
BoundInterfaceConstraint(monty::rc_ptr< ::mosek::fusion::RangedConstraint > origin_,monty::rc_ptr< ::mosek::fusion::Set > shape_,long long first_,std::shared_ptr< monty::ndarray< long long,1 > > strides_,bool islower_);
protected: BoundInterfaceConstraint(p_BoundInterfaceConstraint * _impl);
public:
BoundInterfaceConstraint(const BoundInterfaceConstraint &) = delete;
const BoundInterfaceConstraint & operator=(const BoundInterfaceConstraint &) = delete;
friend class p_BoundInterfaceConstraint;
virtual ~BoundInterfaceConstraint();
virtual void destroy();
typedef monty::rc_ptr< BoundInterfaceConstraint > t;

}; // class BoundInterfaceConstraint;

class ModelConstraint : public ::mosek::fusion::Constraint
{
protected: ModelConstraint(p_ModelConstraint * _impl);
public:
ModelConstraint(const ModelConstraint &) = delete;
const ModelConstraint & operator=(const ModelConstraint &) = delete;
friend class p_ModelConstraint;
virtual ~ModelConstraint();
virtual void destroy();
typedef monty::rc_ptr< ModelConstraint > t;

std::string toString() /* override */;
monty::rc_ptr< ::mosek::fusion::Constraint > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) /* override */;
monty::rc_ptr< ::mosek::fusion::Constraint > slice(int first,int last) /* override */;
}; // class ModelConstraint;

class LinearPSDConstraint : public ::mosek::fusion::ModelConstraint
{
LinearPSDConstraint(monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint > c,monty::rc_ptr< ::mosek::fusion::Model > m);
LinearPSDConstraint(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::Set > shape_,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,int conesize_,int firstcone_,int numcone_,std::shared_ptr< monty::ndarray< long long,1 > > ptrb,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > cof,std::shared_ptr< monty::ndarray< double,1 > > bfix,std::shared_ptr< monty::ndarray< int,1 > > barsubi_,std::shared_ptr< monty::ndarray< int,1 > > barsubj_,std::shared_ptr< monty::ndarray< int,1 > > barsymmatidx_);
protected: LinearPSDConstraint(p_LinearPSDConstraint * _impl);
public:
LinearPSDConstraint(const LinearPSDConstraint &) = delete;
const LinearPSDConstraint & operator=(const LinearPSDConstraint &) = delete;
friend class p_LinearPSDConstraint;
virtual ~LinearPSDConstraint();
virtual void destroy();
typedef monty::rc_ptr< LinearPSDConstraint > t;

std::string toString() /* override */;
}; // class LinearPSDConstraint;

class PSDConstraint : public ::mosek::fusion::ModelConstraint
{
PSDConstraint(monty::rc_ptr< ::mosek::fusion::PSDConstraint > c,monty::rc_ptr< ::mosek::fusion::Model > m);
PSDConstraint(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::Set > shape_,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,int conesize_,int firstcone_,int numcone_,std::shared_ptr< monty::ndarray< long long,1 > > ptrb,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > cof,std::shared_ptr< monty::ndarray< double,1 > > bfix,std::shared_ptr< monty::ndarray< int,1 > > barsubi_,std::shared_ptr< monty::ndarray< int,1 > > barsubj_,std::shared_ptr< monty::ndarray< int,1 > > barsymmatidx_);
protected: PSDConstraint(p_PSDConstraint * _impl);
public:
PSDConstraint(const PSDConstraint &) = delete;
const PSDConstraint & operator=(const PSDConstraint &) = delete;
friend class p_PSDConstraint;
virtual ~PSDConstraint();
virtual void destroy();
typedef monty::rc_ptr< PSDConstraint > t;

std::string toString() /* override */;
}; // class PSDConstraint;

class RangedConstraint : public ::mosek::fusion::ModelConstraint
{
RangedConstraint(monty::rc_ptr< ::mosek::fusion::RangedConstraint > c,monty::rc_ptr< ::mosek::fusion::Model > m);
RangedConstraint(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::Set > shape_,monty::rc_ptr< ::mosek::fusion::RangeDomain > dom_,std::shared_ptr< monty::ndarray< int,1 > > nidxs_,std::shared_ptr< monty::ndarray< long long,1 > > ptrb,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > cof,std::shared_ptr< monty::ndarray< double,1 > > bfix,std::shared_ptr< monty::ndarray< int,1 > > barsubi,std::shared_ptr< monty::ndarray< int,1 > > barsubj,std::shared_ptr< monty::ndarray< int,1 > > barmatidx);
protected: RangedConstraint(p_RangedConstraint * _impl);
public:
RangedConstraint(const RangedConstraint &) = delete;
const RangedConstraint & operator=(const RangedConstraint &) = delete;
friend class p_RangedConstraint;
virtual ~RangedConstraint();
virtual void destroy();
typedef monty::rc_ptr< RangedConstraint > t;

virtual monty::rc_ptr< ::mosek::fusion::Constraint > upperBoundCon();
virtual monty::rc_ptr< ::mosek::fusion::Constraint > lowerBoundCon();
}; // class RangedConstraint;

class ConicConstraint : public ::mosek::fusion::ModelConstraint
{
ConicConstraint(monty::rc_ptr< ::mosek::fusion::ConicConstraint > c,monty::rc_ptr< ::mosek::fusion::Model > m);
ConicConstraint(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::QConeDomain > dom_,monty::rc_ptr< ::mosek::fusion::Set > shape_,std::shared_ptr< monty::ndarray< int,1 > > nativeidxs_,int first_slack_,int last_slack_,int conesize_,int firstcone_,int numcone_,std::shared_ptr< monty::ndarray< long long,1 > > ptrb,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > cof,std::shared_ptr< monty::ndarray< double,1 > > bfix,std::shared_ptr< monty::ndarray< int,1 > > barsubi,std::shared_ptr< monty::ndarray< int,1 > > barsubj,std::shared_ptr< monty::ndarray< int,1 > > barmatidx);
protected: ConicConstraint(p_ConicConstraint * _impl);
public:
ConicConstraint(const ConicConstraint &) = delete;
const ConicConstraint & operator=(const ConicConstraint &) = delete;
friend class p_ConicConstraint;
virtual ~ConicConstraint();
virtual void destroy();
typedef monty::rc_ptr< ConicConstraint > t;

std::string toString() /* override */;
}; // class ConicConstraint;

class LinearConstraint : public ::mosek::fusion::ModelConstraint
{
LinearConstraint(monty::rc_ptr< ::mosek::fusion::LinearConstraint > c,monty::rc_ptr< ::mosek::fusion::Model > m);
LinearConstraint(monty::rc_ptr< ::mosek::fusion::Model > model_,const std::string &  name_,monty::rc_ptr< ::mosek::fusion::LinearDomain > dom_,monty::rc_ptr< ::mosek::fusion::Set > shape_,std::shared_ptr< monty::ndarray< int,1 > > nidxs_,std::shared_ptr< monty::ndarray< long long,1 > > ptrb,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > cof,std::shared_ptr< monty::ndarray< double,1 > > bfix,std::shared_ptr< monty::ndarray< int,1 > > barsubi,std::shared_ptr< monty::ndarray< int,1 > > barsubj,std::shared_ptr< monty::ndarray< int,1 > > barmatidx);
protected: LinearConstraint(p_LinearConstraint * _impl);
public:
LinearConstraint(const LinearConstraint &) = delete;
const LinearConstraint & operator=(const LinearConstraint &) = delete;
friend class p_LinearConstraint;
virtual ~LinearConstraint();
virtual void destroy();
typedef monty::rc_ptr< LinearConstraint > t;

}; // class LinearConstraint;

class Set : public virtual monty::RefCounted
{
public: p_Set * _impl;
protected: Set(p_Set * _impl);
public:
Set(const Set &) = delete;
const Set & operator=(const Set &) = delete;
friend class p_Set;
virtual ~Set();
virtual void destroy();
typedef monty::rc_ptr< Set > t;
int get_nd();
void set_nd(int val);

Set(int nd_,long long size_);
std::string toString() /* override */;
virtual std::string indexToString(long long index) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Set > slice(int first,int last) = 0;
virtual std::shared_ptr< monty::ndarray< int,1 > > idxtokey(long long idx);
virtual std::string getname(std::shared_ptr< monty::ndarray< int,1 > > keya) = 0;
virtual std::string getname(long long key) = 0;
virtual long long stride(int i) = 0;
virtual int dim(int i) = 0;
static  monty::rc_ptr< ::mosek::fusion::Set > make(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Set >,1 > > ss);
static  monty::rc_ptr< ::mosek::fusion::Set > make(monty::rc_ptr< ::mosek::fusion::Set > set1,monty::rc_ptr< ::mosek::fusion::Set > set2);
static  monty::rc_ptr< ::mosek::fusion::Set > make(std::shared_ptr< monty::ndarray< int,1 > > sizes);
static  monty::rc_ptr< ::mosek::fusion::Set > make(int s1,int s2,int s3);
static  monty::rc_ptr< ::mosek::fusion::Set > make(int s1,int s2);
static  monty::rc_ptr< ::mosek::fusion::Set > make(int sz);
static  monty::rc_ptr< ::mosek::fusion::Set > scalar();
static  monty::rc_ptr< ::mosek::fusion::Set > make(std::shared_ptr< monty::ndarray< std::string,1 > > names);
virtual int realnd();
virtual long long getSize();
virtual bool compare(monty::rc_ptr< ::mosek::fusion::Set > other);
}; // class Set;

class BaseSet : public ::mosek::fusion::Set
{
protected: BaseSet(p_BaseSet * _impl);
public:
BaseSet(const BaseSet &) = delete;
const BaseSet & operator=(const BaseSet &) = delete;
friend class p_BaseSet;
virtual ~BaseSet();
virtual void destroy();
typedef monty::rc_ptr< BaseSet > t;

BaseSet(long long size_);
int dim(int i) /* override */;
}; // class BaseSet;

class IntSet : public ::mosek::fusion::BaseSet
{
protected: IntSet(p_IntSet * _impl);
public:
IntSet(const IntSet &) = delete;
const IntSet & operator=(const IntSet &) = delete;
friend class p_IntSet;
virtual ~IntSet();
virtual void destroy();
typedef monty::rc_ptr< IntSet > t;

IntSet(int length);
IntSet(int first_,int last_);
std::string indexToString(long long index) /* override */;
std::string getname(std::shared_ptr< monty::ndarray< int,1 > > key) /* override */;
std::string getname(long long key) /* override */;
monty::rc_ptr< ::mosek::fusion::Set > slice(std::shared_ptr< monty::ndarray< int,1 > > firstidx,std::shared_ptr< monty::ndarray< int,1 > > lastidx) /* override */;
monty::rc_ptr< ::mosek::fusion::Set > slice(int firstidx,int lastidx) /* override */;
virtual int getidx(int key);
long long stride(int i) /* override */;
}; // class IntSet;

class StringSet : public ::mosek::fusion::BaseSet
{
protected: StringSet(p_StringSet * _impl);
public:
StringSet(const StringSet &) = delete;
const StringSet & operator=(const StringSet &) = delete;
friend class p_StringSet;
virtual ~StringSet();
virtual void destroy();
typedef monty::rc_ptr< StringSet > t;

StringSet(std::shared_ptr< monty::ndarray< std::string,1 > > ks);
std::string indexToString(long long index) /* override */;
std::string getname(std::shared_ptr< monty::ndarray< int,1 > > key) /* override */;
std::string getname(long long key) /* override */;
monty::rc_ptr< ::mosek::fusion::Set > slice(std::shared_ptr< monty::ndarray< int,1 > > first_,std::shared_ptr< monty::ndarray< int,1 > > last_) /* override */;
monty::rc_ptr< ::mosek::fusion::Set > slice(int first_,int last_) /* override */;
std::string toString() /* override */;
long long stride(int i) /* override */;
}; // class StringSet;

class NDSet : public ::mosek::fusion::Set
{
protected: NDSet(p_NDSet * _impl);
public:
NDSet(const NDSet &) = delete;
const NDSet & operator=(const NDSet &) = delete;
friend class p_NDSet;
virtual ~NDSet();
virtual void destroy();
typedef monty::rc_ptr< NDSet > t;

NDSet(int size0,int size1,int size2);
NDSet(int size0,int size1);
NDSet(std::shared_ptr< monty::ndarray< int,1 > > dims);
std::string indexToString(long long index) /* override */;
std::string getname(std::shared_ptr< monty::ndarray< int,1 > > key) /* override */;
std::string getname(long long key) /* override */;
int dim(int i) /* override */;
monty::rc_ptr< ::mosek::fusion::Set > slice(std::shared_ptr< monty::ndarray< int,1 > > first,std::shared_ptr< monty::ndarray< int,1 > > last) /* override */;
monty::rc_ptr< ::mosek::fusion::Set > slice(int first,int last) /* override */;
long long stride(int i) /* override */;
}; // class NDSet;

class ProductSet : public ::mosek::fusion::NDSet
{
protected: ProductSet(p_ProductSet * _impl);
public:
ProductSet(const ProductSet &) = delete;
const ProductSet & operator=(const ProductSet &) = delete;
friend class p_ProductSet;
virtual ~ProductSet();
virtual void destroy();
typedef monty::rc_ptr< ProductSet > t;

ProductSet(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Set >,1 > > ss);
std::string indexToString(long long index) /* override */;
}; // class ProductSet;

class QConeDomain : public virtual monty::RefCounted
{
public: p_QConeDomain * _impl;
QConeDomain(mosek::fusion::QConeKey k,std::shared_ptr< monty::ndarray< int,1 > > d,int a);
protected: QConeDomain(p_QConeDomain * _impl);
public:
QConeDomain(const QConeDomain &) = delete;
const QConeDomain & operator=(const QConeDomain &) = delete;
friend class p_QConeDomain;
virtual ~QConeDomain();
virtual void destroy();
typedef monty::rc_ptr< QConeDomain > t;

virtual monty::rc_ptr< ::mosek::fusion::QConeDomain > integral();
virtual int getAxis();
virtual monty::rc_ptr< ::mosek::fusion::QConeDomain > axis(int a);
}; // class QConeDomain;

class LinPSDDomain : public virtual monty::RefCounted
{
public: p_LinPSDDomain * _impl;
LinPSDDomain(monty::rc_ptr< ::mosek::fusion::Set > shp);
protected: LinPSDDomain(p_LinPSDDomain * _impl);
public:
LinPSDDomain(const LinPSDDomain &) = delete;
const LinPSDDomain & operator=(const LinPSDDomain &) = delete;
friend class p_LinPSDDomain;
virtual ~LinPSDDomain();
virtual void destroy();
typedef monty::rc_ptr< LinPSDDomain > t;

}; // class LinPSDDomain;

class PSDDomain : public virtual monty::RefCounted
{
public: p_PSDDomain * _impl;
PSDDomain(mosek::fusion::PSDKey k,monty::rc_ptr< ::mosek::fusion::Set > shp);
protected: PSDDomain(p_PSDDomain * _impl);
public:
PSDDomain(const PSDDomain &) = delete;
const PSDDomain & operator=(const PSDDomain &) = delete;
friend class p_PSDDomain;
virtual ~PSDDomain();
virtual void destroy();
typedef monty::rc_ptr< PSDDomain > t;

}; // class PSDDomain;

class RangeDomain : public virtual monty::RefCounted
{
public: p_RangeDomain * _impl;
RangeDomain(std::shared_ptr< monty::ndarray< double,1 > > lb_,std::shared_ptr< monty::ndarray< double,1 > > ub_,std::shared_ptr< monty::ndarray< int,1 > > dims,std::shared_ptr< monty::ndarray< long long,1 > > inst);
RangeDomain(monty::rc_ptr< ::mosek::fusion::RangeDomain > other);
protected: RangeDomain(p_RangeDomain * _impl);
public:
RangeDomain(const RangeDomain &) = delete;
const RangeDomain & operator=(const RangeDomain &) = delete;
friend class p_RangeDomain;
virtual ~RangeDomain();
virtual void destroy();
typedef monty::rc_ptr< RangeDomain > t;

virtual monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain > symmetric();
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > sparse();
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > integral();
}; // class RangeDomain;

class SymmetricRangeDomain : public ::mosek::fusion::RangeDomain
{
SymmetricRangeDomain(monty::rc_ptr< ::mosek::fusion::RangeDomain > other);
protected: SymmetricRangeDomain(p_SymmetricRangeDomain * _impl);
public:
SymmetricRangeDomain(const SymmetricRangeDomain &) = delete;
const SymmetricRangeDomain & operator=(const SymmetricRangeDomain &) = delete;
friend class p_SymmetricRangeDomain;
virtual ~SymmetricRangeDomain();
virtual void destroy();
typedef monty::rc_ptr< SymmetricRangeDomain > t;

}; // class SymmetricRangeDomain;

class SymmetricLinearDomain : public virtual monty::RefCounted
{
public: p_SymmetricLinearDomain * _impl;
SymmetricLinearDomain(monty::rc_ptr< ::mosek::fusion::LinearDomain > other);
protected: SymmetricLinearDomain(p_SymmetricLinearDomain * _impl);
public:
SymmetricLinearDomain(const SymmetricLinearDomain &) = delete;
const SymmetricLinearDomain & operator=(const SymmetricLinearDomain &) = delete;
friend class p_SymmetricLinearDomain;
virtual ~SymmetricLinearDomain();
virtual void destroy();
typedef monty::rc_ptr< SymmetricLinearDomain > t;

virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > sparse();
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > integral();
}; // class SymmetricLinearDomain;

class LinearDomain : public virtual monty::RefCounted
{
public: p_LinearDomain * _impl;
LinearDomain(mosek::fusion::RelationKey k,std::shared_ptr< monty::ndarray< double,1 > > rhs,std::shared_ptr< monty::ndarray< long long,1 > > sp,std::shared_ptr< monty::ndarray< int,1 > > dims);
LinearDomain(monty::rc_ptr< ::mosek::fusion::LinearDomain > other);
protected: LinearDomain(p_LinearDomain * _impl);
public:
LinearDomain(const LinearDomain &) = delete;
const LinearDomain & operator=(const LinearDomain &) = delete;
friend class p_LinearDomain;
virtual ~LinearDomain();
virtual void destroy();
typedef monty::rc_ptr< LinearDomain > t;

virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > symmetric();
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > sparse();
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > integral();
}; // class LinearDomain;

class Domain : public virtual monty::RefCounted
{
public: p_Domain * _impl;
protected: Domain(p_Domain * _impl);
public:
Domain(const Domain &) = delete;
const Domain & operator=(const Domain &) = delete;
friend class p_Domain;
virtual ~Domain();
virtual void destroy();
typedef monty::rc_ptr< Domain > t;

static  monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain > symmetric(monty::rc_ptr< ::mosek::fusion::RangeDomain > rd);
static  monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > symmetric(monty::rc_ptr< ::mosek::fusion::LinearDomain > ld);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > sparse(monty::rc_ptr< ::mosek::fusion::RangeDomain > rd);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > sparse(monty::rc_ptr< ::mosek::fusion::LinearDomain > ld);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > integral(monty::rc_ptr< ::mosek::fusion::RangeDomain > rd);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > integral(monty::rc_ptr< ::mosek::fusion::LinearDomain > ld);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > integral(monty::rc_ptr< ::mosek::fusion::QConeDomain > c);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > axis(monty::rc_ptr< ::mosek::fusion::QConeDomain > c,int a);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inRotatedQCone(std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inRotatedQCone(int m,int n);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inRotatedQCone(int n);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inRotatedQCone();
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inQCone(std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inQCone(int m,int n);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inQCone(int n);
static  monty::rc_ptr< ::mosek::fusion::QConeDomain > inQCone();
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD(int n,int m);
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD(int n);
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD();
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD(int n,int m);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD(int n);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD();
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone(int n,int m);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone(int n);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone();
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary();
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(int m,int n);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(int n);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(monty::rc_ptr< ::mosek::fusion::Matrix > lbm,monty::rc_ptr< ::mosek::fusion::Matrix > ubm);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(monty::rc_ptr< ::mosek::fusion::Matrix > lbm,double ub);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double lb,monty::rc_ptr< ::mosek::fusion::Matrix > ubm);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > lba,std::shared_ptr< monty::ndarray< double,1 > > uba);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > lba,double ub);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double lb,std::shared_ptr< monty::ndarray< double,1 > > uba);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double lb,double ub);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,1 > > a1,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double b,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double b,int m,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double b,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double b);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,1 > > a1,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double b,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double b,int m,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double b,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double b);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,1 > > a1,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double b,std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double b,int m,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double b,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double b);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(std::shared_ptr< monty::ndarray< int,1 > > dims);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(int m,int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(int n);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded();
}; // class Domain;

class SymmetricExpr : public virtual monty::RefCounted
{
public: p_SymmetricExpr * _impl;
SymmetricExpr(int n,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > Ms,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > xs,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > b);
protected: SymmetricExpr(p_SymmetricExpr * _impl);
public:
SymmetricExpr(const SymmetricExpr &) = delete;
const SymmetricExpr & operator=(const SymmetricExpr &) = delete;
friend class p_SymmetricExpr;
virtual ~SymmetricExpr();
virtual void destroy();
typedef monty::rc_ptr< SymmetricExpr > t;

std::string toString() /* override */;
}; // class SymmetricExpr;

class Expr : public virtual ::mosek::fusion::Expression
{
public: p_Expr * _impl;
Expr(std::shared_ptr< monty::ndarray< long long,1 > > ptrb_,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > v,std::shared_ptr< monty::ndarray< long long,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > cof_,std::shared_ptr< monty::ndarray< double,1 > > bfix_,monty::rc_ptr< ::mosek::fusion::Set > shp,std::shared_ptr< monty::ndarray< long long,1 > > inst_,int unchecked_);
Expr(monty::rc_ptr< ::mosek::fusion::Expression > e);
protected: Expr(p_Expr * _impl);
public:
Expr(const Expr &) = delete;
const Expr & operator=(const Expr &) = delete;
friend class p_Expr;
virtual ~Expr();
virtual void destroy();
typedef monty::rc_ptr< Expr > t;

Expr(std::shared_ptr< monty::ndarray< long long,1 > > ptrb_,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > v,std::shared_ptr< monty::ndarray< long long,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > cof_,std::shared_ptr< monty::ndarray< double,1 > > bfix_,monty::rc_ptr< ::mosek::fusion::Set > shp,std::shared_ptr< monty::ndarray< long long,1 > > inst_);
std::string toString() /* override */;
static  monty::rc_ptr< ::mosek::fusion::Expression > flatten(monty::rc_ptr< ::mosek::fusion::Expression > e);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > e,int dimi,int dimj);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > e,int size);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > e,monty::rc_ptr< ::mosek::fusion::Set > shp);
virtual long long size();
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > eval();
static  monty::rc_ptr< ::mosek::fusion::Expression > zeros(int num);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(int num);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > nda);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(double val);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::Set > shp,double val);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(int size,double val);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< double,2 > > vals2);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< double,1 > > vals1);
virtual long long numNonzeros();
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > expr,int dfirst,int dlast);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > expr,int d);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Variable > v,int dfirst,int dlast);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Variable > v,int d);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > neg(monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > neg(monty::rc_ptr< ::mosek::fusion::Expression > e);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Matrix > mx,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Matrix > mx,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,2 > > a);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(std::shared_ptr< monty::ndarray< double,2 > > a,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > expr,std::shared_ptr< monty::ndarray< double,2 > > a);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(std::shared_ptr< monty::ndarray< double,2 > > a,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Matrix > mx,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > expr,std::shared_ptr< monty::ndarray< double,1 > > vals);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,1 > > vals,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(double val,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > expr,double val);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,2 > > vals2);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,2 > > vals2,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Variable > v,double val);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(double val,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,1 > > vals,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,1 > > vals);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Matrix > mx,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(std::shared_ptr< monty::ndarray< double,1 > > a,monty::rc_ptr< ::mosek::fusion::Expression > e);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Expression > e,std::shared_ptr< monty::ndarray< double,1 > > a);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(std::shared_ptr< monty::ndarray< double,1 > > a,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,1 > > a);
virtual monty::rc_ptr< ::mosek::fusion::Expression > pick(std::shared_ptr< monty::ndarray< int,2 > > indexrows);
virtual monty::rc_ptr< ::mosek::fusion::Expression > pick(std::shared_ptr< monty::ndarray< int,1 > > indexes);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > >,1 > > exprs);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > exprs);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > exprs);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Expression > e3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,monty::rc_ptr< ::mosek::fusion::Variable > v3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2,double a3);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,double a2,monty::rc_ptr< ::mosek::fusion::Expression > e1);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,double a2,monty::rc_ptr< ::mosek::fusion::Variable > v1);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Variable > v1,double a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,double a1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,double a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int dim,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > exprs);
static  monty::rc_ptr< ::mosek::fusion::Expression > repeat(monty::rc_ptr< ::mosek::fusion::Expression > e,int n,int d);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > exps);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > vs);
virtual monty::rc_ptr< ::mosek::fusion::Expression > transpose();
virtual monty::rc_ptr< ::mosek::fusion::Expression > slice(std::shared_ptr< monty::ndarray< int,1 > > firsta,std::shared_ptr< monty::ndarray< int,1 > > lasta);
virtual monty::rc_ptr< ::mosek::fusion::Expression > index(std::shared_ptr< monty::ndarray< int,1 > > firsta);
virtual monty::rc_ptr< ::mosek::fusion::Expression > index(int first);
virtual monty::rc_ptr< ::mosek::fusion::Expression > slice(int first,int last);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > expr,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > expr,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Variable > v);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Expression > expr);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > expr,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > expr,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > expr,monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::NDSparseArray > spm);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Variable > v,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Variable > v,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::NDSparseArray > n,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::NDSparseArray > n);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(double c,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,double c);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::NDSparseArray > n,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::NDSparseArray > n);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(double c,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,double c);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::NDSparseArray > n,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::NDSparseArray > n);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(double c,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,double c);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::NDSparseArray > n,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::NDSparseArray > n);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Matrix > m,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(double c,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,double c);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,2 > > a2,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,1 > > a1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,std::shared_ptr< monty::ndarray< double,2 > > a2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,std::shared_ptr< monty::ndarray< double,1 > > a1);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Variable > v1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Variable > v2);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > e1,monty::rc_ptr< ::mosek::fusion::Expression > e2);
virtual monty::rc_ptr< ::mosek::fusion::Set > shape();
virtual monty::rc_ptr< ::mosek::fusion::Set > getShape();
virtual monty::rc_ptr< ::mosek::fusion::Model > getModel();
public:
}; // class Expr;

class FlatExpr : public virtual monty::RefCounted
{
public: p_FlatExpr * _impl;
protected: FlatExpr(p_FlatExpr * _impl);
public:
FlatExpr(const FlatExpr &) = delete;
const FlatExpr & operator=(const FlatExpr &) = delete;
friend class p_FlatExpr;
virtual ~FlatExpr();
virtual void destroy();
typedef monty::rc_ptr< FlatExpr > t;
std::shared_ptr< monty::ndarray< long long,1 > > get_inst();
void set_inst(std::shared_ptr< monty::ndarray< long long,1 > > val);
monty::rc_ptr< ::mosek::fusion::Set > get_shape();
void set_shape(monty::rc_ptr< ::mosek::fusion::Set > val);
long long get_nnz();
void set_nnz(long long val);
std::shared_ptr< monty::ndarray< double,1 > > get_cof();
void set_cof(std::shared_ptr< monty::ndarray< double,1 > > val);
std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > get_x();
void set_x(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > val);
std::shared_ptr< monty::ndarray< long long,1 > > get_subj();
void set_subj(std::shared_ptr< monty::ndarray< long long,1 > > val);
std::shared_ptr< monty::ndarray< long long,1 > > get_ptrb();
void set_ptrb(std::shared_ptr< monty::ndarray< long long,1 > > val);
std::shared_ptr< monty::ndarray< double,1 > > get_bfix();
void set_bfix(std::shared_ptr< monty::ndarray< double,1 > > val);

FlatExpr(monty::rc_ptr< ::mosek::fusion::FlatExpr > e);
FlatExpr(std::shared_ptr< monty::ndarray< double,1 > > bfix_,std::shared_ptr< monty::ndarray< long long,1 > > ptrb_,std::shared_ptr< monty::ndarray< long long,1 > > subj_,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > x_,std::shared_ptr< monty::ndarray< double,1 > > cof_,monty::rc_ptr< ::mosek::fusion::Set > shape_,std::shared_ptr< monty::ndarray< long long,1 > > inst_);
std::string toString() /* override */;
virtual int size();
}; // class FlatExpr;

class SymmetricMatrix : public virtual monty::RefCounted
{
public: p_SymmetricMatrix * _impl;
SymmetricMatrix(int dim0,int dim1,std::shared_ptr< monty::ndarray< int,1 > > usubi,std::shared_ptr< monty::ndarray< int,1 > > usubj,std::shared_ptr< monty::ndarray< double,1 > > uval,std::shared_ptr< monty::ndarray< int,1 > > vsubi,std::shared_ptr< monty::ndarray< int,1 > > vsubj,std::shared_ptr< monty::ndarray< double,1 > > vval,double scale);
protected: SymmetricMatrix(p_SymmetricMatrix * _impl);
public:
SymmetricMatrix(const SymmetricMatrix &) = delete;
const SymmetricMatrix & operator=(const SymmetricMatrix &) = delete;
friend class p_SymmetricMatrix;
virtual ~SymmetricMatrix();
virtual void destroy();
typedef monty::rc_ptr< SymmetricMatrix > t;

static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > rankOne(int n,std::shared_ptr< monty::ndarray< int,1 > > sub,std::shared_ptr< monty::ndarray< double,1 > > v);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > rankOne(std::shared_ptr< monty::ndarray< double,1 > > v);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > antiDiag(std::shared_ptr< monty::ndarray< double,1 > > vals);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > diag(std::shared_ptr< monty::ndarray< double,1 > > vals);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > add(monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > m);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > sub(monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > m);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > mul(double v);
virtual int getdim();
}; // class SymmetricMatrix;

class NDSparseArray : public virtual monty::RefCounted
{
public: p_NDSparseArray * _impl;
NDSparseArray(std::shared_ptr< monty::ndarray< int,1 > > dims_,std::shared_ptr< monty::ndarray< int,2 > > sub,std::shared_ptr< monty::ndarray< double,1 > > cof_);
NDSparseArray(std::shared_ptr< monty::ndarray< int,1 > > dims_,std::shared_ptr< monty::ndarray< long long,1 > > inst_,std::shared_ptr< monty::ndarray< double,1 > > cof_);
NDSparseArray(monty::rc_ptr< ::mosek::fusion::Matrix > m);
protected: NDSparseArray(p_NDSparseArray * _impl);
public:
NDSparseArray(const NDSparseArray &) = delete;
const NDSparseArray & operator=(const NDSparseArray &) = delete;
friend class p_NDSparseArray;
virtual ~NDSparseArray();
virtual void destroy();
typedef monty::rc_ptr< NDSparseArray > t;

static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(monty::rc_ptr< ::mosek::fusion::Matrix > m);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(std::shared_ptr< monty::ndarray< int,1 > > dims,std::shared_ptr< monty::ndarray< long long,1 > > inst,std::shared_ptr< monty::ndarray< double,1 > > cof);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(std::shared_ptr< monty::ndarray< int,1 > > dims,std::shared_ptr< monty::ndarray< int,2 > > sub,std::shared_ptr< monty::ndarray< double,1 > > cof);
}; // class NDSparseArray;

class Matrix : public virtual monty::RefCounted
{
public: p_Matrix * _impl;
protected: Matrix(p_Matrix * _impl);
public:
Matrix(const Matrix &) = delete;
const Matrix & operator=(const Matrix &) = delete;
friend class p_Matrix;
virtual ~Matrix();
virtual void destroy();
typedef monty::rc_ptr< Matrix > t;

std::string toString() /* override */;
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int num,monty::rc_ptr< ::mosek::fusion::Matrix > mv);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Matrix >,1 > > md);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(int n,double val,int k);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(int n,double val);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int n,double val,int k);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int n,double val);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(std::shared_ptr< monty::ndarray< double,1 > > d,int k);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(std::shared_ptr< monty::ndarray< double,1 > > d);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< double,1 > > d,int k);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< double,1 > > d);
static  monty::rc_ptr< ::mosek::fusion::Matrix > ones(int n,int m);
static  monty::rc_ptr< ::mosek::fusion::Matrix > eye(int n);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(monty::rc_ptr< ::mosek::fusion::Matrix > other);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(int dimi,int dimj,double value);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(int dimi,int dimj,std::shared_ptr< monty::ndarray< double,1 > > data);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(std::shared_ptr< monty::ndarray< double,2 > > data);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(monty::rc_ptr< ::mosek::fusion::Matrix > mx);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Matrix >,1 > >,1 > > blocks);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< double,2 > > data);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int nrow,int ncol);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int nrow,int ncol,std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,double val);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,double val);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > val);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int nrow,int ncol,std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > val);
virtual double get(int i,int j) = 0;
virtual bool isSparse() = 0;
virtual std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() = 0;
virtual void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > val) = 0;
virtual monty::rc_ptr< ::mosek::fusion::Matrix > transpose() = 0;
virtual long long numNonzeros() = 0;
virtual int numColumns();
virtual int numRows();
}; // class Matrix;

class DenseMatrix : public ::mosek::fusion::Matrix
{
DenseMatrix(int dimi_,int dimj_,std::shared_ptr< monty::ndarray< double,1 > > cof);
DenseMatrix(monty::rc_ptr< ::mosek::fusion::Matrix > m_);
DenseMatrix(std::shared_ptr< monty::ndarray< double,2 > > d);
DenseMatrix(int dimi_,int dimj_,double value_);
protected: DenseMatrix(p_DenseMatrix * _impl);
public:
DenseMatrix(const DenseMatrix &) = delete;
const DenseMatrix & operator=(const DenseMatrix &) = delete;
friend class p_DenseMatrix;
virtual ~DenseMatrix();
virtual void destroy();
typedef monty::rc_ptr< DenseMatrix > t;

std::string toString() /* override */;
monty::rc_ptr< ::mosek::fusion::Matrix > transpose() /* override */;
bool isSparse() /* override */;
std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() /* override */;
void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > subi,std::shared_ptr< monty::ndarray< int,1 > > subj,std::shared_ptr< monty::ndarray< double,1 > > cof) /* override */;
double get(int i,int j) /* override */;
long long numNonzeros() /* override */;
}; // class DenseMatrix;

class SparseMatrix : public ::mosek::fusion::Matrix
{
SparseMatrix(int dimi_,int dimj_,std::shared_ptr< monty::ndarray< int,1 > > subi_,std::shared_ptr< monty::ndarray< int,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > val_,long long nelm);
SparseMatrix(int dimi_,int dimj_,std::shared_ptr< monty::ndarray< int,1 > > subi_,std::shared_ptr< monty::ndarray< int,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > val_);
protected: SparseMatrix(p_SparseMatrix * _impl);
public:
SparseMatrix(const SparseMatrix &) = delete;
const SparseMatrix & operator=(const SparseMatrix &) = delete;
friend class p_SparseMatrix;
virtual ~SparseMatrix();
virtual void destroy();
typedef monty::rc_ptr< SparseMatrix > t;

std::string toString() /* override */;
long long numNonzeros() /* override */;
monty::rc_ptr< ::mosek::fusion::Matrix > transpose() /* override */;
bool isSparse() /* override */;
std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() /* override */;
void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > subi_,std::shared_ptr< monty::ndarray< int,1 > > subj_,std::shared_ptr< monty::ndarray< double,1 > > cof_) /* override */;
double get(int i,int j) /* override */;
}; // class SparseMatrix;

class Parameters : public virtual monty::RefCounted
{
public: p_Parameters * _impl;
protected: Parameters(p_Parameters * _impl);
public:
Parameters(const Parameters &) = delete;
const Parameters & operator=(const Parameters &) = delete;
friend class p_Parameters;
virtual ~Parameters();
virtual void destroy();
typedef monty::rc_ptr< Parameters > t;

static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > M,const std::string &  name,double value);
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > M,const std::string &  name,int value);
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > M,const std::string &  name,const std::string &  value);
}; // class Parameters;

}
}
namespace mosek
{
namespace fusion
{
namespace Utils
{
// class mosek.fusion.Utils.StringIntMap
// mosek.fusion.Utils.IntMap from file 'src/fusion/cxx/IntMap.h'
// namespace mosek::fusion::Utils
class IntMap : public monty::RefCounted
{
  std::unique_ptr<p_IntMap> _impl;  
public:
  friend class p_IntMap;
  typedef monty::rc_ptr<IntMap> t;

  IntMap();
  bool hasItem (long long key);
  int  getItem (long long key);
  void setItem (long long key, int val);
  std::shared_ptr<monty::ndarray<long long,1>> keys();
  std::shared_ptr<monty::ndarray<int,1>>       values();

  t clone();
};

class StringIntMap : public monty::RefCounted
{
  std::unique_ptr<p_StringIntMap> _impl;
public:
  friend class p_StringIntMap;
  typedef monty::rc_ptr<StringIntMap> t;

  StringIntMap();
  bool hasItem (const std::string & key);
  int  getItem (const std::string & key);
  void setItem (const std::string & key, int val);
  std::shared_ptr<monty::ndarray<std::string,1>> keys();
  std::shared_ptr<monty::ndarray<int,1>>       values();

  t clone();
};

// End of file 'src/fusion/cxx/IntMap.h'
// mosek.fusion.Utils.StringBuffer from file 'src/fusion/cxx/StringBuffer.h'
// namespace mosek::fusion::Utils
class StringBuffer : public monty::RefCounted
{
private:
  std::unique_ptr<p_StringBuffer> _impl;
public:
  friend class p_StringBuffer;

  typedef monty::rc_ptr<StringBuffer> t;

  StringBuffer();
  t clear ();
  t a (int                 value);
  t a (double              value);
  t a (const std::string & value);
  t a (bool                value);
  t a (std::shared_ptr<monty::ndarray<std::string,1>> value);
  t a (std::shared_ptr<monty::ndarray<int,1>>         value);
  t a (std::shared_ptr<monty::ndarray<long long,1>>   value);
  t a (std::shared_ptr<monty::ndarray<double,1>>      value);
  t lf ();
  std::string toString () const;
};
// End of file 'src/fusion/cxx/StringBuffer.h'
// mosek.fusion.Utils.Tools from file 'src/fusion/cxx/Tools.h'
namespace Tools
{
  template<typename T, int N>
  void
  arraycopy
  ( const std::shared_ptr<monty::ndarray<T,N>> & src,
    int                                          srcoffset,
    const std::shared_ptr<monty::ndarray<T,N>> & tgt,
    int                                          tgtoffset,
    int                                          size)
  {
    std::copy(src->flat_begin()+srcoffset, src->flat_begin()+srcoffset+size, tgt->flat_begin()+tgtoffset);
  }

  template<typename T, int N>
  void
  arraycopy
  ( const std::shared_ptr<monty::ndarray<T,N>> & src,
    long long                                    srcoffset,
    const std::shared_ptr<monty::ndarray<T,N>> & tgt,
    long long                                    tgtoffset,
    long long                                    size)
  {
    std::copy(src->flat_begin()+srcoffset, src->flat_begin()+srcoffset+size, tgt->flat_begin()+tgtoffset);
  }

  template<typename T, int N>
  std::shared_ptr<monty::ndarray<T,N>> 
  arraycopy (const std::shared_ptr<monty::ndarray<T,N>> & a) 
  { 
    return std::shared_ptr<monty::ndarray<T,N>>(new monty::ndarray<T,N>(a->shape, a->flat_begin(), a->flat_end()));
  }

  template<typename T> 
  std::shared_ptr<monty::ndarray<T,1>> range(T last) 
  {     
    return std::shared_ptr<monty::ndarray<T,1>>(new monty::ndarray<T,1>(monty::shape(last), monty::iterable(monty::range_t<T>(0,last))));
  }
  
  template<typename T> 
  std::shared_ptr<monty::ndarray<T,1>> range(T first, T last)
  { 
    return std::shared_ptr<monty::ndarray<T,1>>(new monty::ndarray<T,1>(monty::shape(last-first), monty::iterable(monty::range_t<T>(first,last))));
  }
  
  template<typename T> 
  std::shared_ptr<monty::ndarray<T,1>> range(T first, T last, T step)
  { 
    size_t num = last > first && step > 0 ? (last - first - 1) / step + 1 : 0;
    if (num > 0)
      return std::shared_ptr<monty::ndarray<T,1>>(new monty::ndarray<T,1>(monty::shape(num), monty::iterable(monty::range_t<T>(first,last,step))));
    else
      return std::shared_ptr<monty::ndarray<T,1>>(new monty::ndarray<T,1>(monty::shape(0)));
  }

  static std::shared_ptr<monty::ndarray<double,1>> zeros(int num)            { return std::shared_ptr<monty::ndarray<double,1>>(new monty::ndarray<double,1>(monty::shape(num),0.0));       }
  static std::shared_ptr<monty::ndarray<double,2>> zeros(int dimi, int dimj) { return std::shared_ptr<monty::ndarray<double,2>>(new monty::ndarray<double,2>(monty::shape(dimi,dimj),0.0)); }
  static std::shared_ptr<monty::ndarray<double,1>> ones (int num)            { return std::shared_ptr<monty::ndarray<double,1>>(new monty::ndarray<double,1>(monty::shape(num),1.0));       }

  template<typename T>
  std::shared_ptr<monty::ndarray<T,1>> makevector(T v, int num) { return std::shared_ptr<monty::ndarray<T,1>>(new monty::ndarray<T,1>(monty::shape(num),v)); }

  template<typename T>
  std::shared_ptr<monty::ndarray<T,1>> repeatrange(T first, T last, T num)
  {
    return std::shared_ptr<monty::ndarray<T,1>>(new monty::ndarray<T,1>(monty::shape((last-first)*num),[=](ptrdiff_t i) { return (T)(i%num+first); }));
  }
  
  template<typename T>
  std::string stringvalue (T val) 
  {
    std::stringstream os; os << val;
    return os.str(); 
  }

  static int    toInt(const std::string & v)    { return atoi(v.c_str()); } 
  static double toDouble(const std::string & v) { return atof(v.c_str()); }
  static double sqrt(double v) { return std::sqrt(v); }


  template<typename T>
  void sort (const std::shared_ptr<monty::ndarray<T,1>> & vals,int first,int last)
  {
      //assert(first >= 0);
      //assert(last < vals->size());
      std::sort(vals->flat_begin()+first, vals->flat_begin()+last, [&](T lhs, T rhs){ return lhs < rhs; });
  }

#if (defined(_WIN32) && _WIN32) || (defined(_WIN64) && _WIN64)
  static int randInt(int max) 
  {
    long long lo = rand(), hi = rand();
    return (int)(((double)(((hi << 31) | lo)) / (double)(LLONG_MAX)) * max);
  }
#else
  static int randInt(int max) { return (int)(((double)(random()) / (double)(RAND_MAX)) * max); }
#endif





  template<typename T>
  static void argsort(const std::shared_ptr<monty::ndarray<long long,1>> & perm, 
                      const std::shared_ptr<monty::ndarray<T,1>> & v,
                      long long first,
                      long long last)
  {
    std::sort(perm->begin()+first,perm->begin()+last,[&v](long long lhs, long long rhs){ return (*v)[lhs] < (*v)[rhs]; });
  }
  
  template<typename T>
  static void argsort(const std::shared_ptr<monty::ndarray<long long,1>> & perm, 
                      const std::shared_ptr<monty::ndarray<T,1>> & v0,
                      const std::shared_ptr<monty::ndarray<T,1>> & v1,
                      long long first,
                      long long last)
  {
    std::sort(perm->begin()+first,perm->begin()+last,[&v0,&v1](long long lhs, long long rhs){ return (*v0)[lhs] < (*v0)[rhs] || ((*v0)[lhs] == (*v0)[rhs] && (*v1)[lhs] < (*v1)[rhs]); });
  }


  template <typename T>
  static void bucketsort(const std::shared_ptr<monty::ndarray<long long,1>> & perm,
                         long long first, 
                         long long last,
                         const std::shared_ptr<monty::ndarray<T,1>> & v,
                         T minval,
                         T maxval)
  {
    T N = maxval-minval+1;
    long long M = last-first;
    std::vector<ptrdiff_t> ptrb(N+1);
    std::vector<long long> nperm(M);
    for (ptrdiff_t i = first; i < last; ++i) ++ptrb[(*v)[(*perm)[i]]-minval+1];
    for (ptrdiff_t i = 1; i < N; ++i) ptrb[i] += ptrb[i-1];
    for (ptrdiff_t i = first; i < last; ++i) nperm[ptrb[(*v)[(*perm)[i]]-minval]++] = (*perm)[i];

    std::copy(nperm.begin(),nperm.end(),perm->begin()+first);
  }
}

// End of file 'src/fusion/cxx/Tools.h'
}
}
}
#endif
