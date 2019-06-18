#include "fusion_p.h"
#include <string>
#include <vector>

namespace mosek
{
  namespace fusion
  {
    namespace Utils
    {
      StringBuffer::StringBuffer() : _impl(new p_StringBuffer(this)) {}
      StringBuffer::t StringBuffer::clear ()                                        { return _impl->clear(); }
      StringBuffer::t StringBuffer::a (int         value)                           { return _impl->a(value); }
      StringBuffer::t StringBuffer::a (double      value)                           { return _impl->a(value); }
      StringBuffer::t StringBuffer::a (const std::string & value)                   { return _impl->a(value); }
      StringBuffer::t StringBuffer::a (bool        value)                           { return _impl->a(value); }
      StringBuffer::t StringBuffer::a (std::shared_ptr<monty::ndarray<std::string,1>> value) { return _impl->a(*value.get()); }
      StringBuffer::t StringBuffer::a (std::shared_ptr<monty::ndarray<int,1>>         value) { return _impl->a(*value.get()); }
      StringBuffer::t StringBuffer::a (std::shared_ptr<monty::ndarray<long long,1>>   value) { return _impl->a(*value.get()); }
      StringBuffer::t StringBuffer::a (std::shared_ptr<monty::ndarray<double,1>>      value) { return _impl->a(*value.get()); }
      StringBuffer::t StringBuffer::lf ()                                           { return _impl->lf(); }
      std::string StringBuffer::toString () const                  { return _impl->toString(); }


      StringBuffer::t p_StringBuffer::lf       () { ss << std::endl; return StringBuffer::t(_pubthis); } 
      StringBuffer::t p_StringBuffer::clear    () { ss.str(""); return StringBuffer::t(_pubthis); }
      std::string 
      p_StringBuffer::toString () const           { return std::string(ss.str()); }
    }
  }
}
