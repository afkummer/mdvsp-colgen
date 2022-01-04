#pragma once

#include "Instance.h"

#include <boost/multi_array.hpp>
#include <coin/Cbc_C_Interface.h>

class ModelCbc {
public:
   ModelCbc(const Instance &inst);
   virtual ~ModelCbc();

   auto writeLp(const char *fname) const noexcept -> void;

   auto changeBounds(int k, int i, int j, double lb = 0.0, double ub = 1.0) noexcept -> void;

private:
   const Instance *m_inst;
   
   Cbc_Model *m_model;
   boost::multi_array<int,3> m_x;
};
