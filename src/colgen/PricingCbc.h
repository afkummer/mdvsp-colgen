#pragma once

#include "Instance.h"
#include "CgMasterInterface.h"
#include "CgPricingInterface.h"

#include <boost/multi_array.hpp>
#include <coin/Cbc_C_Interface.h>

class PricingCbc: public CgPricingInterface {
public:
   PricingCbc(const Instance &inst, CgMasterInterface &master, int depotId);
   virtual ~PricingCbc();

   auto writeLp(const char *fname) const noexcept -> void;

   virtual auto depotId() const noexcept -> int override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto generateColumns() const noexcept -> void override;

  private:
   const Instance *m_inst;
   const int m_depotId;
   CgMasterInterface *m_master;
   
   Cbc_Model *m_model;
   boost::multi_array<int,2> m_x;

   auto getPathRecursive(std::vector<int> &path, std::vector<std::vector<int>> &allPaths) const noexcept -> void;
};
