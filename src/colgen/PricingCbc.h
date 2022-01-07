#pragma once

#include "CgPricingBase.h"

#include <boost/multi_array.hpp>
#include <coin/CbcModel.hpp>
#include <coin/Cbc_C_Interface.h>

class PricingCbc: public CgPricingBase {
public:
   PricingCbc(const Instance &inst, CgMasterBase &master, int depotId, int maxPaths = -1);
   virtual ~PricingCbc();

   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto isExact() const noexcept -> bool override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto generateColumns() const noexcept -> int override;

private:
   // Used to model the problem
   std::unique_ptr<OsiClpSolverInterface> m_lpSolver;
   boost::multi_array<int, 2> m_x;

   // Used to solve the problem as integer programming.
   std::unique_ptr<CbcModel> m_model;   

   auto findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void;
};
