#pragma once

#include "CgPricingBase.h"

#include <boost/multi_array.hpp>
#include <glpk.h>

class PricingGlpk: public CgPricingBase {
public:
   PricingGlpk(const Instance &inst, CgMasterBase &master, const int depotId, int maxPaths = -1);
   virtual ~PricingGlpk();

   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto isExact() const noexcept -> bool override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto generateColumns() const noexcept -> int override;

private:
   glp_prob *m_model;
   boost::multi_array<int, 2> m_x;

   auto findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void;

   auto colValue(int j) const noexcept -> double;
};
