#pragma once

#include "CgPricingBase.h"

class PricingSpfa: public CgPricingBase {
public:
   PricingSpfa(const Instance &inst, CgMasterBase &dp, int depotId, bool singlePath = false);
   virtual ~PricingSpfa();
   
   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto isExact() const noexcept -> bool override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto generateColumns() const noexcept -> int override;
   
private:
   std::vector<double> m_dist;
   std::vector<int> m_pred;

   int m_labelExpansionLimitPerNode;

   auto findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void;
};
