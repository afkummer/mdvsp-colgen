#pragma once

#include "CgMasterInterface.h"
#include "CgPricingInterface.h"
#include "Instance.h"
#include "PricingCplex.h"

#include <queue>

class PricingSpfa: public CgPricingInterface {
public:
   PricingSpfa(const Instance &inst, CgMasterInterface &dp, int depotId);
   virtual ~PricingSpfa();

   virtual auto depotId() const noexcept -> int override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto generateColumns() const noexcept -> void override;
   
private:
   const Instance *m_inst;
   CgMasterInterface *m_dp;
   const int m_depotId;

   std::vector<double> m_dist;
   std::vector<int> m_pred;

   std::unique_ptr<PricingCplex> m_cpx;

   auto getPathRecursive(std::vector<int> &path, double &pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void;
};
