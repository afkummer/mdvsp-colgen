#pragma once

#include "CgPricingBase.h"

#include <ilcplex/ilocplex.h>

class PricingCplex: public CgPricingBase {
public:
   PricingCplex(const Instance &inst, CgMasterBase &master, const int depotId, int maxPaths = 5);
   virtual ~PricingCplex();

   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto generateColumns() const noexcept -> int override;

private:   
   IloEnv m_env;
   IloModel m_model;
   IloCplex m_cplex;
   IloObjective m_obj;
   IloArray<IloNumVarArray> m_x;

   auto findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void;
};
