#pragma once

#include "Instance.h"
#include "CgMasterInterface.h"
#include "CgPricingInterface.h"

#include <ilcplex/ilocplex.h>

class PricingCplex: public CgPricingInterface {
public:
   PricingCplex(const Instance &inst, CgMasterInterface &master, int depotId);
   virtual ~PricingCplex();

   auto writeLp(const char *fname) const noexcept -> void;

   virtual auto depotId() const noexcept -> int override;

   virtual auto solve() noexcept -> double override;
   virtual auto generateColumns() const noexcept -> void override;

private:
   const Instance *m_inst;
   const int m_depotId;
   CgMasterInterface *m_master;
   
   IloEnv m_env;
   IloModel m_model;
   IloCplex m_cplex;
   IloObjective m_obj;
   IloArray<IloNumVarArray> m_x;
};
