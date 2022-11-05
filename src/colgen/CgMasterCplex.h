#pragma once

#include "Instance.h"
#include "CgMasterBase.h"

#include <ilcplex/ilocplex.h>

class CgMasterCplex: public CgMasterBase {
public:
   CgMasterCplex(const Instance &inst);
   virtual ~CgMasterCplex();

   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto getTripDual(int i) const noexcept -> double override;
   virtual auto getDepotCapDual(int k) const noexcept -> double override;

   virtual auto setAssignmentType(char sense = 'G') noexcept -> void override;   

   virtual auto getValue(int col) const noexcept -> double override;
   virtual auto getLb(int col) const noexcept -> double override;
   virtual auto setLb(int col, double bound) noexcept -> void override;

private:
   IloEnv m_env;
   IloModel m_model;
   IloCplex m_cplex;
   IloObjective m_obj;

   IloNumVarArray m_dummy;
   IloNumVarArray m_paths;

   IloRangeArray m_range;
   IloNumArray m_duals;

   virtual auto addColumn() noexcept -> void override;
};
