#pragma once

#include "Instance.h"
#include "CgMasterInterface.h"

#include <ilcplex/ilocplex.h>

class CgMasterCplex: public CgMasterInterface {
public:
   CgMasterCplex(const Instance &inst);
   virtual ~CgMasterCplex();

   auto writeLp(const char *fname) const noexcept -> void;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto getTripDual(int i) const noexcept -> double override;
   virtual auto getDepotCapDual(int k) const noexcept -> double override;

   // Methods for adding a column.
   virtual auto beginColumn(int depotId) noexcept -> void override;
   virtual auto addTrip(int trip) noexcept -> void override;
   virtual auto commitColumn() noexcept -> void override;

   virtual auto numColumns() const noexcept -> int;

   virtual auto setAssignmentType(char sense = 'G') noexcept -> void override;   

private:
   const Instance *m_inst;

   IloEnv m_env;
   IloModel m_model;
   IloCplex m_cplex;
   IloObjective m_obj;

   IloNumVarArray m_dummy;
   IloNumVarArray m_paths;

   IloRangeArray m_range;
   IloNumArray m_duals;

   int m_newcolDepotId;
   int m_lastTrip;
   double m_newcolCost;
   IloArray<std::pair<int, double>> m_coefs;

   std::vector<int> m_pathDepot;
   std::vector<std::vector<int>> m_pathTrips;

   auto addColumn(double cost) noexcept -> IloNumVar;
};
