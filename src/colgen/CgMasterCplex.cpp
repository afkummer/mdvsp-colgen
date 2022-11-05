#include "CgMasterCplex.h"

#include "ModelCbc.h"

#include <iostream>
#include <fstream>

using namespace std;

CgMasterCplex::CgMasterCplex(const Instance &inst): CgMasterBase(inst) {
   char buf[128];
   m_model = IloModel(m_env, "mdvsp_master_cplex");
   m_cplex = IloCplex(m_model);
   m_obj = IloObjective(m_env, 0.0, IloObjective::Minimize, "set_partition_cost");

   m_dummy = IloNumVarArray(m_env);
   m_paths = IloNumVarArray(m_env);

   m_range = IloRangeArray(m_env);
   m_duals = IloNumArray(m_env);

   // Adds the assignment constraints.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      snprintf(buf, sizeof buf, "task_assign#%d", i);
      m_range.add(IloRange(m_env, 1.0, IloInfinity, buf));
      m_duals.add(0.0);
   }

   // Adds the depot capacity constraints
   for (int k = 0; k < m_inst->numDepots(); ++k) {
      snprintf(buf, sizeof buf, "depot_cap#%d", k);
      m_range.add(IloRange(m_env, -IloInfinity, m_inst->depotCapacity(k), buf));
      m_duals.add(0.0);
   }

   m_model.add(m_range);
   m_model.add(m_obj);

   // Adds the dummy solution.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      IloNumColumn col = m_obj(1e7);
      col += m_range[i](1.0);
      snprintf(buf, sizeof buf, "dummy#%d", i);

      IloNumVar path = IloNumVar(col, 0.0, IloInfinity, IloNumVar::Float, buf);
      // For now, we do nothing with these artificial vars.
      (void) path;
      col.end();
   }

   m_env.setOut(m_env.getNullStream());
   m_env.setWarning(m_env.getNullStream());
   m_cplex.setOut(m_env.getNullStream());
   m_cplex.setWarning(m_env.getNullStream());
   
   m_cplex.setParam(IloCplex::IntParam::Threads, 1);
}

CgMasterCplex::~CgMasterCplex() {
   m_env.end();
}

auto CgMasterCplex::getSolverName() const noexcept -> std::string {
   return "CPLEX " + to_string(CPX_VERSION);
}

auto CgMasterCplex::writeLp(const char *fname) const noexcept -> void {
   m_cplex.exportModel(fname);
}

auto CgMasterCplex::solve() noexcept -> double {
   if (!m_cplex.solve()) {
      cout << "RMP became infeasible.\n";
      writeLp("rmp_problematic.lp");
      abort();
   }
   m_cplex.getDuals(m_duals, m_range);
   return m_cplex.getObjValue();
}

auto CgMasterCplex::getObjValue() const noexcept -> double {
   return m_cplex.getObjValue();
}

auto CgMasterCplex::getTripDual(int i) const noexcept -> double {
   assert(i >= 0 && i < m_inst->numTrips());
   return m_duals[i];
}

auto CgMasterCplex::getDepotCapDual(int k) const noexcept -> double {
   assert(k >= 0 && k <= m_inst->numDepots());
   return m_duals[k+m_inst->numTrips()];
}

auto CgMasterCplex::setAssignmentType(char sense) noexcept -> void {
   assert(sense == 'E' or sense == 'G');
   auto ub = sense == 'E' ? 1.0 : IloInfinity;
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      m_range[i].setBounds(1.0, ub);
   }
}

auto CgMasterCplex::getValue(int col) const noexcept -> double {
   return m_cplex.getValue(m_paths[col]);
}

auto CgMasterCplex::getLb(int col) const noexcept -> double {
   return m_paths[col].getLB();
}

auto CgMasterCplex::setLb(int col, double bound) noexcept -> void {
   m_paths[col].setLb(bound);
}

auto CgMasterCplex::addColumn() noexcept -> void {
   assert(m_newcolDepot != -1);
   assert(!m_newcolPath.empty());
   IloNumColumn col = m_obj(m_newcolCost);
   for (int i: m_newcolPath) {
      col += m_range[i](1.0);
   }
   col += m_range[m_newcolDepot+m_inst->numTrips()](1.0);

   char buf[128];
   if (m_newcolDepot == -1) {
      snprintf(buf, sizeof buf, "dummy#%d", numColumns());
   } else {
      snprintf(buf, sizeof buf, "path#%d#%d", m_newcolDepot, numColumns());
   }

   IloNumVar path = IloNumVar(col, 0.0, IloInfinity, IloNumVar::Float, buf);
   m_paths.add(path);
   col.end();
}
