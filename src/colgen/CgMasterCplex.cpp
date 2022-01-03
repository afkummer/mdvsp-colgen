#include "CgMasterCplex.h"

#include "ModelCbc.h"

#include <iostream>
#include <fstream>

using namespace std;

CgMasterCplex::CgMasterCplex(const Instance &inst): m_inst(&inst) {
   char buf[128];
   m_model = IloModel(m_env, "mdvsp_master_cplex");
   m_cplex = IloCplex(m_model);
   m_obj = IloObjective(m_env, 0.0, IloObjective::Minimize, "set_partition_cost");

   m_dummy = IloNumVarArray(m_env);
   m_paths = IloNumVarArray(m_env);

   m_range = IloRangeArray(m_env);
   m_coefs = IloArray<std::pair<int, double>>(m_env);
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
      m_coefs.add(make_pair(i, 1.0));
      snprintf(buf, sizeof buf, "dummy#%d", i);
      addColumn(1e7).setName(buf);
   }

   m_paths.clear();

   m_cplex.setParam(IloCplex::IntParam::MIPDisplay, 0);
   m_cplex.setParam(IloCplex::IntParam::SimDisplay, 0);
   m_cplex.setParam(IloCplex::IntParam::BarDisplay, 0);
   m_env.setOut(m_env.getNullStream());
   m_env.setWarning(m_env.getNullStream());
   m_cplex.setOut(m_env.getNullStream());
   m_cplex.setWarning(m_env.getNullStream());

   if (0) {
      ifstream fid("colcache.txt");
      if (fid) {
         int nc;
         fid >> nc;
         cout << "Column cache found. It contains " << nc << " precalculated columns.\n";
         for (int cnt = 0; cnt < nc; ++cnt) {
            int k, tc;
            fid >> k >> tc;
            beginColumn(k);
            for (int cnt2 = 0; cnt2 < tc; ++cnt2) {
               int trip;
               fid >> trip;
               addTrip(trip);
            }
            commitColumn();
         }
      }
   }
}

CgMasterCplex::~CgMasterCplex() {
   if (0) {
      ofstream fid("colcache.txt");
      fid << m_pathTrips.size() << "\n";
      for (size_t i = 0; i < m_pathTrips.size(); ++i) {
         fid << m_pathDepot[i] << " " << m_pathTrips[i].size() << "\n";
         for (int v: m_pathTrips[i])
         fid << v << "\n";
      }
   }
   m_env.end();
}

auto CgMasterCplex::writeLp(const char *fname) const noexcept -> void {
   m_cplex.exportModel(fname);
}

auto CgMasterCplex::solve() noexcept -> double {
   if (!m_cplex.solve()) {
      cout << "RMP became infeasible.\n";
      writeLp("rrmp_problematic.lp");
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

auto CgMasterCplex::beginColumn(int depotId) noexcept -> void {
   m_coefs.add(make_pair(depotId + m_inst->numTrips(), 1.0));
   m_newcolDepotId = depotId;
   m_lastTrip = -1;
   m_newcolCost = 0.0;

   m_pathDepot.push_back(depotId);
   m_pathTrips.push_back({});
}

auto CgMasterCplex::addTrip(int trip) noexcept -> void {
   m_coefs.add(make_pair(trip, 1.0));
   if (m_lastTrip == -1) {
      m_newcolCost += m_inst->sourceCost(m_newcolDepotId, trip);
   } else {
      m_newcolCost += m_inst->deadheadCost(m_lastTrip, trip);
   }
   m_lastTrip = trip;
   m_pathTrips.back().push_back(trip);
}

auto CgMasterCplex::commitColumn() noexcept -> void {
   m_newcolCost += m_inst->sinkCost(m_newcolDepotId, m_lastTrip);
   char buf[128];
   snprintf(buf, sizeof buf, "path#%d#%ld", m_newcolDepotId, m_paths.getSize());
   addColumn(m_newcolCost).setName(buf);
}

auto CgMasterCplex::numColumns() const noexcept -> int {
   return m_paths.getSize();
}

auto CgMasterCplex::setAssignmentType(char sense) noexcept -> void {
   assert(sense == 'E' or sense == 'G');
   auto ub = sense == 'E' ? 1.0 : IloInfinity;
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      m_range[i].setBounds(1.0, ub);
   }
}

auto CgMasterCplex::addColumn(double cost) noexcept -> IloNumVar {
   IloNumColumn col = m_obj(cost);
   for (IloInt i = 0; i < m_coefs.getSize(); i++) {
      const auto p = m_coefs[i];
      col += m_range[p.first](p.second);
   }
   
   IloNumVar path = IloNumVar(col, 0.0, IloInfinity, IloNumVar::Float, nullptr);
   m_paths.add(path);

   m_coefs.clear();   
   col.end();

   return path;
}
