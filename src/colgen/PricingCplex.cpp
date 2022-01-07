#include "PricingCplex.h"
#include "Instance.h"
#include "CgMasterBase.h"

#include <iostream>
#include <cassert>

using namespace std;

PricingCplex::PricingCplex(const Instance &inst, CgMasterBase &master, const int depotId, int maxPaths): CgPricingBase(inst, master, depotId, maxPaths) {
   char buf[128];
   const auto N = numNodes();
   const auto O = sourceNode();
   const auto D = sinkNode();

   snprintf(buf, sizeof buf, "mdvsp_pricing_cplex#%d", m_depotId);
   m_model = IloModel(m_env, buf);
   m_cplex = IloCplex(m_model);

   IloExpr expr{m_env};
   m_x = IloArray<IloNumVarArray>(m_env, N);
   for (int i = 0; i < N; ++i)
      m_x[i] = IloNumVarArray(m_env, N);

   const auto VarTy = IloNumVar::Bool;
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // Creates source arcs.
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "source#%d#%d", m_depotId, i);
         m_x[O][i] = IloNumVar(m_env, 0.0, 1.0, VarTy, buf);
         expr += cost * m_x[O][i];
      }

      // Creates sink arcs.
      if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "sink#%d#%d", m_depotId, i);
         m_x[i][D] = IloNumVar(m_env, 0.0, 1.0, VarTy, buf);
         expr += cost * m_x[i][D];
      }

      // Adds all deadheading arcs.
      for (auto &p: m_inst->deadheadSuccAdj(i)) {
         snprintf(buf, sizeof buf, "deadhead#%d#%d", i, p.first);
         m_x[i][p.first] = IloNumVar(m_env, 0.0, 1.0, VarTy, buf);
         expr += p.second * m_x[i][p.first];
      }
   }

   m_obj = IloObjective(m_env, expr, IloObjective::Minimize, "shortest_path");
   expr.clear();

   // Adds the flow conservation constraints.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto col = m_x[O][i]; col.getImpl()) {
         expr += col;
      }

      if (auto col = m_x[i][D]; col.getImpl()) {
         expr -= col;
      }

      for (auto &p: m_inst->deadheadPredAdj(i)) {
         assert(p.first != -1);
         expr += m_x[p.first][i];
      }

      for (auto &p: m_inst->deadheadSuccAdj(i)) {
         assert(p.first != -1);
         expr -= m_x[i][p.first];
      }

      snprintf(buf, sizeof buf, "flow_consevation#%d", i);
      IloConstraint c = expr == 0;
      c.setName(buf);
      m_model.add(c);
      expr.clear();
   }

   // Optional constraint to force a single path.
   if (maxPaths >= 1) {
      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto col = m_x[O][i]; col.getImpl()) {
            expr += col;
         }
      }

      snprintf(buf, sizeof buf, "max_paths#%d", m_depotId);
      IloConstraint c = expr <= maxPaths;
      c.setName(buf);
      m_model.add(c);
      expr.clear();
   }

   expr.end();

   m_env.setOut(m_env.getNullStream());
   m_env.setWarning(m_env.getNullStream());
   m_cplex.setOut(m_env.getNullStream());
   m_cplex.setWarning(m_env.getNullStream());
}

PricingCplex::~PricingCplex() {
   m_env.end();
}

auto PricingCplex::getSolverName() const noexcept -> std::string {
   return "CPLEX " + to_string(CPX_VERSION);
}

auto PricingCplex::writeLp(const char *fname) const noexcept -> void {
   m_cplex.exportModel(fname);
}

auto PricingCplex::isExact() const noexcept -> bool {
   return true;
}

auto PricingCplex::solve() noexcept -> double {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

   IloExpr expr{m_env};
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // Creates source arcs.
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         expr += (cost - m_master->getDepotCapDual(m_depotId)) * m_x[O][i];
      }

      // Creates sink arcs.
      if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
         expr += (cost - m_master->getTripDual(i)) * m_x[i][D];
      }

      // Adds all deadheading arcs.
      for (auto &p: m_inst->deadheadSuccAdj(i)) {
         assert(m_x[i][p.first].getImpl());
         expr += (p.second - m_master->getTripDual(i)) * m_x[i][p.first];
      }
   }

   m_obj.end();
   m_obj = IloObjective(m_env, expr, IloObjective::Minimize, "shortest_path");
   m_model.add(m_obj);
   expr.end();

   if (!m_cplex.solve()) abort();
   return m_cplex.getObjValue();
}

auto PricingCplex::getObjValue() const noexcept -> double {
   return m_cplex.getObjValue();
}

auto PricingCplex::generateColumns() const noexcept -> int {
   vector<vector<int>> allPaths;

   // I think the algorithm can be accelerated by skipping the calculation 
   // of reduced cost per path. 

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto col = m_x[sourceNode()][i]; col.getImpl() && m_cplex.getValue(col) >= 0.98) {
         vector<int> path = {i};
         double pcost = m_inst->sourceCost(m_depotId, i) - m_master->getDepotCapDual(m_depotId);

         findPathRecursive(path, pcost, allPaths);
      }
   }

   for (const auto &p: allPaths) {
      m_master->beginColumn(m_depotId);
      for (auto it = p.begin(); it != p.end(); ++it) {
         m_master->addTrip(*it);
      }
      m_master->commitColumn();
   }

   return allPaths.size();
}

auto PricingCplex::findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   if (auto col = m_x[path.back()][sinkNode()]; col.getImpl() && m_cplex.getValue(col) >= 0.98) {
      double cst = pcost + (m_inst->sinkCost(m_depotId, path.back()) - m_master->getTripDual(path.back()));
      if (cst <= -0.001) {
         allPaths.push_back(path);
      }
   }

   for (const auto &p: m_inst->deadheadSuccAdj(path.back())) {
      if (auto col = m_x[path.back()][p.first]; col.getImpl() && m_cplex.getValue(col) >= 0.98) {
         double cst = pcost + (p.second - m_master->getTripDual(path.back()));
         path.push_back(p.first);
         findPathRecursive(path, cst, allPaths);
         path.pop_back();
      }
   }
}

// auto PricingCplex::getPathRecursive(std::vector<int> &path, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
//    const auto N = m_inst->numTrips() + 2;
//    const auto O = N - 2;
//    const auto D = N - 1;

//    if (path.back() == O) {
//       for (int i = 0; i < m_inst->numTrips(); ++i) {
//          if (auto col = m_x[O][i]; col.getImpl() and m_cplex.getValue(col) >= 0.5) {
//             path.push_back(i);
//             getPathRecursive(path, allPaths);
//             path.pop_back();
//          }
//       }
//    } else {

//       if (auto col = m_x[path.back()][D]; col.getImpl() && m_cplex.getValue(col) >= 0.5) {
//          path.push_back(D);
//          allPaths.push_back(path);
//          path.pop_back();
//       }

//       for (auto &p: m_inst->deadheadSuccAdj(path.back())) {
//          if (auto col = m_x[path.back()][p.first]; m_cplex.getValue(col) >= 0.5) {
//             path.push_back(p.first);
//             getPathRecursive(path, allPaths);
//             path.pop_back();
//          }
//       }
//    }
// }
