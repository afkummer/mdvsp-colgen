#include "PricingCplex.h"

#include <iostream>

using namespace std;

PricingCplex::PricingCplex(const Instance &inst, CgMasterInterface &master, int depotId): m_inst(&inst), m_depotId(depotId), m_master(&master) {
   char buf[128];
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

   snprintf(buf, sizeof buf, "mdvsp_pricing_cplex#%d", m_depotId);
   m_model = IloModel(m_env, buf);
   m_cplex = IloCplex(m_model);

   IloExpr expr{m_env};
   m_x = IloArray<IloNumVarArray>(m_env, N);
   for (int i = 0; i < N; ++i)
      m_x[i] = IloNumVarArray(m_env, N);

   const auto VarTy = IloNumVar::Float;
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
   if (1) {
      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto col = m_x[O][i]; col.getImpl()) {
            expr += col;
         }
      }

      snprintf(buf, sizeof buf, "single_path#%d", m_depotId);
      IloConstraint c = expr <= 5;
      c.setName(buf);
      m_model.add(c);
      expr.clear();
   }

   expr.end();

   m_cplex.setParam(IloCplex::IntParam::Threads, 1);
   m_cplex.setParam(IloCplex::IntParam::MIPDisplay, 0);
   m_cplex.setParam(IloCplex::IntParam::SimDisplay, 0);
   m_cplex.setParam(IloCplex::IntParam::BarDisplay, 0);
   m_env.setOut(m_env.getNullStream());
   m_env.setWarning(m_env.getNullStream());
   m_cplex.setOut(m_env.getNullStream());
   m_cplex.setWarning(m_env.getNullStream());
}

PricingCplex::~PricingCplex() {
   m_env.end();
}

auto PricingCplex::writeLp(const char *fname) const noexcept -> void {
   m_cplex.exportModel(fname);
}

auto PricingCplex::depotId() const noexcept -> int {
   return m_depotId;
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

auto PricingCplex::generateColumns() const noexcept -> void {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

#if 0
   // Very simple approach to generate columns.
   // If more than a single path exists in the solution and they crosspasses,
   // then this algorithm does not guarantee anything about the sequence of the trips.
   // An recursive algorithm could help, but these are hard to devise and can cause
   // a expressive growth in number of columns of the RMP.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // Check if there is an active arc from source to `i`.
      if (auto col = m_x[O][i]; col.getImpl() && m_cplex.getValue(col) >= 0.01) {
         // There is flow. Begins a new column in the RMP!
         m_master->beginColumn(m_depotId);
         m_master->addTrip(i);
         int lastTrip = i;
         // cout << "BEGIN PATH " << m_depotId << " -> " << i << "\n";

         // Walks the entire path until achieving some sink arc.

         for (int count = 0;; ++count) {
            int nextTrip = -1;
            for (int j = 0; j < m_inst->numTrips(); ++j) {
               if (auto col = m_x[lastTrip][j]; col.getImpl() && m_cplex.getValue(col) >= 0.5) {
                  nextTrip = j;
                  break;
               }
            }

            if (nextTrip != -1) {
               // Found the next one. Adds to the path and continues.
               m_master->addTrip(nextTrip);
               // cout << "   Adds " << nextTrip << "\n";
               lastTrip = nextTrip;
            } else {
               // Apparently arrived at the end of the path.
               // Check if the sink arc is connected or report an flow violation.
               if (auto col = m_x[lastTrip][D]; col.getImpl() && m_cplex.getValue(col) >= 0.5) {
                  // Everything ok. Commits the new column and proceed.
                  m_master->commitColumn();
                  // cout << "END PATH " << lastTrip << " -> " << m_depotId << "\n";
                  break;
               } else {
                  // Error in the path.
                  cout << "Error in the path! Check problematic file 'flow.lp' (depot " << m_depotId << ").\n";
                  exit(EXIT_FAILURE);
               }
            }

            if (count >= N) {
               cout << "Seems like the algorithm entered an infinite loop.\n";
            }
         }
      }
   }
#else
   vector <int> path = {O};
   vector<vector<int>> allPaths;
   getPathRecursive(path, allPaths);
   cout << "   Pricing #" << m_depotId << " with " << allPaths.size() << " new columns.\n";
   for (auto &p: allPaths) {
      assert(p.size() > 2);
      m_master->beginColumn(m_depotId);
      for (size_t pos = 1; pos < p.size()-1; ++pos) {
         m_master->addTrip(p[pos]);
      }
      m_master->commitColumn();
   }
#endif
}

auto PricingCplex::getPathRecursive(std::vector<int> &path, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

   if (path.back() == O) {
      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto col = m_x[O][i]; col.getImpl() and m_cplex.getValue(col) >= 0.5) {
            path.push_back(i);
            getPathRecursive(path, allPaths);
            path.pop_back();
         }
      }
   } else {

      if (auto col = m_x[path.back()][D]; col.getImpl() && m_cplex.getValue(col) >= 0.5) {
         path.push_back(D);
         allPaths.push_back(path);
         path.pop_back();
      }

      for (auto &p: m_inst->deadheadSuccAdj(path.back())) {
         if (auto col = m_x[path.back()][p.first]; m_cplex.getValue(col) >= 0.5) {
            path.push_back(p.first);
            getPathRecursive(path, allPaths);
            path.pop_back();
         }
      }
   }
}
