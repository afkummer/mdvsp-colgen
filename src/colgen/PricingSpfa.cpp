#include "PricingSpfa.h"

#include <iostream>
#include <numeric>

using namespace std;

PricingSpfa::PricingSpfa(const Instance &inst, CgMasterInterface &dp, int depotId): 
m_inst(&inst), m_dp(&dp), m_depotId(depotId) {
   // m_cpx = make_unique<PricingCplex>(inst, dp, depotId);
}

PricingSpfa::~PricingSpfa() {
   // Empty
}

auto PricingSpfa::depotId() const noexcept -> int {
   return m_depotId;
}

auto PricingSpfa::solve() noexcept -> double {
   // Some initial definitions that help understanding the algorithm.
   const auto N = m_inst->numTrips() + 2;  // Number of nodes
   const auto O = N - 2;                   // Source node
   const auto D = N - 1;                   // Sink node

   // Puts data structures to initial state
   m_dist.resize(N);
   fill(m_dist.begin(), m_dist.end(), numeric_limits<double>::infinity());
   m_pred.resize(N);
   fill(m_pred.begin(), m_pred.end(), -1);

   // cout << "\n\nRelaxing for depot " << m_depotId << "\n";
   // cout << "O = " << O << " D = " << D << endl;

   // Set initial state of the data structures.
   // Formally speaking, we would add the source node to then expand the
   // graph using some kind of BFS algorithm. Instead, we know some things
   // about the graph so we can tweak the algorithm for our use case.
   const auto depotDual = m_dp->getDepotCapDual(m_depotId);
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         m_dist[i] = double(cost) - depotDual;
         m_pred[i] = O;
         // cout << "Source relaxing to task " << i << " with pred = " << m_pred[i] << " and dist = " << m_dist[i] << endl;
      }
   }  

   auto doRelaxation = [&] () -> bool {
      bool changed = false;
      // cout << "Starting a new relaxation!\n";

      for (int i = 0; i < m_inst->numTrips(); ++i) {
         const auto iDual = m_dp->getTripDual(i);

         for (auto &p: m_inst->deadheadSuccAdj(i)) {
            int to = p.first;
            double len = double(p.second) - iDual;

            if (m_dist[i] + len < m_dist[to]) {
               // cout << "   Updating destination = " << to << "\n";
               // cout << "       Previous arc: " << m_pred[to] << " -> " << to << " with distance = " << m_dist[to] << "\n";
               // cout << "       New arc: " << i << " -> " << to << " with distance = " << m_dist[i] + len << " (dist[" << i << "] = " << m_dist[i] << ", len=" << len << ")\n";
               m_dist[to] = m_dist[i] + len;
               m_pred[to] = i;
               changed = true;
            }
         }

         if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
            double len = cost - iDual;
            if (m_dist[i] + len < m_dist[D]) {
               m_dist[D] = m_dist[i] + len;
               m_pred[D] = i;
               changed = true;
            }
         }
      }

      return changed;
   };

   for (int rep = 0; rep < N; ++rep) {
      if (!doRelaxation()) {
         // cout << "BF for depot#" << m_depotId << " run for " << rep << " iterations.\n";
         break;
      }
   }

   if (doRelaxation()) {
      cout << "Negative cycle detected.\n";
      abort();
   }

   // m_cpx->solve();
   // double diff = m_cpx->getObjValue() - m_dist[D];
   // #pragma omp critical
   // cout << "Depot " << m_depotId << " with BF: " << m_dist[D] << " and PricingCplex: " << m_cpx->getObjValue() << " DIFF: " << diff << "\n";

   #pragma omp critical
   cout << "Algo for depot " << m_depotId << " is PricingSPFA.\n";
   return m_dist[D];   
}

auto PricingSpfa::getObjValue() const noexcept -> double {
   return m_dist[m_inst->numTrips()+1];
}

auto PricingSpfa::generateColumns() const noexcept -> void {
   if (1) {
   vector<vector<int>> allPaths;

      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
            vector<int> path = {i};
            double pcost = double(cost) - m_dp->getTripDual(i);
            getPathRecursive(path, pcost, allPaths);
         }
      }
   } else {

      const auto N = m_inst->numTrips() + 2;
      const auto O = N - 2;
      const auto D = N - 1;

      vector <int> path;

      int last = m_pred[D];
      while (last != O) {
         path.push_back(last);
         last = m_pred[last];
      }
      
      m_dp->beginColumn(m_depotId);
      for (auto it = path.rbegin(); it != path.rend(); ++it) {
         m_dp->addTrip(*it);
      }
      m_dp->commitColumn();
   }
}

auto PricingSpfa::getPathRecursive(std::vector<int> &path, double &pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

   int pred = m_pred[path.back()];
   if (pred == O) {
      assert(m_inst->sourceCost(m_depotId, path.back()) != -1);
      double cst = pcost + (m_inst->sourceCost(m_depotId, path.back()) - m_dp->getDepotCapDual(m_depotId));
      if (cst <= -0.001) {
         // cout << "Pricing " << m_depotId << " with path value of " << cst << ": ";
         m_dp->beginColumn(m_depotId);
         for (auto it = path.rbegin(); it != path.rend(); ++it) {
            // cout << i << " ";
            m_dp->addTrip(*it);
         }
         // cout << "\n";
         m_dp->commitColumn();
      }
      return;
   }

   assert(m_inst->deadheadCost(pred, path.back()) != -1);
   double cst = pcost + (m_inst->deadheadCost(pred, path.back()) - m_dp->getTripDual(pred));
   path.push_back(pred);
   getPathRecursive(path, cst, allPaths);
   path.pop_back();
}
