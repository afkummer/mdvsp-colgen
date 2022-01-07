#include "PricingBellman.h"
#include "CgMasterBase.h"
#include "Instance.h"

#include <iostream>
#include <numeric>

using namespace std;

PricingBellman::PricingBellman(const Instance &inst, CgMasterBase &master, int depotId, bool singlePath): 
CgPricingBase(inst, master, depotId, singlePath ? 1: 999999) {

   m_dist.resize(numNodes());
   m_pred.resize(numNodes());
}

PricingBellman::~PricingBellman() {
   // Empty
}

auto PricingBellman::getSolverName() const noexcept -> std::string {
   return "Bellman-Ford with negative cycle detection";
}

auto PricingBellman::writeLp(const char *fname) const noexcept -> void {
   cout << "WARNING: Bellman-Ford pricing does not support writing LP files. Command ignored\n";
}

auto PricingBellman::isExact() const noexcept -> bool {
   return true;
}

auto PricingBellman::solve() noexcept -> double {
   // Some initial definitions that help understanding the algorithm.
   const auto N = numNodes();
   const auto O = sourceNode();
   const auto D = sinkNode();

   // Puts data structures to initial state.
   fill(m_dist.begin(), m_dist.end(), numeric_limits<double>::infinity());
   fill(m_pred.begin(), m_pred.end(), -1);

   // Set initial state of the data structures.
   // Formally speaking, we would add the source node to then expand the
   // graph using some kind of BFS algorithm. Instead, we know some things
   // about the graph so we can tweak the algorithm for our use case.
   const auto depotDual = m_master->getDepotCapDual(m_depotId);
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         m_dist[i] = double(cost) - depotDual;
         m_pred[i] = O;
      }
   }  

   auto doRelaxation = [&] () -> bool {
      bool changed = false;

      for (int i = 0; i < m_inst->numTrips(); ++i) {
         const auto iDual = m_master->getTripDual(i);

         for (auto &p: m_inst->deadheadSuccAdj(i)) {
            int to = p.first;
            double len = double(p.second) - iDual;

            if (m_dist[i] + len < m_dist[to]) {
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
         break;
      }
   }

   if (doRelaxation()) {
      cout << "Negative cycle detected.\n";
      abort();
   }

   return m_dist[D];   
}

auto PricingBellman::getObjValue() const noexcept -> double {
   return m_dist[sinkNode()];
}

auto PricingBellman::generateColumns() const noexcept -> int {
   const auto N = numNodes();
   const auto O = sourceNode();
   const auto D = sinkNode();
   
   vector<vector<int>> allPaths;

   if (m_maxPaths == 1) {
      vector<int> path { m_pred[D] };
      assert(m_inst->sinkCost(m_depotId, path.back()) != -1);
      double cost = m_inst->sinkCost(m_depotId, path.back()) - m_master->getTripDual(path.back());
      findPathRecursive(path, cost, allPaths);
   } else {
      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
            vector<int> path = {i};
            double pcost = double(cost) - m_master->getTripDual(i);
            findPathRecursive(path, pcost, allPaths);
         }
      }
   }

   for (const auto &p: allPaths) {
      m_master->beginColumn(m_depotId);
      for (auto it = p.rbegin(); it != p.rend(); ++it) {
         m_master->addTrip(*it);
      }
      m_master->commitColumn();
   }

   return allPaths.size();
}

auto PricingBellman::findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   const auto N = numNodes();
   const auto O = sourceNode();
   const auto D = sinkNode();

   int pred = m_pred[path.back()];
   if (pred == O) {
      assert(m_inst->sourceCost(m_depotId, path.back()));
      double cst = pcost + (m_inst->sourceCost(m_depotId, path.back()) - m_master->getDepotCapDual(m_depotId));
      if (cst <= -0.001) {
         allPaths.push_back(path);
      }
      return;
   }

   assert(m_inst->deadheadCost(pred, path.back()) != -1);
   double cst = pcost + (m_inst->deadheadCost(pred, path.back()) - m_master->getTripDual(pred));
   path.push_back(pred);
   findPathRecursive(path, cst, allPaths);
   path.pop_back();
}
