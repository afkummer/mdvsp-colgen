#include "PricingSpfa.h"
#include "CgMasterBase.h"
#include "Instance.h"

#include <iostream>
#include <numeric>
#include <queue>

using namespace std;

PricingSpfa::PricingSpfa(const Instance &inst, CgMasterBase &master, int depotId, bool singlePath): 
CgPricingBase(inst, master, depotId, singlePath ? 1: 999999) {

   m_dist.resize(numNodes());
   m_pred.resize(numNodes());

}

PricingSpfa::~PricingSpfa() {
   // Empty
}

auto PricingSpfa::getSolverName() const noexcept -> std::string {
   return "Shortest Path Faster Algorithm with negative cycle detection";
}

auto PricingSpfa::writeLp(const char *fname) const noexcept -> void {
   (void) fname;
   cout << "WARNING: SPFA pricing does not support writing LP files. Command ignored\n";
}

auto PricingSpfa::isExact() const noexcept -> bool {
   return true;
}

auto PricingSpfa::solve() noexcept -> double {
   // Some initial definitions that help understanding the algorithm.
   const auto O = sourceNode();
   const auto D = sinkNode();

   // Puts data structures to initial state.
   fill(m_dist.begin(), m_dist.end(), numeric_limits<double>::infinity());
   fill(m_pred.begin(), m_pred.end(), -1);
   vector<int> cnt(numNodes(), 0);
   vector <char> inqueue(numNodes(), false);
   queue<int> qu;
   // priority_queue <pair<int, double>> qu;

   // Set initial state of the data structures.
   // Formally speaking, we would add the source node to then expand the
   // graph using some kind of BFS algorithm. Instead, we know some things
   // about the graph so we can tweak the algorithm for our use case.
   const auto depotDual = m_master->getDepotCapDual(m_depotId);
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         m_dist[i] = double(cost) - depotDual;
         m_pred[i] = O;
         cnt[i]++;
         inqueue[i] = true;
         qu.push(i);
         // qu.push(make_pair(i, m_dist[i]));
      }
   }

   while (!qu.empty()) {
      int v = qu.front();
      // auto [v, _] = qu.top();
      qu.pop();
      inqueue[v] = false;
      const auto iDual = m_master->getTripDual(v);

      int numExpansions = m_maxLabelExpansions;
      for (auto &p : m_inst->deadheadSuccAdj(v)) {
         int to = p.first;
         double len = double(p.second) - iDual;

         if (m_dist[v] + len < m_dist[to]) {
            m_dist[to] = m_dist[v] + len;
            m_pred[to] = v;
            if (!inqueue[to]) {
               qu.push(to);
               // qu.push(make_pair(to, m_dist[to]));
               inqueue[to] = true;
               cnt[to]++;
               if (cnt[to] > numNodes()) {
                  cout << "Negative cycle detected.\n";
                  abort();
               }
            }
            if (--numExpansions == 0)
               break;
         }
      }

      if (auto cost = m_inst->sinkCost(m_depotId, v); cost != -1) {
         double len = cost - iDual;
         if (m_dist[v] + len < m_dist[D]) {
            m_dist[D] = m_dist[v] + len;
            m_pred[D] = v;
         }
      }
   }

   return m_dist[D];   
}

auto PricingSpfa::getObjValue() const noexcept -> double {
   return m_dist[sinkNode()];
}

auto PricingSpfa::generateColumns() const noexcept -> int {
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

auto PricingSpfa::findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   const auto O = sourceNode();

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
