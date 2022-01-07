#include "CgPricingBase.h"
#include "CgMasterBase.h"

#include "Instance.h"
#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;

CgPricingBase::CgPricingBase(const Instance &inst, CgMasterBase &master, const int depotId, const int maxPaths): 
   m_inst(&inst), m_depotId(depotId), m_master(&master), m_maxPaths(maxPaths) {
   // Empty
}

CgPricingBase::~CgPricingBase() {
   // Empty
}

auto CgPricingBase::depotId() const noexcept -> int {
   return m_depotId;
}

// auto CgPricingBase::generateColumns() const noexcept -> int {
//    vector<vector<int>> allPaths;
//    vector<int> path;
//    int colCount = 0;
//    if (m_maxPaths == 1) {
//       double cost = 0.0;
//       getSinglePath(path, cost);
//       if (cost <= -0.001) {
//          allPaths.push_back(path);
//       }      
//    } else {
//       for (int i = 0; i < m_inst->numTrips(); ++i) {
//          if (isArcSet(i, sinkNode())) {
//             path.push_back(i);
//             assert(m_inst->sinkCost(m_depotId, i) != -1);

//             double pcost = m_inst->sinkCost(m_depotId, i) - m_master->getTripDual(i);
//             getPathRecursive(path, pcost, allPaths);
//             path.clear();
//          }
//       }
//    }

//    for (const auto &p: allPaths) {
//       m_master->beginColumn(m_depotId);
//       for (auto it = p.rbegin(); it != p.rend(); ++it) {
//          m_master->addTrip(*it);
//       }
//       m_master->commitColumn();
//       ++colCount;
//    }
//    return colCount;
// }

auto CgPricingBase::numNodes() const noexcept -> int {
   return m_inst->numTrips()+2;
}

auto CgPricingBase::sourceNode() const noexcept -> int {
   return numNodes() - 2;
}

auto CgPricingBase::sinkNode() const noexcept -> int {
   return numNodes() - 1;
}

// auto CgPricingBase::getSinglePath(std::vector<int> &path, double &pcost) const noexcept -> void {
//    // Looks at sink arcs to check if there is one active.
//    int last = -1;
//    for (int i = 0; i < m_inst->numTrips(); ++i) {
//       if (isArcSet(i, sinkNode())) {
//          last = i;
//          break;
//       }
//    }
//    assert(last != -1 && "Pricing seems to contain no path.");
//    assert(m_inst->sinkCost(m_depotId, last) != -1);
//    pcost = m_inst->sinkCost(m_depotId, last) - m_master->getTripDual(last);
//    path.push_back(last);
   
//    while (!isArcSet(sourceNode(), last)) {
//       int i = -1;
//       int icost = -1;
//       for (const auto &p: m_inst->deadheadPredAdj(last)) {
//          if (isArcSet(p.first, last)) {
//             i = p.first;
//             icost = p.second;
//             break;
//          }
//       }
//       assert(i != -1 && "Path structure is broken.");
//       assert(icost != -1); // Obviously true...
      
//       pcost += icost - m_master->getTripDual(i);
//       path.push_back(i);
      
//       last = i;
//    }

//    // Tries to "finish" the path by connecting to the source node.
//    if (isArcSet(sourceNode(), last)) {
//       assert(m_inst->sourceCost(m_depotId, last) != 1);
//       pcost += m_inst->sourceCost(m_depotId, last) - m_master->getDepotCapDual(m_depotId);
//    } else {
//       assert(false && "Path does not connects to source node.");
//    }
// }

// auto CgPricingBase::getPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   
// }
