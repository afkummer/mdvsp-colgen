#include "CgPricingBase.h"
#include "CgMasterBase.h"

#include "Instance.h"
#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;

CgPricingBase::CgPricingBase(const Instance &inst, CgMasterBase &master, const int depotId, const int maxPaths): 
   m_inst(&inst), m_depotId(depotId), m_master(&master), m_maxPaths(maxPaths), m_maxLabelExpansions(std::numeric_limits<int>::max()) {
}

CgPricingBase::~CgPricingBase() {
   // Empty
}

auto CgPricingBase::depotId() const noexcept -> int {
   return m_depotId;
}

void CgPricingBase::setMaxLabelExpansionsPerNode(int maxExpansions) {
   const auto maxVal = std::numeric_limits<int>::max();
   if (maxExpansions <= 0) {
      cout << "WARNING: MaxExpansions = " << maxExpansions << ", setting it to " << maxVal << "\n";
      m_maxLabelExpansions = maxVal;
   } else {
      m_maxLabelExpansions = maxExpansions;
   }
}
auto CgPricingBase::numNodes() const noexcept -> int {
   return m_inst->numTrips()+2;
}

auto CgPricingBase::sourceNode() const noexcept -> int {
   return numNodes() - 2;
}

auto CgPricingBase::sinkNode() const noexcept -> int {
   return numNodes() - 1;
}

