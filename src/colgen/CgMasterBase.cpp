#include "CgMasterBase.h"

#include "Instance.h"
#include <cassert>

using namespace std;

CgMasterBase::CgMasterBase(const Instance &inst): m_inst(&inst) {
   // Empty
}

CgMasterBase::~CgMasterBase() {
   // Empty
}

auto CgMasterBase::beginColumn(int depotId) noexcept -> void {
   assert(depotId >= 0 && depotId < m_inst->numDepots());
   m_newcolDepot = depotId;
   m_newcolCost = 0.0;
   m_newcolLastTrip = -1;
   m_newcolPath.clear();
}

auto CgMasterBase::addTrip(int trip) noexcept -> void {
   assert(trip >= 0 && trip < m_inst->numTrips());
   if (m_newcolLastTrip == -1) {
      assert(m_inst->sourceCost(m_newcolDepot, trip) != -1);
      m_newcolCost += m_inst->sourceCost(m_newcolDepot, trip);
   } else {
      assert(m_inst->deadheadCost(m_newcolLastTrip, trip) != -1);
      m_newcolCost += m_inst->deadheadCost(m_newcolLastTrip, trip);
   }
   m_newcolPath.push_back(trip);
   m_newcolLastTrip = trip;
}

auto CgMasterBase::commitColumn() noexcept -> void {
   assert(m_newcolLastTrip != -1);
   assert(m_inst->sinkCost(m_newcolDepot, m_newcolLastTrip) != -1);
   m_newcolCost += m_inst->sinkCost(m_newcolDepot, m_newcolLastTrip);
   addColumn();
   ++m_numCols;
}

auto CgMasterBase::numColumns() const noexcept -> int {
   return m_numCols;
}
