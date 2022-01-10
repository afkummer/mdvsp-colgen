#include "CgMasterBase.h"

#include "Instance.h"
#include <cassert>
#include <fstream>

using namespace std;

CgMasterBase::CgMasterBase(const Instance &inst): m_inst(&inst) {
   m_newcolDepot = -1;
   m_newcolCost = std::numeric_limits<double>::infinity();
   m_newcolLastTrip = -1;
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

   m_colDepot.push_back(m_newcolDepot);
   m_colTrips.push_back(m_newcolPath);
}

auto CgMasterBase::numColumns() const noexcept -> int {
   return m_numCols;
}

auto CgMasterBase::exportColumns(const char *fname) const noexcept -> void {
   ofstream fid(fname);
   if (!fid) abort();

   fid << m_colDepot.size() << "\n";
   for (size_t i = 0; i < m_colDepot.size(); ++i) {
      fid << m_colDepot[i] << " " << m_colTrips[i].size() << "\n";
      for (int i: m_colTrips[i])
         fid << i << "\n";
   }
}

auto CgMasterBase::importColumns(const char *fname) noexcept -> int {
   ifstream fid(fname);
   if (!fid) abort();

   int nc;
   fid >> nc;
   for (int i = 0; i < nc; ++i) {
      int depot, nt;
      fid >> depot >> nt;
      beginColumn(depot);
      for (int j = 0; j < nt; ++j) {
         int t;
         fid >> t;
         addTrip(t);
      }
      commitColumn();
   }
   
   return nc;
}