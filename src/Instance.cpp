#include "Instance.h"

#include <cassert>
#include <fstream>
#include <iostream>

using namespace std;

Instance::Instance(const char *fname): m_fname{fname} {
   // RAII style: tries to read the instance data in the constructor.
   ifstream fid(fname);
   if (!fid) {
      cerr << "Instance " << fname << " could not be read.\n";
      exit(EXIT_FAILURE);
   }

   // Reads the header. It contains the instance size, and depot capacity.
   fid >> m_numDepots >> m_numTrips;
   m_depotCap.resize(m_numDepots);
   for (auto &cap: m_depotCap)
      fid >> cap;

   // Prepares to read the connection matrix.
   const auto L = m_numDepots + m_numTrips;
   m_matrix.resize(boost::extents[L][L]);
   
   // Reads the entire matrix.
   for (int i = 0; i < L; ++i) {
      for (int j = 0; j < L; ++j) {
         fid >> m_matrix[i][j];
      }
   }

   // Prepares the deadheading cache.
   m_dhCacheSucc.resize(numTrips());
   m_dhCachePred.resize(numTrips());
   for (int i = 0; i < numTrips(); ++i) {
      auto &vecSucc = m_dhCacheSucc[i];
      auto &vecPred = m_dhCachePred[i];
      for (int j = 0; j < numTrips(); ++j) {
         if (auto cost = deadheadCost(i, j); cost != -1) {
            vecSucc.emplace_back(make_pair(j, cost));
         }
         if (auto cost = deadheadCost(j, i); cost != -1) {
            vecPred.emplace_back(make_pair(j, cost));
         }
      }
      vecSucc.shrink_to_fit();
      vecPred.shrink_to_fit();
   }
}

Instance::~Instance() {
   // Empty
}

auto Instance::fileName() const noexcept -> const std::string & {
   return m_fname;
}

auto Instance::numDepots() const noexcept -> int {
   return m_numDepots;
}

auto Instance::numTrips() const noexcept -> int {
   // return 200;
   return m_numTrips;
}

auto Instance::depotCapacity(int k) const noexcept -> int {
   assert(k >= 0 && k < m_numDepots);
   return m_depotCap[k];
}

auto Instance::sourceCost(int k, int trip) const noexcept -> int {
   assert(k >= 0 && k < m_numDepots);
   assert(trip >= 0 && trip < m_numTrips);
   return m_matrix[k][trip + m_numDepots];
}

auto Instance::sinkCost(int k, int trip) const noexcept -> int {
   assert(k >= 0 && k < m_numDepots);
   assert(trip >= 0 && trip < m_numTrips);
   return m_matrix[trip + m_numDepots][k];
}

auto Instance::deadheadCost(int pred, int succ) const noexcept -> int {
   assert(pred >= 0 && pred < m_numTrips);
   assert(succ >= 0 && succ < m_numTrips);
   return m_matrix[m_numDepots+pred][m_numDepots+succ];
}

auto Instance::rawCost(int i, int j) const noexcept -> int {
   assert(i >= 0 && i < m_numDepots + m_numTrips);
   assert(j >= 0 && j < m_numDepots + m_numTrips);
   return m_matrix[i][j];
}

auto Instance::deadheadSuccAdj(int pred) const noexcept -> const std::vector<std::pair<int, int>> & {
   assert(pred >= 0 && pred < m_numTrips);
   return m_dhCacheSucc[pred];
}

auto Instance::deadheadPredAdj(int succ) const noexcept -> const std::vector<std::pair<int, int>> & {
   assert(succ >= 0 && succ < m_numTrips);
   return m_dhCachePred[succ];
}