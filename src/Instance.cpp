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
   return m_numTrips;
}

auto Instance::depotCapacity(int k) const noexcept -> int {
   assert(k >= 0 && k < m_numDepots);
   return m_depotCap[k];
}

auto Instance::sourceCost(int k, int trip) const noexcept -> int {
   assert(k >= 0 && k < m_numDepots);
   assert(trip >= 0 && trip < m_numTrips);
   return m_matrix[k][trip];
}

auto Instance::sinkCost(int k, int trip) const noexcept -> int {
   assert(k >= 0 && k < m_numDepots);
   assert(trip >= 0 && trip < m_numTrips);
   return m_matrix[trip][k];
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
