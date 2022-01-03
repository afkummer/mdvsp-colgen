#pragma once

#include <string>
#include <vector>

#include <boost/multi_array.hpp>

// Handles CTRL+C
extern volatile bool MdvspSigInt;

/**
 * @brief Class representing a instance of a MDVSP problem.
 */
class Instance {
public:
   /// Reads the instance from file.
   Instance(const char *fname);
   virtual ~Instance();

   auto fileName() const noexcept -> const std::string &;

   /// Data regarding the instance size.
   auto numDepots() const noexcept -> int;
   auto numTrips() const noexcept -> int;

   /// Depot data.
   auto depotCapacity(int k) const noexcept -> int;

   /// Arc cost data.
   auto sourceCost(int k, int trip) const noexcept -> int;
   auto sinkCost(int k, int trip) const noexcept -> int;
   auto deadheadCost(int pred, int succ) const noexcept -> int;

   /// Mostly for debugging purposes.
   /// Can also help devising some special loops without
   /// too many branches/ifs.
   auto rawCost(int i, int j) const noexcept -> int;

   // Returns a list of adjacent tasks that can succeed task `pred`.
   // The pairs store the successor task id (first), and the
   // associated deadheading cost (second.)
   auto deadheadSuccAdj(int pred) const noexcept -> const std::vector<std::pair<int, int>> &;
   auto deadheadPredAdj(int pred) const noexcept -> const std::vector<std::pair<int, int>> &;

private:
   const std::string m_fname;
   int m_numDepots, m_numTrips;
   
   // [depot ID] -> number of vehicles available
   std::vector<int> m_depotCap;
   
   // [origin][dest] -> cost relative to arc (origin,dest)
   boost::multi_array<int, 2> m_matrix;

   // Cache of deadheading portion of the matrix.
   // [pred task] -> list of successors
   // <0> -> succeeding task
   // <1> -> associated cost
   std::vector<std::vector<std::pair<int, int>>> m_dhCacheSucc;

   std::vector<std::vector<std::pair<int, int>>> m_dhCachePred;
};
