#pragma once

#include <string>
#include <vector>

#include <boost/multi_array.hpp>

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

private:
   const std::string m_fname;
   int m_numDepots, m_numTrips;
   
   // [depot ID] -> number of vehicles available
   std::vector<int> m_depotCap;
   
   // [origin][dest] -> cost relative to arc (origin,dest)
   boost::multi_array<int, 2> m_matrix;
};
