#pragma once 

#include <vector>
#include <iosfwd>

class Instance;

/**
 * @brief Base class for implementing a Master Problem.
 */
class CgMasterBase {
public:
   CgMasterBase(const Instance &inst);
   virtual ~CgMasterBase();

   virtual auto getSolverName() const noexcept -> std::string = 0;
   virtual auto writeLp(const char *fname) const noexcept -> void = 0;

   virtual auto solve() noexcept -> double = 0;
   virtual auto getObjValue() const noexcept -> double = 0;
   virtual auto getTripDual(int i) const noexcept -> double = 0;
   virtual auto getDepotCapDual(int k) const noexcept -> double = 0;

   // Methods for adding a column
   virtual auto beginColumn(int depotId) noexcept -> void;
   virtual auto addTrip(int trip) noexcept -> void;
   virtual auto commitColumn() noexcept -> void;

   // Queries how many columns exists in the RRMP
   auto numColumns() const noexcept -> int;

   // Changes the type of assignment constraints.
   // If sense is 'E', uses a constraint in the format '== 1'.
   // If sense is 'G', uses a constraint in the format '>='.
   // By default, the model should be created with sense 'G'. 
   virtual auto setAssignmentType(char sense = 'G') noexcept -> void = 0;

   // Capability of exporting/importing columns from file.
   auto exportColumns(const char *fname) const noexcept -> void;
   auto importColumns(const char *fname) noexcept -> int;

protected:
   const Instance *m_inst;
   int m_numCols{0};

   // Items for caching elements of a new column.
   int m_newcolDepot;
   double m_newcolCost;
   int m_newcolLastTrip;
   std::vector <int> m_newcolPath;

   // Cached copy of the columns.
   std::vector<int> m_colDepot;
   std::vector<std::vector<int>> m_colTrips;

   virtual auto addColumn() noexcept -> void = 0;
};
