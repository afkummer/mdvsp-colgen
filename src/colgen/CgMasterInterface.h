#pragma once 

class CgMasterInterface {
public:
   virtual auto solve() noexcept -> double = 0;
   virtual auto getObjValue() const noexcept -> double = 0;
   virtual auto getTripDual(int i) const noexcept -> double = 0;
   virtual auto getDepotCapDual(int k) const noexcept -> double = 0;

   // Methods for adding a column
   virtual auto beginColumn(int depotId) noexcept -> void = 0;
   virtual auto addTrip(int trip) noexcept -> void = 0;
   virtual auto commitColumn() noexcept -> void = 0;

   // Queries how many columns exists in the RRMP
   virtual auto numColumns() const noexcept -> int = 0;

   // Changes the type of assignment constraints.
   // If sense is 'E', uses a constraint in the format '== 1'.
   // If sense is 'G', uses a constraint in the format '>='.
   virtual auto setAssignmentType(char sense = 'G') noexcept -> void = 0;
};
