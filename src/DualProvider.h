#pragma once 

class DualProvider {
public:
   virtual auto getTripDual(int i) const noexcept -> double = 0;
   virtual auto getDepotCapDual(int k) const noexcept -> double = 0;
};
