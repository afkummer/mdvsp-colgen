#pragma once

class CgPricingInterface {
public:
   virtual auto depotId() const noexcept -> int = 0;

   virtual auto solve() noexcept -> double = 0;
   virtual auto getObjValue() const noexcept -> double = 0;
   virtual auto generateColumns() const noexcept -> void = 0;
};
