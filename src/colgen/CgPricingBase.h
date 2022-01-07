#pragma once

#include <vector>
#include <iosfwd>

class Instance;
class CgMasterBase;

class CgPricingBase {
public:
   CgPricingBase(const Instance &inst, CgMasterBase &master, const int depotId, int maxPaths = -1);
   virtual ~CgPricingBase();

   virtual auto getSolverName() const noexcept -> std::string = 0;
   virtual auto writeLp(const char *fname) const noexcept -> void = 0;

   auto depotId() const noexcept -> int;

   virtual auto solve() noexcept -> double = 0;
   virtual auto getObjValue() const noexcept -> double = 0;
   virtual auto generateColumns() const noexcept -> int = 0; // Returns the number of cols generated

protected:
   const Instance *m_inst;
   const int m_depotId;
   CgMasterBase *m_master;
   const int m_maxPaths;

   auto numNodes() const noexcept -> int;
   auto sourceNode() const noexcept -> int;
   auto sinkNode() const noexcept -> int;
};
