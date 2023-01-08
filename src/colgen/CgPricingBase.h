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
   virtual auto isExact() const noexcept -> bool = 0;

   virtual auto solve() noexcept -> double = 0;
   virtual auto getObjValue() const noexcept -> double = 0;
   virtual auto generateColumns() const noexcept -> int = 0; // Returns the number of cols generated

   /**
    * Defines the maximum number of output arcs to evaluate when relaxing a node in the DAG.
    * 
    * This parameter can be used to speedup the column generation algorithm, offering a 
    * trade-off between solution quality and processing times. Depending upon the density
    * of the graph, a tight limit can greatly reduce the total running times of the algorithm,
    * without sacrificing much the quality of the solution found.
    * 
    * Note that this setting is not applied when evaluating source and sink arcs. 
    * 
    * @param maxExpansions maximum number of expansions allowed, per node. If the provided
    * value is less than or equals to zero, the internal control will be initialized with
    * a very large value (infinity from the practical perspective)
   */
   void setMaxLabelExpansionsPerNode(int maxExpansions);

protected:
   const Instance *m_inst;
   const int m_depotId;
   CgMasterBase *m_master;
   const int m_maxPaths;

   int m_maxLabelExpansions;

   auto numNodes() const noexcept -> int;
   auto sourceNode() const noexcept -> int;
   auto sinkNode() const noexcept -> int;
};
