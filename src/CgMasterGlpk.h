#pragma once

#include "Instance.h"
#include "DualProvider.h"

#include <glpk.h>

class CgMasterGlpk: public DualProvider {
public:
   CgMasterGlpk(const Instance &inst, bool quiet = true);
   virtual ~CgMasterGlpk();

   auto writeLp(const char *fname) const noexcept -> void;

   auto solve() const noexcept -> double;

   auto getObjValue() const noexcept -> double;
   virtual auto getTripDual(int i) const noexcept -> double;
   virtual auto getDepotCapDual(int k) const noexcept -> double;

   // These three methods should be used in this specific sequence.
   // It is guaranted that no memory allocation occur when using
   // this API. It also automatically computes the path cost as
   // the user adds trips.
   auto beginColumn(int depotId) noexcept -> void;
   auto addTrip(int trip) noexcept -> void;
   auto commitColumn() noexcept -> void;

private:
   const Instance *m_inst;
   glp_prob *m_model;

   // [task ID] -> column index
   std::vector <int> m_dummyCols;

   // Attributes that manages the inclusion of new columns.
   int m_newcolDepot;
   double m_newcolCost;
   int m_newcolLastTrip;
   std::vector<int> m_newcolRows;
   std::vector<double> m_newcolCoefs;
   char m_newcolBuf[128];
};
