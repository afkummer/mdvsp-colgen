#pragma once

#include "Instance.h"
#include "CgMasterInterface.h"

#include <glpk.h>

class CgMasterGlpk: public CgMasterInterface {
public:
   CgMasterGlpk(const Instance &inst, bool quiet = true);
   virtual ~CgMasterGlpk();

   auto writeLp(const char *fname) const noexcept -> void;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto getTripDual(int i) const noexcept -> double override;
   virtual auto getDepotCapDual(int k) const noexcept -> double override;

   virtual auto beginColumn(int depotId) noexcept -> void override;
   virtual auto addTrip(int trip) noexcept -> void override;
   virtual auto commitColumn() noexcept -> void override;

   virtual auto numColumns() const noexcept -> int override;
   
   virtual auto setAssignmentType(char sense = 'G') noexcept -> void override;

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
