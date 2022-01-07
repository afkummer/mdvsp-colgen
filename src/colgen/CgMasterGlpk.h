#pragma once

#include "Instance.h"
#include "CgMasterBase.h"

#include <glpk.h>

class CgMasterGlpk: public CgMasterBase {
public:
   CgMasterGlpk(const Instance &inst);
   virtual ~CgMasterGlpk();

   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto getTripDual(int i) const noexcept -> double override;
   virtual auto getDepotCapDual(int k) const noexcept -> double override;
   
   virtual auto setAssignmentType(char sense = 'G') noexcept -> void override;

private:
   glp_prob *m_model;

   virtual auto addColumn() noexcept -> void override;
};
