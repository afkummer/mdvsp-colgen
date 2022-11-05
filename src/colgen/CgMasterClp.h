#pragma once

#include "CgMasterBase.h"
#include <coin/OsiClpSolverInterface.hpp>
#include <memory>

class instance;

class CgMasterClp: public CgMasterBase {
public:
   CgMasterClp(const Instance &inst);
   virtual ~CgMasterClp();

   virtual auto getSolverName() const noexcept -> std::string override;
   virtual auto writeLp(const char *fname) const noexcept -> void override;

   virtual auto solve() noexcept -> double override;
   virtual auto getObjValue() const noexcept -> double override;
   virtual auto getTripDual(int i) const noexcept -> double override;
   virtual auto getDepotCapDual(int k) const noexcept -> double override;

   virtual auto setAssignmentType(char sense = 'G') noexcept -> void override;

   virtual auto getValue(int col) const noexcept -> double override;
   virtual auto getLb(int col) const noexcept -> double override;
   virtual auto setLb(int col, double bound) noexcept -> void override;

   virtual auto convertToBinary() noexcept -> void override;
   virtual auto convertToRelaxed() noexcept -> void override;

private:
   std::unique_ptr<OsiClpSolverInterface> m_lpSolver;

   virtual auto addColumn() noexcept -> void override;
};
