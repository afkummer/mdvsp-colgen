#include "CgMasterClp.h"
#include "Instance.h"

#include <vector>

using namespace std;

CgMasterClp::CgMasterClp(const Instance &inst): CgMasterBase(inst) {
   // This implementation does not uses CoinModel because the RMP is really
   // simple to build, and has a very minimal size so the time spent resizing
   // the internal CLP structures is negligible.
   char buf[128];

   m_lpSolver.reset(new OsiClpSolverInterface());
   m_lpSolver->setObjName("mdvsp_master_clp");
   m_lpSolver->setObjSense(1.0);

   // Adds the trip assignment constraints.
   assert(m_lpSolver->getNumRows() == 0);
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      int rowId = m_lpSolver->getNumRows();
      m_lpSolver->addRow(0, nullptr, nullptr, 1.0, COIN_DBL_MAX);
      snprintf(buf, sizeof buf, "task_assign#%d", i);
      m_lpSolver->setRowName(rowId, buf);
   }

   // Adds the depot capacity constraints.
   for (int k = 0; k < m_inst->numDepots(); ++k) {
      int rowId = m_lpSolver->getNumRows();
      m_lpSolver->addRow(0, nullptr, nullptr, COIN_DBL_MIN, m_inst->depotCapacity(k));
      snprintf(buf, sizeof buf, "depot_cap#%d", k);
      m_lpSolver->setRowName(rowId, buf);
   }

   // Adds the dummy solution.
   assert(m_lpSolver->getNumCols() == 0);
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      int colId = m_lpSolver->getNumCols();

      snprintf(buf, sizeof buf, "dummy#%d", colId);

      vector<int> rows = {i};
      vector<double> coeffs = {1.0};
      m_lpSolver->addCol(1, rows.data(), coeffs.data(), 0.0, COIN_DBL_MAX, 1e7, buf);
   }

   // Configures the solver to be quiet.
   m_lpSolver->setLogLevel(0);
   // m_lpSolver->getModelPtr()->setNumberThreads(1);
   m_lpSolver->initialSolve();
}

CgMasterClp::~CgMasterClp() {
   // Empty, probably
}

auto CgMasterClp::getSolverName() const noexcept -> std::string {
   return string("Coin-OR CLP ") + CLP_VERSION;
}

auto CgMasterClp::writeLp(const char *fname) const noexcept -> void {
   m_lpSolver->writeLp(fname, "");
}

auto CgMasterClp::solve() noexcept -> double {
   m_lpSolver->resolve();
   return m_lpSolver->getObjValue();
}

auto CgMasterClp::getObjValue() const noexcept -> double {
   return m_lpSolver->getObjValue();
}

auto CgMasterClp::getTripDual(int i) const noexcept -> double {
   return m_lpSolver->getRowPrice()[i];
}

auto CgMasterClp::getDepotCapDual(int k) const noexcept -> double {
   return m_lpSolver->getRowPrice()[k+m_inst->numTrips()];
}

auto CgMasterClp::setAssignmentType(char sense) noexcept -> void {
   const auto ub = sense == 'E' ? 1.0 : COIN_DBL_MAX;
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      m_lpSolver->setRowBounds(i, 1.0, ub);
   }
}

auto CgMasterClp::getValue(int col) const noexcept -> double {
   return m_lpSolver->getColSolution()[col];
}

auto CgMasterClp::getLb(int col) const noexcept -> double {
   return m_lpSolver->getColLower()[col];
}

auto CgMasterClp::setLb(int col, double bound) noexcept -> void {
   //m_lpSolver->setObjCoeff(col, 0.0);
   m_lpSolver->setColLower(col, bound);
}

auto CgMasterClp::convertToBinary() noexcept -> void {
   for (int col = 0; col < numColumns(); ++col) {
      m_lpSolver->setInteger(col);
   }
}

auto CgMasterClp::convertToRelaxed() noexcept -> void {
   for (int col = 0; col < numColumns(); ++col) {
      m_lpSolver->setContinuous(col);
   }
}

auto CgMasterClp::addColumn() noexcept -> void {
   assert(m_newcolDepot != -1);
   assert(!m_newcolPath.empty());
   vector<int> rows;
   vector<double> coeffs;
   char buf[128];

   for (auto i: m_newcolPath) {
      rows.push_back(i);
      coeffs.push_back(1.0);
   }

   {
      rows.push_back(m_newcolDepot+m_inst->numTrips());
      coeffs.push_back(1.0);
   }

   snprintf(buf, sizeof buf, "path#%d#%d", m_newcolDepot, numColumns());
   m_lpSolver->addCol(rows.size(), rows.data(), coeffs.data(), 0.0, COIN_DBL_MAX, m_newcolCost, buf);
}
