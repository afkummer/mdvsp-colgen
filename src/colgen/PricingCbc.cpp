#include "PricingCbc.h"

#include <algorithm>
#include <iostream>

using namespace std;

PricingCbc::PricingCbc(const Instance &inst, CgMasterInterface &master, int depotId): m_inst(&inst), m_depotId(depotId), m_master(&master) {
   char buf[128];
   m_model = Cbc_newModel();

   snprintf(buf, sizeof buf, "mdvsp_pricing_glpk#%d", m_depotId);
   Cbc_setProblemName(m_model, buf);
   Cbc_setObjSense(m_model, 1);

   // Create the variables.
   const auto N = m_inst->numTrips()+2;
   const auto O = N-2;
   const auto D = N-1;
   m_x.resize(boost::extents[N][N]);
   fill_n(m_x.data(), m_x.num_elements(), -1);

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // Creates source arcs.
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "source#%d#%d", m_depotId, i);
         int colId = Cbc_getNumCols(m_model);
         Cbc_addCol(m_model, buf, 0.0, 1.0, cost, 1, 0, nullptr, nullptr);
         m_x[O][i] = colId;
         
      }

      // Creates sink arcs.
      if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "sink#%d#%d", m_depotId, i);
         int colId = Cbc_getNumCols(m_model);
         Cbc_addCol(m_model, buf, 0.0, 1.0, cost, 1, 0, nullptr, nullptr);
         m_x[i][D] = colId;  
      }

      // Adds all deadheading arcs.
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (auto cost = m_inst->deadheadCost(i, j); cost != -1) {
            snprintf(buf, sizeof buf, "deadhead#%d#%d", i, j);
            int colId = Cbc_getNumCols(m_model);
            Cbc_addCol(m_model, buf, 0.0, 1.0, cost, 1, 0, nullptr, nullptr);
            m_x[i][j] = colId;
         }
      }
   }
  
   // Adds the flow conservation constraints.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      vector<int> cols;
      vector<double> coefs;

      for (int j = 0; j < N; ++j) {
         if (auto colId = m_x[i][j]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(-1.0);
         }
         if (auto colId = m_x[j][i]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(1.0);
         }
      }

      snprintf(buf, sizeof buf, "flow_consevation#%d", i);
      Cbc_addRow(m_model, buf, cols.size(), cols.data(), coefs.data(), 'E', 0.0); 
   }

   // Optional constraint to force a single path.
   if (1) {
      vector<int> cols;
      vector<double> coefs;

      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto colId = m_x[O][i]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(1.0);
         }
      }

      snprintf(buf, sizeof buf, "single_path#%d", m_depotId);
      Cbc_addRow(m_model, buf, cols.size(), cols.data(), coefs.data(), 'L', 5.0);
   }
}

PricingCbc::~PricingCbc() {
   Cbc_deleteModel(m_model);
}

auto PricingCbc::writeLp(const char *fname) const noexcept -> void {
   Cbc_writeLp(m_model, fname);
}

auto PricingCbc::depotId() const noexcept -> int {
   return m_depotId;
}

auto PricingCbc::solve() noexcept -> double {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // First updates the source arcs with the duals relative to
      // the depot capacity.
      if (int colId = m_x[O][i]; colId != -1) {
         auto cost = m_inst->sourceCost(m_depotId, i) - m_master->getDepotCapDual(m_depotId);
         Cbc_setObjCoeff(m_model, colId, cost);
         Cbc_setContinuous(m_model, colId);
         Cbc_setInteger(m_model, colId);
      }

      // Then update the deadheading arcs.
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (int colId = m_x[i][j]; colId != -1) {
            auto cost = m_inst->deadheadCost(i, j) - m_master->getTripDual(i);
            Cbc_setObjCoeff(m_model, colId, cost);
         }
      }

      // And also the sink arcs.
      if (int colId = m_x[i][D]; colId != -1) {
         auto cost = m_inst->sinkCost(m_depotId, i) - m_master->getTripDual(i);
         Cbc_setObjCoeff(m_model, colId, cost);
      }
   }

   Cbc_setLogLevel(m_model, 999);

   Cbc_solve(m_model);
   return Cbc_getObjValue(m_model);
}

auto PricingCbc::getObjValue() const noexcept -> double {
   return Cbc_getObjValue(m_model);
}

auto PricingCbc::generateColumns() const noexcept -> void {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;

   vector<int> path = {O};
   vector<vector<int>> allPaths;
   getPathRecursive(path, allPaths);
   cout << "   Pricing #" << m_depotId << " with " << allPaths.size() << " new columns and cost " << getObjValue()<< ".\n";
   for (auto &p : allPaths) {
      assert(p.size() > 2);
      m_master->beginColumn(m_depotId);
      for (size_t pos = 1; pos < p.size() - 1; ++pos) {
         m_master->addTrip(p[pos]);
      }
      m_master->commitColumn();
   }
}

auto PricingCbc::getPathRecursive(std::vector<int> &path, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   const auto N = m_inst->numTrips() + 2;
   const auto O = N - 2;
   const auto D = N - 1;
   const auto sol = Cbc_getColSolution(m_model);

   if (path.back() == O) {
      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto colId = m_x[O][i]; colId != -1 and sol[colId] >= 0.5) {
            path.push_back(i);
            getPathRecursive(path, allPaths);
            path.pop_back();
         }
      }
   } else {
      if (auto colId = m_x[path.back()][D]; colId != -1 and sol[colId] >= 0.5) {
         path.push_back(D);
         allPaths.push_back(path);
         path.pop_back();
      }

      for (auto &p : m_inst->deadheadSuccAdj(path.back())) {
         if (auto colId = m_x[path.back()][p.first]; colId != -1 and sol[colId] >= 0.5) {
            path.push_back(p.first);
            getPathRecursive(path, allPaths);
            path.pop_back();
         }
      }
   }
}
