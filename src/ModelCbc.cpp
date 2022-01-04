#include "ModelCbc.h"

#include <algorithm>
#include <iostream>

using namespace std;

ModelCbc::ModelCbc(const Instance &inst): m_inst(&inst) {
   char buf[128];
   m_model = Cbc_newModel();
   Cbc_setProblemName(m_model, "compact_mdvsp_coin_cbc");
   Cbc_setObjSense(m_model, 1);

   // Create the variables.
   const auto N = m_inst->numTrips()+2;
   const auto O = N-2;
   const auto D = N-1;
   m_x.resize(boost::extents[m_inst->numDepots()][N][N]);
   fill_n(m_x.data(), m_x.num_elements(), -1);

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // Creates source arcs.
      for (int k = 0; k < m_inst->numDepots(); ++k) {
         if (auto cost = m_inst->sourceCost(k, i); cost != -1) {
            snprintf(buf, sizeof buf, "source#%d#%d#%d", k, O, i);
            int colId = Cbc_getNumCols(m_model);
            Cbc_addCol(m_model, buf, 0.0, 1.0, cost, 1, 0, nullptr, nullptr);
            m_x[k][O][i] = colId;
            
         }
      }

      // Creates sink arcs.
      for (int k = 0; k < m_inst->numDepots(); ++k) {
         if (auto cost = m_inst->sinkCost(k, i); cost != -1) {
            snprintf(buf, sizeof buf, "sink#%d#%d#%d", k, i, D);
            int colId = Cbc_getNumCols(m_model);
            Cbc_addCol(m_model, buf, 0.0, 1.0, cost, 1, 0, nullptr, nullptr);
            m_x[k][i][D] = colId;  
         }
      }

      // Adds all deadheading arcs.
      for (auto &p: m_inst->deadheadSuccAdj(i)) {
         for (int k = 0; k < m_inst->numDepots(); ++k) {
            snprintf(buf, sizeof buf, "deadhead#%d#%d#%d", k, i, p.first);
            int colId = Cbc_getNumCols(m_model);
            Cbc_addCol(m_model, buf, 0.0, 1.0, p.second, 1, 0, nullptr, nullptr);
            m_x[k][i][p.first] = colId;
         }
      }
   }
   
   // Adds the assignment constraints.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      vector <int> cols;
      vector <double> coefs;

      for (int k = 0; k < m_inst->numDepots(); ++k) {
         for (auto &p: m_inst->deadheadSuccAdj(i)) {
            auto colId = m_x[k][i][p.first];
            assert(colId != -1);
            cols.push_back(colId);
            coefs.push_back(1.0);
         }
         if (auto colId = m_x[k][i][D]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(1.0);
         }
      }

      snprintf(buf, sizeof buf, "assignment#%d", i);
      Cbc_addRow(m_model, buf, cols.size(), cols.data(), coefs.data(), 'E', 1.0);
   }

   // Adds the flow conservation constraints.
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      for (int k = 0; k < m_inst->numDepots(); ++k) {
         vector<int> cols;
         vector<double> coefs;

         if (auto colId = m_x[k][O][i]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(1.0);
         }

         if (auto colId = m_x[k][i][D]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(-1.0);
         }

         for (auto &p: m_inst->deadheadPredAdj(i)) {
            auto colId = m_x[k][p.first][i];
            assert(colId != -1);
            cols.push_back(colId);
            coefs.push_back(1.0);
         }

         for (auto &p: m_inst->deadheadSuccAdj(i)) {
            auto colId = m_x[k][i][p.first];
            assert(colId != -1);
            cols.push_back(colId);
            coefs.push_back(-1.0);
         }

         snprintf(buf, sizeof buf, "flow_consevation#%d#%d", k, i);
         Cbc_addRow(m_model, buf, cols.size(), cols.data(), coefs.data(), 'E', 0.0);
      }      
   }

   // Adds the depot capacity constraints.
   for (int k = 0; k < m_inst->numDepots(); ++k) {
      vector<int> cols;
      vector<double> coefs;

      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto colId = m_x[k][O][i]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(1.0);
         }
      }

      snprintf(buf, sizeof buf, "depot_cap#%d", k);
      Cbc_addRow(m_model, buf, cols.size(), cols.data(), coefs.data(), 'L', m_inst->depotCapacity(k));
   }
}

ModelCbc::~ModelCbc() {
   Cbc_deleteModel(m_model);
}

auto ModelCbc::writeLp(const char *fname) const noexcept -> void {
   Cbc_writeLp(m_model, fname);
}

auto ModelCbc::changeBounds(int k, int i, int j, double lb, double ub) noexcept -> void {
   auto colId = m_x[k][i][j];
   assert(colId != -1);
   Cbc_setColLower(m_model, colId, lb);
   Cbc_setColUpper(m_model, colId, ub);
}
