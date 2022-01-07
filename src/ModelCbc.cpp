#include "ModelCbc.h"

#include <algorithm>
#include <iostream>

#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CoinModel.hpp>

using namespace std;

ModelCbc::ModelCbc(const Instance &inst): m_inst(&inst) {
   char buf[128];
   CoinModel builder;

   builder.setProblemName("compact_mdvsp_coin_cbc");
   builder.setOptimizationDirection(1.0);

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
            m_x[k][O][i] = builder.numberColumns();
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, true);
         }
      }

      // Creates sink arcs.
      for (int k = 0; k < m_inst->numDepots(); ++k) {
         if (auto cost = m_inst->sinkCost(k, i); cost != -1) {
            snprintf(buf, sizeof buf, "sink#%d#%d#%d", k, i, D);
            m_x[k][i][D] = builder.numberColumns();
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, true);
         }
      }

      // Adds all deadheading arcs.
      for (auto &p: m_inst->deadheadSuccAdj(i)) {
         for (int k = 0; k < m_inst->numDepots(); ++k) {
            snprintf(buf, sizeof buf, "deadhead#%d#%d#%d", k, i, p.first);
            m_x[k][i][p.first] = builder.numberColumns();
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, p.second, buf, true);
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
      builder.addRow(cols.size(), cols.data(), coefs.data(), 1.0, 1.0, buf);
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
         builder.addRow(cols.size(), cols.data(), coefs.data(), 0.0, 0.0, buf);
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
      builder.addRow(cols.size(), cols.data(), coefs.data(), -numeric_limits<double>::infinity(), m_inst->depotCapacity(k), buf);
   }

   m_lpSolver.reset(new OsiClpSolverInterface);
   m_lpSolver->loadFromCoinModel(builder);
   m_model.reset(new CbcModel(*m_lpSolver));
}

ModelCbc::~ModelCbc() {
   // Empty
}

auto ModelCbc::writeLp(const char *fname) const noexcept -> void {
   m_lpSolver->writeLp(fname, "");
}

auto ModelCbc::changeBounds(int k, int i, int j, double lb, double ub) noexcept -> void {
   auto colId = m_x[k][i][j];
   assert(colId != -1);
   m_lpSolver->setColBounds(colId, lb, ub);
}
