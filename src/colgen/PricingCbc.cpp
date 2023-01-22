#include "PricingCbc.h"
#include "CgMasterBase.h"
#include "Instance.h"

#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CoinModel.hpp>

#include <algorithm>
#include <iostream>

using namespace std;

PricingCbc::PricingCbc(const Instance &inst, CgMasterBase &master, int depotId, int maxPaths): CgPricingBase(inst, master, depotId, maxPaths) {
   // Empty
}

PricingCbc::~PricingCbc() {
   // Empty.
}

auto PricingCbc::getSolverName() const noexcept -> std::string {
   return string("Coin-OR CBC ") + Cbc_getVersion();
}

auto PricingCbc::writeLp(const char *fname) const noexcept -> void {
   m_lpSolver->writeLp(fname, "");
}

auto PricingCbc::isExact() const noexcept -> bool {
   return true;
}

auto PricingCbc::solve() noexcept -> double {
   if (!m_lpSolver)
      buildModel();
   const auto O = sourceNode();
   const auto D = sinkNode();

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // First updates the source arcs with the duals relative to
      // the depot capacity.
      if (int colId = m_x[O][i]; colId != -1) {
         auto cost = m_inst->sourceCost(m_depotId, i) - m_master->getDepotCapDual(m_depotId);
         m_lpSolver->setObjCoeff(colId, cost);
      }

      // Then update the deadheading arcs.
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (int colId = m_x[i][j]; colId != -1) {
            auto cost = m_inst->deadheadCost(i, j) - m_master->getTripDual(i);
            m_lpSolver->setObjCoeff(colId, cost);
         }
      }

      // And also the sink arcs.
      if (int colId = m_x[i][D]; colId != -1) {
         auto cost = m_inst->sinkCost(m_depotId, i) - m_master->getTripDual(i);
         m_lpSolver->setObjCoeff(colId, cost);
      }
   }

   m_model.reset(new CbcModel(*m_lpSolver));
   
   m_lpSolver->setLogLevel(0);
   m_model->setLogLevel(0);
   m_model->setNumberThreads(1);
   
   m_model->branchAndBound();
   return m_model->getObjValue();
}

auto PricingCbc::getObjValue() const noexcept -> double {
   return m_model->getObjValue();
}

auto PricingCbc::generateColumns() const noexcept -> int {
   vector<vector<int>> allPaths;
   const auto sol = m_model->getColSolution();

   // I think the algorithm can be accelerated by skipping the calculation
   // of reduced cost per path.

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto col = m_x[sourceNode()][i]; col != -1 && sol[col] >= 0.98) {
         vector<int> path = {i};
         double pcost = m_inst->sourceCost(m_depotId, i) - m_master->getDepotCapDual(m_depotId);

         findPathRecursive(path, pcost, allPaths);
      }
   }

   for (const auto &p : allPaths) {
      m_master->beginColumn(m_depotId);
      for (auto it = p.begin(); it != p.end(); ++it) {
         m_master->addTrip(*it);
      }
      m_master->commitColumn();
   }

   return allPaths.size();
   return 0.0;
}

auto PricingCbc::findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   const auto sol = m_model->getColSolution();  // hope this call to not be expensive...
   if (auto col = m_x[path.back()][sinkNode()]; col != -1 && sol[col] >= 0.98) {
      double cst = pcost + (m_inst->sinkCost(m_depotId, path.back()) - m_master->getTripDual(path.back()));
      if (cst <= -0.001) {
         allPaths.push_back(path);
      }
   }

   for (const auto &p : m_inst->deadheadSuccAdj(path.back())) {
      if (auto col = m_x[path.back()][p.first]; col != -1 && sol[col] >= 0.98) {
         double cst = pcost + (p.second - m_master->getTripDual(path.back()));
         path.push_back(p.first);
         findPathRecursive(path, cst, allPaths);
         path.pop_back();
      }
   }
}

auto PricingCbc::buildModel() noexcept -> void {
   char buf[128];
   m_lpSolver.reset(new OsiClpSolverInterface());

   // Create the variables.
   const auto N = numNodes();
   const auto O = sourceNode();
   const auto D = sinkNode();
   m_x.resize(boost::extents[N][N]);
   fill_n(m_x.data(), m_x.num_elements(), -1);

   // Class used to build models.
   CoinModel builder;

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // Creates source arcs.
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "source#%d#%d", m_depotId, i);
         m_x[O][i] = builder.numberColumns();
         #ifndef MIP_PRICING_LP
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, true);
         #else
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, false);
         #endif
      }

      // Creates sink arcs.
      if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "sink#%d#%d", m_depotId, i);
         m_x[i][D] = builder.numberColumns();
         #ifndef MIP_PRICING_LP
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, true);
         #else
            builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, false);
         #endif
      }

      // Adds all deadheading arcs, or up to the maximum number of arcs allowed to expand.
      int numExpansions = m_maxLabelExpansions;
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (auto cost = m_inst->deadheadCost(i, j); cost != -1) {
            snprintf(buf, sizeof buf, "deadhead#%d#%d", i, j);
            m_x[i][j] = builder.numberColumns();
            #ifndef MIP_PRICING_LP
               builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, true);
            #else
               builder.addColumn(0, nullptr, nullptr, 0.0, 1.0, cost, buf, false);
            #endif

            if (--numExpansions == 0)
               break;
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
      builder.addRow(cols.size(), cols.data(), coefs.data(), 0.0, 0.0, buf);
   }

   // Optional constraint to force a single path.
   if (m_maxPaths >= 1) {
      vector<int> cols;
      vector<double> coefs;

      for (int i = 0; i < m_inst->numTrips(); ++i) {
         if (auto colId = m_x[O][i]; colId != -1) {
            cols.push_back(colId);
            coefs.push_back(1.0);
         }
      }

      snprintf(buf, sizeof buf, "max_paths#%d", m_depotId);
      builder.addRow(cols.size(), cols.data(), coefs.data(), -numeric_limits<double>::infinity(), m_maxPaths, buf);
   }

   snprintf(buf, sizeof buf, "mdvsp_pricing_coin#%d", m_depotId);
   builder.setProblemName(buf);
   builder.setOptimizationDirection(1.0);

   m_lpSolver->loadFromCoinModel(builder);
   m_lpSolver->setObjName("shortest_path");
}
