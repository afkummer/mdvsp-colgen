#include "PricingGlpk.h"
#include "CgMasterBase.h"
#include "Instance.h"

#include <algorithm>
#include <iostream>

using namespace std;

PricingGlpk::PricingGlpk(const Instance &inst, CgMasterBase &master, const int depotId, int maxPaths): 
   CgPricingBase(inst, master, depotId, maxPaths) {

   char buf[128];
   m_model = glp_create_prob();
   
   // Basic model data.
   snprintf(buf, sizeof buf, "mdvsp_pricing_glpk#%d", m_depotId);
   glp_set_prob_name(m_model, buf);
   glp_set_obj_dir(m_model, GLP_MIN);
   glp_set_obj_name(m_model, "shortest_path");

   // In this implementation, we use the original IDs for trips
   const auto N = numNodes();
   const auto O = sourceNode();
   const auto D = sinkNode();

   // Reserve memory for storing the variables.
   m_x.resize(boost::extents[N][N]);
   fill_n(m_x.data(), m_x.num_elements(), -1);

   // Creates all variables.
   auto addVar = [&](int i, int j, double cost) {
      int colId = glp_add_cols(m_model, 1);
      glp_set_col_name(m_model, colId, buf);
      glp_set_col_kind(m_model, colId, GLP_BV);
      glp_set_col_bnds(m_model, colId, GLP_DB, 0.0, 1.0);
      glp_set_obj_coef(m_model, colId, cost);
      m_x[i][j] = colId;
   };   
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto cost = m_inst->sourceCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "source#%d#%d", m_depotId, i);
         addVar(O, i, cost);
      }
      if (auto cost = m_inst->sinkCost(m_depotId, i); cost != -1) {
         snprintf(buf, sizeof buf, "sink#%d#%d", m_depotId, i);
         addVar(i, D, cost);
      }
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (auto cost = m_inst->deadheadCost(i, j); cost != -1) {
            snprintf(buf, sizeof buf, "deadhead#%d#%d", i, j);
            addVar(i, j, cost);
         }
      }
   }

   // Structures to fill rows of the problem.
   vector <int> matCols {0};
   vector <double> matCoefs {0.0};

   // The model consists of a flow conservation solely.
   int rowId = glp_add_rows(m_model, m_inst->numTrips());
   for (int i = 0; i < m_inst->numTrips(); ++i, ++rowId) {
      snprintf(buf, sizeof buf, "flow_conservation#%d", i);
      glp_set_row_name(m_model, rowId, buf);
      glp_set_row_bnds(m_model, rowId, GLP_FX, 0.0, 0.0);

      // Checks if there is source arc.
      if (int colId = m_x[O][i]; colId != -1) {
         matCols.push_back(colId);
         matCoefs.push_back(1.0);
      }

      // Checks if there is sink arc.
      if (int colId = m_x[i][D]; colId != -1) {
         matCols.push_back(colId);
         matCoefs.push_back(-1.0);
      }

      // Adds the remainder arcs.
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (int colId = m_x[i][j]; colId != -1) {
            matCols.push_back(colId);
            matCoefs.push_back(-1.0);
         }
         if (int colId = m_x[j][i]; colId != -1) {
            matCols.push_back(colId);
            matCoefs.push_back(1.0);
         }
      }

      glp_set_mat_row(m_model, rowId, matCols.size()-1, matCols.data(), matCoefs.data());
      matCols.resize(1);
      matCoefs.resize(1);
   }

   // Single path per solve call.
   if (maxPaths >= 1) {
      for (int i = 0; i < m_inst->numTrips(); ++i) {
         // Checks if there is source arc.
         if (int colId = m_x[O][i]; colId != -1) {
            matCols.push_back(colId);
            matCoefs.push_back(1.0);
         }
      }
      int rowId = glp_add_rows(m_model, maxPaths);
      snprintf(buf, sizeof buf, "max_paths");
      glp_set_row_name(m_model, rowId, buf);
      glp_set_row_bnds(m_model, rowId, GLP_UP, 0.0, maxPaths);
      
      glp_set_mat_row(m_model, rowId, matCols.size() - 1, matCols.data(), matCoefs.data());
      matCols.resize(1);
      matCoefs.resize(1);
   }
}

PricingGlpk::~PricingGlpk() {
   glp_delete_prob(m_model);
}

auto PricingGlpk::getSolverName() const noexcept -> std::string {
   return string("GLPK ") + glp_version();
}

auto PricingGlpk::writeLp(const char *fname) const noexcept -> void {
   glp_write_lp(m_model, nullptr, fname);
}

auto PricingGlpk::isExact() const noexcept -> bool {
   return true;
}

auto PricingGlpk::solve() noexcept -> double {
   const auto O = sourceNode();
   const auto D = sinkNode();

   glp_term_out(GLP_OFF);

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      // First updates the source arcs with the duals relative to
      // the depot capacity.
      if (int colId = m_x[O][i]; colId != -1) {
         auto cost = m_inst->sourceCost(m_depotId, i) - m_master->getDepotCapDual(m_depotId);
         glp_set_obj_coef(m_model, colId, cost);
      }

      // Then update the deadheading arcs.
      for (int j = 0; j < m_inst->numTrips(); ++j) {
         if (int colId = m_x[i][j]; colId != -1) {
            auto cost = m_inst->deadheadCost(i, j) - m_master->getTripDual(i);
            glp_set_obj_coef(m_model, colId, cost);
         }
      }

      // And also the sink arcs.
      if (int colId = m_x[i][D]; colId != -1) {
         auto cost = m_inst->sinkCost(m_depotId, i) - m_master->getTripDual(i);
         glp_set_obj_coef(m_model, colId, cost);
      }
   }

   glp_smcp parm;
   glp_init_smcp(&parm);
   parm.meth = GLP_PRIMAL;
   // parm.msg_lev = GLP_MSG_ALL;

   if (glp_simplex(m_model, &parm) != 0) {
      cout << "Failed to solve relaxation for pricing #" << m_depotId << ".\n";
      writeLp("problematic.lp");
      cout << "Problematic model written to problematic.lp.\n";
      abort();
   }

   glp_iocp mipParm;
   glp_init_iocp(&mipParm);
   mipParm.br_tech = GLP_BR_DTH;
   mipParm.bt_tech = GLP_BT_BPH;
   mipParm.pp_tech = GLP_PP_ROOT;
   mipParm.mip_gap = 1e-8;
   // mipParm.msg_lev = GLP_MSG_ALL;

   if (glp_intopt(m_model, &mipParm) != 0) {
      cout << "Failed to solve pricing #" << m_depotId << " as integer program.\n";
      writeLp("problematicMip.lp");
      cout << "Problematic model written to problematicMip.lp.\n";
      abort();
   }
   
   return glp_mip_obj_val(m_model);
}

auto PricingGlpk::getObjValue() const noexcept -> double {
   return glp_mip_obj_val(m_model);
}

auto PricingGlpk::generateColumns() const noexcept -> int {
   vector<vector<int>> allPaths;

   // I think the algorithm can be accelerated by skipping the calculation
   // of reduced cost per path.

   for (int i = 0; i < m_inst->numTrips(); ++i) {
      if (auto col = m_x[sourceNode()][i]; col != -1 && glp_mip_col_val(m_model, col) >= 0.98) {
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
}

auto PricingGlpk::findPathRecursive(std::vector<int> &path, double pcost, std::vector<std::vector<int>> &allPaths) const noexcept -> void {
   if (auto col = m_x[path.back()][sinkNode()]; col != -1 && glp_mip_col_val(m_model, col) >= 0.98) {
      double cst = pcost + (m_inst->sinkCost(m_depotId, path.back()) - m_master->getTripDual(path.back()));
      if (cst <= -0.001) {
         allPaths.push_back(path);
      }
   }

   for (const auto &p : m_inst->deadheadSuccAdj(path.back())) {
      if (auto col = m_x[path.back()][p.first]; col != -1 && glp_mip_col_val(m_model, col) >= 0.98) {
         double cst = pcost + (p.second - m_master->getTripDual(path.back()));
         path.push_back(p.first);
         findPathRecursive(path, cst, allPaths);
         path.pop_back();
      }
   }
}
