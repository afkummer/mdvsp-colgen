#include "CgMasterGlpk.h"

#include <vector>

using namespace std;

CgMasterGlpk::CgMasterGlpk(const Instance &inst, bool quiet): m_inst(&inst) {
   if (quiet) {
      glp_term_out(GLP_OFF);
   }

   char buf[128];
   m_model = glp_create_prob();

   // Basic model data.
   glp_set_prob_name(m_model, "mdvsp_master_glpk");
   glp_set_obj_dir(m_model, GLP_MIN);
   glp_set_obj_name(m_model, "set_partition_cost");

   // Task assignment constraints.
   int rowId = glp_add_rows(m_model, m_inst->numTrips());
   for (int i = 0; i < m_inst->numTrips(); ++i, ++rowId) {
      snprintf(buf, sizeof buf, "task_assign#%d", i);
      glp_set_row_name(m_model, rowId, buf);
      glp_set_row_bnds(m_model, rowId, GLP_LO, 1.0, 1.0);
   }

   // Depot capacity constraints.
   rowId = glp_add_rows(m_model, m_inst->numDepots());
   for (int k = 0; k < m_inst->numDepots(); ++k, ++rowId) {
      snprintf(buf, sizeof buf, "depot_cap#%d", k);
      glp_set_row_name(m_model, rowId, buf);
      glp_set_row_bnds(m_model, rowId, GLP_UP, 0.0, m_inst->depotCapacity(k));
   }

   // Adds the dummy columns.
   m_dummyCols.reserve(m_inst->numTrips());
   int colId = glp_add_cols(m_model, m_inst->numTrips());
   for (int i = 0; i < m_inst->numTrips(); ++i, ++colId) {
      snprintf(buf, sizeof buf, "dummy#%d", i);
      glp_set_col_name(m_model, colId, buf);
      glp_set_col_kind(m_model, colId, GLP_CV);
      glp_set_col_bnds(m_model, colId, GLP_LO, 0.0, 0.0);
      glp_set_obj_coef(m_model, colId, 1e7);

      vector<double> coefs {0.0, 1.0};
      vector<int> rows {0, i+1};
      glp_set_mat_col(m_model, colId, 1, rows.data(), coefs.data());
   }

   m_newcolRows.reserve(m_inst->numTrips() + m_inst->numDepots());
   m_newcolCoefs.reserve(m_inst->numTrips() + m_inst->numDepots());
}

CgMasterGlpk::~CgMasterGlpk() {
   glp_delete_prob(m_model);
}

auto CgMasterGlpk::writeLp(const char *fname) const noexcept -> void {
   glp_write_lp(m_model, nullptr, fname);
}

auto CgMasterGlpk::solve() noexcept -> double {
   glp_smcp parm;
   glp_init_smcp(&parm);
   parm.meth = GLP_PRIMAL;
   glp_simplex(m_model, &parm);
   return glp_get_obj_val(m_model);
}

auto CgMasterGlpk::getObjValue() const noexcept -> double {
   return glp_get_obj_val(m_model);
}

auto CgMasterGlpk::getTripDual(int i) const noexcept -> double {
   assert(i >= 0 && i < m_inst->numTrips());
   return glp_get_row_dual(m_model, i+1); // GLPK uses base-1 indexing!
}

auto CgMasterGlpk::getDepotCapDual(int k) const noexcept -> double {
   assert(k >= 0 && k < m_inst->numDepots());
   return glp_get_row_dual(m_model, k + m_inst->numTrips() + 1);  // GLPK uses base-1 indexing!
}

auto CgMasterGlpk::beginColumn(int depotId) noexcept -> void {
   assert(depotId >= 0 && depotId < m_inst->numDepots());
   m_newcolDepot = depotId;
   m_newcolCost = 0.0;
   m_newcolLastTrip = -1;
   
   // Acts as a 'clear', because GLPK uses base-1 indexing, and it 
   // starts scanning indices from position 1.
   m_newcolRows.resize(1);
   m_newcolCoefs.resize(1);

   // Sets the depot capacity constraints.
   m_newcolRows.push_back(depotId + m_inst->numTrips()+1);
   m_newcolCoefs.push_back(1.0);
}

auto CgMasterGlpk::addTrip(int trip) noexcept -> void {
   assert(trip >= 0 && trip < m_inst->numTrips());

   if (m_newcolLastTrip == -1) {
      m_newcolCost += m_inst->sourceCost(m_newcolDepot, trip);
   } else {
      m_newcolCost += m_inst->deadheadCost(m_newcolLastTrip, trip);
   }

   m_newcolRows.push_back(trip+1);
   m_newcolCoefs.push_back(1.0);

   m_newcolLastTrip = trip;
}

auto CgMasterGlpk::commitColumn() noexcept -> void {
   m_newcolCost += m_inst->sinkCost(m_newcolDepot, m_newcolLastTrip);

   int colId = glp_add_cols(m_model, 1);
   snprintf(m_newcolBuf, sizeof m_newcolBuf, "path#%d", glp_get_num_cols(m_model) + 1);

   glp_set_col_name(m_model, colId, m_newcolBuf);
   glp_set_col_kind(m_model, colId, GLP_CV);
   glp_set_col_bnds(m_model, colId, GLP_LO, 0.0, 0.0);
   glp_set_obj_coef(m_model, colId, m_newcolCost);

   glp_set_mat_col(m_model, colId, m_newcolRows.size()-1, m_newcolRows.data(), m_newcolCoefs.data());
}

auto CgMasterGlpk::numColumns() const noexcept -> int {
   return glp_get_num_cols(m_model);
}

auto CgMasterGlpk::setAssignmentType(char sense) noexcept -> void {
   int ind = sense == 'G' ? GLP_LO : GLP_FX;
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      glp_set_row_bnds(m_model, i+1, ind, 1.0, 1.0);
   }
}
