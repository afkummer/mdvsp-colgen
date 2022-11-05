#include "CgMasterGlpk.h"

#include <vector>

using namespace std;

CgMasterGlpk::CgMasterGlpk(const Instance &inst): CgMasterBase(inst) {
   // Initializes the basics of GLPK.
   glp_term_out(GLP_OFF);
   m_model = glp_create_prob();
   char buf[128];

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
}

CgMasterGlpk::~CgMasterGlpk() {
   glp_delete_prob(m_model);
}

auto CgMasterGlpk::getSolverName() const noexcept -> std::string {
   return string("GLPK ") + glp_version();
}

auto CgMasterGlpk::writeLp(const char *fname) const noexcept -> void {
   glp_write_lp(m_model, nullptr, fname);
}

auto CgMasterGlpk::solve() noexcept -> double {
   // In this use case, the best method is the primal simplex,
   // mostly because the RMP is always feasible and only requires
   // re-optimization due to additional columns inserted 
   // on-the-fly.
   glp_smcp parm;
   glp_init_smcp(&parm);
   parm.meth = GLP_PRIMAL;
   // parm.presolve = GLP_ON; // TODO CHECK
   // TODO: Build a basis??
   
   // Function that calls simplex/dual simplex, according the parameters.
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

auto CgMasterGlpk::setAssignmentType(char sense) noexcept -> void {
   int ind = sense == 'G' ? GLP_LO : GLP_FX;
   for (int i = 0; i < m_inst->numTrips(); ++i) {
      glp_set_row_bnds(m_model, i+1, ind, 1.0, 1.0);
   }
}

auto CgMasterGlpk::getValue(int col) const noexcept -> double {
   return glp_get_col_prim(m_model, col+1);
}

auto CgMasterGlpk::getLb(int col) const noexcept -> double {
   return glp_get_col_lb(m_model, col+1);
}

auto CgMasterGlpk::setLb(int col, double bound) noexcept -> void {
   double ub = glp_get_col_ub(m_model, col+1);
   glp_set_col_bnds(m_model, col+1, GLP_DB, bound, ub);
}

auto CgMasterGlpk::convertToBinary() noexcept -> void {
   for (int col = 1; col <= numColumns(); ++col) {
      glp_set_col_kind(m_model, col, GLP_BV);
   }
}

auto CgMasterGlpk::convertToRelaxed() noexcept -> void {
   for (int col = 1; col <= numColumns(); ++col) {
      glp_set_col_kind(m_model, col, GLP_CV);
   }
}

auto CgMasterGlpk::addColumn() noexcept -> void {
   assert(m_newcolDepot != -1);
   assert(!m_newcolPath.empty());
   char buf[128];
   snprintf(buf, sizeof buf, "path#%d#%d", m_newcolDepot, numColumns());

   vector<int> rows{0};
   vector<double> coefs{0.0};

   rows.push_back(m_newcolDepot + m_inst->numTrips() + 1);
   coefs.push_back(1.0);

   for (int i: m_newcolPath) {
      rows.push_back(i+1);
      coefs.push_back(1.0);
   }

   int colId = glp_add_cols(m_model, 1);

   glp_set_col_name(m_model, colId, buf);
   glp_set_col_kind(m_model, colId, GLP_CV);
   glp_set_col_bnds(m_model, colId, GLP_LO, 0.0, 0.0);
   glp_set_obj_coef(m_model, colId, m_newcolCost);

   glp_set_mat_col(m_model, colId, rows.size() - 1, rows.data(), coefs.data());
}
