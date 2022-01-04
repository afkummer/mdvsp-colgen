#include "Instance.h"
#include "ModelCbc.h"
#include "Timer.h"
#include "colgen/CgMasterCplex.h"
#include "colgen/CgMasterGlpk.h"
#include "colgen/PricingCplex.h"
#include "colgen/PricingSpfa.h"
#include "colgen/PricingCbc.h"
#include "colgen/PricingGlpk.h"

#include <iostream>
#include <iomanip>
#include <csignal>

using namespace std;

volatile bool MdvspSigInt = false;

void sigintHandler(int s) {
   if (s == SIGINT) {
      MdvspSigInt = true;      
      cout << "MDVSP: SIGINT received." << endl;
   }
}

auto main(int argc, char *argv[]) noexcept -> int {
   if (argc != 2) {
      cout << "Usage: " << argv[0] << " <1:instance path>\n";
      return EXIT_FAILURE;
   }

   // Basic app initialization.
   cout.precision(12);
   char buf[128];
   Timer tm;
   signal(SIGINT, sigintHandler);
   
   Instance inst{argv[1]};
   cout << "--- MDVSP solver ---\n" <<
      "Instance: " << inst.fileName() << "\n" <<
      "Number of depots: " << inst.numDepots() << "\n" <<
      "Number of trips: " << inst.numTrips() << "\n\n";

   cout << "Using GNU GLPK " << glp_version() << "\n";
   cout << "Using Coin-OR CBC " << Cbc_getVersion() << "\n";
   cout << "Using IBM ILOG CPLEX " << CPX_VERSION << "\n";
   

   if (0) {
      ModelCbc comp{inst};
      comp.writeLp("cbc.lp");
   }

   // Optimization toolkit initialization.
   tm.start();
   cout << "Initializing algorithms..." << endl;
   CgMasterGlpk master{inst};
   bool relaxed = true;
   
   // Then creates the pricing subproblems.
   using PricingAlg = PricingGlpk;
   vector<unique_ptr<PricingAlg>> pricing;
   for (int k = 0; k < inst.numDepots(); ++k) {
      pricing.emplace_back(make_unique<PricingAlg>(inst, master, k));
   }

   cout << "Time spent preparing the algorithms: " << tm.elapsed() << " sec.\n";

   // Column generation - main loop
   tm.start();
   int threads = 1;
   for (int iter = 0; iter < 1500 and !MdvspSigInt; ++iter) {
      const auto rmp = master.solve();

      // snprintf(buf, sizeof buf, "m%d.lp", iter);
      // master.writeLp(buf);

      // Solves the pricing subproblems.
      double lbPricing = rmp;
      bool newCols = false;

      #pragma omp parallel for default(shared) private(buf) schedule(static, 1) num_threads(threads)
      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         sp->solve();
      }

      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         const auto pobj = sp->getObjValue();

         lbPricing += pobj;
         if (pobj <= -0.01) {
            sp->generateColumns();
            newCols = true;
         }
      }

      cout << "@>>> Iter: " << iter << "   Elapsed: " << tm.elapsed() << " sec   Primal: " << rmp << "   Dual: " << lbPricing << "   NC: " << master.numColumns() << "\n";
      
      if (!newCols) {
         if (relaxed) {
            relaxed = false;
            cout << "\n***** CONVERTING MASTER RELAXATION. *****\n";
            master.setAssignmentType('E');
         } else {
            cout << "\nNo new columns generated.\nStopping the algorithm.\n";
            break;
         }
      }
      threads = 8;
   }

   master.solve();
   cout << "\n" << master.getObjValue() << "\n";

   master.writeLp("masterFinal.lp");
   return EXIT_SUCCESS;
}
