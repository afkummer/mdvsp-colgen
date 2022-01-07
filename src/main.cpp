#include "Instance.h"
#include "ModelCbc.h"
#include "Timer.h"
#include "colgen/CgMasterBase.h"
#include "colgen/CgMasterCplex.h"
// #include "colgen/CgMasterGlpk.h"
#include "colgen/PricingCplex.h"
// #include "colgen/PricingSpfa.h"
#include "colgen/PricingCbc.h"
// #include "colgen/PricingGlpk.h"

#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <csignal>

using namespace std;

volatile bool MdvspSigInt = false;

void sigintHandler(int s) {
   if (s == SIGINT) {
      MdvspSigInt = true;      
      cout << "MDVSP: SIGINT received." << endl;
   }
}

/*
double memUsageMB() {
   ifstream fid("/proc/self/status");
   if (!fid)
      return 0.0;
   string line;
   int memkb;
   while (getline(fid, line)) {
      if (line.find("VmRSS:", 0) != string::npos) {
         sscanf(line.data(), "%*s %d", &memkb);         
      }
   }
   return double(memkb) / 1024;
}

struct {
   string mode {"??"};
   int iter;
   double ub;
   double lb;
   double rmpTime;
   double pricingTime;
   double totalTime;
   int numCols;
   double memMB;

   int linesPrint{0};

   auto printHeaders() noexcept -> void {
      cout <<
         setw(3) << "Asg" << 
         setw(6) << "Iter" <<
         setw(10) << "T.Time" << 
         setw(10) << "T.RMP" << 
         setw(10) << "T.SP" << 
         setw(16) << "LB   " << 
         setw(16) << "UB   " << 
         setw(12) << "Gap(%)" << 
         setw(10) << "N.Cols" <<
         setw(10) << "MemMB" << 
      "\n";
   }

   auto printData() noexcept -> void {
      if (linesPrint % 15 == 0) {
         if (linesPrint != 0)
            cout << "\n\n";
         printHeaders();
      }

      double gap = (ub-lb)/ub*100.0;
      cout <<
         fixed << 
         setw(3) << mode << 
         setw(6) << iter << 
         setw(10) << setprecision(2) << totalTime << 
         setw(10) << setprecision(2) << rmpTime << 
         setw(10) << setprecision(2) << pricingTime << 
         setw(16) << setprecision(2) << lb << 
         setw(16) << setprecision(2) << ub << 
         setw(12) << setprecision(2) << gap << 
         setw(10) << numCols <<
         setw(10) << setprecision(2) << memMB << 
      "\n";
      ++linesPrint;
   }

} TtyOutput;
*/

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

   // cout << "Using GNU GLPK " << glp_version() << "\n";
   // cout << "Using Coin-OR CBC " << Cbc_getVersion() << "\n";
   cout << "Using IBM ILOG CPLEX " << CPX_VERSION << "\n";

   // Optimization toolkit initialization.
   tm.start();
   cout << "\nInitializing algorithms..." << endl;

   if (1) {
      ModelCbc comp{inst};
      comp.writeLp("cbc.lp");
   }
   
   CgMasterCplex master{inst};
   bool relaxed = true;
   bool heurPricing = true;
   
   // Then creates the pricing subproblems.
   vector<unique_ptr<CgPricingBase>> pricing;
   for (int k = 0; k < inst.numDepots(); ++k) {
      pricing.emplace_back(make_unique<PricingCbc>(inst, master, k, 5));
   }

   cout << "Time spent preparing the algorithms: " << tm.elapsed() << " sec.\n\n";

   // Column generation - main loop
   tm.start();
   // TtyOutput.mode = "G+H";
   int threads = 1;
   for (int iter = 0; iter < 1500 and !MdvspSigInt; ++iter) {
      Timer tm2;
      tm2.start();
      const auto rmp = master.solve();
      tm2.finish();
      // TtyOutput.iter = iter;
      // TtyOutput.ub = rmp;
      // TtyOutput.rmpTime = tm2.elapsed();

      // snprintf(buf, sizeof buf, "m%d.lp", iter);
      // master.writeLp(buf);

      // Solves the pricing subproblems.
      double lbPricing = rmp;
      bool newCols = false;

      tm2.start();
      #pragma omp parallel for default(shared) private(buf) schedule(static, 1) num_threads(threads)
      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         sp->solve();
      }

      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         const auto pobj = sp->getObjValue();

         lbPricing += pobj;
         if (pobj <= -0.0001) {
            sp->generateColumns();
            newCols = true;
         }
      }
      tm2.finish();

      // TtyOutput.lb = lbPricing;
      // TtyOutput.pricingTime = tm2.elapsed();
      // TtyOutput.totalTime = tm.elapsed();
      // TtyOutput.numCols = master.numColumns();
      // TtyOutput.memMB = memUsageMB();

      // TtyOutput.printData();

      // cout << "\n\n";

      cout << "@>>> Iter: " << iter << "   Elapsed: " << tm.elapsed() << " sec   Primal: " << rmp << "   Dual: " << lbPricing << "   NC: " << master.numColumns() << "\n";
      
      if (!newCols) {
         if (relaxed) {
            relaxed = false;
            // TtyOutput.mode = "E+" + heurPricing ? 'H' : 'E';
            cout << "***** CONVERTING MASTER RELAXATION. *****\n";
            master.setAssignmentType('E');
         } else {
            cout << "\nNo new columns generated.\nStopping the algorithm.\n";
            break;
         }
      }
      // if (iter == 4000) {
      //    for (int k = 0; k < inst.numDepots(); ++k) {
      //       pricing[k].reset(new PricingGlpk(inst, master, k));
      //    }
      //    cout << "Replacing pricing algorithms.\n";
      //    threads = 1;
      //    TtyOutput.mode = "E+E";
      // } else {
      //    threads = 8;
      // }
   }

   // master.solve();
   // cout << "\n" << master.getObjValue() << "\n";

   // master.writeLp("masterFinal.lp");
   return EXIT_SUCCESS;
}
