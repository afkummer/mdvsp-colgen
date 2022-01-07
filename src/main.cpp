#include "Instance.h"
#include "ModelCbc.h"
#include "Timer.h"
#include "MemQuery.h"
#include "colgen/CgMasterBase.h"
#include "colgen/CgMasterCplex.h"
#include "colgen/CgMasterGlpk.h"
#include "colgen/PricingCplex.h"
#include "colgen/PricingBellman.h"
#include "colgen/PricingSpfa.h"
#include "colgen/PricingCbc.h"
#include "colgen/PricingGlpk.h"

#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <csignal>

#include <boost/program_options.hpp>

#include <omp.h>

using namespace std;

using CmdParm = boost::program_options::variables_map;

// Signals if the program received a CTRL+C signal.
volatile bool MdvspSigInt = false;

// Method to set the SigInt flag.
void sigintHandler(int s) {
   if (s == SIGINT) {
      MdvspSigInt = true;      
      cout << "MDVSP: SIGINT received." << endl;
   }
}

// Parses the command line arguments using boost::program_options library.
auto parseCommandline(int argc, char **argv) noexcept -> CmdParm;

// Solves a problem using compact formulation.
auto solveCompactModel(const CmdParm &parm, const Instance &inst) noexcept -> int;

// Solves a problem using column generation.
auto solveColumnGeneration(const CmdParm &parm, const Instance &inst) noexcept -> int;

auto main(int argc, char *argv[]) noexcept -> int {
   // Basic app initialization.
   const auto parm = parseCommandline(argc, argv);
   signal(SIGINT, sigintHandler);
   
   // Parses the problem data.
   Instance inst(parm["instance"].as<string>().c_str());

   cout << "--- MDVSP solver ---\n" <<
      "Instance: " << inst.fileName() << "\n" <<
      "Number of depots: " << inst.numDepots() << "\n" <<
      "Number of trips: " << inst.numTrips() << "\n\n";

   // Decides if compact formulation should be used when solving the problem.
   const auto methodName = parm["method"].as<string>();
   if (methodName == "compact") {
      return solveCompactModel(parm, inst);  
   } else if(methodName == "cg") {
      return solveColumnGeneration(parm, inst);
   } else {
      cout << "Unknown solution method: " << methodName << ".\n";
      return EXIT_FAILURE;
   }

   return EXIT_FAILURE;
}

auto parseCommandline(int argc, char **argv) noexcept -> CmdParm {
   namespace po = boost::program_options;
   po::options_description desc("Accepted command options are");
   desc.add_options()
      ("help,h", "shows this text")

      ("instance,i", po::value<string>(), "path to the instance file")

      ("method", po::value<string>()->default_value("cg"), "defines the algorithm to be employed "
      "for solving the problem. Accepted values: compact, cg")

      ("master,m", po::value<string>()->default_value("glpk"), "defines the implementation "
      "backend to use while solving the restricted relaxed master problem. Only has effect when "
      "the solution method is cg. Accepted values: glpk, cplex")

      ("pricing,p", po::value<string>()->default_value("spfa"), "defines the implementation "
      "backend to use while solving the pricing subproblems. Only has effect when "
      "the solution method is cg. Accepted values: spfa, bellman, glpk, cbc, cplex")

      ("max-paths", po::value<int>()->default_value(1), "defines the maximum number of paths, "
       "per iteration, to extract from pricing subproblems. The exact number of paths is only "
       "available when solving with glpk, cbc, or cplex. When using spfa and bellman algorithms, "
       "generates as many paths as possible with no limitation. Only has effect when "
       "the solution method is cg.")

      ("import-cols", po::value<string>(), "import columns stored in the text file 'arg'")
   ;

   po::variables_map vm;
   po::store(po::parse_command_line(argc, argv, desc), vm);
   po::notify(vm);    

   if (vm.count("help") > 0 or vm.count("instance") == 0) {
      cout << desc << "\n";
      exit(EXIT_FAILURE);
   }

   return vm;
}

auto solveCompactModel(const CmdParm &parm, const Instance &inst) noexcept -> int {
   unique_ptr <ModelCbc> compact;
   
   cout << "Solution method of choice: compact formulation (Coin-OR CBC)\n";
  
   auto mem = getMemoryUsageKb();
   Timer tm;
   tm.start();

   compact.reset(new ModelCbc(inst));
   double buildTime = tm.elapsed();

   tm.start();
   compact->writeLp("compact.lp");
   double writeTime = tm.elapsed();
   double memMB = (getMemoryUsageKb()-mem)/1024.0;

   cout << "Building the model took " << buildTime << " sec.\n";
   cout << "Memory consumed by the model: " << memMB << " MB.\n";
   cout << "Compact model written to 'compact.lp'.\n";
   cout << "Time spent writing file: " << writeTime << " sec.\n";
   
   return EXIT_SUCCESS; 
}

auto solveColumnGeneration(const CmdParm &parm, const Instance &inst) noexcept -> int {
   // Uses smart pointers to hold the implementation of solution methods.
   unique_ptr <CgMasterBase> master;
   vector<unique_ptr<CgPricingBase>> pricing;

   // Timer for computing elapsed time while buildining the models.
   Timer tm;
   cout << "Solution method of choice: column generation\n";

   // Creates the master problem.
   const auto masterImpl = parm["master"].as<string>();
   tm.start();
   cout << "\nBuilding master problem.\n";
   if (masterImpl == "glpk") {
      master.reset(new CgMasterGlpk(inst));
   } else  if (masterImpl == "cplex") {
      master.reset(new CgMasterCplex(inst));
   } else {
      cout << "Unknown implementation for RMP solver: " << masterImpl << ".\n";
      return EXIT_FAILURE;
   }
   cout << "Solver: " << master->getSolverName() << "\n";
   cout << "Time spent: " << tm.elapsed() << " sec\n";
   cout << "Current memory usage: " << setprecision(2) << getMemoryUsageKb()/1024.0 << " MB\n";

   if (parm.count("import-cols") != 0) {
      int nc = master->importColumns(parm["import-cols"].as<string>().c_str());
      cout << "Imported " << nc << " columns from " << parm["import-cols"].as<string>() << ".\n";
   }

   const int maxPaths = parm["max-paths"].as<int>();
   if (maxPaths <= 0) {
      cout << "Parameter --max-paths need to be >= 1.\n";
      return EXIT_FAILURE;
   }

   // Creates the pricing subproblems.
   int maxThreads = omp_get_max_threads(); 
   const auto pricingImpl = parm["pricing"].as<string>();
   tm.start();
   cout << "\nBuilding pricing subproblems.\n";
   string warnMsg = "";
   for (int k = 0; k < inst.numDepots(); ++k) {
      if (pricingImpl == "spfa") {
         pricing.emplace_back(make_unique<PricingSpfa>(inst, *master, k, maxPaths == 1));
         if (maxPaths > 1) {
            warnMsg = "Fine-grained control of path generation not available.";
         }
      } else if (pricingImpl == "bellman") {
         pricing.emplace_back(make_unique<PricingBellman>(inst, *master, k, maxPaths == 1));
         if (maxPaths > 1) {
            warnMsg = "Fine-grained control of path generation not available.";
         }
      } else if (pricingImpl == "glpk") {
         pricing.emplace_back(make_unique<PricingGlpk>(inst, *master, k, maxPaths));
         maxThreads = 1; // To circumvent problems with GLPK and multi-threading apps
      } else if (pricingImpl == "cbc") {
         pricing.emplace_back(make_unique<PricingCbc>(inst, *master, k, maxPaths));
      } else if (pricingImpl == "cplex") {
         pricing.emplace_back(make_unique<PricingCplex>(inst, *master, k, maxPaths));
      } else {
         cout << "Unknown implementation for pricing solver: " << pricingImpl << ".\n";
         return EXIT_FAILURE;
      }
   }
   tm.finish();
   cout << "Solver: " << pricing.front()->getSolverName() << "\n";
   if (!warnMsg.empty()) {
      cout << "WARNING: " << warnMsg << "\n";
   }
   cout << "Time spent: " << tm.elapsed() << " sec\n";
   cout << "Current memory usage: " << setprecision(2) << getMemoryUsageKb() / 1024.0 << " MB\n"; 
   
   bool masterRelax = true;
   double timeMaster = 0.0, timePricing = 0.0;
   double rmpObj = 0.0, lbObj = 0.0;
   int newCols = 0;

   // Lambda used to print optimization log.
   Timer tmPrint;
   int linesPrinted = 0;
   auto printLog = [&](int iter, bool force = false) -> void {
      if (linesPrinted % 15 == 0) {
         if (linesPrinted != 0)
            cout << "\n\n";
         cout <<
            setw(3) << "Asg" << 
            setw(6) << "Iter" <<
            setw(10) << "T.Time" << 
            setw(10) << "T.RMP" << 
            setw(10) << "T.SP" << 
            setw(16) << "UB   " << 
            setw(16) << "LB   " << 
            setw(12) << "Gap(%)" << 
            setw(15) << "N.Cols" <<
            setw(10) << "MemMB" << 
         "\n";         
      }

      if (force || tmPrint.elapsed() >= 0.3 or linesPrinted % 15 == 0) {
         double gap = (rmpObj-lbObj)/rmpObj*100.0;
         cout <<
            fixed << 
            setw(3) << (masterRelax ? "R" : "E") << 
            setw(6) << iter << 
            setw(10) << setprecision(2) << tm.elapsed() << 
            setw(10) << setprecision(2) << timeMaster << 
            setw(10) << setprecision(2) << timePricing << 
            setw(16) << setprecision(2) << rmpObj << 
            setw(16) << setprecision(2) << lbObj << 
            setw(12) << setprecision(2) << gap << 
            setw(15) << (to_string(master->numColumns()) + string("+") + to_string(newCols)) <<
            setw(10) << setprecision(2) << getMemoryUsageKb()/1024.0 << 
         "\n";
         ++linesPrinted;
         tmPrint.start();
      }
   };

   cout << "\nStarting column generation.\n";
   tm.start();
   tmPrint.start();
   int iter = 0;
   for (iter = 0; iter < 5000 and !MdvspSigInt; ++iter) {
      Timer tmInner;
      tmInner.start();
      rmpObj = master->solve();
      timeMaster = tmInner.elapsed();

      // Solves the pricing subproblems.
      // This step can be done in parallel, with some observation when
      // using GLPK to solve the pricing subproblems.
      lbObj = rmpObj;      
      newCols = 0;
      tmInner.start();
      #pragma omp parallel for default(shared) schedule(static, 1) num_threads(maxThreads)
      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         sp->solve();
      }
      timePricing = tmInner.elapsed();

      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         const auto pobj = sp->getObjValue();

         lbObj += pobj;
         if (pobj <= -0.0001) {
            newCols += sp->generateColumns();
         }
      }
      
      printLog(iter);

      // double mem = getMemoryUsageKb()/1024.0;
      // cout.precision(12);
      // cout << "@>>> Iter: " << iter << "   Elapsed: " << tm.elapsed() << " sec   Primal: " << rmp << "   Dual: " << lbPricing << "   NC: " << master->numColumns() << "   Mem: " << mem << "\n";
      
      if (!newCols) {
         if (masterRelax) {
            masterRelax = false;
            cout << "***** CONVERTING MASTER RELAXATION. *****\n";
            master->setAssignmentType('E');
         } else {
            cout << "\nNo new columns generated.\nStopping the algorithm.\n";
            break;
         }
      }
      maxThreads = omp_get_max_threads();
   }
   printLog(iter, true);
   const auto totalTime = tm.elapsed();

   // Final solve to guarantee consistency.
   master->solve();

   if (MdvspSigInt) {
      cout << "WARNING: Optimization was interrupted. RMP relaxation likely to not be optimal.\n";
   }
   cout << "\nValue of optimal RMP relaxation: " << master->getObjValue() << "\n";
   cout << "Total time spent: " << totalTime << " sec\n";
   cout << "Current memory consumption: " << setprecision(2) << getMemoryUsageKb()/1024.0 << " MB\n";

   master->writeLp("masterFinal.lp");
   cout << "RMP exported to 'masterFinal.lp'.\n";
   master->exportColumns("cols.txt");
   cout << "RMP columns exported to 'cols.txt'.\n";  

   return EXIT_SUCCESS;
}