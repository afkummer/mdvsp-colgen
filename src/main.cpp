#include "Instance.h"
#include "ModelCbc.h"
#include "Timer.h"
#include "MemQuery.h"
#include "colgen/CgMasterBase.h"
#include "colgen/CgMasterCplex.h"
#include "colgen/CgMasterGlpk.h"
#include "colgen/CgMasterClp.h"
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
      "the solution method is cg. Accepted values: glpk, cplex, clp")

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
   (void) parm;
   unique_ptr <ModelCbc> compact;
   cout << "Solution method of choice: compact formulation (Coin-OR CBC)\n";
  
   // Builds the model, recording elapsed time and memory consumption.
   Timer tm;
   tm.start();
   auto mem = getMemoryUsageKb();
   compact.reset(new ModelCbc(inst));
   double buildTime = tm.elapsed();
   double memMB = (getMemoryUsageKb()-mem)/1024.0;

   // Exports the model as a LP file, and computes the time spent in the operation.
   tm.start();
   compact->writeLp("compact.lp");
   double writeTime = tm.elapsed();  

   // Prints some useful information.
   cout << "Building the model took " << buildTime << " sec.\n";
   cout << "Memory consumed by the model: " << memMB << " MB\n";
   cout << "Compact model written to 'compact.lp'.\n";
   cout << "Time spent writing file: " << writeTime << " sec.\n";
   
   return EXIT_SUCCESS; 
}

auto solveColumnGeneration(const CmdParm &parm, const Instance &inst) noexcept -> int {
   // Uses smart pointers to manage the implementation of solution methods.
   // These automatically release all memory and any other resources when the 
   // objects go out of the scope (i.e., when this function returns).
   unique_ptr <CgMasterBase> master;
   vector<unique_ptr<CgPricingBase>> pricing;
   cout << "Solution method of choice: column generation\n";

   // Creates the master problem.
   const auto masterImpl = parm["master"].as<string>();
   Timer tm;
   tm.start();
   cout << "\nBuilding master problem.\n";
   if (masterImpl == "glpk") {
      master.reset(new CgMasterGlpk(inst));
   } else if (masterImpl == "cplex") {
      master.reset(new CgMasterCplex(inst));
   } else if (masterImpl == "clp") {
      master.reset(new CgMasterClp(inst));
   } else {
      cout << "Unknown implementation for RMP solver: " << masterImpl << ".\n";
      return EXIT_FAILURE;
   }
   cout << "Solver: " << master->getSolverName() << "\n";
   cout << "Build time: " << tm.elapsed() << " sec\n";
   cout << "Current memory usage: " << fixed << setprecision(2) << getMemoryUsageKb()/1024.0 << " MB\n";

   // Checks whether the option for import columns from a text file was set.
   if (parm.count("import-cols") != 0) {
      int nc = master->importColumns(parm["import-cols"].as<string>().c_str());
      cout << "Imported " << nc << " columns from " << parm["import-cols"].as<string>() << ".\n";
   }

   // Parses the parameter of max-paths.
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
   bool pricingPathControl = true;
   for (int k = 0; k < inst.numDepots(); ++k) {
      if (pricingImpl == "spfa") {
         pricing.emplace_back(make_unique<PricingSpfa>(inst, *master, k, maxPaths == 1));
         pricingPathControl = false;
      } else if (pricingImpl == "bellman") {
         pricing.emplace_back(make_unique<PricingBellman>(inst, *master, k, maxPaths == 1));
         pricingPathControl = false;
      } else if (pricingImpl == "glpk") {
         pricing.emplace_back(make_unique<PricingGlpk>(inst, *master, k, maxPaths));
         maxThreads = 1; // To circumvent problems with GLPK and multi-threading applications
         #ifdef MIP_PRICING_LP
            cout << "Solving pricing subproblems as LP.\n";
         #endif
      } else if (pricingImpl == "cbc") {
         pricing.emplace_back(make_unique<PricingCbc>(inst, *master, k, maxPaths));
         #ifdef MIP_PRICING_LP
            cout << "Solving pricing subproblems as LP.\n";
         #endif
      } else if (pricingImpl == "cplex") {
         pricing.emplace_back(make_unique<PricingCplex>(inst, *master, k, maxPaths));
         #ifdef MIP_PRICING_LP
            cout << "Solving pricing subproblems as LP.\n";
         #endif
      } else {
         cout << "Unknown implementation for pricing solver: " << pricingImpl << ".\n";
         return EXIT_FAILURE;
      }
   }
   tm.finish();
   cout << "Solver: " << pricing.front()->getSolverName() << "\n";
   if (maxPaths == 1) {
      cout << "Generating a single path per subproblem.\n";
   } else {
      if (pricingPathControl)
         cout << "Generating up to " << maxPaths << " paths per subproblem.\n";
      else
         cout << "WARNING: Fine-grained control of generated paths unavailable for this pricing solver.\n";
   }
   cout << "Build time: " << tm.elapsed() << " sec\n";
   cout << "Current memory usage: " << fixed << setprecision(2) << getMemoryUsageKb() / 1024.0 << " MB\n"; 
   
   // Variables that store the progress of the optimization.
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

   // CG - main loop
   cout << "\nStarting column generation.\n";
   tm.start();
   tmPrint.start();
   int iter = 0;
   for (iter = 0; !MdvspSigInt; ++iter) {
      Timer tmInner;
      tmInner.start();
      rmpObj = master->solve();
      timeMaster += tmInner.elapsed();

      // Solves the pricing subproblems.
      // This step can be done in parallel, with some observation when
      // using GLPK to solve the pricing subproblems.
      lbObj = rmpObj;      
      newCols = 0;
      tmInner.start();
      #pragma omp parallel for default(shared) schedule(static, 1) num_threads(maxThreads)
      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         // This method already takes the dual multipliers from the master.
         // All the work of updating subproblem obj is managed internally.
         sp->solve();
      }
      timePricing += tmInner.elapsed();

      // Test for negative reduced costs and add columns into RMP.
      for (size_t i = 0; i < pricing.size(); ++i) {
         auto &sp = pricing[i];
         const auto pobj = sp->getObjValue();

         lbObj += pobj;
         if (pobj <= -0.0001) {
            // We could find new columns with negative reduced cost!
            // Add them into RMP.
            newCols += sp->generateColumns();
         }
      }
      
      // Prints a log row.
      printLog(iter);
      
      // Check for stopping criterion.
      if (!newCols) {
         if (masterRelax) {
            masterRelax = false;
            cout << "***** CONVERTING MASTER RELAXATION. *****\n";
            master->setAssignmentType('E');
         } else {
            // Does a last print to ensure the correct output to the user.
            printLog(iter, true);

            cout << "\nNo new columns generated.\nStopping the algorithm.\n";
            break;
         }
      }

      // In case of using GLPK as subproblem solver: this changes the number of 
      // parallel threads to its maximum, from iteration 2 and forth.
      // This seems to be enough to circumvent GLPK difficulties when solving
      // multiple independent problems in parallel.
      maxThreads = omp_get_max_threads();
   }
   const auto totalTime = tm.elapsed();

   // Final solve to guarantee consistency.
   master->solve();

   if (MdvspSigInt) {
      cout << "\n\nWARNING: Optimization was interrupted. RMP relaxation likely to not be optimal.\n";
   } else {
      cout << "\n\n";
   }
   cout << "Value of RMP relaxation: " << master->getObjValue() << "\n";
   cout << "Total time spent: " << totalTime << " sec\n";
   cout << "Current memory consumption: " << setprecision(2) << getMemoryUsageKb()/1024.0 << " MB\n";

   master->writeLp("masterFinal.lp");
   cout << "\nRMP exported to 'masterFinal.lp'.\n";
   master->exportColumns("cols.txt");
   cout << "RMP columns exported to 'cols.txt'.\n";  

   return EXIT_SUCCESS;
}