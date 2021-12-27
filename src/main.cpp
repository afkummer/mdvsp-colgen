#include "Instance.h"
#include <iostream>
#include <iomanip>

using namespace std;

auto main(int argc, char *argv[]) noexcept -> int {
   if (argc != 2) {
      cout << "Usage: " << argv[0] << " <1:instance path>\n";
      return EXIT_FAILURE;
   }

   Instance inst{argv[1]};
   
   cout << inst.numDepots() << "\t" << inst.numTrips();
   for (int k = 0; k < inst.numDepots(); ++k) {
      cout << "\t" << inst.depotCapacity(k);
   }
   cout << "\n";

   const auto L = inst.numDepots() + inst.numTrips();
   for (int i = 0; i < L ; ++i) {
      for (int j = 0; j < L; ++j) {
         cout << inst.rawCost(i, j) << '\t';
      }
      cout << "\n";
   }

   return EXIT_SUCCESS;
}
