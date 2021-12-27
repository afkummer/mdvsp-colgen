#include "Instance.h"
#include "CgMasterGlpk.h"

#include <iostream>
#include <iomanip>

using namespace std;

auto main(int argc, char *argv[]) noexcept -> int {
   if (argc != 2) {
      cout << "Usage: " << argv[0] << " <1:instance path>\n";
      return EXIT_FAILURE;
   }

   Instance inst{argv[1]};
   CgMasterGlpk master{inst};

   master.beginColumn(0);
   master.addTrip(0);
   master.addTrip(10);
   master.commitColumn();

   master.writeLp("master.lp");

   const auto obj = master.solve();
   cout << "RMP = " << obj << "\n";

   // for (int i = 0; i < inst.numTrips(); ++i)
   //    cout << "Trip #" << i << " dual: " << master.getTripDual(i) << "\n";

   // for (int k = 0; k < inst.numDepots(); ++k)
   //    cout << "depot #" << k << " dual: " << master.getDepotCapDual(k) << "\n";


   return EXIT_SUCCESS;
}
