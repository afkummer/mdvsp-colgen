#pragma once

#include <iostream>
#include <string>
#include <limits>

/**
 * Defines the maximum number of arcs to explore during the pricing solver.
 * It only affects the root node of the column generation. 
 * Default value: max value of int32
 */
#define MAX_LABEL_EXPANSIONS "MAX_LABEL_EXPANSIONS"

/**
 * Defines the maximum number of arcs to explore during the pricing solver.
 * It only affects the truncated column generation step of the solver.
 * Default value: max value of int32
 */
#define MAX_LABEL_EXPANSIONS_TCG "MAX_LABEL_EXPANSIONS_TCG"

/**
 * Flags whether the adjacency list of deadhead arcs should be sorted by cost.
 * Default value: 0 (false)
 */
#define SORT_DEADHEAD_ARCS "SORT_DEADHEAD_ARCS"

/**
 * Limits the number of column generation iterations in each step of the
 * truncated column generation algorithm.
 * Default value: 20
 */
#define TCG_MAX_SUB_ITERATIONS "TCG_MAX_SUB_ITERATIONS"

inline auto getEnvMaxLabelExpansions() noexcept -> int {
   if (getenv(MAX_LABEL_EXPANSIONS)) {
      int value = std::stoi(getenv(MAX_LABEL_EXPANSIONS));
      if (value > 0) {
         std::cout << "Read MAX_LABEL_EXPANSIONS = " << value << "\n";
      } else {
         std::cout << "Bad value for MAX_LABEL_EXPANSIONS: " << getenv(MAX_LABEL_EXPANSIONS) << std::endl;
         exit(EXIT_FAILURE);
      }
   }
   return std::numeric_limits<int>::max();
}

inline auto getEnvMaxLabelExpansionsTcg() noexcept -> int {
   if (getenv(MAX_LABEL_EXPANSIONS_TCG)) {
      int value = std::stoi(getenv(MAX_LABEL_EXPANSIONS_TCG));
      if (value > 0) {
         std::cout << "Read MAX_LABEL_EXPANSIONS_TCG = " << value << "\n";
      } else {
         std::cout << "Bad value for MAX_LABEL_EXPANSIONS_TCG: " << getenv(MAX_LABEL_EXPANSIONS_TCG) << std::endl;
         exit(EXIT_FAILURE);
      }
   }
   return std::numeric_limits<int>::max();
}

inline auto getEnvSortDeadheadArcs() noexcept -> bool {
   if (getenv(SORT_DEADHEAD_ARCS)) {
      bool value = std::stoi(getenv(SORT_DEADHEAD_ARCS)) == 0 ? false : true;
      std::cout << "Read SORT_DEADHEAD_ARCS = " << value << "\n";
      return value;      
   }
   return false;
}

inline auto getEnvMaxTcgSubIter() noexcept -> int {
   if (getenv(TCG_MAX_SUB_ITERATIONS)) {
      int value = std::stoi(getenv(TCG_MAX_SUB_ITERATIONS));
      if (value > 0) {
         std::cout << "Read TCG_MAX_SUB_ITERATIONS = " << value << "\n";
      } else {
         std::cout << "Bad value for TCG_MAX_SUB_ITERATIONS: " << getenv(TCG_MAX_SUB_ITERATIONS) << std::endl;
         exit(EXIT_FAILURE);
      }
      return value;
   }
   return 20;
}
